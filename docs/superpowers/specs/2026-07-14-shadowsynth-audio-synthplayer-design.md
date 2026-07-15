# ShadowSynth — `audio_synthplayer.h` Rewrite Design

**Date:** 2026-07-14
**Status:** Approved for implementation
**Scope:** Full rewrite of `audio_synthplayer.h` from scratch. Does not reuse the current (empty) file or the `_chat` / `_old` / `_weird_vibecode` variants, though patterns may be drawn from `audio_synthplayer_chat.h`.

---

## 1. Purpose

A single-voice subtractive synthesizer for the Teensy Audio library, driven by shadow-blob
data (`LDRBlob`) arriving over UART. Four oscillators feed one shared filter; a 1 kHz
`IntervalTimer` performs all smoothing, LFO modulation, and hardware writes. The engine also
hosts the SD-backed WAV player used elsewhere in the project.

The header is included by `Brain.ino` as `#include "audio_synthplayer.h"` (see `Brain.ino:10`).
It relies on `globals.h` for the `LDRBlob` struct, `DEBUG_MODE`, and reverb globals.

---

## 2. Audio graph

Single-voice subtractive synth. Four oscillators sum into one mixer, through one Biquad
filter, into the final/master mix with reverb, out to I2S as mono (duplicated L+R). The WAV
player joins at the final mixer.

```
saw ┐
sq  ├─ oscMixer ──→ biquadFilter ──→ finalMixer ──→ masterMixer ──→ I2S out (L + R, mono)
tri ┤                     ↑               ↑                ↑
sin ┘               (LFO sweeps      wavPlayer        freeverb (wet)
                     cutoff)
```

**Nodes**
- `AudioSynthWaveform sawWave, squareWave, triWave, sineWave`
- `AudioMixer4 oscMixer` — 4 oscillators on channels 0..3
- `AudioFilterBiquad biquadFilter` — one filter, hard-switched LP / off / HP
- `AudioPlaySdWav wavPlayer`
- `AudioMixer4 finalMixer` — filtered synth (dry) + wavPlayer
- `AudioEffectFreeverb freeverb`
- `AudioMixer4 masterMixer` — dry + wet
- `AudioOutputI2S audioOutput`, `AudioControlSGTL5000 sgtl5000_1`
- `IntervalTimer lfoTimer`

Waveform assignment: **sine, square, triangle, sawtooth** (the spec's "sine, square, tri, and
square" is corrected — the duplicate square becomes sawtooth).

---

## 3. Controllable static variables (top of file)

All values that a user might tune live at the top, grouped and commented. Logic reads these;
no magic numbers buried in functions.

### LFO frequency
- `float LFO_FREQ_MIN = 1.0f;` — Hz at `centerAngle` 0
- `float LFO_FREQ_MAX = 20.0f;` — Hz at `centerAngle` 100 (exponential map)

### Filter cutoff (Board 1)
- `float LP_CUT_MIN = 300.0f;` — low-pass cutoff at angle 0
- `float LP_CUT_MAX = 19000.0f;` — low-pass cutoff at angle 45
- `float HP_CUT_MIN = 20.0f;` — high-pass cutoff at angle 55
- `float HP_CUT_MAX = 4000.0f;` — high-pass cutoff at angle 100
- `float FILTER_Q = 0.7f;`
- `float FILTER_ABS_MIN = 0.0f;` — absolute lower bound for any cutoff write
- `float FILTER_ABS_MAX = 20000.0f;` — absolute upper bound for any cutoff write

### LFO depth + taper (applied to filter cutoff)
- `float LFO_DEPTH_OCT = 2.0f;` — full modulation depth, ± octaves
- `float LFO_DEPTH_OCT_MIN = 0.2f;` — tapered depth near the 20 kHz high end
- `float LFO_TAPER_START_HZ = 16000.0f;` — base cutoff above which depth begins tapering toward `LFO_DEPTH_OCT_MIN` at 20 kHz. Set near the 20 kHz high end so full ±2 oct is retained across the low and mid range; only the top ~4 kHz tapers.

### Oscillators (per-oscillator, index 0=sine, 1=square, 2=tri, 3=saw)
- `float oscVolume[4]        = { … };` — mix gain per oscillator
- `int8_t oscSemitoneOffset[4] = { … };` — integer semitone transpose from the selected note
- `float oscCentsOffset[4]   = { … };` — fine detune in cents

### WAV player
- `float WAV_PLAYER_VOLUME = 0.5f;`

### Smoothing amounts (0.0 … 1.0; higher = smoother/slower glide)
- `float smoothedCutoffFreqAmount = 0.90f;`
- `float smoothedLfoFreqAmount    = 0.90f;`
- `float smoothedNoteAmount       = 0.85f;`
- `float smoothedVolumeAmount     = 0.85f;`

Smoothing filter form (per the user's example):
`smoothedX = smoothedX * amount + target * (1.0 - amount);`

### Reverb
- `float ROOM_SIZE`, `float DAMPING`, `float DRY_GAIN`, `float WET_GAIN`
  (Reuses the reverb globals already defined in `globals.h`; redeclare here only if not pulled in.)

### Pins
- `#define SDCARD_CS_PIN 10`

---

## 4. Scale convention (important)

`LDRBlob.centerAngle` is a `uint8_t` percentage **0–100** (per `globals.h:39`). The spec's
numeric thresholds are interpreted **directly on this 0–100 scale**, not as raw 0–255 or as
literal degrees:

- Board 0: `centerAngle` 0 → 1 Hz, 100 → 20 Hz.
- Board 1: `centerAngle` 0–45 → low-pass, 45–55 → off, 55–100 → high-pass.
- Board 2: `centerAngle` 0–100 maps across the 10-note table.

---

## 5. Public API

```cpp
void audioSetup();
void modulateSynthFromBoards(LDRBlob blobs[3][3], uint8_t distancePct[3]);
void modulateSynthFromBoard0(LDRBlob boardBlobs[3], uint8_t distancePct); // LFO
void modulateSynthFromBoard1(LDRBlob boardBlobs[3], uint8_t distancePct); // filter
void modulateSynthFromBoard2(LDRBlob boardBlobs[3], uint8_t distancePct); // note
void lfoUpdate();                                    // 1 kHz IntervalTimer callback
void playWAVFile(const char *filename, bool waitToFinish = true);
void stopWAVPlayer();
```

Called from the main loop as `modulateSynthFromBoards(LDRBlobs, ultrasonicDistancePercent)`.

---

## 6. `audioSetup()`

1. `AudioMemory(120);`
2. `sgtl5000_1.enable(); sgtl5000_1.volume(…);`
3. WAV/SD init:
   ```cpp
   SPI.setMOSI(11);
   SPI.setSCK(13);
   if (!(SD.begin(SDCARD_CS_PIN))) {
     while (1) { Serial.println("Unable to access the SD card"); delay(500); }
   }
   ```
4. Begin all 4 oscillators (`WAVEFORM_SINE / _SQUARE / _TRIANGLE / _SAWTOOTH`), apply
   `oscVolume[]` to `oscMixer`, set an initial frequency.
5. Initialize `biquadFilter` (default centerAngle=50 → filter off / wide pass), reverb params,
   final/master mix gains.
6. `lfoTimer.begin(lfoUpdate, 1000);` (1000 µs = 1 kHz).

---

## 7. Modulation dispatch & fallback

`modulateSynthFromBoards`:
- Calls `modulateSynthFromBoard0/1/2` with each board's blob row and distance.
- Computes `synthActive = (blobs[0][0].size > 0) || (blobs[1][0].size > 0) || (blobs[2][0].size > 0)`.
  Writes it to a `volatile bool` target used by the volume glide.

Each `modulateSynthFromBoardN`:
- If `boardBlobs[0].size == 0`, use a local `effectiveAngle = 50` (the neutral default) instead
  of the blob's angle. The board still contributes its default; it does **not** by itself force silence.
- Writes **raw `volatile` target variables only**. No smoothing, no hardware writes here — the
  1 kHz timer owns those.

**Silence rule:** oscillator/master output goes to 0 **only when all three boards are empty**
(`synthActive == false`). Any shadow on any board keeps sound alive.

**Unused-hook inputs:** `centerRadius`, `size` (beyond the empty check), and
`distancePct` are accepted and wired in as **named, commented placeholders** for future
mapping. They do not affect sound in this build.

---

## 8. Board 0 — LFO frequency

`effectiveAngle` (0–100) → exponential map to `[LFO_FREQ_MIN, LFO_FREQ_MAX]`:

```
x = effectiveAngle / 100
targetLfoFreq = LFO_FREQ_MIN * pow(LFO_FREQ_MAX / LFO_FREQ_MIN, x)
```

Written to `volatile float g_targetLfoFreq`.

---

## 9. Board 1 — filter cutoff (one Biquad, hard-switch at midpoint)

From `effectiveAngle` (0–100):

- **0 … 45 → low-pass.** `t = angle/45`, log-map `LP_CUT_MIN → LP_CUT_MAX`.
  `filterMode = LOWPASS`.
- **45 … 55 → off.** Filter set to pass-through (very wide low-pass at `FILTER_ABS_MAX`, or the
  library's flat setting). `filterMode = OFF`. LFO does not sweep an off filter.
- **55 … 100 → high-pass.** `t = (angle-55)/45`, log-map `HP_CUT_MIN → HP_CUT_MAX`.
  `filterMode = HIGHPASS`.

Log map: `cut = lo * pow(hi/lo, t)`.

Hard switch at the midpoint — no crossfade between LP and HP. Writes `volatile` targets:
`g_targetCutoff`, `g_filterMode`.

---

## 10. Board 2 — note selection

`effectiveAngle` (0–100) indexes the 10-note table:

```
E2, A2, D3, E3, A3, D4, E4, A4, D5, E5
```

Frequencies (equal temperament, A4=440):
`E2=82.41, A2=110.00, D3=146.83, E3=164.81, A3=220.00, D4=293.66, E4=329.63, A4=440.00, D5=587.33, E5=659.25`

`index = constrain((angle * 10) / 100, 0, 9)` → `targetNoteFreq = NOTE_TABLE[index]`.
Written to `volatile float g_targetNoteFreq`.

Per-oscillator frequency (applied in the timer):
```
oscFreq[i] = g_smoothedNoteFreq
             * pow(2, oscSemitoneOffset[i] / 12.0)
             * pow(2, oscCentsOffset[i] / 1200.0)
```

---

## 11. `lfoUpdate()` — 1 kHz timer (owns all smoothing + writes)

Each tick (`dt = 0.001 s`):

1. **Glide smoothed values** toward their volatile targets, each with its own amount:
   - `g_smoothedLfoFreq`, `g_smoothedCutoff`, `g_smoothedNoteFreq`, `g_smoothedVolume`
     (volume target = `synthActive ? 1.0 : 0.0`).

2. **LFO phase:** `lfoPhase += 2π * g_smoothedLfoFreq * dt; wrap at 2π`.

3. **Depth taper (from smoothed BASE cutoff — decided: option A):**
   ```
   if base <= LFO_TAPER_START_HZ:  depthOct = LFO_DEPTH_OCT
   else: linear-interp depthOct from LFO_DEPTH_OCT (at LFO_TAPER_START_HZ)
         down to LFO_DEPTH_OCT_MIN (at FILTER_ABS_MAX = 20000)
   ```
   Taper only engages near the 20 kHz high end; the low end keeps full ±2 oct.

4. **Modulated cutoff:**
   `modCut = base * pow(2, depthOct * sin(lfoPhase))`

5. **Asymmetric clamp:** clamp `modCut` to `[FILTER_ABS_MIN, FILTER_ABS_MAX]`. Only the side
   that exceeds a bound is capped; the opposite side retains full swing (a natural consequence
   of clamping the instantaneous value each tick rather than shrinking depth symmetrically).
   Also respect the Biquad's practical minimum (a few Hz) so the library stays stable.

6. **Apply filter** by mode:
   - `LOWPASS`: `biquadFilter.setLowpass(0, modCut, FILTER_Q)`
   - `HIGHPASS`: `biquadFilter.setHighpass(0, modCut, FILTER_Q)`
   - `OFF`: flat pass-through (set once; LFO sweep skipped)

7. **Oscillator writes:** for each osc `i`, set frequency from `g_smoothedNoteFreq` +
   per-osc semitone/cents offsets (section 10), and set `oscMixer.gain(i, oscVolume[i] * g_smoothedVolume)`.

---

## 12. WAV player

```cpp
void playWAVFile(const char *filename, bool waitToFinish = true) {
  if (DEBUG_MODE) { Serial.print("Playing file: "); Serial.println(filename); }
  wavPlayer.play(filename);
  if (waitToFinish) { delay(25); while (wavPlayer.isPlaying()) {} }
}
void stopWAVPlayer() { if (wavPlayer.isPlaying()) wavPlayer.stop(); }
```
WAV mixed into `finalMixer` at `WAV_PLAYER_VOLUME`.

---

## 13. Concurrency & error handling

- Variables shared between the `modulate*` functions (loop context) and `lfoUpdate` (ISR) are
  `volatile`. All are single-word (`float`, `int`, `bool`), so writes are atomic on Teensy 4.x —
  no locking required.
- SD init failure → infinite halt loop with serial message (matches the supplied snippet).
- Filter cutoff is always clamped to `[FILTER_ABS_MIN, FILTER_ABS_MAX]` and a stable Biquad
  minimum before any hardware write, so the LFO can never drive it out of bounds.

---

## 14. Testing / verification

This is Teensy-hardware-dependent (Teensy Audio library, SGTL5000, SD), so there is no host
unit-test harness. Verification is on-device via a compact `DEBUG_MODE` serial block that prints
each computed value per update: `synthActive`, `g_smoothedLfoFreq`, `filterMode`,
base + modulated cutoff, `depthOct`, selected note index + frequency, and per-osc frequency.
The developer confirms the mappings (exponential LFO curve, LP/off/HP switch points, note table,
taper near 20 kHz, asymmetric clamp) against these prints on hardware.

---

## 15. Out of scope (YAGNI)

- Polyphony (single voice only).
- Mapping of `centerRadius`, `size`, and `ultrasonicDistancePercent` to sound (wired as hooks only).
- Crossfaded / morphing filter transition (hard switch by decision).
- Any change to `globals.h`, `blobMaker.h`, `Brain.ino` beyond what including the new header requires.
```
