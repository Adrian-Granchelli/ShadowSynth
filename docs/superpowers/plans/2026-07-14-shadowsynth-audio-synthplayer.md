# ShadowSynth `audio_synthplayer.h` Rewrite — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite `audio_synthplayer.h` from scratch as a single-voice subtractive Teensy synth driven by shadow-blob data, with a 1 kHz LFO/smoothing timer, a hard-switched Biquad filter, per-oscillator detune, and the SD-backed WAV player.

**Architecture:** Four `AudioSynthWaveform` oscillators (sine/square/tri/saw) sum into one `AudioMixer4`, feed one `AudioFilterBiquad` (LP / off / HP hard-switched by Board 1), then into a final mixer (with the WAV player) → reverb → master mixer → mono I2S out. The three `modulateSynthFromBoardN` functions only write `volatile` target variables; a single `IntervalTimer` callback (`lfoUpdate`, 1 kHz) owns all EMA smoothing, LFO phase, cutoff taper/clamp, and every hardware write.

**Tech Stack:** C++ (Arduino/Teensyduino), Teensy Audio Library (`Audio.h`), `IntervalTimer`, SGTL5000 audio shield, SD via SPI.

## Global Constraints

- **Single header, full rewrite:** replace the entire contents of `audio_synthplayer.h`. Do not `#include` or copy from `audio_synthplayer_chat.h`, `_old.h`, or `_weird_vibecode.h`.
- **Include guard:** `#ifndef audio_h` / `#define audio_h` / `#endif` (this exact guard name — `Brain.ino:10` includes it as `"audio_synthplayer.h"` and the project convention uses `audio_h`).
- **Depends on `globals.h`** for: `struct LDRBlob { uint8_t centerRadius, centerAngle, size; }`, `DEBUG_MODE`, and reverb globals `DRY_GAIN`, `WET_GAIN`, `ROOM_SIZE`, `DAMPING`. Do NOT redeclare these; `globals.h` is included before this header in `Brain.ino`.
- **Scale convention:** `centerAngle` is a `uint8_t` percentage **0–100**. All thresholds (45, 55, note splits) are on this 0–100 scale, NOT 0–255 or degrees.
- **Oscillator index convention (fixed everywhere):** `0 = sine`, `1 = square`, `2 = triangle`, `3 = sawtooth`.
- **Smoothing form (verbatim from spec):** `smoothedX = smoothedX * amount + target * (1.0f - amount);` where `amount` ∈ [0,1], higher = smoother/slower.
- **Concurrency:** every variable shared between the `modulate*` functions (loop context) and `lfoUpdate` (ISR) is declared `volatile`. All are single-word (`float`/`int`/`bool`) — atomic on Teensy 4.x, no locking.
- **Cutoff safety:** the value written to the Biquad is always clamped to `[FILTER_ABS_MIN, FILTER_ABS_MAX]` = `[0, 20000]` and to a stable Biquad floor before any hardware write.
- **Pins:** `#define SDCARD_CS_PIN 10`; SD init uses `SPI.setMOSI(11); SPI.setSCK(13);`.

## Verification Note (read before starting)

This is Teensy-hardware code with no host unit-test harness (Teensy Audio, SGTL5000, SD). "Tests" in this plan are of two kinds:

1. **Compile-verify** — the header must compile as part of the sketch. If `arduino-cli` is installed, use:
   `arduino-cli compile --fqbn teensy:avr:teensy40 /Users/marcbalaban/Desktop/Code/ShadowSynth` (adjust `teensy40`/`teensy41` to the actual board). If not installed, the developer compiles in the Arduino IDE (Teensyduino) — "Verify" button — and confirms 0 errors. **This environment has no arduino-cli**, so compile-verify is a manual developer step; each task states the expected result.
2. **Math self-check** — pure numeric mappings (LFO exp curve, LP/HP log maps, note table index, taper, asymmetric clamp) are verified by reasoning through documented sample inputs → expected outputs, printed on-device via the `DEBUG_MODE` block (Task 9). Each math task lists the sample points to confirm.

Commit after each task.

---

## File Structure

- **Modify (full rewrite):** `audio_synthplayer.h` — the entire synth engine. One file by project convention (the existing variants are all single headers). All sections below build up this one file top-to-bottom.
- **No other files change.** `Brain.ino` already `#include "audio_synthplayer.h"` and calls `audioSetup()` and (in the loop) will call `modulateSynthFromBoards(...)`. Wiring that call into the loop is out of scope for this plan (spec §15); the header only needs to expose the API.

Because everything lives in one header, tasks are ordered so the file compiles (or is at least self-consistent) at each commit: declarations first, then setup, then the pure-math modulation writers, then the timer that consumes them, then debug + final compile.

---

### Task 1: Header skeleton, includes, and controllable static variables

**Files:**
- Modify (rewrite): `/Users/marcbalaban/Desktop/Code/ShadowSynth/audio_synthplayer.h`

**Interfaces:**
- Consumes: `globals.h` symbols (`LDRBlob`, `DEBUG_MODE`, `DRY_GAIN`, `WET_GAIN`, `ROOM_SIZE`, `DAMPING`) — included transitively via `Brain.ino` before this header.
- Produces: all top-of-file tunable statics + `#define SDCARD_CS_PIN 10`, used by every later task. Exact names below.

- [ ] **Step 1: Replace the whole file with the guard, includes, and tunable-variable block**

```cpp
///////////////////////////////////////////////////
//// ShadowSynth — single-voice subtractive synth //
//// Driven by LDRBlob shadow data over UART      //
///////////////////////////////////////////////////

#ifndef audio_h
#define audio_h

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <IntervalTimer.h>

// NOTE: globals.h (LDRBlob, DEBUG_MODE, DRY_GAIN, WET_GAIN, ROOM_SIZE, DAMPING)
// is included by Brain.ino before this header. Do not redeclare those here.

// =====================================================================
//  CONTROLLABLE STATIC VARIABLES  (tune these; logic reads them)
// =====================================================================

// --- LFO frequency (Board 0) ---
float LFO_FREQ_MIN = 1.0f;    // Hz at centerAngle 0
float LFO_FREQ_MAX = 20.0f;   // Hz at centerAngle 100 (exponential map)

// --- Filter cutoff (Board 1) ---
float LP_CUT_MIN = 300.0f;    // low-pass cutoff at angle 0
float LP_CUT_MAX = 19000.0f;  // low-pass cutoff at angle 45
float HP_CUT_MIN = 20.0f;     // high-pass cutoff at angle 55
float HP_CUT_MAX = 4000.0f;   // high-pass cutoff at angle 100
float FILTER_Q   = 0.7f;
float FILTER_ABS_MIN = 0.0f;      // absolute lower bound for any cutoff write
float FILTER_ABS_MAX = 20000.0f;  // absolute upper bound for any cutoff write
float FILTER_STABLE_MIN = 20.0f;  // practical Biquad floor (never write below this)

// --- LFO depth + taper (applied to filter cutoff) ---
float LFO_DEPTH_OCT     = 2.0f;   // full modulation depth, +/- octaves
float LFO_DEPTH_OCT_MIN = 0.2f;   // tapered depth near the 20 kHz high end
float LFO_TAPER_START_HZ = 16000.0f; // base cutoff above which depth tapers toward
                                     // LFO_DEPTH_OCT_MIN at FILTER_ABS_MAX (20 kHz)

// --- Oscillators (index 0=sine, 1=square, 2=triangle, 3=saw) ---
float  oscVolume[4]         = { 0.25f, 0.20f, 0.25f, 0.20f };
int8_t oscSemitoneOffset[4] = { 0,     -12,   12,    0 };
float  oscCentsOffset[4]    = { 0.0f,  0.0f,  3.0f,  -3.0f };

// --- WAV player ---
float WAV_PLAYER_VOLUME = 0.5f;

// --- Smoothing amounts (0..1; higher = smoother/slower glide) ---
float smoothedCutoffFreqAmount = 0.90f;
float smoothedLfoFreqAmount    = 0.90f;
float smoothedNoteAmount       = 0.85f;
float smoothedVolumeAmount     = 0.85f;

// --- Reverb (reuses globals.h values; kept here as the tuning entry point) ---
// DRY_GAIN, WET_GAIN, ROOM_SIZE, DAMPING come from globals.h.

#define SDCARD_CS_PIN 10

#endif // audio_h
```

- [ ] **Step 2: Compile-verify (developer step)**

Run (developer, in Arduino IDE / Teensyduino, or `arduino-cli compile --fqbn teensy:avr:teensy40 .`):
Expected: **0 errors.** The sketch still uses `audio_synthplayer.h`; only tunables exist so far. (If `audioSetup()`/`modulateSynthFromBoards` are already referenced elsewhere, expect "not declared" errors from those call sites — that is acceptable at this task and resolved by Tasks 3 & 7. Note them and continue.)

- [ ] **Step 3: Commit**

```bash
git add audio_synthplayer.h
git commit -m "feat(synth): header skeleton + controllable static variables"
```

---

### Task 2: Audio graph — nodes and connections

**Files:**
- Modify: `/Users/marcbalaban/Desktop/Code/ShadowSynth/audio_synthplayer.h` (insert before `#define SDCARD_CS_PIN` — i.e. after the tunables, before the guard close)

**Interfaces:**
- Consumes: nothing new.
- Produces: audio objects `sineWave, squareWave, triWave, sawWave, oscMixer, biquadFilter, wavPlayer, finalMixer, freeverb, masterMixer, audioOutput, sgtl5000_1, lfoTimer` and their `AudioConnection`s. `oscMixer` channels 0..3 = sine/square/tri/saw. Consumed by Tasks 3 and 7.

- [ ] **Step 1: Insert the audio-graph declaration block** (immediately after the reverb comment, before `#define SDCARD_CS_PIN 10`)

```cpp
// =====================================================================
//  AUDIO GRAPH
// =====================================================================

// Oscillators (index convention: 0=sine 1=square 2=triangle 3=saw)
AudioSynthWaveform sineWave;
AudioSynthWaveform squareWave;
AudioSynthWaveform triWave;
AudioSynthWaveform sawWave;

AudioMixer4        oscMixer;      // ch0=sine ch1=square ch2=tri ch3=saw
AudioFilterBiquad  biquadFilter;  // one filter: LP / off / HP (hard switch)

AudioPlaySdWav     wavPlayer;

AudioMixer4        finalMixer;    // ch0=filtered synth (dry), ch1=wav
AudioEffectFreeverb freeverb;
AudioMixer4        masterMixer;   // ch0=dry, ch1=wet

AudioOutputI2S       audioOutput;
AudioControlSGTL5000 sgtl5000_1;

IntervalTimer lfoTimer;

// Routing: oscillators -> oscMixer
AudioConnection c_osc0(sineWave,   0, oscMixer, 0);
AudioConnection c_osc1(squareWave, 0, oscMixer, 1);
AudioConnection c_osc2(triWave,    0, oscMixer, 2);
AudioConnection c_osc3(sawWave,    0, oscMixer, 3);

// oscMixer -> filter -> finalMixer (dry synth)
AudioConnection c_oscToFilter(oscMixer,     0, biquadFilter, 0);
AudioConnection c_filterToFinal(biquadFilter, 0, finalMixer,  0);

// wav -> finalMixer (mono: take left channel of the WAV player)
AudioConnection c_wavToFinal(wavPlayer, 0, finalMixer, 1);

// finalMixer -> masterMixer (dry) and -> freeverb -> masterMixer (wet)
AudioConnection c_finalToMasterDry(finalMixer, 0, masterMixer, 0);
AudioConnection c_finalToVerb(finalMixer,      0, freeverb,    0);
AudioConnection c_verbToMasterWet(freeverb,    0, masterMixer, 1);

// masterMixer -> both output channels (mono)
AudioConnection c_masterToOutL(masterMixer, 0, audioOutput, 0);
AudioConnection c_masterToOutR(masterMixer, 0, audioOutput, 1);
```

- [ ] **Step 2: Compile-verify (developer step)**

Expected: **0 new errors** from the graph block (same caveat as Task 1 about `audioSetup`/`modulate*` call sites not yet defined).

- [ ] **Step 3: Commit**

```bash
git add audio_synthplayer.h
git commit -m "feat(synth): audio graph nodes and connections"
```

---

### Task 3: `audioSetup()` — init audio, SD/WAV, oscillators, filter, reverb, timer

**Files:**
- Modify: `/Users/marcbalaban/Desktop/Code/ShadowSynth/audio_synthplayer.h` (insert after the audio graph, before `#endif`)

**Interfaces:**
- Consumes: all tunables (Task 1), all audio nodes (Task 2), and `lfoUpdate` (Task 7 — forward-declared here so setup can start the timer).
- Produces: `void audioSetup();` called by `Brain.ino:27`.

- [ ] **Step 1: Add a forward declaration for `lfoUpdate` and the smoothed-state + volatile-target variables**

Insert **after the audio-graph block** (these are needed by both `audioSetup` and later tasks; defining them here keeps the file compiling in order):

```cpp
// =====================================================================
//  MODULATION STATE
//  volatile targets: written by modulate*(), read by lfoUpdate() ISR
//  smoothed values: owned entirely by lfoUpdate()
// =====================================================================
enum FilterMode : uint8_t { FILTER_LOWPASS = 0, FILTER_OFF = 1, FILTER_HIGHPASS = 2 };

volatile float g_targetLfoFreq  = 1.0f;      // Hz
volatile float g_targetCutoff   = 1000.0f;   // Hz (base, pre-LFO)
volatile uint8_t g_filterMode   = FILTER_OFF;
volatile float g_targetNoteFreq = 82.41f;    // Hz (E2)
volatile bool  g_synthActive    = false;     // false => glide volume to 0

// smoothed (ISR-owned)
float g_smoothedLfoFreq  = 1.0f;
float g_smoothedCutoff   = 1000.0f;
float g_smoothedNoteFreq = 82.41f;
float g_smoothedVolume   = 0.0f;

float g_lfoPhase = 0.0f;

void lfoUpdate(); // defined in Task 7
```

- [ ] **Step 2: Add `audioSetup()`**

```cpp
// =====================================================================
//  SETUP
// =====================================================================
void audioSetup() {
  AudioMemory(120);

  sgtl5000_1.enable();
  sgtl5000_1.volume(0.75f);

  // --- SD / WAV player init (SPI pins per hardware) ---
  SPI.setMOSI(11);
  SPI.setSCK(13);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    while (1) { Serial.println("Unable to access the SD card"); delay(500); }
  }

  // --- Oscillators ---
  sineWave.begin(WAVEFORM_SINE);
  squareWave.begin(WAVEFORM_SQUARE);
  triWave.begin(WAVEFORM_TRIANGLE);
  sawWave.begin(WAVEFORM_SAWTOOTH);

  AudioSynthWaveform* oscs[4] = { &sineWave, &squareWave, &triWave, &sawWave };
  for (uint8_t i = 0; i < 4; i++) {
    oscs[i]->amplitude(1.0f);            // per-osc level lives in oscMixer gains
    oscs[i]->frequency(g_targetNoteFreq);
    oscMixer.gain(i, oscVolume[i] * 0.0f); // start silent; volume glides up when active
  }

  // --- Filter: start in "off"/wide-open state (centerAngle 50 default) ---
  biquadFilter.setLowpass(0, FILTER_ABS_MAX, FILTER_Q);

  // --- Final / master mix ---
  finalMixer.gain(0, 1.0f);              // filtered synth (dry)
  finalMixer.gain(1, WAV_PLAYER_VOLUME); // wav
  masterMixer.gain(0, DRY_GAIN);
  masterMixer.gain(1, WET_GAIN);

  // --- Reverb ---
  freeverb.roomsize(ROOM_SIZE);
  freeverb.damping(DAMPING);

  // --- Start the 1 kHz modulation timer (1000 us) ---
  lfoTimer.begin(lfoUpdate, 1000);
}
```

- [ ] **Step 3: Compile-verify (developer step)**

Expected: **0 errors** for `audioSetup` itself. `lfoUpdate` is forward-declared, so linking succeeds once Task 7 defines it; before Task 7 the IDE "Verify" will error on the missing `lfoUpdate` definition — that is expected and resolved in Task 7. If executing tasks strictly in order, note it and proceed.

- [ ] **Step 4: Commit**

```bash
git add audio_synthplayer.h
git commit -m "feat(synth): audioSetup + modulation state variables"
```

---

### Task 4: Board 0 writer — LFO frequency (exponential map)

**Files:**
- Modify: `/Users/marcbalaban/Desktop/Code/ShadowSynth/audio_synthplayer.h` (insert after `audioSetup`, before `#endif`)

**Interfaces:**
- Consumes: `LFO_FREQ_MIN/MAX`, `g_targetLfoFreq`.
- Produces: `void modulateSynthFromBoard0(LDRBlob boardBlobs[3], uint8_t distancePct);` and helper `float mapAngleExp(float angle01, float lo, float hi);`. Board2/Board1 writers reuse the empty-check pattern; the exp helper is Board0-only.

- [ ] **Step 1: Add the exponential helper and Board 0 writer**

```cpp
// =====================================================================
//  MODULATION WRITERS  (loop context; write volatile targets only)
// =====================================================================

// Exponential map: angle01 in [0,1] -> [lo,hi] exponentially. lo>0, hi>0.
inline float mapAngleExp(float angle01, float lo, float hi) {
  if (angle01 < 0.0f) angle01 = 0.0f;
  if (angle01 > 1.0f) angle01 = 1.0f;
  return lo * powf(hi / lo, angle01);
}

// BOARD 0 -> LFO frequency.
// Empty board (blob[0].size==0) falls back to centerAngle=50 (neutral),
// it does NOT force silence (see modulateSynthFromBoards).
// distancePct: unused hook for now.
void modulateSynthFromBoard0(LDRBlob boardBlobs[3], uint8_t distancePct) {
  (void)distancePct; // UNUSED HOOK: reserved for future mapping
  uint8_t angle = (boardBlobs[0].size == 0) ? 50 : boardBlobs[0].centerAngle;
  // UNUSED HOOKS: boardBlobs[0].centerRadius, boardBlobs[0].size (beyond empty check)

  float a01 = angle / 100.0f;
  g_targetLfoFreq = mapAngleExp(a01, LFO_FREQ_MIN, LFO_FREQ_MAX);
}
```

- [ ] **Step 2: Math self-check (reason through, confirm on-device via Task 9 prints)**

- `angle=0`   → `a01=0`   → `1.0 * (20/1)^0  = 1.0 Hz`  ✓ (LFO_FREQ_MIN)
- `angle=100` → `a01=1`   → `1.0 * (20/1)^1  = 20.0 Hz` ✓ (LFO_FREQ_MAX)
- `angle=50`  → `a01=0.5` → `1.0 * 20^0.5   ≈ 4.47 Hz` (geometric midpoint — exponential, as specified)
- `size==0`   → uses angle 50 → ≈4.47 Hz (fallback), not silence.

- [ ] **Step 3: Compile-verify (developer step)** — Expected: 0 new errors.

- [ ] **Step 4: Commit**

```bash
git add audio_synthplayer.h
git commit -m "feat(synth): Board 0 LFO-frequency modulation writer"
```

---

### Task 5: Board 1 writer — filter mode + cutoff (LP / off / HP, hard switch)

**Files:**
- Modify: `/Users/marcbalaban/Desktop/Code/ShadowSynth/audio_synthplayer.h` (insert after Board 0 writer)

**Interfaces:**
- Consumes: `LP_CUT_MIN/MAX`, `HP_CUT_MIN/MAX`, `g_targetCutoff`, `g_filterMode`, `FilterMode` enum, `FILTER_ABS_MAX`.
- Produces: `void modulateSynthFromBoard1(LDRBlob boardBlobs[3], uint8_t distancePct);` and helper `float mapLog(float t01, float lo, float hi);`.

- [ ] **Step 1: Add the log helper and Board 1 writer**

```cpp
// Log (== exponential-in-t) map: t01 in [0,1] -> [lo,hi]. lo>0, hi>0.
inline float mapLog(float t01, float lo, float hi) {
  if (t01 < 0.0f) t01 = 0.0f;
  if (t01 > 1.0f) t01 = 1.0f;
  return lo * powf(hi / lo, t01);
}

// BOARD 1 -> filter mode + base cutoff. Hard switch at the 45/55 boundaries.
//   angle 0..45   -> low-pass,  LP_CUT_MIN..LP_CUT_MAX  (log)
//   angle 45..55  -> filter off (wide-open low-pass at FILTER_ABS_MAX)
//   angle 55..100 -> high-pass, HP_CUT_MIN..HP_CUT_MAX  (log)
void modulateSynthFromBoard1(LDRBlob boardBlobs[3], uint8_t distancePct) {
  (void)distancePct; // UNUSED HOOK
  uint8_t angle = (boardBlobs[0].size == 0) ? 50 : boardBlobs[0].centerAngle;
  // UNUSED HOOKS: boardBlobs[0].centerRadius, boardBlobs[0].size (beyond empty check)

  if (angle <= 45) {
    float t = angle / 45.0f;                    // 0..1 across the LP band
    g_targetCutoff = mapLog(t, LP_CUT_MIN, LP_CUT_MAX);
    g_filterMode   = FILTER_LOWPASS;
  } else if (angle >= 55) {
    float t = (angle - 55) / 45.0f;             // 0..1 across the HP band
    g_targetCutoff = mapLog(t, HP_CUT_MIN, HP_CUT_MAX);
    g_filterMode   = FILTER_HIGHPASS;
  } else {
    g_targetCutoff = FILTER_ABS_MAX;            // off: wide open
    g_filterMode   = FILTER_OFF;
  }
}
```

- [ ] **Step 2: Math self-check**

- `angle=0`  → LP, `t=0`   → `300 Hz`   ✓
- `angle=45` → LP, `t=1`   → `19000 Hz` ✓
- `angle=50` → OFF → cutoff `20000`, mode OFF ✓ (neutral default region)
- `angle=55` → HP, `t=0`   → `20 Hz`    ✓
- `angle=100`→ HP, `t=1`   → `4000 Hz`  ✓
- `size==0`  → angle 50 → OFF (default), not silence ✓

- [ ] **Step 3: Compile-verify (developer step)** — Expected: 0 new errors.

- [ ] **Step 4: Commit**

```bash
git add audio_synthplayer.h
git commit -m "feat(synth): Board 1 filter mode + cutoff modulation writer"
```

---

### Task 6: Board 2 writer — note selection (10-note table)

**Files:**
- Modify: `/Users/marcbalaban/Desktop/Code/ShadowSynth/audio_synthplayer.h` (insert after Board 1 writer)

**Interfaces:**
- Consumes: `g_targetNoteFreq`.
- Produces: `void modulateSynthFromBoard2(LDRBlob boardBlobs[3], uint8_t distancePct);` and `const float NOTE_TABLE[10];`.

- [ ] **Step 1: Add the note table and Board 2 writer**

```cpp
// 10-note table: E2 A2 D3 E3 A3 D4 E4 A4 D5 E5 (equal temperament, A4=440)
const float NOTE_TABLE[10] = {
  82.41f,  // E2
  110.00f, // A2
  146.83f, // D3
  164.81f, // E3
  220.00f, // A3
  293.66f, // D4
  329.63f, // E4
  440.00f, // A4
  587.33f, // D5
  659.25f  // E5
};

// BOARD 2 -> selected note frequency (index into NOTE_TABLE across angle).
void modulateSynthFromBoard2(LDRBlob boardBlobs[3], uint8_t distancePct) {
  (void)distancePct; // UNUSED HOOK
  uint8_t angle = (boardBlobs[0].size == 0) ? 50 : boardBlobs[0].centerAngle;
  // UNUSED HOOKS: boardBlobs[0].centerRadius, boardBlobs[0].size (beyond empty check)

  int idx = (angle * 10) / 100;         // 0..10
  if (idx > 9) idx = 9;                  // angle==100 clamps to last note
  if (idx < 0) idx = 0;
  g_targetNoteFreq = NOTE_TABLE[idx];
}
```

- [ ] **Step 2: Math self-check**

- `angle=0`   → `idx=0`  → E2 (82.41) ✓
- `angle=9`   → `idx=0`  → E2 (each note spans a 10-wide angle band)
- `angle=10`  → `idx=1`  → A2 (110.00) ✓
- `angle=95`  → `idx=9`  → E5 (659.25)
- `angle=100` → `idx=10`→ clamped to 9 → E5 ✓
- `angle=50`  → `idx=5`  → D4 (293.66) (default when board empty) ✓

- [ ] **Step 3: Compile-verify (developer step)** — Expected: 0 new errors.

- [ ] **Step 4: Commit**

```bash
git add audio_synthplayer.h
git commit -m "feat(synth): Board 2 note-selection modulation writer"
```

---

### Task 7: `lfoUpdate()` — 1 kHz smoothing, LFO, taper, asymmetric clamp, hardware writes

**Files:**
- Modify: `/Users/marcbalaban/Desktop/Code/ShadowSynth/audio_synthplayer.h` (insert after Board 2 writer)

**Interfaces:**
- Consumes: all `g_target*` / `g_smoothed*` state, all tunables, `oscMixer`, `biquadFilter`, oscillator objects, `oscSemitoneOffset/CentsOffset/Volume`.
- Produces: `void lfoUpdate();` (definition matching the Task 3 forward declaration). Consumed by the `IntervalTimer` started in `audioSetup`.

- [ ] **Step 1: Add a small EMA helper (once) and the `lfoUpdate` definition**

```cpp
// =====================================================================
//  1 kHz MODULATION TIMER  (owns all smoothing + hardware writes)
// =====================================================================

// EMA glide: higher `amount` = smoother/slower. (spec's smoothing form)
inline float glide(float smoothed, float target, float amount) {
  return smoothed * amount + target * (1.0f - amount);
}

void lfoUpdate() {
  const float dt = 0.001f; // 1 kHz

  // 1) Glide smoothed values toward volatile targets.
  g_smoothedLfoFreq  = glide(g_smoothedLfoFreq,  g_targetLfoFreq,  smoothedLfoFreqAmount);
  g_smoothedCutoff   = glide(g_smoothedCutoff,   g_targetCutoff,   smoothedCutoffFreqAmount);
  g_smoothedNoteFreq = glide(g_smoothedNoteFreq, g_targetNoteFreq, smoothedNoteAmount);
  float volTarget    = g_synthActive ? 1.0f : 0.0f;
  g_smoothedVolume   = glide(g_smoothedVolume,   volTarget,        smoothedVolumeAmount);

  // 2) Advance LFO phase.
  g_lfoPhase += 2.0f * PI * g_smoothedLfoFreq * dt;
  if (g_lfoPhase > 2.0f * PI) g_lfoPhase -= 2.0f * PI;

  // 3) Oscillator frequencies + volume (per-osc semitone/cents offset).
  for (uint8_t i = 0; i < 4; i++) {
    float f = g_smoothedNoteFreq
              * powf(2.0f, oscSemitoneOffset[i] / 12.0f)
              * powf(2.0f, oscCentsOffset[i] / 1200.0f);
    switch (i) {
      case 0: sineWave.frequency(f);   break;
      case 1: squareWave.frequency(f); break;
      case 2: triWave.frequency(f);    break;
      case 3: sawWave.frequency(f);    break;
    }
    oscMixer.gain(i, oscVolume[i] * g_smoothedVolume);
  }

  // 4) Filter. If OFF, set wide-open once and skip the LFO sweep.
  if (g_filterMode == FILTER_OFF) {
    biquadFilter.setLowpass(0, FILTER_ABS_MAX, FILTER_Q);
    return;
  }

  // 4a) Depth taper from the smoothed BASE cutoff (option A):
  //     full LFO_DEPTH_OCT below LFO_TAPER_START_HZ, tapering linearly to
  //     LFO_DEPTH_OCT_MIN at FILTER_ABS_MAX (20 kHz).
  float base = g_smoothedCutoff;
  float depthOct = LFO_DEPTH_OCT;
  if (base > LFO_TAPER_START_HZ) {
    float span = FILTER_ABS_MAX - LFO_TAPER_START_HZ; // e.g. 20000-16000
    float t = (base - LFO_TAPER_START_HZ) / span;     // 0..1
    if (t > 1.0f) t = 1.0f;
    depthOct = LFO_DEPTH_OCT + (LFO_DEPTH_OCT_MIN - LFO_DEPTH_OCT) * t;
  }

  // 4b) LFO-modulated cutoff, then asymmetric clamp (clamp the instantaneous
  //     value each tick; the non-clipping side keeps full swing).
  float modCut = base * powf(2.0f, depthOct * sinf(g_lfoPhase));
  if (modCut > FILTER_ABS_MAX) modCut = FILTER_ABS_MAX;
  if (modCut < FILTER_ABS_MIN) modCut = FILTER_ABS_MIN;
  if (modCut < FILTER_STABLE_MIN) modCut = FILTER_STABLE_MIN; // Biquad stability floor

  // 4c) Apply by mode.
  if (g_filterMode == FILTER_LOWPASS) {
    biquadFilter.setLowpass(0, modCut, FILTER_Q);
  } else { // FILTER_HIGHPASS
    biquadFilter.setHighpass(0, modCut, FILTER_Q);
  }
}
```

- [ ] **Step 2: Math self-check**

- **Depth taper:** `base=1000` → `depthOct=2.0` (full); `base=16000` → `2.0`; `base=18000` → `t=0.5` → `2.0 + (0.2-2.0)*0.5 = 1.1`; `base=20000` → `t=1` → `0.2`. ✓ (only the top ~4 kHz tapers).
- **Asymmetric clamp:** LP with `base=15000`, depth 2 oct: peak `= 15000*2^2 = 60000` → clamped to `20000`; trough `= 15000*2^-2 = 3750` → passes untouched. So the low side keeps full swing while only the high side caps. ✓
- **OFF mode:** early-return sets wide-open filter, no sweep. ✓
- **Volume glide:** `g_synthActive=false` → target 0 → volume EMA-glides down (no click). ✓

- [ ] **Step 3: Compile-verify (developer step)**

Expected: **0 errors, full sketch links now** (`lfoUpdate` definition satisfies the Task 3 forward declaration and the `lfoTimer.begin` reference). This is the first task where the whole header is expected to compile clean end-to-end (modulo Task 8's dispatcher, which `Brain.ino`'s loop will eventually call).

- [ ] **Step 4: Commit**

```bash
git add audio_synthplayer.h
git commit -m "feat(synth): 1kHz lfoUpdate — smoothing, LFO taper, asymmetric clamp, HW writes"
```

---

### Task 8: `modulateSynthFromBoards()` dispatcher + silence rule

**Files:**
- Modify: `/Users/marcbalaban/Desktop/Code/ShadowSynth/audio_synthplayer.h` (insert after `lfoUpdate`)

**Interfaces:**
- Consumes: `modulateSynthFromBoard0/1/2`, `g_synthActive`.
- Produces: `void modulateSynthFromBoards(LDRBlob blobs[3][3], uint8_t distancePct[3]);` — the entry point called from `Brain.ino`'s loop as `modulateSynthFromBoards(LDRBlobs, ultrasonicDistancePercent)`.

- [ ] **Step 1: Add the dispatcher**

```cpp
// =====================================================================
//  TOP-LEVEL DISPATCH
//  Called as: modulateSynthFromBoards(LDRBlobs, ultrasonicDistancePercent)
// =====================================================================
void modulateSynthFromBoards(LDRBlob blobs[3][3], uint8_t distancePct[3]) {
  modulateSynthFromBoard0(blobs[0], distancePct[0]); // LFO
  modulateSynthFromBoard1(blobs[1], distancePct[1]); // filter
  modulateSynthFromBoard2(blobs[2], distancePct[2]); // note

  // Silence ONLY when all three boards are empty. Any shadow keeps sound alive.
  g_synthActive = (blobs[0][0].size > 0) ||
                  (blobs[1][0].size > 0) ||
                  (blobs[2][0].size > 0);
}
```

- [ ] **Step 2: Signature check against the call site**

`globals.h` declares `LDRBlob LDRBlobs[3][3];` and `uint8_t ultrasonicDistancePercent[3];`. The parameter types `LDRBlob blobs[3][3]` and `uint8_t distancePct[3]` match exactly, so `modulateSynthFromBoards(LDRBlobs, ultrasonicDistancePercent)` compiles. Each `blobs[n]` decays to `LDRBlob[3]`, matching `modulateSynthFromBoardN(LDRBlob[3], uint8_t)`. ✓

- [ ] **Step 3: Compile-verify (developer step)** — Expected: 0 errors.

- [ ] **Step 4: Commit**

```bash
git add audio_synthplayer.h
git commit -m "feat(synth): modulateSynthFromBoards dispatcher + all-empty silence rule"
```

---

### Task 9: WAV player functions + optional debug block

**Files:**
- Modify: `/Users/marcbalaban/Desktop/Code/ShadowSynth/audio_synthplayer.h` (insert after the dispatcher, before `#endif`)

**Interfaces:**
- Consumes: `wavPlayer`, `DEBUG_MODE`, modulation state.
- Produces: `void playWAVFile(const char*, bool);`, `void stopWAVPlayer();`, `void debugPrintSynthState();`. `playWAVFile`/`stopWAVPlayer` are used by `Brain.ino` (`playWAVFile("START.WAV", false)` etc.).

- [ ] **Step 1: Add the WAV functions (exact signatures the project already uses)**

```cpp
// =====================================================================
//  WAV PLAYER
// =====================================================================
void playWAVFile(const char *filename, bool waitToFinish = true) {
  if (DEBUG_MODE) { Serial.print("Playing file: "); Serial.println(filename); }
  wavPlayer.play(filename);
  if (waitToFinish) {
    delay(25);
    while (wavPlayer.isPlaying()) {}
  }
}

void stopWAVPlayer() {
  if (wavPlayer.isPlaying()) {
    wavPlayer.stop();
  }
}
```

- [ ] **Step 2: Add the debug print (on-device verification of every mapping)**

```cpp
// =====================================================================
//  DEBUG  — call from the main loop (throttled) to verify mappings.
// =====================================================================
void debugPrintSynthState() {
  if (!DEBUG_MODE) return;
  Serial.print("active="); Serial.print(g_synthActive);
  Serial.print(" lfoHz="); Serial.print(g_smoothedLfoFreq, 2);
  Serial.print(" mode=");  Serial.print(g_filterMode); // 0=LP 1=OFF 2=HP
  Serial.print(" baseCut="); Serial.print(g_smoothedCutoff, 0);
  Serial.print(" note=");  Serial.print(g_smoothedNoteFreq, 2);
  Serial.print(" vol=");   Serial.println(g_smoothedVolume, 3);
}
```

- [ ] **Step 3: Compile-verify (developer step)**

Expected: **0 errors; full sketch compiles and links.** This is the final end-to-end compile gate. Developer confirms in Arduino IDE / Teensyduino (or `arduino-cli compile --fqbn teensy:avr:teensyXX .`).

- [ ] **Step 4: On-device smoke check (developer step, hardware required)**

Flash to the Teensy. With `DEBUG_MODE = true`, call `debugPrintSynthState()` from the loop (throttled). Confirm against the math self-checks in Tasks 4–7:
- No shadows on any board → `active=0`, `vol` glides to ~0 (silence). ✓
- Board 0 angle sweep 0→100 → `lfoHz` moves ~1→20 exponentially.
- Board 1 angle 20 → `mode=0` (LP); angle 50 → `mode=1` (OFF); angle 80 → `mode=2` (HP).
- Board 2 angle sweep → `note` steps through 82.41 … 659.25.
- Filter audibly wobbles (LFO) and never clips silent at extremes (asymmetric clamp).

- [ ] **Step 5: Commit**

```bash
git add audio_synthplayer.h
git commit -m "feat(synth): WAV player functions + debug state print"
```

---

## Self-Review

**1. Spec coverage** (each spec section → task):
- §2 audio graph → Task 2 ✓
- §3 controllable statics → Task 1 ✓
- §4 scale convention (0–100) → applied in Tasks 4/5/6 (`/100`, thresholds 45/55, `*10/100`) ✓
- §5 public API → Tasks 3,4,5,6,7,8,9 (all signatures present) ✓
- §6 audioSetup → Task 3 ✓
- §7 dispatch + fallback + silence + unused hooks → Task 8 (dispatch/silence) + Tasks 4/5/6 (per-board `size==0`→angle 50 fallback, `(void)distancePct` + UNUSED HOOK comments) ✓
- §8 Board 0 exp LFO → Task 4 ✓
- §9 Board 1 LP/off/HP hard switch → Task 5 ✓
- §10 Board 2 note table + per-osc offsets → Task 6 (table/select) + Task 7 (per-osc semitone/cents applied) ✓
- §11 lfoUpdate smoothing/LFO/taper/asymmetric clamp/writes → Task 7 ✓
- §12 WAV player → Task 9 ✓
- §13 concurrency (volatile) + SD halt → Task 1/3 (volatile decls, SD while(1)) ✓
- §14 testing (DEBUG print) → Task 9 debug block ✓
- §15 out-of-scope (hooks only, single voice, hard switch, no globals.h change) → respected; no task touches other files ✓

No spec requirement is left without a task.

**2. Placeholder scan:** No "TBD/TODO/implement later." `(void)distancePct` + "UNUSED HOOK" are deliberate, spec-mandated placeholders (§7), each with a real comment, not vague instructions. Every code step shows complete code.

**3. Type consistency:**
- `g_targetLfoFreq/Cutoff/NoteFreq` (float), `g_filterMode` (uint8_t, `FilterMode` enum values), `g_synthActive` (bool) — declared once in Task 3, read/written with matching types in Tasks 4–8.
- `modulateSynthFromBoardN(LDRBlob[3], uint8_t)` — consistent across Tasks 4/5/6 and the Task 8 dispatcher's `blobs[n]` decay.
- `modulateSynthFromBoards(LDRBlob[3][3], uint8_t[3])` — matches `globals.h` `LDRBlobs[3][3]` / `ultrasonicDistancePercent[3]` (Task 8 step 2).
- `lfoUpdate()` forward-declared in Task 3, defined in Task 7 — same signature.
- Oscillator index convention 0=sine/1=square/2=tri/3=saw — consistent in Tasks 1, 2, 3, 7.
- Helpers `mapAngleExp`, `mapLog`, `glide` each defined once, before first use.

No inconsistencies found.
