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

void lfoUpdate(); // defined below (after the modulation writers)

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

// =====================================================================
//  MODULATION WRITERS  (loop context; write volatile targets only)
// =====================================================================

// Exponential map: angle01 in [0,1] -> [lo,hi] exponentially. lo>0, hi>0.
inline float mapAngleExp(float angle01, float lo, float hi) {
  if (angle01 < 0.0f) angle01 = 0.0f;
  if (angle01 > 1.0f) angle01 = 1.0f;
  return lo * powf(hi / lo, angle01);
}

// Log (== exponential-in-t) map: t01 in [0,1] -> [lo,hi]. lo>0, hi>0.
inline float mapLog(float t01, float lo, float hi) {
  if (t01 < 0.0f) t01 = 0.0f;
  if (t01 > 1.0f) t01 = 1.0f;
  return lo * powf(hi / lo, t01);
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

#endif // audio_h
