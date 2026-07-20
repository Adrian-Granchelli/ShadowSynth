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
float LFO_FREQ_MIN = 0.2f;    // Hz at centerAngle 0
float LFO_FREQ_MAX = 18.0f;   // Hz at centerAngle 100 (exponential map)

// --- Filter cutoff (Board 1) ---
float LP_CUT_MIN = 150.0f;    // low-pass cutoff at angle 0
float LP_CUT_MAX = 16000.0f;  // low-pass cutoff at angle 45
float HP_CUT_MIN = 1.0f;     // high-pass cutoff at angle 55
float HP_CUT_MAX = 4500.0f;   // high-pass cutoff at angle 100
float FILTER_Q   = 4.0f; // 0.707 is 0% resonance, 40% is Q ≈ 4.0
float FILTER_ABS_MIN = 100.0f;      // absolute lower bound for any cutoff write
float FILTER_ABS_MAX = 19000.0f;  // absolute upper bound for any cutoff write
float FILTER_STABLE_MIN = 20.0f;  // practical Biquad floor (never write below this)

// --- LFO depth + taper (applied to filter cutoff) ---
float LFO_DEPTH_OCT     = 3.0f;   // full modulation depth, +/- octaves
float LFO_DEPTH_OCT_MIN = 0.1f;   // tapered depth near the 20 kHz high end
float LFO_TAPER_START_HZ = LP_CUT_MIN; // base cutoff above which depth tapers toward
                                     // LFO_DEPTH_OCT_MIN at FILTER_ABS_MAX (20 kHz)

// --- Noise ---
float NOISE_MAX = 0.30f;
float NOISE_MIN = 0.00f;

// --- BASE Settings ---
uint8_t holdLastSetting    = 10; // how many iterations of create synth to hold the last option before reverting to standard

uint8_t defaultBoard0Angle = 25; // LFO 
uint8_t defaultBoard1Angle = 50; // Filter 
uint8_t defaultBoard2Angle = 40; // Notes

// --- Oscillators (index 0=sine, 1=square, 2=triangle, 3=saw) ---
float oscVolume_Base[4]          = { 0.42f, 0.15f, 0.25f, 0.05f };
float oscSemitoneOffset_Base[4]  = { 0,     -12,   12,    0     };
float oscCentsOffset_Base[4]     = { 0.0f,  2.0f,  0.0f,  -1.0f };
// float oscVolume_Base[4]          = { 0.42f, 0.25f, 0.25f, 0.20f };
// float oscSemitoneOffset_Base[4]  = { 0,     -12,   12,    0     };
// float oscCentsOffset_Base[4]     = { 0.0f,  3.0f,  1.0f,  -2.0f };


float oscVolume_Filth[4]         = { 0.07f, 0.30f, 0.08f, 0.30f };
float oscSemitoneOffset_Filth[4] = { -24,   0,     12,     -12  };
float oscCentsOffset_Filth[4]    = { 0.0f,  0.0f,  1.0f,  5.0f  };
// float oscVolume_Filth[4]         = { 0.12f, 0.35f, 0.18f, 0.35f };
// float oscSemitoneOffset_Filth[4] = { -24,   0,     12,     -12  };
// float oscCentsOffset_Filth[4]    = { 0.0f,  0.0f,  1.0f,  5.0f  };


//float  oscVolume[4]         = { 0.25f, 0.20f, 0.25f, 0.20f };
//int8_t oscSemitoneOffset[4] = { 0,     -12,   12,    0 };
//float  oscCentsOffset[4]    = { 0.0f,  0.0f,  3.0f,  -3.0f };

// --- WAV player ---
float WAV_PLAYER_VOLUME = 0.3f;

// --- Smoothing amounts (0..1; higher = smoother/slower glide) ---
static uint16_t lfoUpdatedFreq = 1000; // this doesn't actually change the dt well... 

float cutoffSmoothTime = 0.15f;  // in s, i.e: 150 ms
float lfoSmoothTime    = 0.20f;  // 500 ms
float noteSmoothTime   = 0.08f;  // 80 ms
float volumeSmoothTime = 0.75f;  // 300 ms
float oscSmoothTime    = 0.05f;  // 250 ms

// --- Reverb (reuses globals.h values; kept here as the tuning entry point) ---
float DRY_GAIN = 0.9; // Set gain for the dry signal
float WET_GAIN = 0.1; // Set gain for the wet signal (reverb)
float ROOM_SIZE = 0.5; // 0 is small 1.00 large 
float DAMPING = 0.8; // 0 is high free decay slow, 1 is low decay slow

#define SDCARD_CS_PIN 10

// =====================================================================
// AUDIO GRAPH
// =====================================================================
// Voice 1 Oscillators
AudioSynthWaveform sine1;
AudioSynthWaveform square1;
AudioSynthWaveform tri1;
AudioSynthWaveform saw1;

AudioMixer4 oscMixer1;     // sine, square, tri, saw

// Voice 2 Oscillators
AudioSynthWaveform sine2;
AudioSynthWaveform square2;
AudioSynthWaveform tri2;
AudioSynthWaveform saw2;

AudioMixer4 oscMixer2;

// Voice 3 Oscillators
AudioSynthWaveform sine3;
AudioSynthWaveform square3;
AudioSynthWaveform tri3;
AudioSynthWaveform saw3;

AudioMixer4 oscMixer3;

// Noise source
AudioSynthNoiseWhite noiseGen;

// Voice summing and effects
AudioMixer4        voiceMixer;      // ch0=v1 ch1=v2 ch2=v3 ch3=noise
AudioFilterBiquad  biquadFilter;

AudioPlaySdWav     wavPlayer;

AudioMixer4        finalMixer;      // ch0=synth ch1=wav
AudioEffectFreeverb freeverb;
AudioMixer4        masterMixer;     // ch0=dry ch1=wet

AudioOutputI2S       audioOutput;
AudioControlSGTL5000 sgtl5000_1;

IntervalTimer lfoTimer;


// ROUTING
// =====================================================================
// Voice 1
AudioConnection c_v1_0(sine1,   0, oscMixer1, 0);
AudioConnection c_v1_1(square1, 0, oscMixer1, 1);
AudioConnection c_v1_2(tri1,    0, oscMixer1, 2);
AudioConnection c_v1_3(saw1,    0, oscMixer1, 3);

// Voice 2
AudioConnection c_v2_0(sine2,   0, oscMixer2, 0);
AudioConnection c_v2_1(square2, 0, oscMixer2, 1);
AudioConnection c_v2_2(tri2,    0, oscMixer2, 2);
AudioConnection c_v2_3(saw2,    0, oscMixer2, 3);

// Voice 3
AudioConnection c_v3_0(sine3,   0, oscMixer3, 0);
AudioConnection c_v3_1(square3, 0, oscMixer3, 1);
AudioConnection c_v3_2(tri3,    0, oscMixer3, 2);
AudioConnection c_v3_3(saw3,    0, oscMixer3, 3);

// Combine all synth voices + noise
AudioConnection c_mix0(oscMixer1, 0, voiceMixer, 0);
AudioConnection c_mix1(oscMixer2, 0, voiceMixer, 1);
AudioConnection c_mix2(oscMixer3, 0, voiceMixer, 2);
AudioConnection c_mix3(noiseGen,  0, voiceMixer, 3);

// Filter entire synth engine---
AudioConnection c_filterIn (voiceMixer,    0, biquadFilter, 0);
AudioConnection c_filterOut(biquadFilter, 0, finalMixer,   0);


// WAV player
AudioConnection c_wavToFinal(wavPlayer, 0, finalMixer, 1);

// Reverb path
AudioConnection c_finalToMasterDry(finalMixer, 0, masterMixer, 0);
AudioConnection c_finalToVerb(finalMixer,      0, freeverb,    0);
AudioConnection c_verbToMasterWet(freeverb,    0, masterMixer, 1);

// Output
AudioConnection c_masterToOutL(masterMixer, 0, audioOutput, 0);
AudioConnection c_masterToOutR(masterMixer, 0, audioOutput, 1);

// --- Oscillators ---
  AudioSynthWaveform* voiceOscs[3][4] = {
    { &sine1, &square1, &tri1, &saw1 },
    { &sine2, &square2, &tri2, &saw2 },
    { &sine3, &square3, &tri3, &saw3 }
  };
  AudioMixer4* oscMixers[3] = {
    &oscMixer1,
    &oscMixer2,
    &oscMixer3
  };
  const int waveformTypes[4] = {
    WAVEFORM_SINE,
    WAVEFORM_SQUARE,
    WAVEFORM_TRIANGLE,
    WAVEFORM_SAWTOOTH
  };

  // ---------------- LFO shape definitions ----------------
  enum LfoShape {
      LFO_SINE,
      LFO_SAW_DOWN,
      LFO_SQUARE
  };
  volatile LfoShape g_lfoShape = LFO_SINE;

// =====================================================================
//  MODULATION STATE
//  volatile targets: written by modulate*(), read by lfoUpdate() ISR
//  smoothed values: owned entirely by lfoUpdate()
// =====================================================================
enum FilterMode : uint8_t { FILTER_LOWPASS = 0, FILTER_OFF = 1, FILTER_HIGHPASS = 2 };

volatile float g_targetLfoFreq  = 1.0f;      // Hz
volatile float g_targetCutoff   = 1000.0f;   // Hz (base, pre-LFO)
volatile uint8_t g_filterMode   = FILTER_OFF;
volatile float g_targetNoise = 0.0f;
volatile float g_targetNoteFreq[3] = {82.41f, 82.41f, 82.41f};    // Hz (E2)
volatile bool  g_synthActive    = false;     // false => glide volume to 0
volatile uint8_t noDataCount[3] = {0, 0, 0}; // tracks how many zeroes there were 
float  g_targetOscVolume[4]         = { 0.25f, 0.20f, 0.25f, 0.20f };
int8_t g_targetOscSemitoneOffset[4] = { 0,     -12,   12,    0 };
float  g_targetOscCentsOffset[4]    = { 0.0f,  0.0f,  3.0f,  -3.0f };

// smoothed (ISR-owned)
float g_smoothedLfoFreq  = 1.0f;
float g_smoothedCutoff   = 1000.0f;
float g_smoothedNoteFreq[3] = {82.41f, 82.41f, 82.41f};
float g_smoothedVolume[3]   = {0.0f, 0.0f, 0.0f};
float volTarget[3] = {0.0f, 0.0f, 0.0f};
float g_smoothedNoise = 0.0f;
// --- Oscillators (index 0=sine, 1=square, 2=triangle, 3=saw) ---
float  g_smoothedOscVolume[4]         = { 0.25f, 0.20f, 0.25f, 0.20f };
int8_t g_smoothedOscSemitoneOffset[4] = { 0,     -12,   12,    0 };
float  g_smoothedOscCentsOffset[4]    = { 0.0f,  0.0f,  3.0f,  -3.0f };

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
    while (1) { if (DEBUG_MODE) { Serial.println("Unable to access the SD card"); } delay(500); }
  }

  // ----------------------------------------------------
  // Initialize oscillators
  // ----------------------------------------------------
  for (uint8_t voice = 0; voice < 3; voice++) {
    for (uint8_t osc = 0; osc < 4; osc++) {
      voiceOscs[voice][osc]->begin(waveformTypes[osc]); 
      voiceOscs[voice][osc]->amplitude(1.0f); // oscillator internal amplitude
      voiceOscs[voice][osc]->frequency(g_targetNoteFreq[0]); // initial pitch
      oscMixers[voice]->gain( osc, g_smoothedOscVolume[osc] * 0.0f ); // mixer controls actual output volume
    }
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

  // --- Start the modulation timer ---
  lfoTimer.begin(lfoUpdate, lfoUpdatedFreq);
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
  if (boardBlobs[0].size == 0) {
    if (noDataCount[0] < holdLastSetting) {
      noDataCount[0]++;
      return;
    } 
  }
  noDataCount[0] = 0;

  (void)distancePct; // UNUSED HOOK: reserved for future mapping
  uint8_t angle = (boardBlobs[0].size == 0) ? defaultBoard0Angle : boardBlobs[0].centerAngle;
  // UNUSED HOOKS: boardBlobs[0].centerRadius, boardBlobs[0].size (beyond empty check)

  float a01 = angle / 100.0f;
  g_targetLfoFreq = mapAngleExp(a01, LFO_FREQ_MIN, LFO_FREQ_MAX);
  g_targetNoise = NOISE_MAX * std::pow(a01, 2.0); // zero is zero, i.e., ignoerin the NOISE_MIN Variable

  // set the LFO Shape 
  if (boardBlobs[2].size > 1) { g_lfoShape = LFO_SQUARE; }
  else if (boardBlobs[1].size > 1) { g_lfoShape = LFO_SAW_DOWN; }
  else { g_lfoShape = LFO_SINE; }
}

// BOARD 1 -> filter mode + base cutoff. Hard switch at the 45/55 boundaries.
//   angle 0..45   -> low-pass,  LP_CUT_MIN..LP_CUT_MAX  (log)
//   angle 45..55  -> filter off (wide-open low-pass at FILTER_ABS_MAX)
//   angle 55..100 -> high-pass, HP_CUT_MIN..HP_CUT_MAX  (log)
void modulateSynthFromBoard1(LDRBlob boardBlobs[3], uint8_t distancePct) {
  if (boardBlobs[0].size == 0) {
    if (noDataCount[1] < holdLastSetting) {
      noDataCount[1]++;
      return;
    } 
  }
  noDataCount[1] = 0;

  (void)distancePct; // UNUSED HOOK
  uint8_t angle = (boardBlobs[0].size == 0) ? defaultBoard1Angle : boardBlobs[0].centerAngle;
  // UNUSED HOOKS: boardBlobs[0].centerRadius, boardBlobs[0].size (beyond empty check)

  float t = angle / 100.0f;                    // 0..1 across the LP band
  g_targetCutoff = mapLog(t, LP_CUT_MIN, LP_CUT_MAX);
  g_filterMode   = FILTER_LOWPASS;

  // Shifting between low-pass to hi-pass
  // if (angle <= 45) {
  //   float t = angle / 65.0f;                    // 0..1 across the LP band
  //   g_targetCutoff = mapLog(t, LP_CUT_MIN, LP_CUT_MAX);
  //   g_filterMode   = FILTER_LOWPASS;
  // } else if (angle >= 55) {
  //   float t = (angle - 55) / 65.0f;             // 0..1 across the HP band
  //   g_targetCutoff = mapLog(t, HP_CUT_MIN, HP_CUT_MAX);
  //   g_filterMode   = FILTER_HIGHPASS;
  // } else {
  //   g_targetCutoff = FILTER_ABS_MAX;            // off: wide open
  //   g_filterMode   = FILTER_OFF;
  // }
  
  // -- changing the wave table 
  if (boardBlobs[2].size > 1) { } 
  else if (boardBlobs[1].size > 1) { 
    for (uint8_t osc = 0; osc < 4; osc++) {
      g_targetOscVolume[osc]          = oscVolume_Filth[osc];
      g_targetOscSemitoneOffset[osc]  = oscSemitoneOffset_Filth[osc];
      g_targetOscCentsOffset[osc]     = oscCentsOffset_Filth[osc];
    }
  }
  else { 
    for (uint8_t osc = 0; osc < 4; osc++) {
      g_targetOscVolume[osc]          = oscVolume_Base[osc];
      g_targetOscSemitoneOffset[osc]  = oscSemitoneOffset_Base[osc];
      g_targetOscCentsOffset[osc]     = oscCentsOffset_Base[osc];
    }
  }
}

//10-note table: E2 A2 D3 E3 A3 D4 E4 A4 D5 E5 (equal temperament, A4=440)
const float NOTE_TABLE[11] = {
  73.42f,  // D2
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
// 19-note table: A1 B1 D2 E2 G2 A2 B2 D3 E3 G3 A3 B3 D4 E4 G4 A4 B4 D5 E5
// Equal temperament, A4 = 440 Hz
// const float NOTE_TABLE[19] = {
//    55.00f,  // A1
//    61.74f,  // B1
//    73.42f,  // D2
//    82.41f,  // E2
//    98.00f,  // G2
//   110.00f,  // A2
//   123.47f,  // B2
//   146.83f,  // D3
//   164.81f,  // E3
//   196.00f,  // G3
//   220.00f,  // A3
//   246.94f,  // B3
//   293.66f,  // D4
//   329.63f,  // E4
//   392.00f,  // G4
//   440.00f,  // A4
//   493.88f,  // B4
//   587.33f,  // D5
//   659.25f   // E5
// };

// BOARD 2 -> selected note frequency (index into NOTE_TABLE across angle).
void modulateSynthFromBoard2(LDRBlob boardBlobs[3], uint8_t distancePct) {
  if (boardBlobs[0].size == 0) {
    if (noDataCount[2] < holdLastSetting) {
      noDataCount[2]++;
      return;
    } 
  }
  noDataCount[2] = 0;

  // FOR BLOB 0
  (void)distancePct; // UNUSED HOOK
  uint8_t angle = (boardBlobs[0].size == 0) ? defaultBoard2Angle : boardBlobs[0].centerAngle;
  // UNUSED HOOKS: boardBlobs[0].centerRadius, boardBlobs[0].size (beyond empty check)

  const int NUM_NOTES = sizeof(NOTE_TABLE) / sizeof(NOTE_TABLE[0]);

  int idx = (angle * NUM_NOTES) / 100;
  idx = constrain(idx, 0, NUM_NOTES - 1);

  g_targetNoteFreq[0] = NOTE_TABLE[idx];

  // FOR BLOB 1
  if (boardBlobs[1].size > 1) {
    (void)distancePct; // UNUSED HOOK
    angle = boardBlobs[1].centerAngle;
    // UNUSED HOOKS: boardBlobs[1].centerRadius, boardBlobs[1].size (beyond empty check)

    idx = (angle * NUM_NOTES) / 100;
    idx = constrain(idx, 0, NUM_NOTES - 1);

    g_targetNoteFreq[1] = NOTE_TABLE[idx];
    volTarget[1] = 0.75f;
  } else {
    volTarget[1] = 0.0f;
  }

  // FOR BLOB 2
  if (boardBlobs[2].size > 1) {
    (void)distancePct; // UNUSED HOOK
    angle = boardBlobs[2].centerAngle;
    // UNUSED HOOKS: boardBlobs[1].centerRadius, boardBlobs[1].size (beyond empty check)

    idx = (angle * NUM_NOTES) / 100;
    idx = constrain(idx, 0, NUM_NOTES - 1);

    g_targetNoteFreq[2] = NOTE_TABLE[idx];
    volTarget[2] = 0.5f;
  } else {
    volTarget[2] = 0.0f;
  }
}

// =====================================================================
//  1 kHz MODULATION TIMER  (owns all smoothing + hardware writes)
// =====================================================================
inline float glide(float current, float target, float smoothTime, float dt) {
  float alpha = dt / smoothTime;

  if (alpha > 1.0f) alpha = 1.0f;

  return current + (target - current) * alpha;
}

void lfoUpdate() {
  const float dt = 1.0f/lfoUpdatedFreq; //0.001f; // 1 kHz

  // 1) Glide smoothed values toward volatile targets.
  g_smoothedLfoFreq  = glide(g_smoothedLfoFreq,  g_targetLfoFreq,  lfoSmoothTime, dt);
  g_smoothedCutoff   = glide(g_smoothedCutoff,   g_targetCutoff,   cutoffSmoothTime, dt);
  volTarget[0]    = g_synthActive ? 1.0f : 0.0f;
  for (uint8_t i = 0; i <= 2; i++) {
    g_smoothedNoteFreq[i] = glide(g_smoothedNoteFreq[i], g_targetNoteFreq[i], noteSmoothTime, dt);
    g_smoothedVolume[i]  = glide(g_smoothedVolume[i],   volTarget[i],        volumeSmoothTime, dt);
  }
  g_smoothedNoise = glide(g_smoothedNoise, g_targetNoise,  volumeSmoothTime, dt);
  for (uint8_t osc = 0; osc < 4; osc++) {
    g_smoothedOscVolume[osc] = glide(g_smoothedOscVolume[osc], g_targetOscVolume[osc], oscSmoothTime, dt);
    g_smoothedOscSemitoneOffset[osc] = glide(g_smoothedOscSemitoneOffset[osc], g_targetOscSemitoneOffset[osc], oscSmoothTime, dt);
    g_smoothedOscCentsOffset[osc] = glide(g_smoothedOscCentsOffset[osc], g_targetOscCentsOffset[osc], oscSmoothTime, dt);
  }


  // 2) Advance LFO phase.
  g_lfoPhase += 2.0f * PI * g_smoothedLfoFreq * dt;
  if (g_lfoPhase > 2.0f * PI) g_lfoPhase -= 2.0f * PI;

  // 3) Oscillator frequencies + volume (per-osc semitone/cents offset).
  for (uint8_t voice = 0; voice < 3; voice++) {
    for (uint8_t osc = 0; osc < 4; osc++) {
      float f = g_smoothedNoteFreq[voice] * powf(2.0f, g_smoothedOscSemitoneOffset[osc] / 12.0f) * powf(2.0f, g_smoothedOscCentsOffset[osc] / 1200.0f);
      voiceOscs[voice][osc]->frequency(f);
      oscMixers[voice]->gain(osc, g_smoothedOscVolume[osc] * g_smoothedVolume[voice]);
    }
  }

  // 3.5 Noise amount 
  noiseGen.amplitude(g_smoothedNoise);

  // 4) Filter. If OFF, set wide-open once and skip the LFO sweep.
  if (g_filterMode == FILTER_OFF) {
    biquadFilter.setLowpass(0, FILTER_ABS_MAX, FILTER_Q);
    return;
  }

  // 4a) Depth taper from the smoothed BASE LFO cutoff (option A):
  float base = g_smoothedLfoFreq;
  float depthOct = LFO_DEPTH_OCT;
  if (base > LFO_FREQ_MIN) {
    float span = LFO_FREQ_MAX - LFO_FREQ_MIN; 
    float t = (base - LFO_FREQ_MIN) / span;   
    if (t > 1.0f) t = 1.0f;
    depthOct = LFO_DEPTH_OCT + (LFO_DEPTH_OCT_MIN - LFO_DEPTH_OCT) * t;
  }

  // 4b) LFO-modulated cutoff, then asymmetric clamp (clamp the instantaneous
  //     value each tick; the non-clipping side keeps full swing).
  // Calculate current LFO value (-1 to +1)
  float lfoValue;
  switch (g_lfoShape) {
      case LFO_SINE:
          lfoValue = sinf(g_lfoPhase);
          break;
      case LFO_SAW_DOWN:
          // +1 -> -1 over one cycle
          lfoValue = 1.0f - (g_lfoPhase / PI);
          break;
      case LFO_SQUARE:
          // Toggle between +1 and -1
          lfoValue = (g_lfoPhase < PI) ? 1.0f : -1.0f;
          break;
      default:
          lfoValue = sinf(g_lfoPhase);
          break;
  }
  // Apply LFO to filter cutoff in octave space
  float modCut = g_targetCutoff * powf(2.0f, depthOct * lfoValue);
  if (modCut > FILTER_ABS_MAX) modCut = FILTER_ABS_MAX;
  if (modCut < FILTER_ABS_MIN) modCut = FILTER_ABS_MIN;
  if (modCut < FILTER_STABLE_MIN) modCut = FILTER_STABLE_MIN; // Biquad stability floor

  // 4c) Apply by mode.
  if (g_filterMode == FILTER_LOWPASS) {
    biquadFilter.setLowpass(0, modCut, FILTER_Q);
  } else { // FILTER_HIGHPASS
    biquadFilter.setHighpass(0, modCut, FILTER_Q);
  }

  // exit function if not in debug mode
  return; // leaves
  if (!DEBUG_MODE) return;
  static uint32_t throttleCount = 0;
  throttleCount++;
  if (throttleCount > dt*1000) {
    throttleCount = 0;
    Serial.print("Vol%:"); Serial.print(g_smoothedVolume[0]*100);
    //Serial.print("LFO Hz:"); Serial.print(g_smoothedLfoFreq);
    Serial.print("\tNote:"); Serial.print(g_smoothedNoteFreq[0]);
    Serial.print("\tCutoffFreq:"); Serial.print(modCut); 
    Serial.print("\tCutoffFreqAvg:"); Serial.print(g_smoothedCutoff); 
    Serial.print("\tCutoffFreqMin:"); Serial.print(max(g_targetCutoff * powf(2.0f, depthOct * -1), FILTER_ABS_MIN));
    Serial.print("\tCutoffFreqMax:"); Serial.println(min(g_targetCutoff * powf(2.0f, depthOct * 1), FILTER_ABS_MAX));
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
  Serial.print(" note=");  Serial.print(g_smoothedNoteFreq[0], 2);
  Serial.print(" vol=");   Serial.println(g_smoothedVolume[0], 3);
}

#endif // audio_h
