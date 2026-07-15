#ifndef audio_h
#define audio_h

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// === Audio Objects ===

// --- Voice 1 (largest blob) — full richness ---
AudioSynthWaveform          sawWave1;
AudioSynthWaveform          squareWave1;
AudioSynthNoiseWhite        noiseGen1;
AudioMixer4                 synthMixer1;
AudioFilterBiquad           lpFilter1; // dual-purpose: acts as LOWPASS or HIGHPASS depending on angle (see below)

// --- Voice 2 (second blob) — "the body" ---
AudioSynthWaveform          sawWave2;
AudioSynthWaveform          squareWave2;
AudioMixer4                 synthMixer2;
AudioFilterBiquad           lpFilter2;

// --- Voice 3 (third blob) — "the magic": FM sine + custom phaser ---
AudioSynthWaveform          fmModulator3;   // audio-rate FM modulator
AudioSynthWaveformModulated sineWave3;      // carrier, FM'd by fmModulator3
AudioFilterBiquad           phaserStage1;   // allpass stage 1 (custom phaser)
AudioFilterBiquad           phaserStage2;   // allpass stage 2 (custom phaser)
AudioMixer4                 phaserMixer3;   // 0=dry sine3, 1=phased/wet sine3

// --- Combine all three voices before the existing finalMixer ---
AudioMixer4                 voiceMixer; // 0=voice1, 1=voice2, 2=voice3

// --- Per-voice note envelopes, driven by noteSynthFromBlob(). Inserted
// between each voice's timbre chain and voiceMixer, so note gating (this
// file's newest addition) layers on top of -- without disturbing --
// modulateSynthFromBlob()'s filter/waveform/texture work upstream of them. ---
AudioEffectEnvelope          noteEnvelope1;
AudioEffectEnvelope          noteEnvelope2;
AudioEffectEnvelope          noteEnvelope3;

AudioMixer4              finalMixer;  // Combines processed synth and WAV player
AudioPlaySdWav            wavPlayer;

AudioEffectFreeverb      freeverb;
AudioMixer4              masterMixer; 
AudioOutputI2S           audioOutput;

// --- Patch cords ---
AudioConnection patchV1a(squareWave1, 0, synthMixer1, 0);
AudioConnection patchV1b(sawWave1, 0, synthMixer1, 1);
AudioConnection patchV1c(noiseGen1, 0, synthMixer1, 2);
AudioConnection patchV1d(synthMixer1, 0, lpFilter1, 0);
AudioConnection patchV1e(lpFilter1, 0, noteEnvelope1, 0);       // Voice1 -> note envelope
AudioConnection patchV1env(noteEnvelope1, 0, voiceMixer, 0);    // note envelope -> voiceMixer

AudioConnection patchV2a(squareWave2, 0, synthMixer2, 0);
AudioConnection patchV2b(sawWave2, 0, synthMixer2, 1);
AudioConnection patchV2c(synthMixer2, 0, lpFilter2, 0);
AudioConnection patchV2d(lpFilter2, 0, noteEnvelope2, 0);       // Voice2 -> note envelope
AudioConnection patchV2env(noteEnvelope2, 0, voiceMixer, 1);    // note envelope -> voiceMixer

// Voice 3: FM modulator -> carrier -> split into dry + phaser(wet) -> phaserMixer3 -> note envelope -> voiceMixer
AudioConnection patchV3fm(fmModulator3, 0, sineWave3, 0);      // FM modulation input
AudioConnection patchV3dry(sineWave3, 0, phaserMixer3, 0);     // dry path
AudioConnection patchV3wetA(sineWave3, 0, phaserStage1, 0);    // wet path, stage 1
AudioConnection patchV3wetB(phaserStage1, 0, phaserStage2, 0); // wet path, stage 2
AudioConnection patchV3wetC(phaserStage2, 0, phaserMixer3, 1); // wet path -> mixer
AudioConnection patchV3out(phaserMixer3, 0, noteEnvelope3, 0); // Voice3 -> note envelope
AudioConnection patchV3env(noteEnvelope3, 0, voiceMixer, 2);   // note envelope -> voiceMixer

AudioConnection          patchCordVoiceOut(voiceMixer, 0, finalMixer, 0); // combined voices into final mix
AudioConnection          patchCord7(wavPlayer, 0, finalMixer, 2);       // WAV Left
AudioConnection          patchCord8(wavPlayer, 1, finalMixer, 3);       // WAV Right
AudioConnection          patchCord9(finalMixer, 0, masterMixer, 0);     // DRY Path (Channel 0)
AudioConnection          patchCord10(finalMixer, 0, freeverb, 0);       // Send to Reverb
AudioConnection          patchCord11(freeverb, 0, masterMixer, 1);      // WET Path (Channel 1)
AudioConnection          patchCord12(masterMixer, 0, audioOutput, 0);   // Master to Left Speaker
AudioConnection          patchCord13(masterMixer, 0, audioOutput, 1);   // Master to Right Speaker

AudioControlSGTL5000     sgtl5000_1;

#define SDCARD_CS_PIN    10

// Root note reference, shared by audioSetup() and noteSynthFromBlob().
const float ROOT_C2_FREQ = 65.41;

#include <IntervalTimer.h>

IntervalTimer lfoTimer;

// Shared state, updated by modulateSynthFromBlob(), read by the timer ISR.
// Index 0 = voice1, 1 = voice2 (voice3 has its own FM/phaser system, not this array)
volatile float g_cutoffFreq[2]   = {1500.0, 2500.0};
volatile float g_lfoDepth[2]     = {0.0, 0.2};
volatile float g_lfoFreq[2]      = {1.0, 0.3};
volatile float g_filterQ[2]      = {1.0, 0.65};

// Voice 1 only: which filter TYPE lpFilter1 is currently acting as.
// 0 = lowpass (angle 0.0-0.5), 1 = highpass (angle 0.5-1.0). The LFO sweep
// (g_lfoFreq[0]/g_lfoDepth[0]) is applied identically either way — only the
// filter type and its safe frequency ceiling/floor differ.
volatile short g_filterType0 = 0;

// Voice 2's root pitch, written by noteSynthFromBlob() (the interval
// relative to Voice 1), read by modulateSynthFromBlob() which layers its
// own small live detune on top before writing sawWave2's actual frequency.
volatile float g_voice2RootFreq = ROOT_C2_FREQ * pow(2.0, 7.0 / 12.0);

float lfoPhase[2]      = {0.0, 0.0};
float smoothedFreq[2]  = {1500.0, 2500.0};

float currentVolume = 0; // legacy global, unused below (kept for compatibility)

// ----------------------------------------------------------------------
// Helper: compute a 2nd-order allpass biquad (RBJ cookbook formula) and
// push it into an AudioFilterBiquad stage. Used to build the custom
// phaser for Voice 3, since the Teensy Audio Library has no built-in
// phaser object.
// CAVEAT: setCoefficients()'s exact argument format has varied slightly
// across Teensy Audio Library versions. This uses the common
// {b0,b1,b2,a1,a2} (all pre-divided by a0) convention — verify against
// your installed library version and adjust sign/order if the phaser
// sounds wrong (e.g. silent, or a flat non-sweeping tone) on hardware.
// ----------------------------------------------------------------------
void setAllpassCoeffs(AudioFilterBiquad &filt, uint8_t stage, float freq, float Q) {
  if (freq < 20.0) freq = 20.0;
  if (freq > 18000.0) freq = 18000.0;

  float w0 = 2.0 * PI * freq / AUDIO_SAMPLE_RATE_EXACT;
  float alpha = sin(w0) / (2.0 * Q);
  float cosw0 = cos(w0);

  float b0 = 1.0 - alpha;
  float b1 = -2.0 * cosw0;
  float b2 = 1.0 + alpha;
  float a0 = 1.0 + alpha;
  float a1 = -2.0 * cosw0;
  float a2 = 1.0 - alpha;

  double coeffs[5] = { b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0 };
  filt.setCoefficients(stage, coeffs);
}

// Runs at 1kHz: sweeps voice1 & voice2's filter cutoffs through their LFO
// cycles and pushes the smoothed result into each biquad's coefficients.
void lfoUpdate() {
  const float dt = 0.001;

  // --- Voice 1 filter sweep ---
  // Same LFO sweep math regardless of mode — only the resulting filter TYPE
  // call (setLowpass vs setHighpass) and the safe frequency clamp differ.
  lfoPhase[0] += 2.0 * PI * g_lfoFreq[0] * dt;
  if (lfoPhase[0] > 2.0 * PI) lfoPhase[0] -= 2.0 * PI;
  float target1 = g_cutoffFreq[0] * pow(2.0, g_lfoDepth[0] * sin(lfoPhase[0]));

  if (g_filterType0 == 0) {
    if (target1 > 19000.0) target1 = 19000.0;
    smoothedFreq[0] = (smoothedFreq[0] * 0.85) + (target1 * 0.15);
    lpFilter1.setLowpass(0, smoothedFreq[0], g_filterQ[0]);
  } else {
    if (target1 > 12000.0) target1 = 12000.0;
    if (target1 < 20.0) target1 = 20.0;
    smoothedFreq[0] = (smoothedFreq[0] * 0.85) + (target1 * 0.15);
    lpFilter1.setHighpass(0, smoothedFreq[0], g_filterQ[0]);
  }

  lfoPhase[1] += 2.0 * PI * g_lfoFreq[1] * dt;
  if (lfoPhase[1] > 2.0 * PI) lfoPhase[1] -= 2.0 * PI;
  float target2 = g_cutoffFreq[1] * pow(2.0, g_lfoDepth[1] * sin(lfoPhase[1]));
  smoothedFreq[1] = (smoothedFreq[1] * 0.85) + (target2 * 0.15);
  lpFilter2.setLowpass(0, smoothedFreq[1], g_filterQ[1]);
}

void audioSetup() {
  AudioMemory(250); // 3 voices + 2 extra biquad stages + FM modulator + phaser mixer + 3 note envelopes

  sgtl5000_1.enable();
  sgtl5000_1.dacVolume(1.0);
  sgtl5000_1.volume(0.75);

  SPI.setMOSI(11);
  SPI.setSCK(13);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    while (1) { Serial.println("Unable to access the SD card"); delay(500); }
  }

  const float C2_FREQ = ROOT_C2_FREQ;

  // --- Voice 1: main voice (largest blob) — full richness ---
  sawWave1.begin(WAVEFORM_BANDLIMIT_SAWTOOTH);
  sawWave1.frequency(C2_FREQ); // overwritten live by noteSynthFromBlob()
  sawWave1.amplitude(0.65); // live-modulated down slightly by radius (noise/saw texture dial)

  squareWave1.begin(WAVEFORM_SINE); // morphs via ultrasonic distance (biased 4-zone system)
  squareWave1.frequency(C2_FREQ / 2.0); // overwritten live by noteSynthFromBlob()
  squareWave1.amplitude(0.40);
  squareWave1.pulseWidth(0.5);

  noiseGen1.amplitude(0.0);

  synthMixer1.gain(0, 0.25);
  synthMixer1.gain(1, 0.58);
  synthMixer1.gain(2, 1.0);

  lpFilter1.setLowpass(0, 1500.0, 1.0);

  // --- Voice 2: "the body" (second blob) — pitched a fifth up ---
  const float V2_ROOT = C2_FREQ * pow(2.0, 7.0 / 12.0);

  sawWave2.begin(WAVEFORM_BANDLIMIT_SAWTOOTH);
  sawWave2.frequency(V2_ROOT); // overwritten live: root from noteSynthFromBlob(), detune from modulateSynthFromBlob()
  sawWave2.amplitude(1.0);

  squareWave2.begin(WAVEFORM_SINE);
  squareWave2.frequency(V2_ROOT); // overwritten live by noteSynthFromBlob() (no detune applied to this one)
  squareWave2.amplitude(1.0);

  synthMixer2.gain(0, 0.6);
  synthMixer2.gain(1, 0.2);

  lpFilter2.setLowpass(0, 2500.0, 0.65);

  // --- Voice 3: "the magic" (third blob) — FM sine + custom phaser ---
  const float V3_ROOT = C2_FREQ * 4.0;

  fmModulator3.begin(WAVEFORM_SINE);
  fmModulator3.amplitude(1.0);
  fmModulator3.frequency(5.0);

  sineWave3.begin(WAVEFORM_SINE);
  sineWave3.frequency(V3_ROOT); // overwritten live by noteSynthFromBlob()
  sineWave3.amplitude(0.0);
  sineWave3.frequencyModulation(0.0);

  setAllpassCoeffs(phaserStage1, 0, 800.0, 0.7);
  setAllpassCoeffs(phaserStage2, 0, 1000.0, 0.7);
  phaserMixer3.gain(0, 1.0);
  phaserMixer3.gain(1, 0.0);

  // --- Combine voices ---
  voiceMixer.gain(0, 1.0);
  voiceMixer.gain(1, 0.0);
  voiceMixer.gain(2, 0.0);
  voiceMixer.gain(3, 0.0);

  // --- Note envelopes: silent until noteSynthFromBlob() triggers noteOn().
  // attack/decay/sustain are re-set live per note by configureNoteDuration()/
  // configureNoteVelocity(); these are just safe starting defaults. ---
  noteEnvelope1.attack(10.0);  noteEnvelope1.hold(0.0);  noteEnvelope1.decay(300.0);  noteEnvelope1.sustain(0.0);  noteEnvelope1.release(150.0);  noteEnvelope1.noteOff();
  noteEnvelope2.attack(10.0);  noteEnvelope2.hold(0.0);  noteEnvelope2.decay(300.0);  noteEnvelope2.sustain(0.0);  noteEnvelope2.release(150.0);  noteEnvelope2.noteOff();
  noteEnvelope3.attack(10.0);  noteEnvelope3.hold(0.0);  noteEnvelope3.decay(300.0);  noteEnvelope3.sustain(0.0);  noteEnvelope3.release(150.0);  noteEnvelope3.noteOff();

  lfoTimer.begin(lfoUpdate, 1000);

  finalMixer.gain(0, 1.0);
  finalMixer.gain(2, 0.5);
  finalMixer.gain(3, 0.5);

  freeverb.roomsize(ROOM_SIZE);
  freeverb.damping(DAMPING);

  masterMixer.gain(0, DRY_GAIN);
  masterMixer.gain(1, WET_GAIN);
  masterMixer.gain(2, 0.0);
  masterMixer.gain(3, 0.0);
}

// ============================================================================
// modulateSynthFromBlob — TIMBRE only (filter, waveform, texture, ambience).
// Pitch and note gating are owned by noteSynthFromBlob() instead -- this
// function never calls .frequency() or noteOn()/noteOff().
//
// IDLE DEFAULT: each voice's timbre math now runs unconditionally, using
// the real blob's size/radius/angle when present, or a fixed neutral
// default (size=1, radius=1, angle=0.5) when the blob is absent, so the
// timbre is always settled at a known baseline. The existing presence-based
// volume glide-to-silence is untouched -- still silent while idle, just
// "tuned" underneath.
// ============================================================================
void modulateSynthFromBlob(LDRBlob blobs[3], uint8_t ultrasonicDistancePercent) {

  float ultrasonicDistanceFraction = ultrasonicDistancePercent / 100.0;
  const float DISTANCE_TAPER_POWER = 0.3;
  float easedDistanceFraction = pow(ultrasonicDistanceFraction, DISTANCE_TAPER_POWER);

  int activeCount = 0;
  if (blobs[0].size > 0) activeCount++;
  if (blobs[1].size > 0) activeCount++;
  if (blobs[2].size > 0) activeCount++;
  float shadowIntensity = constrain((activeCount - 1) / 2.0, 0.0, 1.0);

  static float smoothedDryGain = DRY_GAIN;
  static float smoothedWetGain = WET_GAIN;
  float targetDryGain = DRY_GAIN * (1.0 - (easedDistanceFraction * 0.6));
  float targetWetGain = WET_GAIN + (easedDistanceFraction * (0.9 - WET_GAIN)) + (shadowIntensity * 0.08);
  targetWetGain = constrain(targetWetGain, 0.0, 0.95);
  smoothedDryGain = (smoothedDryGain * 0.9) + (targetDryGain * 0.1);
  smoothedWetGain = (smoothedWetGain * 0.9) + (targetWetGain * 0.1);
  masterMixer.gain(0, smoothedDryGain);
  masterMixer.gain(1, smoothedWetGain);

  // === VOICE 1 ===
  {
    static float v1_smoothedCutoff = 1500.0;
    static float v1_currentVolume = 0.0;
    static short v1_currentSubWaveform = WAVEFORM_SINE;
    static short v1_currentSawWaveform = WAVEFORM_BANDLIMIT_SAWTOOTH;
    static short v1_filterMode = 0;
    static short v1_prevFilterMode = 0;
    static float v1_smoothedRadiusFraction = 0.0;

    LDRBlob blob = blobs[0];
    bool v1_present = (blob.size > 0);

    if (!v1_present) {
      v1_currentVolume = (v1_currentVolume * 0.7);
    } else {
      v1_currentVolume = (v1_currentVolume * 0.3) + (1.0 * 0.7);
    }

    float sizeFactor, rawRadiusFraction, angleFraction;
    if (v1_present) {
      sizeFactor = blob.size / 100.0;
      rawRadiusFraction = blob.centerRadius / 100.0;
      angleFraction = blob.centerAngle / 100.0;
    } else {
      sizeFactor = 1.0;
      rawRadiusFraction = 1.0;
      angleFraction = 0.5;
    }

    v1_smoothedRadiusFraction = (v1_smoothedRadiusFraction * 0.3) + (rawRadiusFraction * 0.7);
    float radiusFraction = pow(v1_smoothedRadiusFraction, 0.6);

    const float LFO_FREQ_MAX = 16.0;
    float lfoFreq = LFO_FREQ_MAX * pow(0.5 / LFO_FREQ_MAX, sizeFactor);

    float freqFraction = constrain((lfoFreq - 1.0) / (LFO_FREQ_MAX - 1.0), 0.0, 1.0);
    float lfoDepth = 4.0 * pow(0.2, freqFraction);

    const float ANGLE_SPLIT = 0.5;
    const float ANGLE_HYST = 0.03;
    if (v1_filterMode == 0 && angleFraction > ANGLE_SPLIT + ANGLE_HYST) v1_filterMode = 1;
    else if (v1_filterMode == 1 && angleFraction < ANGLE_SPLIT - ANGLE_HYST) v1_filterMode = 0;

    bool modeJustSwitched = (v1_filterMode != v1_prevFilterMode);

    float distFromCrossover = fabs(angleFraction - ANGLE_SPLIT) / ANGLE_SPLIT;
    distFromCrossover = constrain(distFromCrossover, 0.0, 1.0);

    float purity = 1.0 - distFromCrossover;
    synthMixer1.gain(1, 0.58 - (purity * 0.30));
    synthMixer1.gain(0, 0.25 + (purity * 0.25));

    float cutoffFreq;
    if (v1_filterMode == 0) {
      float t = constrain(angleFraction / ANGLE_SPLIT, 0.0, 1.0);
      cutoffFreq = 300.0 * pow(19000.0 / 300.0, t);
    } else {
      float t = constrain((angleFraction - ANGLE_SPLIT) / (1.0 - ANGLE_SPLIT), 0.0, 1.0);
      cutoffFreq = 30.0 * pow(4000.0 / 30.0, t);
    }

    if (modeJustSwitched) {
      v1_smoothedCutoff = cutoffFreq;
    } else {
      v1_smoothedCutoff = (v1_smoothedCutoff * 0.3) + (cutoffFreq * 0.7);
    }
    v1_prevFilterMode = v1_filterMode;

    g_cutoffFreq[0] = v1_smoothedCutoff;
    g_lfoDepth[0] = lfoDepth;
    g_lfoFreq[0] = lfoFreq;
    g_filterType0 = v1_filterMode;

    float v1_ampBoost = 1.0 + (0.2 * distFromCrossover);
    voiceMixer.gain(0, v1_currentVolume * v1_ampBoost);

    float noiseGainAmt = radiusFraction * 0.1;
    float sawAmp = 0.65 - (radiusFraction * 0.15);
    noiseGen1.amplitude(noiseGainAmt);
    sawWave1.amplitude(sawAmp);

    float ultrasonicFractionForZone = v1_present ? easedDistanceFraction : 1.0;

    const float ZONE_WIDTH = 0.25, HYSTERESIS = 0.03;
    float calmBias = (1.0 - shadowIntensity) * 0.2;
    float effectiveDistance = ultrasonicFractionForZone + calmBias * (1.0 - ultrasonicFractionForZone);

    short targetSubWaveform = v1_currentSubWaveform;
    short targetSawWaveform = v1_currentSawWaveform;
    if (effectiveDistance < ZONE_WIDTH - HYSTERESIS) {
      targetSubWaveform = WAVEFORM_BANDLIMIT_PULSE;
      targetSawWaveform = WAVEFORM_BANDLIMIT_SAWTOOTH;
    } else if (effectiveDistance > ZONE_WIDTH + HYSTERESIS && effectiveDistance < ZONE_WIDTH*2 - HYSTERESIS) {
      targetSubWaveform = WAVEFORM_BANDLIMIT_SAWTOOTH;
      targetSawWaveform = WAVEFORM_BANDLIMIT_SAWTOOTH;
    } else if (effectiveDistance > ZONE_WIDTH*2 + HYSTERESIS && effectiveDistance < ZONE_WIDTH*3 - HYSTERESIS) {
      targetSubWaveform = WAVEFORM_TRIANGLE;
      targetSawWaveform = WAVEFORM_TRIANGLE;
    } else if (effectiveDistance > ZONE_WIDTH*3 + HYSTERESIS) {
      targetSubWaveform = WAVEFORM_SINE;
      targetSawWaveform = WAVEFORM_SINE;
    }

    if (targetSubWaveform != v1_currentSubWaveform) {
      squareWave1.begin(targetSubWaveform);
      v1_currentSubWaveform = targetSubWaveform;
    }
    if (targetSawWaveform != v1_currentSawWaveform) {
      sawWave1.begin(targetSawWaveform);
      v1_currentSawWaveform = targetSawWaveform;
    }

    if (DEBUG_MODE) {
      Serial.print("  |  V1: ");
      Serial.print("Vol:"); Serial.print(v1_currentVolume, 2);
      Serial.print(" Mode:"); Serial.print(g_filterType0 == 0 ? "LP" : "HP");
      Serial.print(" Cut:"); Serial.print(g_cutoffFreq[0], 0);
      Serial.print(" LFO:"); Serial.print(g_lfoFreq[0], 1);
      Serial.print(" Depth:"); Serial.print(g_lfoDepth[0], 2);
      Serial.print(" Wv:"); Serial.println(v1_currentSubWaveform);
    }
  }

  // === VOICE 2 ===
  {
    static float v2_smoothedDetuneCents = 0.0;
    static float v2_currentVolume = 0.0;

    LDRBlob blob = blobs[1];
    bool v2_present = (blob.size > 0);

    if (!v2_present) {
      v2_currentVolume = (v2_currentVolume * 0.7);
    } else {
      v2_currentVolume = (v2_currentVolume * 0.6) + (0.4 * 0.4);
    }
    voiceMixer.gain(1, v2_currentVolume);

    float sizeFactor, radiusFraction, angleFraction;
    if (v2_present) {
      sizeFactor = blob.size / 100.0;
      radiusFraction = blob.centerRadius / 100.0;
      angleFraction = blob.centerAngle / 100.0;
    } else {
      sizeFactor = 1.0;
      radiusFraction = 1.0;
      angleFraction = 0.5;
    }

    float richness = sizeFactor;
    synthMixer2.gain(1, 0.2 + (richness * 0.5));
    synthMixer2.gain(0, 0.6 - (richness * 0.3));

    const float MIN_Q2 = 0.65, MAX_Q2 = 2.0;
    g_filterQ[1] = MIN_Q2 + (angleFraction * (MAX_Q2 - MIN_Q2));

    float detuneCents = radiusFraction * 6.0;
    v2_smoothedDetuneCents = (v2_smoothedDetuneCents * 0.9) + (detuneCents * 0.1);
    sawWave2.frequency(g_voice2RootFreq * pow(2.0, v2_smoothedDetuneCents / 1200.0));

    if (DEBUG_MODE) {
      Serial.print("  |  V2: ");
      Serial.print("Vol:"); Serial.print(v2_currentVolume, 2);
      Serial.print(" Richness:"); Serial.print(0.2 + (richness * 0.5), 2);
      Serial.print(" Q:"); Serial.println(g_filterQ[1], 2);
    }
  }

  // === VOICE 3 ===
  {
    static float v3_currentVolume = 0.0;
    static float v3_phaserSweepPhase = 0.0;
    static unsigned long v3_lastUpdateMs = millis();

    LDRBlob blob = blobs[2];
    bool v3_present = (blob.size > 0);

    unsigned long nowMs = millis();
    float dt = (nowMs - v3_lastUpdateMs) / 1000.0;
    v3_lastUpdateMs = nowMs;

    if (!v3_present) {
      v3_currentVolume = (v3_currentVolume * 0.7);
    } else {
      v3_currentVolume = (v3_currentVolume * 0.5) + (0.35 * 0.5);
    }
    voiceMixer.gain(2, v3_currentVolume);
    sineWave3.amplitude(v3_currentVolume * 0.6);

    float sizeFactor, angleFraction, radiusFraction;
    if (v3_present) {
      sizeFactor = blob.size / 100.0;
      angleFraction = blob.centerAngle / 100.0;
      radiusFraction = blob.centerRadius / 100.0;
    } else {
      sizeFactor = 1.0;
      angleFraction = 0.5;
      radiusFraction = 1.0;
    }

    float fmDepthOctaves = sizeFactor * 2.0;
    sineWave3.frequencyModulation(fmDepthOctaves);

    float fmModSpeed = 1.0 + (angleFraction * 29.0);
    fmModulator3.frequency(fmModSpeed);

    const float PHASER_SWEEP_HZ = 0.15;
    v3_phaserSweepPhase += 2.0 * PI * PHASER_SWEEP_HZ * dt;
    if (v3_phaserSweepPhase > 2.0 * PI) v3_phaserSweepPhase -= 2.0 * PI;

    float sweepRangeHz = 200.0 + (radiusFraction * 1800.0);
    float phaserCenterFreq = 800.0 + (sweepRangeHz * sin(v3_phaserSweepPhase));
    if (phaserCenterFreq < 100.0) phaserCenterFreq = 100.0;

    setAllpassCoeffs(phaserStage1, 0, phaserCenterFreq, 0.7);
    setAllpassCoeffs(phaserStage2, 0, phaserCenterFreq * 1.3, 0.7);

    float wetMix = radiusFraction * 0.8;
    phaserMixer3.gain(0, 1.0 - wetMix);
    phaserMixer3.gain(1, wetMix);

    if (DEBUG_MODE) {
      Serial.print("  |  V3: ");
      Serial.print("Vol:"); Serial.print(v3_currentVolume, 2);
      Serial.print(" FMDepth:"); Serial.print(fmDepthOctaves, 2);
      Serial.print(" FMSpeed:"); Serial.print(fmModSpeed, 1);
      Serial.print(" PhaserWet:"); Serial.println(wetMix, 2);
    }
  }

  if (DEBUG_MODE) {
    Serial.println();
  }
}

// ----------------------------------------------------------------------
// Helper: Voice 1's continuous root frequency from its own angle.
// angle=0.0 -> C2, 0.5 -> C4, 1.0 -> C6 (4 octaves total, exponential so
// the glide is musically even). Also used as the reference root for
// Voices 2/3's relative-interval math below.
// ----------------------------------------------------------------------
float rootFreqFromAngle(float angleFraction) {
  return ROOT_C2_FREQ * pow(2.0, angleFraction * 4.0);
}

// ----------------------------------------------------------------------
// Helper: configures an envelope's decay/sustain for a radius-driven note
// duration. radius=0 -> quick percussive stab that dies out on its own
// (sustain=0, short decay). radius=1 -> fully sustained tone (sustain=1,
// long decay/swell) that holds until explicitly released via noteOff().
// Decay time is log-scaled between the two.
// CAVEAT: this is called every frame the blob is present, not just on
// trigger -- on Teensy's AudioEffectEnvelope, updating decay/sustain/
// release live (without calling noteOn()/noteOff()) does not restart the
// current envelope phase, only changes the values used going forward.
// Verify that still holds on your installed library version.
// ----------------------------------------------------------------------
void configureNoteDuration(AudioEffectEnvelope &env, float durationFraction) {
  durationFraction = constrain(durationFraction, 0.0, 1.0);
  const float MIN_DECAY_MS = 60.0, MAX_DECAY_MS = 1800.0;
  float decayMs = MIN_DECAY_MS * pow(MAX_DECAY_MS / MIN_DECAY_MS, durationFraction);
  env.hold(0.0);
  env.decay(decayMs);
  env.sustain(durationFraction);
  env.release(150.0);
}

// ----------------------------------------------------------------------
// Helper: configures an envelope's attack time from blob size, used as a
// rough "velocity" -- a bigger blob triggers a faster, more percussive
// attack; a smaller blob triggers a slower, softer swell-in. (This was
// left up to me for what "size" should do here -- easy to swap for a
// different mapping if you'd rather it do something else.)
// ----------------------------------------------------------------------
void configureNoteVelocity(AudioEffectEnvelope &env, float sizeFraction) {
  sizeFraction = constrain(sizeFraction, 0.0, 1.0);
  const float SLOW_ATTACK_MS = 120.0, FAST_ATTACK_MS = 3.0;
  float attackMs = SLOW_ATTACK_MS * pow(FAST_ATTACK_MS / SLOW_ATTACK_MS, sizeFraction);
  env.attack(attackMs);
}

// ----------------------------------------------------------------------
// Helper: shared logic for Voices 2/3 -- computes a pitch relative to
// Voice 1's current root, and drives the given envelope's duration/
// velocity/gating. `wasPresent` and `intervalClass` are per-voice state
// owned by the caller (each voice needs its own, so they're passed by
// reference rather than kept as statics in here).
//
// INTERVAL CLASS: how far this blob's angle sits from the root blob's
// angle selects the interval above the root -- close = same pitch class
// (just a different octave), mid = a perfect 4th, far = a perfect 5th.
// Hysteresis avoids flicker at the band edges.
// OCTAVE REGISTER: this blob's OWN angle (independent of that distance)
// picks which octave the resulting note lands in, quantized to whole
// octaves (not a continuous glide like Voice 1 -- these two voices are
// meant to sound like discrete harmony notes, not portamento).
//
// Returns the frequency to apply when present; the return value is
// meaningless when not present (gating already went to noteOff()).
// ----------------------------------------------------------------------
float updateRelativeVoice(LDRBlob thisBlob, LDRBlob rootBlob, AudioEffectEnvelope &env,
                           bool &wasPresent, short &intervalClass) {
  bool present = (thisBlob.size > 0);
  if (!present) {
    if (wasPresent) env.noteOff();
    wasPresent = false;
    return 0.0;
  }

  float thisAngle = thisBlob.centerAngle / 100.0;
  float rootAngle = rootBlob.centerAngle / 100.0;
  float radiusFraction = thisBlob.centerRadius / 100.0;
  float sizeFraction = thisBlob.size / 100.0;

  float angleDelta = fabs(thisAngle - rootAngle);
  const float BAND1 = 0.15, BAND2 = 0.35, HYST = 0.02;
  if (intervalClass == 0 && angleDelta > BAND1 + HYST) intervalClass = 1;
  else if (intervalClass == 1 && angleDelta < BAND1 - HYST) intervalClass = 0;
  else if (intervalClass == 1 && angleDelta > BAND2 + HYST) intervalClass = 2;
  else if (intervalClass == 2 && angleDelta < BAND2 - HYST) intervalClass = 1;
  float semitoneOffset = (intervalClass == 0) ? 0.0 : (intervalClass == 1 ? 5.0 : 7.0); // root/octave, P4, P5

  int octaveIndex = (int) round(thisAngle * 4.0); // 0..4 -> C2..C6, quantized
  float freq = ROOT_C2_FREQ * pow(2.0, (float) octaveIndex) * pow(2.0, semitoneOffset / 12.0);

  configureNoteDuration(env, radiusFraction);
  configureNoteVelocity(env, sizeFraction);

  if (!wasPresent) env.noteOn();
  wasPresent = true;
  return freq;
}

// ============================================================================
// noteSynthFromBlob — drives discrete musical PITCH and note GATING for all
// three voices, from the same blobs used by modulateSynthFromBlob(). Timbre
// stays entirely owned by modulateSynthFromBlob() -- this function only
// calls .frequency(), writes g_voice2RootFreq, and drives noteEnvelope1/2/3.
//
// Call this BEFORE modulateSynthFromBlob() each loop, so Voice 2's detune
// (in modulateSynthFromBlob) layers on this frame's root pitch rather than
// last frame's.
//
// blobs[0] (angle)  -> Voice 1 root pitch, continuous glide: angle 0.0 = C2,
//                      0.5 = C4, 1.0 = C6.
// blobs[0] (radius) -> Voice 1 note duration (stab at 0, sustained at 1).
// blobs[0] (size)   -> Voice 1 note velocity (attack speed).
// blobs[1]/[2]      -> Voices 2/3, pitched relative to blobs[0]'s current
//                      root (see updateRelativeVoice() above for the
//                      interval/octave logic); same duration/velocity
//                      mapping as Voice 1, using each blob's own radius/size.
// ============================================================================
void noteSynthFromBlob(LDRBlob blobs[3], uint8_t ultrasonicDistancePercent) {

  // === VOICE 1 — root pitch (continuous) + duration + velocity ===
  {
    static bool v1_wasPresent = false;

    LDRBlob blob = blobs[0];
    bool present = (blob.size > 0);

    if (present) {
      float angleFraction = blob.centerAngle / 100.0;
      float radiusFraction = blob.centerRadius / 100.0;
      float sizeFraction = blob.size / 100.0;

      float freq = rootFreqFromAngle(angleFraction);
      sawWave1.frequency(freq);
      squareWave1.frequency(freq / 2.0);

      configureNoteDuration(noteEnvelope1, radiusFraction);
      configureNoteVelocity(noteEnvelope1, sizeFraction);

      if (!v1_wasPresent) noteEnvelope1.noteOn();

      if (DEBUG_MODE) {
        Serial.print("  |  Note V1: Freq:"); Serial.print(freq, 1);
        Serial.print(" Dur:"); Serial.print(radiusFraction, 2);
        Serial.print(" Vel:"); Serial.println(sizeFraction, 2);
      }
    } else if (v1_wasPresent) {
      noteEnvelope1.noteOff();
    }
    v1_wasPresent = present;
  }

  // === VOICE 2 — interval relative to Voice 1's root ===
  {
    static bool v2_wasPresent = false;
    static short v2_intervalClass = 0;
    float freq = updateRelativeVoice(blobs[1], blobs[0], noteEnvelope2, v2_wasPresent, v2_intervalClass);
    if (blobs[1].size > 0) {
      g_voice2RootFreq = freq;
      squareWave2.frequency(freq);
      if (DEBUG_MODE) {
        Serial.print("  |  Note V2: Freq:"); Serial.print(freq, 1);
        Serial.print(" Interval:"); Serial.println(v2_intervalClass);
      }
    }
  }

  // === VOICE 3 — interval relative to Voice 1's root ===
  {
    static bool v3_wasPresent = false;
    static short v3_intervalClass = 0;
    float freq = updateRelativeVoice(blobs[2], blobs[0], noteEnvelope3, v3_wasPresent, v3_intervalClass);
    if (blobs[2].size > 0) {
      sineWave3.frequency(freq);
      if (DEBUG_MODE) {
        Serial.print("  |  Note V3: Freq:"); Serial.print(freq, 1);
        Serial.print(" Interval:"); Serial.println(v3_intervalClass);
      }
    }
  }
}

void playWAVFile(const char *filename, bool waitToFinish = true) {
  if (DEBUG_MODE) {Serial.print("Playing file: "); Serial.println(filename);}
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

#endif
