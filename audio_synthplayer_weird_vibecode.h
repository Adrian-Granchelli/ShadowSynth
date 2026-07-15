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
// between each voice's timbre chain and voiceMixer. These are the ONLY
// thing that gates a voice's amplitude on/off -- modulateSynthFromBlob()
// never touches loudness, only tone color, so a voice sounding is purely
// a function of noteSynthFromBlob()'s gating decision (see below). ---
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

// Shared filter movement, updated by modulateSynthFromBlob() (from blob
// one only), read by the timer ISR. Cutoff frequency, LP/HP mode, and LFO
// sweep are now SHARED between lpFilter1 (voice1) and lpFilter2 (voice2)
// -- blob one's low-pass/high-pass sweep drives both filters together, so
// the two voices move through the spectrum as one gesture. Resonance (Q)
// stays per-voice (see g_filterQ[2] below) so each voice keeps its own
// character even while their cutoffs move in lockstep.
volatile float g_cutoffFreq   = 1500.0;
volatile float g_lfoDepth     = 0.0;
volatile float g_lfoFreq      = 1.0;

// Per-voice resonance: index 0 = voice1, 1 = voice2 (voice3 has its own
// FM/phaser system, not this array). Voice2's Q still comes from its own
// blob (see modulateSynthFromBlob's VOICE 2 block).
volatile float g_filterQ[2]      = {1.0, 0.65};

// Shared filter TYPE for both lpFilter1 and lpFilter2, driven by blob
// one's angle. 0 = lowpass (angle 0.0-0.5), 1 = highpass (angle 0.5-1.0).
// The LFO sweep (g_lfoFreq/g_lfoDepth) is applied identically either way —
// only the filter type and its safe frequency ceiling/floor differ.
volatile short g_filterType0 = 0;

// Voice 2's root pitch, written by noteSynthFromBlob() (the interval
// relative to Voice 1), read by modulateSynthFromBlob() which layers its
// own small live detune on top before writing sawWave2's actual frequency.
volatile float g_voice2RootFreq = ROOT_C2_FREQ * pow(2.0, 7.0 / 12.0);

float lfoPhase      = 0.0;
float smoothedFreq  = 1500.0;

// Portamento (pitch glide) time constant used by noteSynthFromBlob().
// While a voice is held and its blob moves, frequency eases toward the
// new target over roughly this many seconds (exponential approach) rather
// than jumping instantly -- smooths out jitter and gives a gentle legato
// slide between pitches. A brand-new note attack still SNAPS straight to
// its target pitch (only sustained movement glides), so attacks stay
// crisp. Raise for a slower, dreamier slide; lower for snappier response.
const float PITCH_GLIDE_TIME_S = 0.12;

// ----------------------------------------------------------------------
// TWO INDEPENDENT LIGHT ARRAYS
//
// Array A ("pitch") drives noteSynthFromBlob(): which notes sound, when
// they start/stop, how long they sustain, and how hard they're struck.
//
// Array B ("timbre") drives modulateSynthFromBlob(): filter movement,
// waveform choice, texture, FM depth, phaser sweep, reverb balance.
//
// Each array has a "neutral / not in use" reading of angle=0.5 (fraction),
// radius=1.0 (fraction), size=1.0 (fraction) -- i.e. LDRBlob.centerAngle=50,
// centerRadius=100, size=100 on whatever 0-100 scale the blob detector
// reports. A blob with size==0 means "nothing there."
//
// GATING RULE (how the two arrays combine): a voice is audible if EITHER
// array has an active blob in that voice's slot. This is what lets the
// arrays work solo or together:
//   - Array A alone: notes play, using Array A's pitch/duration/velocity;
//     timbre free-runs at its neutral default (already how
//     modulateSynthFromBlob()'s IDLE DEFAULT branch behaves below).
//   - Array B alone: timbre is fully expressive, but since nothing plays
//     without a note, Array B's presence is ALSO treated as a gate --
//     the voice sounds, using the neutral default pitch/duration/velocity
//     (root note, sustained, moderate attack) while Array B colors it.
//   - Both present: Array A supplies the note, Array B supplies the color,
//     independently and simultaneously, exactly as normal.
// ----------------------------------------------------------------------

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

// Runs at 1kHz: sweeps the SHARED filter cutoff (derived from blob one)
// through its LFO cycle and applies it to BOTH lpFilter1 (voice1) and
// lpFilter2 (voice2), each with its own resonance (Q). Same LFO sweep math
// regardless of mode — only the resulting filter TYPE call (setLowpass vs
// setHighpass) and the safe frequency clamp differ.
void lfoUpdate() {
  const float dt = 0.001;

  lfoPhase += 2.0 * PI * g_lfoFreq * dt;
  if (lfoPhase > 2.0 * PI) lfoPhase -= 2.0 * PI;
  float target = g_cutoffFreq * pow(2.0, g_lfoDepth * sin(lfoPhase));

  if (g_filterType0 == 0) {
    if (target > 19000.0) target = 19000.0;
    smoothedFreq = (smoothedFreq * 0.85) + (target * 0.15);
    lpFilter1.setLowpass(0, smoothedFreq, g_filterQ[0]);
    lpFilter2.setLowpass(0, smoothedFreq, g_filterQ[1]);
  } else {
    if (target > 12000.0) target = 12000.0;
    if (target < 20.0) target = 20.0;
    smoothedFreq = (smoothedFreq * 0.85) + (target * 0.15);
    lpFilter1.setHighpass(0, smoothedFreq, g_filterQ[0]);
    lpFilter2.setHighpass(0, smoothedFreq, g_filterQ[1]);
  }
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

  lpFilter2.setLowpass(0, 2500.0, 0.65); // cutoff/mode overwritten live by lfoUpdate() (shared w/ voice1); Q overwritten by modulateSynthFromBlob()

  // --- Voice 3: "the magic" (third blob) — FM sine + custom phaser ---
  const float V3_ROOT = C2_FREQ * 4.0;

  fmModulator3.begin(WAVEFORM_SINE);
  fmModulator3.amplitude(1.0);
  fmModulator3.frequency(5.0);

  sineWave3.begin(WAVEFORM_SINE);
  sineWave3.frequency(V3_ROOT); // overwritten live by noteSynthFromBlob()
  sineWave3.amplitude(0.6); // fixed baseline -- loudness now comes entirely from noteEnvelope3's ADSR
  sineWave3.frequencyModulation(0.0);

  setAllpassCoeffs(phaserStage1, 0, 800.0, 0.7);
  setAllpassCoeffs(phaserStage2, 0, 1000.0, 0.7);
  phaserMixer3.gain(0, 1.0);
  phaserMixer3.gain(1, 0.0);

  // --- Combine voices ---
  // Fixed at (near-)unity gain: with gating now owned entirely by the note
  // envelopes below, voiceMixer no longer needs to ramp per-voice volume
  // up/down itself -- that used to double up with the envelopes and could
  // silence a voice whose OTHER array (timbre) had gone idle even while
  // its pitch array was actively playing it. Voice 1 keeps a small live
  // ampBoost from modulateSynthFromBlob() for its filter-crossover swell.
  voiceMixer.gain(0, 1.0);
  voiceMixer.gain(1, 1.0);
  voiceMixer.gain(2, 1.0);
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
// modulateSynthFromBlob — TIMBRE only (filter, waveform, texture, ambience),
// driven by Array B (blobsTimbre). Pitch and note gating are owned by
// noteSynthFromBlob() (Array A) instead -- this function never calls
// .frequency() or noteOn()/noteOff(), and as of this revision it no longer
// touches loudness at all (see the voiceMixer/sineWave3 comments in
// audioSetup()) -- amplitude is 100% the note envelopes' job now, which is
// what lets Array B run solo, run alongside Array A, or sit idle without
// ever being able to silence a voice Array A is actively playing.
//
// IDLE DEFAULT: each voice's timbre math runs unconditionally, using the
// real blob's size/radius/angle when present, or the neutral default
// (size=1, radius=1, angle=0.5) when Array B's blob for that voice is
// absent -- so Array B alone, Array A alone, or both together all land on
// settled, intentional timbre rather than an undefined in-between.
// ============================================================================
void modulateSynthFromBlob(LDRBlob blobsTimbre[3], uint8_t ultrasonicDistancePercent) {

  float ultrasonicDistanceFraction = ultrasonicDistancePercent / 100.0;
  const float DISTANCE_TAPER_POWER = 0.3;
  float easedDistanceFraction = pow(ultrasonicDistanceFraction, DISTANCE_TAPER_POWER);

  int activeCount = 0;
  if (blobsTimbre[0].size > 0) activeCount++;
  if (blobsTimbre[1].size > 0) activeCount++;
  if (blobsTimbre[2].size > 0) activeCount++;
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
    static short v1_currentSubWaveform = WAVEFORM_SINE;
    static short v1_currentSawWaveform = WAVEFORM_BANDLIMIT_SAWTOOTH;
    static short v1_filterMode = 0;
    static short v1_prevFilterMode = 0;
    static float v1_smoothedRadiusFraction = 0.0;

    LDRBlob blob = blobsTimbre[0];
    bool v1_present = (blob.size > 0);

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

    // Shared with voice2: blob one's LP/HP sweep now drives lpFilter1 AND
    // lpFilter2 together (see lfoUpdate()). Voice2's own blob still shapes
    // its resonance (g_filterQ[1], set in the VOICE 2 block below).
    g_cutoffFreq = v1_smoothedCutoff;
    g_lfoDepth = lfoDepth;
    g_lfoFreq = lfoFreq;
    g_filterType0 = v1_filterMode;

    // Small live swell as Voice 1 nears the LP/HP crossover, layered on
    // top of the fixed unity gain set in audioSetup() -- this is now the
    // ONLY thing modulateSynthFromBlob() contributes to voiceMixer.
    float v1_ampBoost = 1.0 + (0.2 * distFromCrossover);
    voiceMixer.gain(0, v1_ampBoost);

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
      Serial.print("Present:"); Serial.print(v1_present ? "Y" : "n");
      Serial.print(" Mode:"); Serial.print(g_filterType0 == 0 ? "LP(shared)" : "HP(shared)");
      Serial.print(" Cut:"); Serial.print(g_cutoffFreq, 0);
      Serial.print(" LFO:"); Serial.print(g_lfoFreq, 1);
      Serial.print(" Depth:"); Serial.print(g_lfoDepth, 2);
      Serial.print(" Wv:"); Serial.println(v1_currentSubWaveform);
    }
  }

  // === VOICE 2 ===
  {
    static float v2_smoothedDetuneCents = 0.0;
    static float v2_smoothedRichness = 0.35; // eases synthMixer2 gain steps to avoid zipper noise

    LDRBlob blob = blobsTimbre[1];
    bool v2_present = (blob.size > 0);

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
    v2_smoothedRichness = (v2_smoothedRichness * 0.85) + (richness * 0.15);
    synthMixer2.gain(1, 0.2 + (v2_smoothedRichness * 0.5));
    synthMixer2.gain(0, 0.6 - (v2_smoothedRichness * 0.3));

    // Resonance only -- cutoff frequency and LP/HP mode for lpFilter2 now
    // come from blob one, shared with voice1 (see g_filterType0/
    // g_cutoffFreq and lfoUpdate() above).
    const float MIN_Q2 = 0.65, MAX_Q2 = 2.0;
    g_filterQ[1] = MIN_Q2 + (angleFraction * (MAX_Q2 - MIN_Q2));

    float detuneCents = radiusFraction * 6.0;
    v2_smoothedDetuneCents = (v2_smoothedDetuneCents * 0.9) + (detuneCents * 0.1);
    sawWave2.frequency(g_voice2RootFreq * pow(2.0, v2_smoothedDetuneCents / 1200.0));

    if (DEBUG_MODE) {
      Serial.print("  |  V2: ");
      Serial.print("Present:"); Serial.print(v2_present ? "Y" : "n");
      Serial.print(" Richness:"); Serial.print(0.2 + (v2_smoothedRichness * 0.5), 2);
      Serial.print(" Q:"); Serial.println(g_filterQ[1], 2);
    }
  }

  // === VOICE 3 ===
  {
    static float v3_phaserSweepPhase = 0.0;
    static unsigned long v3_lastUpdateMs = millis();

    LDRBlob blob = blobsTimbre[2];
    bool v3_present = (blob.size > 0);

    unsigned long nowMs = millis();
    float dt = (nowMs - v3_lastUpdateMs) / 1000.0;
    v3_lastUpdateMs = nowMs;

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
      Serial.print("Present:"); Serial.print(v3_present ? "Y" : "n");
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
// CAVEAT: this is called every frame the voice is gated on, not just on
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
// Helper: exponential glide from `current` toward `target`, framerate-
// independent (uses elapsed dt and a time constant rather than a fixed
// per-call blend factor, so it sounds the same whether the loop runs at
// 100Hz or 1000Hz). Used to smooth pitch changes in noteSynthFromBlob() --
// a held note's frequency eases toward wherever its blob currently points
// instead of snapping there instantly.
// ----------------------------------------------------------------------
float glideToward(float current, float target, float dt, float glideTimeSeconds) {
  if (glideTimeSeconds <= 0.0 || dt <= 0.0) return target;
  float alpha = 1.0 - exp(-dt / glideTimeSeconds);
  return current + (target - current) * alpha;
}

// ----------------------------------------------------------------------
// Helper: shared logic for Voices 2/3 -- computes a pitch relative to
// Voice 1's current root, and drives the given envelope's duration/
// velocity/gating. `wasPresent` and `intervalClass` are per-voice state
// owned by the caller (each voice needs its own, so they're passed by
// reference rather than kept as statics in here).
//
// pitchBlob  -- this voice's Array A (pitch) blob. When absent, the voice
//               (if gated on at all -- see `present` below) rests in
//               unison with the current root at a sustained, moderate
//               note, rather than freezing on stale numbers.
// timbreBlob -- this voice's Array B (timbre) blob. Only its presence
//               matters here (as a possible gate source); its values are
//               read by modulateSynthFromBlob() instead.
// rootAngleFraction -- Voice 1's current effective angle (0..1), supplied
//               by the caller, since Voice 1's angle may itself be a
//               default when Array A is idle there too.
//
// INTERVAL CLASS: how far this blob's angle sits from the root's angle
// selects the interval above the root -- close = same pitch class (just a
// different octave), mid = a perfect 4th, far = a perfect 5th. Hysteresis
// avoids flicker at the band edges.
// OCTAVE REGISTER: this blob's OWN angle (independent of that distance)
// picks which octave the resulting note lands in, quantized to whole
// octaves (not a continuous glide like Voice 1 -- these two voices are
// meant to sound like discrete harmony notes, not portamento).
//
// Returns the (glided) frequency to apply when gated on; the return value
// is meaningless when not present (gating already went to noteOff()).
//
// `smoothedFreq` is the caller-owned glide state for this voice: on a
// fresh note attack it SNAPS straight to the target pitch (so attacks
// stay crisp), and while the note stays held it eases toward the target
// over PITCH_GLIDE_TIME_S each call -- smoothing both the interval/octave
// jumps and any jitter in the blob reading, for a gentle slide instead of
// a hard retune.
// ----------------------------------------------------------------------
float updateRelativeVoice(LDRBlob pitchBlob, LDRBlob timbreBlob, float rootAngleFraction,
                           AudioEffectEnvelope &env, bool &wasPresent, short &intervalClass,
                           float &smoothedFreq, float dt) {
  bool present = (pitchBlob.size > 0) || (timbreBlob.size > 0);
  if (!present) {
    if (wasPresent) env.noteOff();
    wasPresent = false;
    return smoothedFreq;
  }

  float thisAngle, radiusFraction, sizeFraction;
  if (pitchBlob.size > 0) {
    thisAngle = pitchBlob.centerAngle / 100.0;
    radiusFraction = pitchBlob.centerRadius / 100.0;
    sizeFraction = pitchBlob.size / 100.0;
  } else {
    // Array A idle here but Array B woke this voice -- rest in unison
    // with the root, sustained, at a moderate attack.
    thisAngle = rootAngleFraction;
    radiusFraction = 1.0;
    sizeFraction = 1.0;
  }

  float angleDelta = fabs(thisAngle - rootAngleFraction);
  const float BAND1 = 0.15, BAND2 = 0.35, HYST = 0.02;
  if (intervalClass == 0 && angleDelta > BAND1 + HYST) intervalClass = 1;
  else if (intervalClass == 1 && angleDelta < BAND1 - HYST) intervalClass = 0;
  else if (intervalClass == 1 && angleDelta > BAND2 + HYST) intervalClass = 2;
  else if (intervalClass == 2 && angleDelta < BAND2 - HYST) intervalClass = 1;
  float semitoneOffset = (intervalClass == 0) ? 0.0 : (intervalClass == 1 ? 5.0 : 7.0); // root/octave, P4, P5

  int octaveIndex = (int) round(thisAngle * 4.0); // 0..4 -> C2..C6, quantized
  float targetFreq = ROOT_C2_FREQ * pow(2.0, (float) octaveIndex) * pow(2.0, semitoneOffset / 12.0);

  configureNoteDuration(env, radiusFraction);
  configureNoteVelocity(env, sizeFraction);

  if (!wasPresent) {
    smoothedFreq = targetFreq; // fresh attack: snap straight to pitch
    env.noteOn();
  } else {
    smoothedFreq = glideToward(smoothedFreq, targetFreq, dt, PITCH_GLIDE_TIME_S);
  }
  wasPresent = true;
  return smoothedFreq;
}

// ============================================================================
// noteSynthFromBlob — drives discrete musical PITCH and note GATING for all
// three voices, from Array A (blobsPitch). Timbre stays entirely owned by
// modulateSynthFromBlob() / Array B -- this function only calls
// .frequency(), writes g_voice2RootFreq, and drives noteEnvelope1/2/3.
//
// GATING: a voice is gated ON if EITHER array has an active blob in that
// voice's slot (blobsPitch[i].size>0 OR blobsTimbre[i].size>0). This is
// what makes the two arrays independent-but-combinable: Array A can play
// solo, Array B can "borrow" a voice and play it at its neutral default
// pitch while coloring it, and when both are active on a voice, Array A's
// real values are used.
//
// PITCH GLIDE: a brand-new note attack snaps straight to its target pitch
// (so the attack stays crisp), but once a voice is held, its frequency
// eases toward wherever its blob currently points (see PITCH_GLIDE_TIME_S
// / glideToward()) instead of jumping instantly -- smooths out blob-
// reading jitter and gives Voice 1's continuous glide, and Voices 2/3's
// interval/octave jumps, a gentle slide rather than a hard retune.
//
// Call this BEFORE modulateSynthFromBlob() each loop, so Voice 2's detune
// (in modulateSynthFromBlob) layers on this frame's root pitch rather than
// last frame's.
//
// blobsPitch[0] (angle)  -> Voice 1 root pitch, continuous glide: angle
//                           0.0 = C2, 0.5 = C4, 1.0 = C6. Defaults to 0.5
//                           (C4) when Array A is idle here but Array B
//                           has gated the voice on.
// blobsPitch[0] (radius) -> Voice 1 note duration (stab at 0, sustained
//                           at 1; defaults to 1 / sustained).
// blobsPitch[0] (size)   -> Voice 1 note velocity/attack speed (defaults
//                           to 1 / fast attack).
// blobsPitch[1]/[2]      -> Voices 2/3, pitched relative to Voice 1's
//                           current root (see updateRelativeVoice() above
//                           for the interval/octave logic and idle-default
//                           behavior); same duration/velocity mapping as
//                           Voice 1, using each blob's own radius/size.
// blobsTimbre[]           -> only consulted here for its .size (presence),
//                           to decide gating; its angle/radius values are
//                           read by modulateSynthFromBlob() instead.
// ============================================================================
void noteSynthFromBlob(LDRBlob blobsPitch[3], LDRBlob blobsTimbre[3], uint8_t ultrasonicDistancePercent) {

  static unsigned long lastUpdateMs = millis();
  unsigned long nowMs = millis();
  float dt = (nowMs - lastUpdateMs) / 1000.0;
  lastUpdateMs = nowMs;
  if (dt < 0.0 || dt > 0.5) dt = 0.02; // guard against rollover / first-call / long stalls

  float v1RootAngleFraction = 0.5; // shared with Voices 2/3 below

  // === VOICE 1 — root pitch (continuous) + duration + velocity ===
  {
    static bool v1_wasPresent = false;
    static float v1_smoothedFreq = ROOT_C2_FREQ * pow(2.0, 0.5 * 4.0); // starts at neutral C4

    LDRBlob pitchBlob = blobsPitch[0];
    LDRBlob timbreBlob = blobsTimbre[0];
    bool present = (pitchBlob.size > 0) || (timbreBlob.size > 0);

    if (present) {
      float radiusFraction, sizeFraction;
      if (pitchBlob.size > 0) {
        v1RootAngleFraction = pitchBlob.centerAngle / 100.0;
        radiusFraction = pitchBlob.centerRadius / 100.0;
        sizeFraction = pitchBlob.size / 100.0;
      } else {
        // Array A idle, Array B woke Voice 1 -- rest at the neutral home
        // note: C4, sustained, moderate attack.
        v1RootAngleFraction = 0.5;
        radiusFraction = 1.0;
        sizeFraction = 1.0;
      }

      float targetFreq = rootFreqFromAngle(v1RootAngleFraction);
      if (!v1_wasPresent) {
        v1_smoothedFreq = targetFreq; // fresh attack: snap straight to pitch
      } else {
        v1_smoothedFreq = glideToward(v1_smoothedFreq, targetFreq, dt, PITCH_GLIDE_TIME_S);
      }
      sawWave1.frequency(v1_smoothedFreq);
      squareWave1.frequency(v1_smoothedFreq / 2.0);

      configureNoteDuration(noteEnvelope1, radiusFraction);
      configureNoteVelocity(noteEnvelope1, sizeFraction);

      if (!v1_wasPresent) noteEnvelope1.noteOn();

      if (DEBUG_MODE) {
        Serial.print("  |  Note V1: Freq:"); Serial.print(v1_smoothedFreq, 1);
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
    static float v2_smoothedFreq = ROOT_C2_FREQ * pow(2.0, 7.0 / 12.0);
    float freq = updateRelativeVoice(blobsPitch[1], blobsTimbre[1], v1RootAngleFraction,
                                      noteEnvelope2, v2_wasPresent, v2_intervalClass,
                                      v2_smoothedFreq, dt);
    if (blobsPitch[1].size > 0 || blobsTimbre[1].size > 0) {
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
    static float v3_smoothedFreq = ROOT_C2_FREQ * 4.0;
    float freq = updateRelativeVoice(blobsPitch[2], blobsTimbre[2], v1RootAngleFraction,
                                      noteEnvelope3, v3_wasPresent, v3_intervalClass,
                                      v3_smoothedFreq, dt);
    if (blobsPitch[2].size > 0 || blobsTimbre[2].size > 0) {
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