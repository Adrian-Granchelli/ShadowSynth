
#ifndef AUDIO_SYNTHPLAYER_H
#define AUDIO_SYNTHPLAYER_H

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <IntervalTimer.h>

#ifndef DEBUG_MODE
#define DEBUG_MODE false
#endif

#define ROOM_SIZE 0.65f
#define DAMPING   0.35f
#define DRY_GAIN  0.85f
#define WET_GAIN  0.25f
#define SDCARD_CS_PIN 10


// ---------- AUDIO GRAPH ----------

// Core sound
AudioSynthWaveform sawWave;
AudioSynthWaveform sineWave;
AudioSynthWaveform bodyWave;
AudioSynthNoiseWhite noiseGen;

AudioMixer4 oscMixer;
AudioFilterBiquad lpFilter;

// Fake phaser path (parallel so it never silences the synth)
AudioFilterBiquad phaserA;
AudioFilterBiquad phaserB;
AudioFilterBiquad phaserC;

AudioMixer4 finalMixer;
AudioEffectFreeverb freeverb;
AudioMixer4 masterMixer;
AudioPlaySdWav wavPlayer;
AudioOutputI2S audioOutput;
AudioControlSGTL5000 sgtl5000_1;

// Routing
AudioConnection p1(sawWave,0,oscMixer,0);
AudioConnection p2(sineWave,0,oscMixer,1);
AudioConnection p3(bodyWave,0,oscMixer,2);
AudioConnection p4(noiseGen,0,oscMixer,3);

AudioConnection p5(oscMixer,0,lpFilter,0);

// dry synth path
AudioConnection p6(lpFilter,0,finalMixer,0);

// phaser path in parallel
AudioConnection p7(lpFilter,0,phaserA,0);
AudioConnection p8(phaserA,0,phaserB,0);
AudioConnection p9(phaserB,0,phaserC,0);
AudioConnection p10(phaserC,0,finalMixer,1);

// wav
AudioConnection p11(wavPlayer,0,finalMixer,2);
AudioConnection p12(wavPlayer,1,finalMixer,3);

AudioConnection p13(finalMixer,0,masterMixer,0);
AudioConnection p14(finalMixer,0,freeverb,0);
AudioConnection p15(freeverb,0,masterMixer,1);

AudioConnection p16(masterMixer,0,audioOutput,0);
AudioConnection p17(masterMixer,0,audioOutput,1);

IntervalTimer lfoTimer;

volatile float g_cutoffFreq = 1500.0f;
volatile float g_lfoDepth = 0.5f;
volatile float g_lfoFreq = 1.0f;
volatile float g_filterQ = 0.8f;
volatile float g_phaserAmount = 0.0f;

float lfoPhase = 0;
float phaserPhase = 0;
float smoothedFreq = 1500;

inline float logMap(float x,float low,float high){
  return low * powf(high/low, x);
}

void lfoUpdate(){
  const float dt = 0.001f;

  lfoPhase += 2.0f * PI * g_lfoFreq * dt;
  if(lfoPhase > 2.0f*PI) lfoPhase -= 2.0f*PI;

  float target = g_cutoffFreq * powf(2.0f, g_lfoDepth * sinf(lfoPhase));
  smoothedFreq = smoothedFreq*0.85f + target*0.15f;

  lpFilter.setLowpass(0, constrain(smoothedFreq,40.0f,19000.0f), g_filterQ);

  phaserPhase += (0.0002f + g_phaserAmount*0.002f);

  phaserA.setBandpass(0,700 + 200*sin(phaserPhase),1.2);
  phaserB.setBandpass(0,1500 + 400*sin(phaserPhase*0.7),1.1);
  phaserC.setBandpass(0,3200 + 700*sin(phaserPhase*0.45),1.0);
}

void audioSetup(){

  AudioMemory(120);

  sgtl5000_1.enable();
  sgtl5000_1.volume(0.75);
  sgtl5000_1.dacVolume(1.0);

  SPI.setMOSI(11);
  SPI.setSCK(13);
  SD.begin(SDCARD_CS_PIN);

  sawWave.begin(WAVEFORM_BANDLIMIT_SAWTOOTH);
  sineWave.begin(WAVEFORM_SINE);
  bodyWave.begin(WAVEFORM_TRIANGLE);

  const float ROOT = 65.41f;

  sawWave.frequency(ROOT);
  sineWave.frequency(ROOT);
  bodyWave.frequency(ROOT);

  sawWave.amplitude(0.65f);
  sineWave.amplitude(0.0f);
  bodyWave.amplitude(0.15f);

  noiseGen.amplitude(0);

  oscMixer.gain(0,1.0);
  oscMixer.gain(1,1.0);
  oscMixer.gain(2,1.0);
  oscMixer.gain(3,1.0);

  finalMixer.gain(0,0.8); // dry synth
  finalMixer.gain(1,0.15); // phaser
  finalMixer.gain(2,0.5); // wav L
  finalMixer.gain(3,0.5); // wav R

  masterMixer.gain(0,DRY_GAIN);
  masterMixer.gain(1,WET_GAIN);

  lpFilter.setLowpass(0,1500,0.8);

  freeverb.roomsize(ROOM_SIZE);
  freeverb.damping(DAMPING);

  lfoTimer.begin(lfoUpdate,1000);
}

void modulateSynthFromBlob(LDRBlob blobs[3], uint8_t ultrasonicDistancePercent){

  float sizeNorm = constrain(blobs[0].size/30.0f,0.0f,1.0f);
  float angleNorm = constrain(blobs[0].centerAngle/180.0f,0.0f,1.0f);
  float radiusNorm = constrain((blobs[0].centerRadius-50.0f)/40.0f,0.0f,1.0f);

  // blob1 motion
  g_lfoFreq = 20.0f * powf(0.05f,sizeNorm);
  g_lfoDepth = 0.2f + 1.8f*powf(sizeNorm,1.5f);
  g_cutoffFreq = logMap(angleNorm,300.0f,19000.0f);
  noiseGen.amplitude(radiusNorm*0.15f);

  // blob2 body
  float b2s = constrain(blobs[1].size/30.0f,0.0f,1.0f);
  float b2a = constrain(blobs[1].centerAngle/180.0f,0.0f,1.0f);
  float b2r = constrain((blobs[1].centerRadius-50.0f)/40.0f,0.0f,1.0f);

  bodyWave.amplitude(0.05f + b2s*0.35f);
  g_filterQ = 0.7f + b2a*2.8f;
  bodyWave.frequency(65.41f * powf(2.0f,(b2r*20.0f-10.0f)/1200.0f));

  // blob3 atmosphere
  g_phaserAmount = constrain(blobs[2].size/30.0f,0.0f,1.0f);
  finalMixer.gain(1,0.05f + 0.25f*g_phaserAmount);

  // ultrasonic perspective
  float d = constrain(ultrasonicDistancePercent/100.0f,0.0f,1.0f);

  sawWave.amplitude((1.0f-d)*(0.65f-radiusNorm*0.15f));
  sineWave.amplitude(d*0.65f);

  freeverb.roomsize(0.25f + d*0.65f);

#if DEBUG_MODE
  Serial.print("Cutoff=");Serial.print(g_cutoffFreq);
  Serial.print(" LFOHz=");Serial.print(g_lfoFreq);
  Serial.print(" DepthOct=");Serial.print(g_lfoDepth);
  Serial.print(" Q=");Serial.print(g_filterQ);
  Serial.print(" Noise=");Serial.print(radiusNorm*0.15f);
  Serial.print(" Phaser=");Serial.print(g_phaserAmount);
  Serial.print(" Saw=");Serial.print((1.0f-d));
  Serial.print(" Sine=");Serial.println(d);
#endif
}

void playWAVFile(const char *filename, bool waitToFinish=true){
#if DEBUG_MODE
  Serial.print("Playing file: ");
  Serial.println(filename);
#endif

  wavPlayer.play(filename);

  if(waitToFinish){
    delay(25);
    while(wavPlayer.isPlaying()){}
  }
}

void stopWAVPlayer(){
  if(wavPlayer.isPlaying()){
    wavPlayer.stop();
  }
}

#endif
