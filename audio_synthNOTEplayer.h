///////////////////////////////////////////////////
//// I/O Mimicry                               ////
//// A light tag game, canvas, or competition  ////
//// Code By Adrian Granchelli                 ////
//// (Converted to Polyphonic Synth Engine)    ////
///////////////////////////////////////////////////

#ifndef audio_h
#define audio_h

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// If you no longer need the loadSounds.h file for file names, you can comment this out
#include "loadSounds.h" 

uint8_t playerGain = 15;
const int NUM_SYNTHS = MAX_SOUND_FILES - 1; // Keeping your previous polyphony count

// 1. DECLARE THE SYNTHESIZERS (Replacing Flash Players)
AudioPlaySdWav audioPlayerWAV;
AudioSynthWaveform synths[NUM_SYNTHS]; 

// Tracks the actual, smoothed volume of all 10 synths to prevent clicking
float currentVolumes[NUM_SYNTHS]; 

AudioOutputI2S audioOutput;

// 2. DECLARE MIXERS
AudioMixer4 audioMixer1, audioMixer2, audioMixer3, audioMixer4, audioMixer5, audioMixer6;
AudioMixer4 audioFinalMixer; 
AudioMixer4 audioSecondaryFinalMixer; 
AudioMixer4 audioMasterMixer; 

// 3. DECLARE EFFECTS
AudioEffectFreeverb freeverb;

// 4. DECLARE CONNECTIONS
AudioConnection* audioConnections[24];
AudioConnection* audioWAVConnection;
AudioConnection* audioFinalConnections[7];

// Master/Effect Connections
AudioConnection* drySignalToMasterMixer;
AudioConnection* freeverbSendConnection; 
AudioConnection* wetSignalToMasterMixer; 
AudioConnection* masterMixerToOutputLeft;
AudioConnection* masterMixerToOutputRight;

AudioControlSGTL5000 sgtl5000_1;

#define SDCARD_CS_PIN    10

// Helper function to return a pointer to the appropriate mixer
AudioMixer4* getMixer(uint8_t index) {
  switch(index) {
    case 0: return &audioMixer1;
    case 1: return &audioMixer2;
    case 2: return &audioMixer3;
    case 3: return &audioMixer4;
    case 4: return &audioMixer5;
    case 5: return &audioMixer6;
    default: return nullptr;
  }
}

// ==========================================
// SETUP FUNCTION
// ==========================================
void audioSetup() {
  // 1. ALLOCATE ALL CONNECTIONS FIRST
  for (uint8_t i = 0; i < NUM_SYNTHS; i++) {
    currentVolumes[i] = 0.0;

    uint8_t mixerNumber = i / 4;  
    uint8_t mixerChannel = i % 4; 
    
    // Connect Synth -> Mixer
    audioConnections[i] = new AudioConnection(synths[i], 0, *(getMixer(mixerNumber)), mixerChannel);
    getMixer(mixerNumber)->gain(mixerChannel, playerGain/100.00); 
  }

  // Connect first level mixers to final mixers
  for (uint8_t i = 0; i < 4; i++) {
    audioFinalConnections[i] = new AudioConnection(*(getMixer(i)), 0, audioFinalMixer, i);
  }
  for (uint8_t i = 4; i < 6; i++) {
    audioFinalConnections[i] = new AudioConnection(*(getMixer(i)), 0, audioSecondaryFinalMixer, i - 4);
  }

  audioFinalConnections[6] = new AudioConnection(audioFinalMixer, 0, audioSecondaryFinalMixer, 2);
  audioWAVConnection = new AudioConnection(audioPlayerWAV, 0, audioSecondaryFinalMixer, 3);
  // Set the WAV player volume to 2%
  audioSecondaryFinalMixer.gain(3, 0.02);

  // Mono and Freeverb Integration
  drySignalToMasterMixer = new AudioConnection(audioSecondaryFinalMixer, 0, audioMasterMixer, 0);
  freeverbSendConnection = new AudioConnection(audioSecondaryFinalMixer, 0, freeverb, 0);
  wetSignalToMasterMixer = new AudioConnection(freeverb, 0, audioMasterMixer, 1);
  masterMixerToOutputLeft = new AudioConnection(audioMasterMixer, 0, audioOutput, 0); 
  masterMixerToOutputRight = new AudioConnection(audioMasterMixer, 0, audioOutput, 1); 

  // 2. START THE AUDIO SYSTEM
  AudioMemory(120); 
  sgtl5000_1.enable();
  sgtl5000_1.volume(1);

  // 3. INITIALIZE SYNTHS TO BE SILENT ON BOOT
  for (uint8_t i = 0; i < NUM_SYNTHS; i++) {
    synths[i].begin(WAVEFORM_SINE); // You can change to WAVEFORM_SAWTOOTH, etc.
    synths[i].amplitude(0.0);       // Start completely silent
    synths[i].frequency(440.0);     // Default pitch (A4)
  }

  // 4. SET VOLUMES & EFFECTS
  // (Assuming DRY_GAIN, WET_GAIN, ROOM_SIZE, DAMPING are defined elsewhere)
  // audioMasterMixer.gain(0, DRY_GAIN); 
  // audioMasterMixer.gain(1, WET_GAIN); 
  // freeverb.roomsize(ROOM_SIZE); 
  // freeverb.damping(DAMPING); 

  // 5. HARDWARE INITIALIZATION (SD Only)
  if (!(SD.begin(SDCARD_CS_PIN))) {
    if (DEBUG_MODE) { Serial.println("No SD card found (or not needed)"); }
  }
}


// ==========================================
// SYNTH CONTROL FUNCTIONS
// ==========================================

// Turns a specific synth oscillator on at a specific frequency
void playSynth(uint8_t n, float frequency, float volume = 0.5) {
  if (n < NUM_SYNTHS) {
    synths[n].frequency(frequency);
    synths[n].amplitude(volume); 
  }
}

// Mutes a specific synth oscillator
void stopSynth(uint8_t n) {
  if (n < NUM_SYNTHS) {
    synths[n].amplitude(0.0);
  }
}

// Mutes all synth oscillators instantly
void stopAllSynths() {
  for (uint8_t i = 0; i < NUM_SYNTHS; i++) {
    synths[i].amplitude(0.0);
  }
}


// ==========================================
// MIXER / WAV CONTROL (Unchanged)
// ==========================================

void setPlayerVolume(uint8_t playerIndex, uint8_t volume) {
    uint8_t mixerNumber = playerIndex / 4;    
    uint8_t mixerChannel = playerIndex % 4;   
    AudioMixer4* mixer = getMixer(mixerNumber);
    if (mixer != nullptr) {
        mixer->gain(mixerChannel, playerGain * volume / 10000.00);
    }
}

void setPlayersVolume(uint8_t volume) {
  for (uint8_t i = 0; i < 24; i++) {
    setPlayerVolume(i, volume);
  }
}

void playWAVFile(const char *filename, bool waitToFinish = true) {
  if (DEBUG_MODE) {Serial.print("Playing file: "); Serial.println(filename);}
  audioPlayerWAV.play(filename);
  if (waitToFinish) {
    delay(25);
    while (audioPlayerWAV.isPlaying()) {}
  }
}

void stopWAVPlayer() {
  if (audioPlayerWAV.isPlaying()) {
    audioPlayerWAV.stop();
  }
}

// 10-Note Custom Voicing 
const float CHORD_TUNING[10] = {
  55.00,   // A1
  73.42,   // D2
  82.41,   // E2
  
  110.00,  // A2
  146.83,  // D3
  164.81,  // E3

  220.00,  // A3
  293.66,  // D4
  329.63,  // E4
  
  440.00,  // A4
};

const float MAX_ROW_VOLUME = 0.1;

// Each row is a wave of root, 4th, 5th, etc. 
void mutliphonicRowSynths(uint8_t freq) {
  int synthIndex = 0; // Tracks which of the 10 synths we are currently updating

  // Loop through all 19 visual groups in your layout
  for (int i = 0; i < NUM_GROUPS; i++) {
    // We ONLY want to apply this logic to the rows that have 5 sensors
    if (groupedLDRs[freq][i].size == 5) {
      // 1. Count the shadows in this specific row
      int activeShadows = 0;
      for (int j = 0; j < 5; j++) {
        if (groupedLDRs[freq][i].values[j] == 0) { 
          activeShadows++;
        }
      }

      // 2. Calculate the volume (e.g., 3 shadows / 5.0 = 60% volume) 
        // 1. Find the raw linear fraction (0.0 to 1.0)
        float linearFraction = activeShadows / 5.0;

        // 2. Square the fraction to create an exponential "Audio Taper", then apply max volume
        float targetVolume = (linearFraction * linearFraction) * MAX_ROW_VOLUME;

      // 3. THE SHOCK ABSORBER (Slew Limiting)
      // This glides the volume instead of snapping it. 
      // 0.9 controls the speed. (0.9 is smooth/slow, 0.5 is fast)
      currentVolumes[synthIndex] = (currentVolumes[synthIndex] * 0.5) + (targetVolume * 0.5);
      
      // 4. Apply the heavily smoothed volume to the synth
      synths[synthIndex].amplitude(currentVolumes[synthIndex]);
      
      // 5. Assign the pitch 
      synths[synthIndex].frequency(CHORD_TUNING[synthIndex]);

      // Move to the next synthesizer for the next row of 5
      synthIndex++; 
    }
  }
}

#endif