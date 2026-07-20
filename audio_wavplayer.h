///////////////////////////////////////////////////
//// I/O Mimicry                               ////
//// A light tag game, canvas, or competition  ////
//// Code By Adrian Granchelli                 ////
///////////////////////////////////////////////////

#ifndef audio_h
#define audio_h

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#include "loadSounds.h"

uint8_t playerGain = 15;

// Create a struct to hold each sound and their particularities
struct AudioSet {
  // which audioplayer is played from the file, 255 is null
  uint8_t audioPlayer;
  char filename; // the filename

  // Function to initialize the struct with default values
  void initialize() {
    audioPlayer = 255;  // Default value indicating no player
  }
};

// Declare an array of AudioPlaySdWav objects
AudioPlaySdWav audioPlayerWAV;
AudioPlaySerialflashRaw audioPlayers[MAX_SOUND_FILES-1]; // THIS IS 15 right now

AudioOutputI2S audioOutput;

// Declare mixers
AudioMixer4 audioMixer1, audioMixer2, audioMixer3, audioMixer4, audioMixer5, audioMixer6;
AudioMixer4 audioFinalMixer;  // First final mixer that combines first 4 mixers
AudioMixer4 audioSecondaryFinalMixer;  // Second final mixer that combines the remaining mixers
AudioMixer4 audioMasterMixer; // NEW: Master mixer for final mono sum

// Declare Freeverb effect
AudioEffectFreeverb freeverb;

// Connections from players to their respective mixers
AudioConnection* audioConnections[24];
AudioConnection* audioWAVConnection;

// Connections from first level mixers to final mixers
AudioConnection* audioFinalConnections[7];

// NEW: Connections for the master mixer and Freeverb
AudioConnection* drySignalToMasterMixer;
AudioConnection* freeverbSendConnection; // Connection to send audio to freeverb
AudioConnection* wetSignalToMasterMixer; // Connection for freeverb output to master mixer
AudioConnection* masterMixerToOutputLeft;
AudioConnection* masterMixerToOutputRight;

// REMOVE these as the new audioMasterMixer will handle the final output connections
// AudioConnection finalMixConnection(audioSecondaryFinalMixer, 0, audioOutput, 0);
// AudioConnection finalMixConnectionRight(audioSecondaryFinalMixer, 0, audioOutput, 1);

AudioControlSGTL5000 sgtl5000_1;

#define FLASH_CS_PIN 6 // Teensy 4.0 Audio Shield SPI Flash CS pin
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
    default: return nullptr;  // Return nullptr if out of bounds
  }
}

void audioSetup() {
  // Audio connections require memory to work.
  // You might need to increase this if you add more complex effects.
  AudioMemory(35); // Increased memory for potentially more connections and effects

  sgtl5000_1.enable();
  sgtl5000_1.volume(1); // Set master volume

  if (!(SD.begin(SDCARD_CS_PIN))) {
    while (1) {
      if (DEBUG_MODE) { Serial.println("Unable to access the SD card"); }
      delay(500);
    }
  }

  SerialFlash.begin(FLASH_CS_PIN);
  if (DEBUG_MODE) {
    if (SerialFlash.begin(FLASH_CS_PIN)) {
      Serial.println("SerialFlash initialized successfully!");
    } else {
      Serial.println("SerialFlash initialization FAILED!");
    }
  }

  if (DEBUG_MODE) {
    uint8_t id[5];
    SerialFlash.readID(id);
    Serial.print("SerialFlash ID: ");
    for (uint8_t i = 0; i < 5; i++) {
      Serial.print(id[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    Serial.println("SerialFlash initialized successfully.");
  }

  // Initialize connections for each player to their respective mixers
  for (uint8_t i = 0; i < MAX_SOUND_FILES; i++) {
    uint8_t mixerNumber = i / 4;  // Determines which mixer to connect to (0 to 5)
    uint8_t mixerChannel = i % 4; // Each mixer has 4 channels (0 to 3)
    audioConnections[i] = new AudioConnection(audioPlayers[i], 0, *(getMixer(mixerNumber)), mixerChannel);
    getMixer(mixerNumber)->gain(mixerChannel, playerGain/100.00); // Setting a conservative initial gain
  }

  // Connect each of the first level mixers to the final mixers
  for (uint8_t i = 0; i < 4; i++) {
    audioFinalConnections[i] = new AudioConnection(*(getMixer(i)), 0, audioFinalMixer, i);
  }
  for (uint8_t i = 4; i < 6; i++) {
    audioFinalConnections[i] = new AudioConnection(*(getMixer(i)), 0, audioSecondaryFinalMixer, i - 4);
  }

  // Connect audioFinalMixer (which combines mixers 1-4) to an input of audioSecondaryFinalMixer.
  // This means audioSecondaryFinalMixer's output (channel 0) effectively sums all player audio.
  audioFinalConnections[6] = new AudioConnection(audioFinalMixer, 0, audioSecondaryFinalMixer, 2);

  // Connect WAV player to a channel of audioSecondaryFinalMixer
  audioWAVConnection = new AudioConnection(audioPlayerWAV, 0, audioSecondaryFinalMixer, 3);

  // **** Mono and Freeverb Integration ****

  // 1. Send the combined dry signal (from audioSecondaryFinalMixer) to the master mixer.
  //    This will be the 'dry' part of your wet/dry mix.
  drySignalToMasterMixer = new AudioConnection(audioSecondaryFinalMixer, 0, audioMasterMixer, 0);
  audioMasterMixer.gain(0, 0.7); // Set initial gain for dry signal (e.g., 70%)

  // 2. Send the combined dry signal to the Freeverb's input.
  freeverbSendConnection = new AudioConnection(audioSecondaryFinalMixer, 0, freeverb, 0);

  // 3. Connect Freeverb's output (we only take the left channel for mono) to the master mixer.
  //    This will be the 'wet' part of your wet/dry mix.
  wetSignalToMasterMixer = new AudioConnection(freeverb, 0, audioMasterMixer, 1);
  audioMasterMixer.gain(1, 0.3); // Set initial gain for wet signal (e.g., 30%)

  // 4. Set Freeverb parameters
  freeverb.roomsize(0.7); // Adjust for desired room size (0.0 to 1.0)
  freeverb.damping(0.5);  // Adjust for high-frequency damping (0.0 to 1.0)

  // 5. Connect the final mono master mixer output to both left and right channels of the AudioOutput.
  masterMixerToOutputLeft = new AudioConnection(audioMasterMixer, 0, audioOutput, 0); // Master output to Left
  masterMixerToOutputRight = new AudioConnection(audioMasterMixer, 0, audioOutput, 1); // Master output to Right


  audioMasterMixer.gain(0, DRY_GAIN); // Set gain for the dry signal channel (second value)
  audioMasterMixer.gain(1, WET_GAIN); // Set gain for the wet signal (reverb) channel (second value)
  freeverb.roomsize(ROOM_SIZE); // 0 is small 1.00 large 
  freeverb.damping(DAMPING); // 0 is high free decay slow, 1 is low decay slow
}


// Example function to set the gain for a specific player
void setPlayerVolume(uint8_t playerIndex, uint8_t volume) {
    uint8_t mixerNumber = playerIndex / 4;    // Determines which mixer (0 to 5)
    uint8_t mixerChannel = playerIndex % 4;   // Which channel on the mixer (0 to 3)

    // Retrieve the appropriate mixer based on the player index and set the gain
    AudioMixer4* mixer = getMixer(mixerNumber);
    if (mixer != nullptr) {
        mixer->gain(mixerChannel, playerGain * volume / 10000.00);
        //Serial.print("Volume set to: "); Serial.println(playerGain * volume / 10000.00);
    }
}


// Example function to set the gain for a specific player
void setPlayersVolume(uint8_t volume) {
  // Initialize connections for each player to their respective mixers
  for (uint8_t i = 0; i < 24; i++) {
    setPlayerVolume(i, volume);
  }
}


///////////////////////////////////////
// PLAYING AUDIO
///////////////////////////////////////
void playFlash(uint8_t n, bool waitToFinish = true) {
  if (!audioPlayers[n].isPlaying()) {
    String filename = "/" + String(n) + ".RAW";
    if (DEBUG_MODE) {Serial.print("Playing file: "); Serial.println(filename);}

    // Check if the file exists on SerialFlash
    if (!SerialFlash.exists(filename.c_str())) {
      if (DEBUG_MODE) {
        Serial.print("ERROR: File does not exist on SerialFlash: ");
        Serial.println(filename);
            }
      return; // Exit if file not found
    }

    // Start playing the file.  This sketch continues to run while the file plays.
    audioPlayers[n].play(filename.c_str());
    //Serial.print("Max Audio Usage: "); Serial.println(AudioMemoryUsageMax());
    //Serial.print("Max Processor Usage: "); Serial.println(AudioProcessorUsageMax()); AudioProcessorUsageMaxReset();

    // Simply wait for the file to finish playing.
    if (waitToFinish) {
      // A brief delay for the library read WAV info
      delay(25);
      while (audioPlayers[n].isPlaying()) {
      }
    }
  }
}

void stopFlashPlayer(uint8_t n) {
  // Check if the player is currently playing and stop it
  if (audioPlayers[n].isPlaying()) {
    audioPlayers[n].stop();
  }
}

void stopAllFlashPlayer() {
  for (uint8_t i=0; i < MAX_SOUND_FILES-1; i++) {
    stopFlashPlayer(i);
  }
}


void playWAVFile(const char *filename, bool waitToFinish = true) {
  if (DEBUG_MODE) {Serial.print("Playing file: "); Serial.println(filename);}
  // Start playing the file.  This sketch continues to

  // run while the file plays.
  audioPlayerWAV.play(filename);
  //Serial.print("Max Audio Usage: "); Serial.println(AudioMemoryUsageMax());
  //Serial.print("Max Processor Usage: "); Serial.println(AudioProcessorUsageMax()); AudioProcessorUsageMaxReset();

  // Simply wait for the file to finish playing.
  if (waitToFinish) {
    // A brief delay for the library read WAV info
    delay(25);
    while (audioPlayerWAV.isPlaying()) {
    }
  }
}



void stopWAVPlayer() {
  // Check if the player is currently playing and stop it
  if (audioPlayerWAV.isPlaying()) {
    audioPlayerWAV.stop();
  }
}



///////////////////////////////////////
// GET AUDIO filenameS
///////////////////////////////////////
void listFoldersInDirectory(const char* parentPath = "/") {
  // Reset folderCount before starting a new scan
  folderCount = 0;

  if (DEBUG_MODE) {
    Serial.print("Scanning directory: "); Serial.println(parentPath);
  }

  // Open the parent directory
  File dir = SD.open(parentPath);

  // Check if the directory was opened successfully
  if (!dir) {
    if (DEBUG_MODE) {
      Serial.print("Failed to open directory: ");
      Serial.println(parentPath);
    }
    return; // Exit the function if the directory can't be opened
  }

  // Check if the opened item is actually a directory
  if (!dir.isDirectory()) {
    if (DEBUG_MODE) {
      Serial.print(parentPath);
      Serial.println(" is not a directory.");
    }
    dir.close(); // Close the file object
    return; // Exit the function
  }

  // Loop through all entries (files and subdirectories) in the opened directory
  while (true) {
    File entry = dir.openNextFile(); // Get the next entry

    // If there are no more entries, break out of the loop
    if (!entry) {
      break;
    }

    // Check if the current entry is a directory
    if (entry.isDirectory()) {
      // If we haven't exceeded our maximum capacity for storing folder names
      if (folderCount < MAX_FOLDERS) {
        // Store the folder's name in our global array
        folderNames[folderCount] = entry.name();
        folderCount++; // Increment the count of found folders
      } else {
        if (DEBUG_MODE) {Serial.println("Warning: Maximum folder limit reached. Some folders may not be stored.");}
      }
    }
    entry.close(); // Close the current entry before moving to the next
  }

  dir.close(); // Close the parent directory once scanning is complete
  if (DEBUG_MODE) {Serial.println("Finished scanning directory.");}
}


#endif