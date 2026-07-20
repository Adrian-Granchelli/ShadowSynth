bool DEBUG_MODE = false;
bool RELOAD_ALL_FILES = false;

#include <HardwareSerial.h>
#include <SerialFlash.h> // Include the SerialFlash library

#include "globals.h"
#include "UART.h"

#include "audio_synthplayer.h"
#include "loadSounds.h"

#include "action.h"

void setup() {
  // This uses the USB connection to your computer.
  if (DEBUG_MODE) {
    Serial.begin(9600);
    Serial.println("");Serial.println("");Serial.println("");Serial.println("Starting up");
  }
  // sets Uart - Set HardwareSerial ports with the defined baud rate.
  Serial4.setTimeout(serialTimeout); Serial4.begin(115200); // yellow wire -> treble 
  Serial6.setTimeout(serialTimeout); Serial6.begin(115200); // green wire -> Bass
  Serial7.setTimeout(serialTimeout); Serial7.begin(115200); // grey -> Mid

  pinMode(button_pin, INPUT_PULLUP); // Set the button pin as an input
  audioSetup(); // initialize the audio and sd card 
  playWAVFile("START.WAV", false);

    ///////////////////////////////////////
  // -- get the names of the sound folders  --
  ///////////////////////////////////////
   
  /*listFoldersInDirectory("/SOUNDS"); // if playing sound files  

  lastSoundChange = millis();

  if (DEBUG_MODE) {
    // Print the names of the folders found
    if (folderCount > 0) {
      Serial.print("\nFound ");
      Serial.print(folderCount);
      Serial.println(" folders:");
      for (int i = 0; i < folderCount; i++) {
        Serial.print("  - ");
        Serial.println(folderNames[i]);
      }
    } else {
      Serial.println("\nNo folders found in the specified directory.");
    }
  }

  if (RELOAD_ALL_FILES) {
    stopWAVPlayer(); // free up the memory
    // Copy Sounds to the SPI FLASH
    eraseAllFlashFiles();
    copySoundsFromSdToFlash(("SOUNDS/" + folderNames[folder_n]).c_str());
  } 

  if (DEBUG_MODE) {
    listFilesonFlash();
  }*/

  if (DEBUG_MODE) {Serial.println("Setup Complete.");}
  playWAVFile("STARTDONE.WAV", true);

}

void loop() {
  now = millis();
  bool updateSynth = false;

  // check if serial incoming
  if (Serial7.available() || Serial6.available() || Serial4.available()) {
    ultrasonicDistanceHistoryCount++; 
    updateSynth = true;
    if (ultrasonicDistanceHistoryCount > ultrasonicDistanceHistoryMax) {
      ultrasonicDistanceHistoryCount = 0; 
    }
  }
  
  // read the serial data 
  if (Serial7.available()) {
    updateSynth = true;
    // Read the string until the Pico sends the newline ('\n') character
    uint16_t bytesRead = Serial7.readBytesUntil('\n', uartBuffer, BUFFER_SIZE - 1);
    parseIncomingUART(0, bytesRead);
    //printUART(0);
    //printBlobs(0);
  }
  if (Serial6.available()) {
    updateSynth = true;
    // 1. Scoop up the incoming data - Read the string until the Pico sends the newline ('\n') character
    uint16_t bytesRead = Serial6.readBytesUntil('\n', uartBuffer, BUFFER_SIZE - 1);
    parseIncomingUART(1, bytesRead);
    printUART(1);
    printBlobs(1);
  }
  if (Serial4.available()) {
    updateSynth = true;
    // Read the string until the Pico sends the newline ('\n') character
    uint16_t bytesRead = Serial4.readBytesUntil('\n', uartBuffer, BUFFER_SIZE - 1);
    parseIncomingUART(2, bytesRead);
    //printUART(2);
    //printBlobs(2);
  }

  if (updateSynth) {
    modulateSynthFromBoards(LDRBlobs, ultrasonicDistancePercent);
  }

  if (now - LAST_LOOP > LOOP_WAIT ) {
    LAST_LOOP = now;

    // do if button is pressed 
    if (digitalRead(button_pin) == LOW) {
      if (DEBUG_MODE) {
          Serial.println("Button Pressed");
        }
      }
  }
}