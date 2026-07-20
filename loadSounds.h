#ifndef loadsounds_h
#define loadsounds_h

#include "globals.h" // Assuming this contains MAX_SOUND_FILES and DEBUG_MODE
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// Buffer size for copying data. Larger is faster but uses more RAM during copy.
#define COPY_BUFFER_SIZE 512


/**
 * @brief Copies WAV files from SD card to SPI Flash.
 * @param sdBaseFolderName The name of the folder on the SD card containing the WAV files.
 * Files are expected to be named "0.RAW", "1.RAW", ..., "MAX_BUTTON-1.RAW".
 * @details This function iterates from 0 to MAX_BUTTON-1, constructs filenames,
 * opens the file from SD, creates/overwrites the file on SPI Flash, and copies
 * the content block by block.
 */
void copySoundsFromSdToFlash(const char* sdBaseFolderName) {
  if (DEBUG_MODE) {
    Serial.print("Attempting to copy "); Serial.print(MAX_SOUND_FILES); Serial.print(" sounds from SD folder: /"); Serial.print(sdBaseFolderName); Serial.print(" to Flash folder: /");
  }

  byte copyBuffer[COPY_BUFFER_SIZE]; // Buffer for copying data

  for (uint8_t i = 0; i < MAX_SOUND_FILES-1; i++) {
    char sdFilename[64];
    char flashFilename[64];

    // Construct SD card filename
    sprintf(sdFilename, "/%s/%d.RAW", sdBaseFolderName, i);

    // Construct SPI Flash filename 
    sprintf(flashFilename, "/%d.RAW", i); // Store in root of flash


    if (DEBUG_MODE) {
      Serial.print("Copying "); Serial.print(sdFilename); Serial.print(" to "); Serial.print(flashFilename); Serial.print("...");
    }

    File sdFile = SD.open(sdFilename);
    if (!sdFile) {
      if (DEBUG_MODE) {Serial.println(" FAILED to open on SD!");}
      continue; // Move to the next file
    }

    size_t fileSize = sdFile.size();
    if (DEBUG_MODE) {
      Serial.print(" Size: "); Serial.print(fileSize); Serial.print(" bytes. ");
    }

    // Check if file already exists on flash and remove if desired (optional)
    if (SerialFlash.exists(flashFilename)) {
      if (DEBUG_MODE) {Serial.print("File exists on Flash, overwriting...");}
      SerialFlash.remove(flashFilename); // Remove existing file
    }

    // *** FIX IS HERE ***
    // 1. Call SerialFlash.create() and store its boolean return value.
    bool createSuccess = SerialFlash.create(flashFilename, fileSize);
    
    if (!createSuccess) {
      if (DEBUG_MODE) {Serial.println(" FAILED to create on Flash! (Is there enough space or too many files?)");}
      sdFile.close();
      continue; // Move to the next file
    }

    // 2. ONLY if creation was successful, then open the file to get the SerialFlashFile object.
    SerialFlashFile flashFile = SerialFlash.open(flashFilename);
    if (!flashFile) { // This check is mostly for robustness; should usually succeed if create was true
      if (DEBUG_MODE) {Serial.println(" FAILED to open the newly created file on Flash!");}
      sdFile.close();
      // Consider trying to remove the partially created file if open fails
      SerialFlash.remove(flashFilename);
      continue;
    }

    // Copy data block by block
    size_t bytesCopied = 0;
    while (bytesCopied < fileSize) {
      size_t bytesToRead = min((size_t)COPY_BUFFER_SIZE, fileSize - bytesCopied);
      int bytesRead = sdFile.read(copyBuffer, bytesToRead);
      if (bytesRead <= 0) {
        if (DEBUG_MODE) {Serial.println(" FAILED to read from SD card!");}
        flashFile.close(); // Close partial flash file
        sdFile.close();
        // You might want to remove the partially written file from flash here
        SerialFlash.remove(flashFilename);
        break; // Exit loop for current file
      }

      flashFile.write(copyBuffer, bytesRead);
      bytesCopied += bytesRead;
      if (DEBUG_MODE) {Serial.print(".");} // Show progress}
    }

    flashFile.close(); // IMPORTANT: Close the flash file when done writing
    sdFile.close();

    if (DEBUG_MODE) {
      if (bytesCopied == fileSize) {
        Serial.println(" SUCCESS!");
      } else {
        Serial.println(" FAILED: Incomplete copy!");
      }
    }
  }
  if (DEBUG_MODE) {Serial.println("Finished attempting to copy all sounds from SD to Flash.");}
}


/**
 * @brief Removes all files from the SPI Flash. Use with caution!
 */
void eraseAllFlashFiles() {
  if (DEBUG_MODE) {
    Serial.println("\nErasing all files on SPI Flash... This may take a moment.");
  }
  SerialFlash.eraseAll();
  while (!SerialFlash.ready()) {
    delay(10); // Wait for erase to complete
  }
  if (DEBUG_MODE) {
    Serial.println("SPI Flash erased.");
  }
}

void listFilesonFlash() {
  if (DEBUG_MODE) {Serial.println("\n--- Listing Files on SPI Flash ---");}

  char filename[64];   // Buffer to hold the filename
  uint32_t filesize;   // Variable to hold the file size

  // Prepare the SerialFlash for reading directory entries.
  // This needs to be called once before starting to read files with readdir().
  SerialFlash.opendir();

  // Loop through files using readdir().
  // readdir() returns true if a file is found, false otherwise.
  // It populates 'filename' and 'filesize'.
  while (SerialFlash.readdir(filename, sizeof(filename), filesize)) {
    if (DEBUG_MODE) {
      Serial.print("File: "); Serial.print(filename); Serial.print(", Size: "); Serial.print(filesize); Serial.println(" bytes");}
  }

  if (DEBUG_MODE) { Serial.println("--- End of File List ---"); }
}

#endif