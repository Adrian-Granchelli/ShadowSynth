#ifndef uart_h
#define uart_h

#include "globals.h"
#include "blobMaker.h"

// Tracks the last millis() timestamp each individual LDR was seen as 1.
// Sized to match groupedLDRs[freq][NUM_GROUPS].values[5]
unsigned long ldrLastTimeWas1[NUM_BOARDS][NUM_GROUPS][5];
bool ldrFailsafeInitialized = false;

void initLDRFailsafeTimestamps() {
  // Unsigned underflow trick: makes every LDR look like it expired
  // FAILSAFE_TIMEOUT_MS + 1 ago, so the very first check after boot
  // will already consider a 0 reading "stuck" and fail it open to 1.
  unsigned long expiredValue = 0UL - FAILSAFE_TIMEOUT_MS - 1UL;
  for (uint8_t f = 0; f < NUM_BOARDS; f++) {
    for (uint8_t i = 0; i < NUM_GROUPS; i++) {
      for (uint8_t j = 0; j < 5; j++) {
        ldrLastTimeWas1[f][i][j] = expiredValue;
      }
    }
  }
  ldrFailsafeInitialized = true;
}

// Call after groupedLDRs[freq] has been populated for this board.
// Any LDR reading 0 that hasn't read 1 in over FAILSAFE_TIMEOUT_MS
// gets forced to 1.
void failsafeGroupedLDRs(uint8_t freq) {
  if (!ldrFailsafeInitialized) initLDRFailsafeTimestamps();

  unsigned long now = millis();

  for (uint8_t i = 0; i < NUM_GROUPS; i++) {
    for (uint8_t j = 0; j < groupedLDRs[freq][i].size; j++) {
      if (groupedLDRs[freq][i].values[j] == 1) {
        ldrLastTimeWas1[freq][i][j] = now; // still healthy, refresh timestamp
      } else {
        // wraparound-safe elapsed-time check (handles millis() overflow)
        unsigned long elapsed = now - ldrLastTimeWas1[freq][i][j];
        if (elapsed > FAILSAFE_TIMEOUT_MS) {
          if (DEBUG_MODE) {Serial.print("LDR Stuck on: freq: "); Serial.print(freq); Serial.print("\tAngle: "); Serial.print(i); Serial.print("\tRadius: "); Serial.println(j);}
          groupedLDRs[freq][i].values[j] = 1; // stuck at 0 too long -> fail open
        }
      }
    }
  }
}

int averageDistance(uint8_t freq) {
  uint16_t sum = 0; 
  for (uint8_t i = 0; i < 5; i++) {
    sum += ultrasonicDistance[freq][i]; 
  }
  return sum / 5;
}

void distanceToPercentage(uint8_t freq) {
  float distanceCalc = 1.0 * averageDistance(freq) / ultrasonicDistanceMax;
  distanceCalc = pow(distanceCalc, distancePowerFactor);
  ultrasonicDistancePercent[freq] = distanceCalc * 100;
  // Serial.print(distanceCalc); Serial.print(" - "); Serial.println(ultrasonicDistancePercent[freq]);
}

// --- PARSING FUNCTION ---
// for each board, [0], [1], or [2]
void parseIncomingUART(uint8_t freq, int bytesRead) {
  // 2. Only proceed if we actually caught something
  if (bytesRead > 0) {
    if (uartBuffer[bytesRead - 1] == '\r') { bytesRead--; } // remove the windows style next line
    uartBuffer[bytesRead] = '\0';
    // just return if the packet is corrupted
    for (int i = 0; i < bytesRead; i++) {
      char c = uartBuffer[i];
      if (!((c >= '0' && c <= '9') || c == ',')) { return; }
    }
   
    uint8_t rawLDRs[TOTAL_LDRS] = {};
    
    // 1. CHOP THE STRING BY COMMAS
    // strtok() replaces the first ',' it finds with a null terminator and returns the chunk
    char* token = strtok(uartBuffer, ",");
    uint8_t count = 0;
    
    // Extract all 59 LDR values
    while (token != NULL && count < TOTAL_LDRS) {
      int parsedValue = atoi(token); // Convert text to integer
  
      // If the parsed value is exactly 0, save as 0. Otherwise, save as 1.
      rawLDRs[count] = (parsedValue == 0) ? 0 : 1;

      token = strtok(NULL, ",");    // Grab the next chunk
      count++;
    }
    
    // Extract the final integer (Ultrasonic Distance)
    if (token != NULL) {
      int parsedDistance = atoi(token);
      // If it's invalid within range, save it, otherwise skip 
      if (parsedDistance <= ultrasonicDistanceMax) {
        ultrasonicDistance[freq][ultrasonicDistanceHistoryCount] = parsedDistance;
      }
    }
      
    // 2. ORGANIZE INTO THE 5-1-5-1 GROUPING
    int rawIndex = 0;
    
    for (uint8_t i = 0; i < NUM_GROUPS; i++) {
      // Even indices (0, 2, 4...) get 5 items. Odd indices (1, 3, 5...) get 1 item.
      if (i % 2 == 0) {
        groupedLDRs[freq][i].size = 5;
        for (uint8_t j = 0; j < 5; j++) {
          if (rawIndex < TOTAL_LDRS) {
            groupedLDRs[freq][i].values[j] = rawLDRs[rawIndex++];
          }
        }
      } else {
        groupedLDRs[freq][i].size = 1;
        if (rawIndex < TOTAL_LDRS) {
          groupedLDRs[freq][i].values[0] = rawLDRs[rawIndex++];
        }
      }
    }

    failsafeGroupedLDRs(freq); 

    calculateBlobs(freq);
    distanceToPercentage(freq);

  }
}


void printUART(uint8_t freq) {
  if (DEBUG_MODE) {
    // --- DEBUG VERIFICATION (VISUAL GRID) ---
    Serial.println("--- Sensor Grid ---");

    // Loop through all 19 groups
    for (uint8_t i = 0; i < NUM_GROUPS; i++) {
      
      // Print every item in this specific group on the SAME line
      for (uint8_t j = 0; j < groupedLDRs[freq][i].size; j++) {
        Serial.print(groupedLDRs[freq][i].values[j]);
        Serial.print(" "); // Add a space between the numbers
      }
      
      // Hit 'Enter' to move to the next line for the next group
      Serial.println(); 
    }

    Serial.print("Distance: ");
    Serial.println(averageDistance(freq));
    Serial.println("-------------------");
  }
}

#endif