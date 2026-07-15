#ifndef uart_h
#define uart_h

#include "globals.h"
#include "blobMaker.h"

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
// for FREQ = BASS is [0], MID is [1], TREB is [2]
void parseIncomingUART(uint8_t freq, int bytesRead) {
    // 2. Only proceed if we actually caught something
    if (bytesRead > 0) {
    // 3. CAP THE STRING: Place a null terminator exactly at the end of the new data
    uartBuffer[bytesRead] = '\0';
   
    uint8_t rawLDRs[TOTAL_LDRS];
    
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
  }

  calculateBlobs(freq);
  distanceToPercentage(freq);
}


void printUART(uint8_t freq) {
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

#endif