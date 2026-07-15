#ifndef blobMaker_h
#define blobMaker_h

#include "globals.h"

// Recursive Flood Fill function
void floodFill(int x, int y, int &sumX, int &sumY, int &count, int radiusCounts[5], uint8_t grid[19][5], bool visited[19][5]) {
  // 1. Boundary Check: Stop if we go off the edges of the 5x19 grid
  if (x < 0 || x >= 5 || y < 0 || y >= 19) return;
  
  // 2. State Check: Stop if we already counted this sensor, or if it is light (0)
  // Note: Because the physical empty spaces on your board are 0, this naturally 
  // prevents shadows from jumping vertically without using the middle bridge!
  if (visited[y][x] || grid[y][x] == 0) return;

  // 3. Mark as visited and add to our blob totals
  visited[y][x] = true;
  sumX += x;
  sumY += y;
  count++;

  radiusCounts[x]++;

  // 4. Look in all 4 directions for touching shadows (The Maze logic)
  floodFill(x + 1, y, sumX, sumY, count, radiusCounts, grid, visited);
  floodFill(x - 1, y, sumX, sumY, count, radiusCounts, grid, visited);
  floodFill(x, y + 1, sumX, sumY, count, radiusCounts, grid, visited);
  floodFill(x, y - 1, sumX, sumY, count, radiusCounts, grid, visited);
}

void calculateBlobs(uint8_t freq) {
  uint8_t grid[19][5] = {0};   
  bool visited[19][5] = {false}; 

  // --- STEP 1: TRANSLATE INPUT TO THE GRID (Left-aligned single sensors) ---
  for (int y = 0; y < 19; y++) {
    if (groupedLDRs[freq][y].size == 5) {
      for (int x = 0; x < 5; x++) {
        grid[y][x] = 1 - groupedLDRs[freq][y].values[x];
      }
    } else if (groupedLDRs[freq][y].size == 1) {
      grid[y][0] = 1 - groupedLDRs[freq][y].values[0]; // Row of 1 is at index 0 (Radius 5)
    }
  }

  // --- STEP 2: FIND ALL BLOBS USING POLAR TRANSFORMS ---
  LDRBlob foundBlobs[30]; 
  int blobCount = 0;

  for (int y = 0; y < 19; y++) {
    for (int x = 0; x < 5; x++) {
      
      if (grid[y][x] == 1 && !visited[y][x]) {
        int sumX = 0; 
        int sumY = 0; 
        int count = 0;
        int radiusCounts[5] = {0, 0, 0, 0, 0}; // Create the empty buckets

        // Pass the new array into the maze runner
        floodFill(x, y, sumX, sumY, count, radiusCounts, grid, visited);

        // --- NEW NOISE-FILTERED RADIUS MATH ---
        int selectedRadiusX = -1; 
        
        // Loop from the outer edge (4) inward to the center (0)
        for (int r = 4; r >= 0; r--) {
          if (radiusCounts[r] > 0) {
            // We found the furthest out radius! Does it pass the 1/6th rule?
            if (radiusCounts[r] * 6 >= count) {
              selectedRadiusX = r;
              break; // It passes! Lock it in and stop searching.
            }
            // If it fails, the loop naturally continues to the next inner ring.
          }
        }

        // Apply the chosen radius (Fallback to average if the shape was completely chaotic)
        if (selectedRadiusX != -1) {
          // Store it in our standard 50-90 fixed-point format
          foundBlobs[blobCount].centerRadius = (selectedRadiusX + 5) * 10;
        } else {
          // Fallback to average (just in case)
          int actualRadiusSum = sumX + (count * 5);
          foundBlobs[blobCount].centerRadius = (actualRadiusSum * 10) / count;
        }

        // turn to percentage
        foundBlobs[blobCount].centerRadius = (foundBlobs[blobCount].centerRadius - 50) * 10 / 4;
        foundBlobs[blobCount].centerAngle = (sumY * 100) / 18 / count;
        foundBlobs[blobCount].size = constrain(count, 1, blobMax) * 100 / blobMax; // 0.0 (Small) to 1.0 (Large)
      
        //Serial.print(sumY);Serial.print("-");Serial.println(foundBlobs[blobCount].centerAngle);

        blobCount++;
      }
    }
  }

  // --- STEP 3: SORT BY SIZE (Largest to Smallest) ---
  for (int i = 0; i < blobCount - 1; i++) {
    for (int j = 0; j < blobCount - i - 1; j++) {
      if (foundBlobs[j].size < foundBlobs[j + 1].size) {
        LDRBlob temp = foundBlobs[j];
        foundBlobs[j] = foundBlobs[j + 1];
        foundBlobs[j + 1] = temp;
      }
    }
  }

  // --- STEP 4: SAVE TOP 3 ---
  for (int i = 0; i < 3; i++) {
    if (i < blobCount) {
      LDRBlobs[freq][i] = foundBlobs[i];
    } else {
      LDRBlobs[freq][i] = {0, 0, 0}; 
    }
  }
}

void printBlobs(uint8_t freq) {
  Serial.println("=== ACTIVE BLOBS ===");
  
  int activeCount = 0;
  for (int i = 0; i < 3; i++) {
    if (LDRBlobs[freq][i].size > 0) {
      activeCount++;
      Serial.print("Blob [");
      Serial.print(i);
      Serial.print("] -> ");
      
      Serial.print("Size: ");
      Serial.print(LDRBlobs[freq][i].size);
      
      // Divide by 10.0 to convert the fixed-point integer back into a readable decimal
      Serial.print(" | Radius: ");
      Serial.print(LDRBlobs[freq][i].centerRadius, 1); 
      
      Serial.print(" | Angle Step: ");
      Serial.println(LDRBlobs[freq][i].centerAngle, 1);
    }
  }
  
  if (activeCount == 0) {
    Serial.println("(No shadows detected)");
  }
  
  Serial.println("--------------------");
}

#endif