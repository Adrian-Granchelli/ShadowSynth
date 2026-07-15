#ifndef globals_h
#define globals_h

unsigned long now; 
unsigned long LAST_LOOP; 

// --- Pin Definitions ---
const uint8_t button_pin = 2;

// --- UART ---
const uint8_t serialTimeoutStartup = 100;
const uint8_t serialTimeout = 40;

const uint16_t BUFFER_SIZE = 257; // Big enough to hold "1,0,1,..." + distance + \n
char uartBuffer[BUFFER_SIZE];

// --- DATA STRUCTURES ---
const uint8_t TOTAL_LDRS = 59;
const uint8_t NUM_GROUPS = 19;

// A custom structure to hold a variable-sized group of LDRs
struct LDRGroup {
  uint8_t values[5]; // Max size is 5
  uint8_t size;      // Will be either 5 or 1 so you know how many to loop through
};

// A custom structure to hold LDR blobs 
struct LDRBlob {
  uint8_t centerRadius; // percentage of total (0 to 100) - representing distance from centre
  uint8_t centerAngle;  // percentage of total: (0 to 100) - representing the angle 
  uint8_t size; // percentage of total: (0 to 100) - representing the number of LDRs covered 
};

uint8_t blobMax = 18; // largest blob size to calculate percentage
// Global variables for your parsed data
LDRGroup groupedLDRs[3][NUM_GROUPS]; // BASS is [0], MID is [1], TREB is [2]
LDRBlob LDRBlobs[3][3] = {}; // First Number: BASS is [0], MID is [1], TREB is [2], Second number is tracking each blob
uint8_t ultrasonicDistanceHistoryCount = 0;
uint8_t ultrasonicDistanceHistoryMax = 5;
uint16_t ultrasonicDistance[3][5] = {
                                 {0, 0, 0, 0, 0},
                                 {0, 0, 0, 0, 0},
                                 {0, 0, 0, 0, 0}
                                 };

const uint16_t ultrasonicDistanceMax = 300;
uint8_t ultrasonicDistancePercent[3];
float distancePowerFactor = 0.6; // lower = more exaggerated near-field sensitivity

// --- DELAYS --- 
const uint16_t LOOP_WAIT = 100; 


// SOUND FILES 
const uint8_t MAX_SOUND_FILES = 16;
const uint8_t MAX_FOLDERS = 20; 
String folderNames[MAX_FOLDERS]; 
uint8_t folderCount = 0; 

unsigned long lastSoundChange; 
uint8_t folder_n = 0;
uint8_t sound_switch_delay = 240; // in minutes (max 255)

  
#endif