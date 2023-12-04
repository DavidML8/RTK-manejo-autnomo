#include <Adafruit_MCP2515.h>

#ifdef ESP8266
   #define CS_PIN    2
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3)
   #define CS_PIN    14
#elif defined(TEENSYDUINO)
   #define CS_PIN    8
#elif defined(ARDUINO_STM32_FEATHER)
   #define CS_PIN    PC5
#elif defined(ARDUINO_NRF52832_FEATHER)  /* BSP 0.6.5 and higher! */
   #define CS_PIN    27
#elif defined(ARDUINO_MAX32620FTHR) || defined(ARDUINO_MAX32630FTHR)
   #define CS_PIN    P3_2
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040)
   #define CS_PIN    7
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_CAN)
   #define CS_PIN    PIN_CAN_CS
#elif defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_RASPBERRY_PI_PICO_W) // PiCowbell CAN Bus
   #define CS_PIN    20
#else
    // Anything else, defaults!
   #define CS_PIN    9
#endif

// Set CAN bus baud rate
#define CAN_BAUDRATE (125000)

Adafruit_MCP2515 mcp(CS_PIN);

const int bufferSize = 8;
char buffer[bufferSize];
int receivedVars[2]; // To store the received variables

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);

  Serial.println("MCP2515 Sender test!");

  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  Serial.println("MCP2515 chip found");
}

void loop() {
  if (Serial.available() > 0) {
    // Read data from serial until newline character is received
    Serial.readBytesUntil('\n', buffer, bufferSize);

    // Tokenize received data using delimiter (',') and convert to integers
    char *ptr = strtok(buffer, ",");
    int i = 0;
    while (ptr != NULL && i < 2) {
      receivedVars[i] = atoi(ptr);
      ptr = strtok(NULL, ",");
      i++;
    }

    // Store received variables in individual variables
    int var1 = receivedVars[0];
    int var2 = receivedVars[1];

    mcp.beginPacket(83);
    mcp.write(var1);
    mcp.endPacket();
    delay(100);

    mcp.beginPacket(84);
    mcp.write(var2);
    mcp.write(1);
    mcp.write(1);
    mcp.write(0);
    mcp.write(1);
    mcp.endPacket();
    delay(100);
  }
}
