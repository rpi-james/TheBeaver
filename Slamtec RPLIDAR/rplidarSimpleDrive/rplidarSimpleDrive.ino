#define RPLIDAR_TX 17  // TX2
#define RPLIDAR_RX 16  // RX2

HardwareSerial LIDAR(2);  // Use UART2 for LIDAR communication

// ✅ Buffer size changed to 84 bytes directly
#define PACKET_SIZE 84   // Now the buffer accumulates all 84 bytes
#define DATA_PACKET_LEN 84  // Length of valid data packet (40 points + header)

// Buffer and parsing variables
uint8_t rawdata[DATA_PACKET_LEN];  // Buffer to hold parsed data
int dataIndex = 0;                 // To track parsed bytes
float lastAngle = 0.0;             // To calculate angle differences
float angleList[40];               // Array to store angles
float distList[40];                // Array to store distances
bool newTurn = false;              // Flag for new 360-degree rotation

void setup() {
    Serial.begin(115200);  // For debugging output
    LIDAR.begin(460800, SERIAL_8N1, RPLIDAR_RX, RPLIDAR_TX);
    delay(1000);

    // Clear any previous data in the LIDAR buffer
    while (LIDAR.available()) {
        LIDAR.read();
    }

    Serial.println("Sending reset command...");
    uint8_t resetCmd[] = {0xA5, 0x40};  // Reset command
    LIDAR.write(resetCmd, sizeof(resetCmd));
    delay(1000);  // Allow LIDAR to reboot

    Serial.println("Sending start scan command...");
    uint8_t startScan[] = {0xA5, 0x20};  // Start scan command
    LIDAR.write(startScan, sizeof(startScan));
}

void loop() {
    while (LIDAR.available()) {
        uint8_t byte = LIDAR.read();  // Read incoming byte

        // ✅ Add byte to rawdata and increment dataIndex
        rawdata[dataIndex++] = byte;

        // ✅ If full 84-byte packet is received, process data
        if (dataIndex >= DATA_PACKET_LEN) {
            parseData();  // Call parsing function
            dataIndex = 0;  // Reset for next packet
        }
    }
}

// ✅ **Parse Raw Data and Extract Angles + Distances**
void parseData() {
    Serial.println("Parsing...");

    // Extract starting angle and determine turn
    float startAngle = (((rawdata[3] & 0x7F) << 8) | rawdata[2]) / 64.0;
    if (lastAngle > startAngle) {
        newTurn = true;  // Completed one full 360-degree turn
    }
    lastAngle = startAngle;

    // ✅ Parse 40 points (each point is 2 bytes distance)
    for (int i = 0; i < 40; i++) {
        // Distance calculation (high + low byte)
        uint16_t distance = (rawdata[2 * i + 5] << 8) | rawdata[2 * i + 4];
        distList[i] = distance / 4.0;  // Distance in mm (C1 scale factor)

        // Angle increment for each point
        float dAngle = (startAngle + (9.0 * i)) / 40.0;
        angleList[i] = fmod(dAngle, 360.0);  // Wrap angle to 0-360°
    }

    // ✅ Step 4: Output first point as a sanity check
    Serial.print("Angle: ");
    Serial.print(angleList[0]);
    Serial.print("°, Distance: ");
    Serial.print(distList[0]);
    Serial.println(" mm");

    // ✅ Step 5: Handle new turn if needed
    if (newTurn) {
        Serial.println("Completed 360-degree sweep!");
        newTurn = false;
    }
}

// ✅ **Restart Scan Safely**
void restartScan() {
    uint8_t endCmd[] = {0xA5, 0x25};  // Stop scan command
    LIDAR.write(endCmd, sizeof(endCmd));
    delay(1000);  // Allow reset time

    // Restart scan
    uint8_t startScan[] = {0xA5, 0x20};
    LIDAR.write(startScan, sizeof(startScan));
    Serial.println("Restarting scan...");
}
