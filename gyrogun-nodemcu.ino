/*
 * Copyright Notice
 * 
 * i2cdevlib MPU6050 examples - MIT Licensed by Jeff Rowberg
 */

#include "ESP8266WiFi.h"
#include "WiFiUdp.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// MPU6050 SCL -> GPIO 5 (D1)
// MPU6050 SDA -> GPIO 4 (D2)
// MPU6050 INT -> GPIO 14 (D5)
#define PIN_MPU_INT 14
// BUZZ -> GPIO 12 (D6)
#define PIN_BUZZ 12
// SWITCH -> GPIO 13 (D7)
#define PIN_SWITCH 13

/*
 * #define MECHANET_SSID "SSID"
 * #define MECHANET_PW "password"
 * #define HOST_IP (127, 0, 0, 1)
 * #define HOST_PORT (11076)
 * #define CLIENT_ID (0)
 */
#include "secrets.h"

MPU6050 mpu;
WiFiClient client;
WiFiUDP udp;
IPAddress host HOST_IP;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void ICACHE_RAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

int switch_before = 0;
int count = 0;
int buzz_counter = 0;

void init_mpu();
void connect_wifi();
void advertise_id();

void setup() {
    init_mpu();

    connect_wifi();

    pinMode(PIN_SWITCH, INPUT_PULLUP);
    pinMode(PIN_BUZZ, OUTPUT);
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;    
  
    bool switch_was_clicked = false;
  
    int switch_val = digitalRead(PIN_SWITCH);
    if (switch_val == 0) {
        if(switch_before == 1) {
            switch_was_clicked = true;
            buzz_counter = 5;
        }
            
        if(buzz_counter > 0) {
            digitalWrite(PIN_BUZZ, HIGH);
            buzz_counter--;
        } else {
            digitalWrite(PIN_BUZZ, LOW);
        }
    } else {
        digitalWrite(PIN_BUZZ, LOW);
    }
    switch_before = switch_val;
    
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
                // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
    }

    float yaw = ypr[0] * 180/M_PI, pitch = ypr[1] *180/M_PI, roll = ypr[2] * 180/M_PI;

    uint8_t packet[16] = {};

    uint32_t message_type = 0;
    if (switch_was_clicked) message_type = 1;

    packet[0] = message_type >> 24 % 256;
    packet[1] = message_type >> 16 % 256;
    packet[2] = message_type >> 8 % 256;
    packet[3] = message_type >> 0 % 256;

    uint32_t yaw_uint = 0;
    memcpy(&yaw_uint, &yaw, sizeof(uint32_t));

    packet[4] = yaw_uint >> 24 % 256;
    packet[5] = yaw_uint >> 16 % 256;
    packet[6] = yaw_uint >> 8 % 256;
    packet[7] = yaw_uint >> 0 % 256;

    uint32_t pitch_uint = 0;
    memcpy(&pitch_uint, &pitch, sizeof(uint32_t));

    packet[8] = pitch_uint >> 24 % 256;
    packet[9] = pitch_uint >> 16 % 256;
    packet[10] = pitch_uint >> 8 % 256;
    packet[11] = pitch_uint >> 0 % 256;

    uint32_t roll_uint = 0;
    memcpy(&roll_uint, &roll, sizeof(uint32_t));

    packet[12] = roll_uint >> 24 % 256;
    packet[13] = roll_uint >> 16 % 256;
    packet[14] = roll_uint >> 8 % 256;
    packet[15] = roll_uint >> 0 % 256;

    if(!client.connected()) {
        client.connect(host, HOST_PORT);
        client.setNoDelay(true);
        advertise_id();
    }

    if (message_type == 0) {
      udp.beginPacket(host, HOST_PORT);
      udp.write(packet+4, 12);
      udp.endPacket();
    } else {
      client.write(packet, 16);
    }

    delay(7);
}

void init_mpu() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    delay(1000);

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(PIN_MPU_INT, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(PIN_MPU_INT));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void connect_wifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(MECHANET_SSID, MECHANET_PW);

    while(WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    if(!client.connected()) {
        client.connect(host, HOST_PORT);
        client.setNoDelay(true);
    }

    advertise_id();
}

void advertise_id() {
    uint8_t packet[16] = {};

    uint32_t message_type = 3;

    packet[0] = message_type >> 24 % 256;
    packet[1] = message_type >> 16 % 256;
    packet[2] = message_type >> 8 % 256;
    packet[3] = message_type >> 0 % 256;

    uint32_t client_index = CLIENT_ID;

    packet[4] = client_index >> 24 % 256;
    packet[5] = client_index >> 16 % 256;
    packet[6] = client_index >> 8 % 256;
    packet[7] = client_index >> 0 % 256;

    client.write(packet, 16);
}
