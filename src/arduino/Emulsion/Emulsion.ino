/*
 * A microgravity experiment studying the behavior of emulsified liquids.
 * This code autonomously mixes an emulsion when an accelerometer is triggered.
 * 
 === Wiring map ===
  Accelerometer
    VCC -> 5V
    GND -> GND
    SCL -> A5
    SDA -> A4
    XDA ->
    XCL ->
    ADO ->
    INT -> 2

  LED
    + -> 13
    - -> DIGITAL GND
  
  Relay
    VCC -> 5V
    GND -> GND
    SIG -> RELAY_PIN
  
  Servo
    VCC -> 5V
    GND -> GND
    SIG -> SERVO_PIN

  Limit Switch
    VCC -> 5V
    GND -> GND
    SIG -> LIMIT_SWITCH_PIN

  RaspberryPi
    RASPPI_PIN -> RaspberryPi GPIO 27
 ================

Accelerometer code from:
  === Contact & Support ===
  Website: http://eeenthusiast.com/
  Youtube: https://www.youtube.com/EEEnthusiast
  Facebook: https://www.facebook.com/EEEnthusiast/
  Patreon: https://www.patreon.com/EE_Enthusiast
  Revision: 1.0 (July 13th, 2016)
  === Hardware ===
  - Arduino Uno R3
  - MPU-6050 (Available from: http://eeenthusiast.com/product/6dof-mpu-6050-accelerometer-gyroscope-temperature/)
  === Software ===
  - Latest Software: https://github.com/VRomanov89/EEEnthusiast/tree/master/MPU-6050%20Implementation/MPU6050_Implementation
  - Arduino IDE v1.6.9
  - Arduino Wire library
  === Terms of use ===
  The software is provided by EEEnthusiast without warranty of any kind. In no event shall the authors or 
  copyright holders be liable for any claim, damages or other liability, whether in an action of contract, 
  tort or otherwise, arising from, out of or in connection with the software or the use or other dealings in 
  the software.
*/

#include <Wire.h>
#include <Servo.h>
Servo servo;

// === Constants ===

// Electrical pins
#define LED_PIN 13
#define RELAY_PIN 12
#define SERVO_PIN 7
#define LIMIT_SWITCH_PIN 3

// RaspberryPi
#define RASPI_PIN 8

// Timing
#define DELAY_TIME 1000 // ms, time in between taking force data, prevents starting from just a quick jolt
#define LED_MIX_BLINK_TIME 250 // ms, time it takes for LED to blink during mix
#define LED_COOLDOWN_BLINK_TIME 1000 // ms, time it takes for LED to blink during cooldown
#define LED_ACTION_BLINK_TIME 50 // ms, time it takes for LED to blink during an action
#define LED_ACTION_DURATION 300 // ms, time it takes to complete action blinking
#define COOLDOWN_TIME 45000 // ms, time to prevent mixing multiple times in a row
#define MIX_TIME 10000 // ms

// Servo
#define SERVO_POS 100 // degrees

// Activation
#define MINIMUM_FORCE 0.5 // g, The minimum acceptable force before experiment starts

// === Gyro ====
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

boolean inFall = false;

// === LED ===
boolean ledState = LOW;

void setup() {
  Serial.begin(9600);
  
  Wire.begin();
  setupMPU();

  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RASPI_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
}

void loop() {
  // Record data
  recordAccelRegisters();
  recordGyroRegisters();

  // Print force
  Serial.println("Force: " + String(gForceZ));

  // Set if falling
  inFall = gForceZ <= MINIMUM_FORCE;

  if(inFall) {
    // === Mixing sequence ===
  
    delayBlinkAction();
    digitalWrite(LED_PIN, HIGH);
    
    // Send signal to pi
    digitalWrite(RASPI_PIN, HIGH);
    
    // Move servo out of the way
    Serial.println("Moving servo out of way");
    servo.write(0);
    delay(1000);
    
    // Start motor
    Serial.println("Begin mixing for " + String(MIX_TIME / 1000.0) + "s.");
    digitalWrite(RELAY_PIN, HIGH);

    // Wait for MIX_TIME
    delay(MIX_TIME);

    while (true) {
      // Stop motor
      digitalWrite(RELAY_PIN, LOW);
      delay(500);
  
      // Move in servo
      Serial.println("Moving servo to stop motor");
      servo.write(SERVO_POS);
      delay(1000);
      
      // Spin motor until upright
      Serial.println("Rotating motor");
      digitalWrite(RELAY_PIN, HIGH);
      delay(500);
      
      if (switchIsPressed()) break;
      else {
        Serial.println("Bottle not upright! Spinning again");
  
        // Move servo out of the way
        Serial.println("Moving servo out of way");
        servo.write(0);
        delay(1000);
      }
    }

    // Stop motor
    digitalWrite(RELAY_PIN, LOW);

    // Begin cooldown and blink LED
    Serial.println("Begin cooldown for " + String(COOLDOWN_TIME / 1000.0) + "s.");
    delay(COOLDOWN_TIME);
    Serial.println("Cooldown finished");

    digitalWrite(LED_PIN, LOW);
  } else {
    // Do when not in fall
    digitalWrite(RASPI_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    servo.write(SERVO_POS);
    digitalWrite(RELAY_PIN, LOW);
  }

  delay(DELAY_TIME);
}

boolean switchIsPressed() {
  return digitalRead(LIMIT_SWITCH_PIN) == LOW; // Pressing switch 
}

void delayBlink(double delayTime, double blinkTime) {
  ledState = HIGH;
  double timeRemaining = delayTime;
  for (int i = 0; i < delayTime; i += blinkTime) {
    // Set LED
    digitalWrite(LED_PIN, ledState);
    
    // Wait
    (timeRemaining > blinkTime) ? delay(blinkTime) : delay(timeRemaining);
    
    timeRemaining -= blinkTime;
    
    // Toggle state
    (ledState == HIGH) ? ledState = LOW : ledState = HIGH;
   }
   
  // Turn off LED
  ledState = LOW;
  digitalWrite(LED_PIN, ledState);
}

void delayBlinkAction() {
  delayBlink(LED_ACTION_DURATION, LED_ACTION_BLINK_TIME);
}

void setupMPU() {
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.print(gForceZ);
  Serial.println("");
}
