/*
Accelerometer code from

===Contact & Support===
Website: http://eeenthusiast.com/
Youtube: https://www.youtube.com/EEEnthusiast
Facebook: https://www.facebook.com/EEEnthusiast/
Patreon: https://www.patreon.com/EE_Enthusiast
Revision: 1.0 (July 13th, 2016)
===Hardware===
- Arduino Uno R3
- MPU-6050 (Available from: http://eeenthusiast.com/product/6dof-mpu-6050-accelerometer-gyroscope-temperature/)
===Software===
- Latest Software: https://github.com/VRomanov89/EEEnthusiast/tree/master/MPU-6050%20Implementation/MPU6050_Implementation
- Arduino IDE v1.6.9
- Arduino Wire library
===Terms of use===
The software is provided by EEEnthusiast without warranty of any kind. In no event shall the authors or 
copyright holders be liable for any claim, damages or other liability, whether in an action of contract, 
tort or otherwise, arising from, out of or in connection with the software or the use or other dealings in 
the software.
*/

#include <Wire.h>
#include <Servo.h>
Servo servo;

// Electrical pins
int RELAY_PIN = 13;
int SERVO_PIN = 7;

// Gyro
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

boolean inFall = false;
int DELAY_TIME = 1000; // time in between taking force data, prevents starting from just a quick jolt
int COOLDOWN_TIME = 10000; // time to prevent mixing multiple times in a row
int MIX_TIME = 10000; // ms
int SERVO_POS = 100;

void setup() {
  Serial.begin(9600);
  
  Wire.begin();
  setupMPU();

  pinMode(RELAY_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
}

void loop() {
  // Record data
  recordAccelRegisters();
  recordGyroRegisters();

  // Set if falling
  Serial.println("Force: " + String(gForceZ));
  
  if(gForceZ <= 0.5)
  {
    inFall = true;
  }
  else
  {
    inFall = false;
  }

  if(inFall)
  {
    // Move servo out of the way
    Serial.println("Moving servo out of way");
    servo.write(0);
    delay(1000);
    
    // Start motor
    Serial.println("Begin mixing for " + String(MIX_TIME / 1000.0) + "s.");
    digitalWrite(RELAY_PIN, HIGH);
    delay(MIX_TIME);

    // Stop motor
    digitalWrite(RELAY_PIN, LOW);
    delay(500);

    // Move in servo
    Serial.println("Moving servo to stop motor");
    servo.write(SERVO_POS);
    delay(1000);
    
    // Spin motor until upright
    digitalWrite(RELAY_PIN, HIGH);
    delay(500);

    // Stop motor
    digitalWrite(RELAY_PIN, LOW);

    // Begin cooldown
    Serial.println("Begin cooldown for " + String(COOLDOWN_TIME / 1000.0) + "s.");
    delay(COOLDOWN_TIME);
  }
  else
  {
    servo.write(SERVO_POS);
    digitalWrite(RELAY_PIN, LOW);
  }

  delay(DELAY_TIME);
}

void setupMPU(){
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
