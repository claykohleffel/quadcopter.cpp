//DRONE CODE 

#include <RF24.h> 
#include <Servo.h> 
#include <Wire.h> 
#include <MPU6050_light.h>


const uint64_t address = 0x4F8D2A1C9ELL;
bool armed = false; 
bool linkAlive = false;
unsigned long lastCheck = 0;
// 11,12,13 for SPI and 1 for Serial
// ESC pins are motor pins
RF24 radio(7,8);

Servo mtr1, mtr2, mtr3, mtr4; 

// use same struct as the controller so that data will be properly stored
struct ctrlpkt {
  uint8_t seq; 
  uint16_t thr, pit, yaw, roll; 
  uint8_t arm;
};

ctrlpkt latestpkt = {0, 1000, 500, 500, 500, 0}; 

//INSERTING PID WORK HERE
//create object mpu and initialize communication route
MPU6050 mpu(Wire); 

//amplitude of correction constants for drone stabilization; test and optimize
//PID = proportional, integral, derivative
float Kp = 1.5; 
float Ki = .03; 
float Kd = .6; 

//create a time constant to use in integral and derivative equations
unsigned long lastTime = 0;
unsigned long currentTime; 

float pitchError, pitchPrevError = 0, pitchInt = 0, pitchDer = 0; 
float rollError, rollPrevError = 0, rollInt = 0, rollDer = 0; 
//target angles 
float targetPitch = 0; 
float targetRoll = 0; 


void setup() {

mtr1.attach(3); 
mtr2.attach(5); 
mtr3.attach(6); 
mtr4.attach(9); 

for (int i = 0; i < 100; i++) {
  mtr1.writeMicroseconds(1000);
  mtr2.writeMicroseconds(1000);
  mtr3.writeMicroseconds(1000);
  mtr4.writeMicroseconds(1000);
  delay(20); 
}

Serial.begin(115200); 
//start radio
radio.begin(); 
radio.setPayloadSize(sizeof(ctrlpkt)); 
radio.setAddressWidth(5); 

//set radio data
radio.setChannel(108); 
radio.setDataRate(RF24_250KBPS); 
radio.setPALevel(RF24_PA_MIN); 
radio.setCRCLength(RF24_CRC_16); 
radio.setAutoAck(true); 
radio.setRetries(5, 15); 
// openReadingPipe takes two arguments, pipe number and address
radio.openReadingPipe(0, address); 
radio.startListening(); 

//MPU
Wire.begin();

//use byte when you're using flags, on/off, 0/1, etc.
byte mstatus = mpu.begin(); 
Serial.print("MPU status: "); 
if (mstatus != 0) {
  Serial.print("Error initializing MPU. Troubleshoot");
  while(1);
}
Serial.println(mstatus);

delay(1000);
mpu.calcOffsets();
Serial.print("MPU ready");


}


void loop() {
ctrlpkt p; 
// Check for availiable data
if (radio.available()) {  
// store info in new array, declare size
  radio.read(&p, sizeof(ctrlpkt));
  Serial.println("Received");
  latestpkt = p;
  // check if link is alive; difference between pkts < 200 ms
  lastCheck = millis();
  linkAlive = true; 
  armed = (p.arm == 1);
}
 

// Scale motors controls to throttle so that 2000 = 1 and 1000 = 0 for smooth descents
float scale = float(latestpkt.thr - 1000) / 1000; 
// subtract 500 from values so that joystick values will provide neg values for controls
float newYaw = (latestpkt.yaw - 500) * scale; 
float newPit = (latestpkt.pit - 500) * scale; 
float newRoll = (latestpkt.roll - 500) * scale; 



//MPU/////////////////////
mpu.update(); 

//time change in seconds
currentTime = millis(); 
float dt = (currentTime - lastTime) / 1000.0;
lastTime = currentTime; 


Serial.print("Pitch Angle"); Serial.println(mpu.getAngleX());
Serial.print("Roll Angle"); Serial.println(mpu.getAngleY());


float currentPitch = mpu.getAngleX(); 
float currentRoll = mpu.getAngleY(); 

//create setpoint angles so that the imu can make corrections
float pitSetPoint = map(p.pit, 0, 1000, -15, 15); //assumes that drone will tilt +-15 degrees
float rollSetPoint = map(p.roll, 0, 1000, -15, 15); 

pitchError = pitSetPoint - currentPitch;
rollError = rollSetPoint - currentRoll;
 
//PID CALCULATIONS
//pitch PID

pitchInt += pitchError * dt; 
pitchDer = (pitchError - pitchPrevError) / dt; 


//roll PID

rollInt += rollError * dt; 
rollDer = (rollError - rollPrevError) / dt; 

//calculate

float finalPit = Kp * (pitchError) + (pitchInt * Ki) + (pitchDer * Kd);
float finalRoll = Kp * (rollError) + (rollInt * Ki) + (rollDer * Kd);


pitchPrevError = pitchError;
rollPrevError = rollError;

delay(50); 

/////////////////


int m1 = latestpkt.thr - finalPit + newYaw - finalRoll; 
int m2 = latestpkt.thr - finalPit - newYaw + finalRoll; 
int m3 = latestpkt.thr + finalPit - newYaw - finalRoll; 
int m4 = latestpkt.thr + finalPit + newYaw + finalRoll; 

m1 = constrain(m1, 1000, 2000); 
m2 = constrain(m2, 1000, 2000); 
m3 = constrain(m3, 1000, 2000); 
m4 = constrain(m4, 1000, 2000); 

// write power to the motors
if (armed) {
mtr1.writeMicroseconds(m1); 
mtr2.writeMicroseconds(m2); 
mtr3.writeMicroseconds(m3); 
mtr4.writeMicroseconds(m4);
}
else {
mtr1.writeMicroseconds(1000); 
mtr2.writeMicroseconds(1000); 
mtr3.writeMicroseconds(1000); 
mtr4.writeMicroseconds(1000);
}


//create boolean
linkAlive = (millis() - lastCheck) <= 200;
if (!(linkAlive)) {
  armed = false;  
  latestpkt.thr = 1000;
}

}
