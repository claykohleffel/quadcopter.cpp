// MPU6050 code
#include <Wire.h>
#include <MPU6050_light.h>

//create object mpu and initialize communication route
MPU6050 mpu(Wire); 

//amplitude of correction constants for drone stabilizatioWn; test and optimize
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

Serial.begin(115200); 
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

}
