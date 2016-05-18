#ifndef MPU6050_h
#define MPU6050_h
#include <Energia.h>
#include<Wire.h>
#define STOP_SEND true
#define CONT_SEND false
#define RAD_TO_DEG 57.29577951
#include "Kalman.h"
#define RESTRICT_PITCH
class Mpu6050{
	// Create the Kalman instances
	Kalman kalmanX; 
	Kalman kalmanY;
	/* IMU Data */
	int16_t accX, accY, accZ;
	int16_t gyroX, gyroY, gyroZ;

	double gyroXangle, gyroYangle; // Angle calculate using the gyro only
	double compAngleX, compAngleY; // Calculated angle using a complementary filter
	double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
	uint32_t timer;
	double roll, pitch;
	const uint8_t address = 0x68;
	const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication
	uint8_t i2cData[14]; // Buffer for I2C data
	uint8_t setBytes(uint8_t registerAddress, uint8_t * data, uint8_t length, boolean command ){
    Wire.beginTransmission(this->address);
    Wire.write(registerAddress);
    Wire.write(data, length);
    uint8_t rcode = Wire.endTransmission(command); // Returns 0 on success
    if (rcode) {
      Serial.print(F("i2cWrite failed: "));
      Serial.println(rcode);
    }
    else {
      Serial.print("Send Byte OK:");
      Serial.println(rcode);
    }
    return rcode;
  }
	uint8_t getBytes(uint8_t registerAddress, uint8_t *buffer, uint8_t nbytes){
    uint32_t timeOutTimer;
    Wire.beginTransmission(this->address);
    Wire.write(registerAddress);
    uint8_t rcode = Wire.endTransmission(CONT_SEND); // Don't release the bus
    if (rcode) {
      Serial.print(F("i2cRead failed: "));
      Serial.println(rcode);
      return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
    }
    Wire.requestFrom((int)this->address, (int)nbytes, (int)STOP_SEND); // Send a repeated start and then release the bus after reading
    for (uint8_t i = 0; i < nbytes; i++) {
      if (Wire.available()){
        buffer[i] = Wire.read();
      }
      else {
        timeOutTimer = micros();
        while (((micros() - timeOutTimer) < this->I2C_TIMEOUT) && !Wire.available());
        if (Wire.available()){
          buffer[i] = Wire.read();
        }else{
          Serial.println(F("i2cRead timeout"));
          return 5; // This error value is not already taken by endTransmission
        }
      } 
    }
    return 0;
  }
public:
	Mpu6050(){
    
  }    
	void init(){
    Wire.begin();
    accX = accY = accZ = gyroX = gyroY = gyroZ = 0;
    memset(this->i2cData,0,14);
    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    Serial.print("Send1\n");
    while(this->setBytes(0x19,this->i2cData,4,STOP_SEND));
    uint8_t x = 0x01;
        Serial.print("Send2\n");
    while(this->setBytes(0x6B,&x,1,STOP_SEND));
     Serial.print("finish Send2\n");
    this->getBytes(0x75,this->i2cData,1);
    if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
      Serial.print(F("Error reading sensor"));
      while (1);
    }
    else{
      Serial.print("Sensor OK\n");
    }
     delay(100); // Wait for sensor to stabilize
	this->getRawAccValue();
	this->getRawGyroValue();
	this->getRollAngle();
	this->getPitchAngle();
	this->kalmanX.setAngle(this->roll);
	this->kalmanY.setAngle(this->pitch);
	this->gyroXangle = this->roll;
	this->gyroYangle = this->pitch;
	this->compAngleX = this->roll;
	this->compAngleY = this->pitch;
	this->timer = micros();
  }
	void getRawAccValue(){
		this->getBytes(0x3B, this->i2cData,6);
		this->accX = (((int16_t)this->i2cData[0]) << 8) | this->i2cData[1];
		this->accY = (((int16_t)this->i2cData[2]) << 8) | this->i2cData[3];
		this->accZ = (((int16_t)this->i2cData[4]) << 8) | this->i2cData[5];
  
  }
	void getRawGyroValue(){
		this->getBytes(0x43, this->i2cData,6);
		this->gyroX = (((int16_t)this->i2cData[0]) << 8) | this->i2cData[1];
		this->gyroY = (((int16_t)this->i2cData[2]) << 8) | this->i2cData[3];
		this->gyroZ = (((int16_t)this->i2cData[4]) << 8) | this->i2cData[5];
  
  }
	void compute(){
		this->getRawAccValue();
		this->getRawGyroValue();
		
		#ifdef RESTRICT_PITCH // Eq. 25 and 26
		  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
		  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
		#else // Eq. 28 and 29
		  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
		  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
		#endif

		  double gyroXrate = gyroX / 131.0; // Convert to deg/s
		  double gyroYrate = gyroY / 131.0; // Convert to deg/s
		double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
		#ifdef RESTRICT_PITCH
		  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
			kalmanX.setAngle(roll);
			compAngleX = roll;
			kalAngleX = roll;
			gyroXangle = roll;
		  } else
			kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // 

		  if (abs(kalAngleX) > 90)
			gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
		  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
		#else
		  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
			kalmanY.setAngle(pitch);
			compAngleY = pitch;
			kalAngleY = pitch;
			gyroYangle = pitch;
		  } else
			kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

		  if (abs(kalAngleY) > 90)
			gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
		  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
		#endif

		  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
		  gyroYangle += gyroYrate * dt;
		  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
		  //gyroYangle += kalmanY.getRate() * dt;

		  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
		  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

		  // Reset the gyro angle when it has drifted too much
		  if (gyroXangle < -180 || gyroXangle > 180)
			gyroXangle = kalAngleX;
		  if (gyroYangle < -180 || gyroYangle > 180)
			gyroYangle = kalAngleY;
//		Serial.print(roll); Serial.print("\t");
//		Serial.print(gyroXangle); Serial.print("\t");
//		Serial.print(compAngleX); Serial.print("\t");
		Serial.print(kalAngleX); Serial.print("\t");

//		Serial.print("\t");

//		Serial.print(pitch); Serial.print("\t");
//		Serial.print(gyroYangle); Serial.print("\t");
//		Serial.print(compAngleY); Serial.print("\t");
//		Serial.print(kalAngleY); Serial.print("\t");	
		Serial.print("\r\n");
//                Serial.println(kalAngleX);
	}
	double getRollAngle(){
		return this->kalAngleX;
		
	}
	double getPitchAngle(){
		return this->kalAngleY;
		
	}
	void updateTimer(){
		this->timer = micros();
		
	}
	};
#endif
