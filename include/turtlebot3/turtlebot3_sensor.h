/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef TURTLEBOT3_SENSOR_H_
#define TURTLEBOT3_SENSOR_H_

#include <IMU.h>

#include "OLLO.h"

// add: 온습도 센서 헤더 추가
#include "DHT.h"

#define ACCEL_FACTOR                      0.000598550415   // (ADC_Value / Scale) * 9.80665            => Range : +- 2[g]
                                                           //                                             Scale : +- 16384
#define GYRO_FACTOR                       0.0010642        // (ADC_Value/Scale) * (pi/180)             => Range : +- 2000[deg/s]
                                                           //                                             Scale : +- 16.4[deg/s]

#define MAG_FACTOR                        15e-8

typedef struct LED_PIN_ARRAY
{
  int front_left;
  int front_right;
  int back_left;
  int back_right;
}LedPinArray;
 
typedef struct SONAR_PIN
{
  int trig;
  int echo;
}SonarPin;

enum DHTState {
  IDLE,
  REQUESTED,
  READY
};

class Turtlebot3Sensor
{
 public:
  Turtlebot3Sensor();
  ~Turtlebot3Sensor();

  bool init(void);

  // IMU
  void initIMU(void);
  float* getIMU(void);
  void updateIMU(void);
  void calibrationGyro(void);

  float* getImuAngularVelocity(void);
  float* getImuLinearAcc(void);
  float* getImuMagnetic(void);
  float* getOrientation(void);
  
  // Battery
  float checkVoltage(void);

  // Button
  uint8_t checkPushButton(void);

  // Sound
  void onMelody();
  void makeMelody(uint8_t index);  

  // Bumper
  void initBumper(void);
  bool getBumper1State();
  bool getBumper2State();
  uint8_t checkPushBumper(void);

  // Cliff sensor
  void initIR(void);
  float getIRsensorData(void);

  // Sonar sensor
  void initSonar(void);
  void updateSonar(uint32_t t);
  float getSonarData(void);

  // Illumination sensor
  float getIlluminationData(void);

  // led pattern
  void initLED(void);
  void setLedPattern(double linear_vel, double angular_vel);

  // add: 불꽃 센서
  void initFlame(void);
  bool getFlameDigitalData(void);
  float getFlameAnalogData(void);

  // add: 가스 센서
  void initGas(void);
  bool getGasDigitalData(void);
  float getGasAnalogData(void);

  // add: 온습도 센서
  void initDHT(void);
  void updateDHT_Async(); // 이 함수가 상태 머신의 핵심입니다.
  uint32_t getTemp() { return last_temp_; }
  uint32_t getHumi() { return last_humi_; }

 private:
  cIMU imu_;
  OLLO ollo_;

  LedPinArray led_pin_array_;
  SonarPin sonar_pin_;

  float sonar_data_;

  bool is_melody_play_complete_;
  uint16_t melody_note_[8];
  uint8_t melody_duration_[8];

  // add: 불꽃 센서
  int flame_pin_;         // D2 (Digital)
  int flame_analog_pin_;  // A1 (Analog)

  // add: 가스센서 핀
  int gas_digital_pin_;   // D8 (Digital)
  int gas_analog_pin_;    // A0 (Analog)

  // add: DHT 관련 변수 추가
  DHT *p_dht;
  int dht_pin_;
  
  DHTState dht_state_ = IDLE;
  uint32_t last_request_time_ = 0;
  uint32_t last_temp_ = 0;
  uint32_t last_humi_ = 0;
};

#endif // TURTLEBOT3_SENSOR_H_
