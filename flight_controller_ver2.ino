/*
============================================================
🛩 Teensy 4.1 Flight Controller Pin Mapping
============================================================

[00] ESC1     [05] SRV1
[01] ESC2     [06] SRV2     [15] PPM Receiver
[02] ESC3     [07] SRV3     [16] RX4 ⇨ GPS TX
[03] ESC4     [08] SRV4     [17] TX4 ⇦ GPS RX
[04] ESC5     [09] SRV5     [18] SDA ⇨ BMP280
              [10] SRV6     [19] SCL ⇨ BMP280
                            [24] CS  ⇨ MPU9250 NCS

========================================================================================================================
Table of Contents [Ctrl + F]

1. Definition
  a. Libraries
  b. Variables
2. Main Setup
  a. Sensors Int
  b. PWM Int
  c. 
3. Loop
  a. Sensors Loop
  b. Receiver Loop
  c. GPS Loop


========================================================================================================================*/





/*
============================================================
🛩 Libraries
============================================================*/
#include "mpu9250.h"
#include <BMP280.h>
#include <PulsePosition.h>
#include <TinyGPS++.h>
#include <PWMServo.h>
#include <BasicLinearAlgebra.h>
#include <PID_v1.h>

/* Mpu9250 object, SPI bus, CS on pin 24 */
bfs::Mpu9250 imu(&SPI, 24);
BMP280 bmp280;
PulsePositionInput ReceiverInput(RISING);
TinyGPSPlus gps;
HardwareSerial &gpsSerial = Serial4;
using namespace BLA;

/*
============================================================
🛩 Variables
============================================================*/
unsigned long channel_1_fs = 1500; //elev
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1000; //thro
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 2000; //throttle cut
unsigned long channel_6_fs = 1000; //aux1

float accx, accy, accz;
float gyrox, gyroy, gyroz;
float magx, magy, magz;
float phi_a, theta_a, psi_a;
float phi_g, theta_g, psi_g;
float psi_m;


float phi, theta, psi;

float sin_roll = sin(phi_a);
float cos_roll = cos(phi_a);
float sin_pitch = sin(theta_a);
float cos_pitch = cos(theta_a);

float mag_x = magx * cos_pitch + magz * sin_pitch;
float mag_y = magx * sin_roll * sin_pitch + magy * cos_roll - magz * sin_roll * cos_pitch;


float temperature;
float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
float ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8;
float channelAvailable;
float lat,lon,alt,Gvel,sat;

int ChannelNumber=0;

//Kalman Filter Parameters
float gyrox_cal, gyroy_cal, gyroz_cal;
int RateCalibrationNumber;
float phi_k=0, theta_k=0, psi_k=0;
float phi_k_U=2*0.01745*2*0.01745,theta_k_U=2*0.01745*2*0.01745,psi_k_U=2*0.01745*2*0.01745; //Uncertainty
float Kalman1DOutput[]={0,0};

//Controller parameters (take note of defaults before modifying!): 
float maxRoll = 5.0*(PI/180);     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxPitch = 5.0*(PI/180);    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0*(PI/180);     //Max yaw rate in deg/sec

//OneShot125 ESC pin outputs:
const int m1Pin = 0;
const int m2Pin = 1;
const int m3Pin = 2;
const int m4Pin = 3;
const int m5Pin = 4;
//PWM servo or ESC outputs:
const int servo1Pin = 5;
const int servo2Pin = 6;
const int servo3Pin = 7;
const int servo4Pin = 8;
const int servo5Pin = 9;
const int servo6Pin = 10;
PWMServo servo1;
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;
PWMServo servo5;
PWMServo servo6;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float p_des, q_des, r_des;

//PID
double Kp_phi=0.3, Kp_theta=0.3, Kp_psi=2;
double Ki_phi=0.05, Ki_theta=0.05, Ki_psi=0.1;
double Kd_phi=0.001, Kd_theta=0.001, Kd_psi=0;

//testing Lab-------------------------------------------------------
double phi_Setpoint, phi_Input, phi_PID;
PID myphiPID(&phi_Input, &phi_PID, &phi_Setpoint, Kp_phi, Ki_phi, Kd_phi, DIRECT);
double theta_Setpoint, theta_Input, theta_PID;
PID mythetaPID(&theta_Input, &theta_PID, &theta_Setpoint, Kp_theta, Ki_theta, Kd_theta, DIRECT);
//------------------------------------------------------------------

//Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

//Flight status
bool armedFly = false;

uint32_t pressure;
uint32_t LoopTimer;

//Definition
#define PI 3.1415926535897932384626433832795

/* Biases */
float gyro_bias[3] = {0};
float acc_bias[3] = {0};
float mag_bias[3] = {0};

void read_receiver(void){
  ChannelNumber = ReceiverInput.available();
  if(ChannelNumber > 0) {
    for(int i=1; i<=ChannelNumber;i++){
      ReceiverValue[i-1]=ReceiverInput.read(i);
    }
  }
}

/*
============================================================
🛩 Main Setup
============================================================
*/
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(115200);
  IMUint();
  calibrateIMU(5000);  // 5sec calibration
  ReceiverInput.begin(15);  //PPM pin
  Wire.begin();
  bmp280.begin();
  Serial.println("Wating for ESC...");
  PWMint();


//testing Setup-----------------------
  phi_Input = phi_k;
  theta_Input = theta_k;

  myphiPID.SetOutputLimits(-255, 255);
  mythetaPID.SetOutputLimits(-255, 255);
  myphiPID.SetMode(AUTOMATIC);
  mythetaPID.SetMode(AUTOMATIC);

//------------------------------------
}

/*
============================================================
🛩 Main Loop
============================================================
*/
void loop() {
  IMUdata();
  BMPdata();
  armedStatus();
  getDesState();
  PPMinput();
  AngleEstimate();
  Phi_Theta_Est();
  controlMixer();
  scaleCommands();
  throttleCut();
  commandMotors();
  PWMloop();
  failSafe();
  GPS();
  MagCorrection();

  //Loop test section------------------
  phi_Input = phi_k;
  theta_Input = theta_k;

  phi_Setpoint = roll_des;
  theta_Setpoint = pitch_des;
  if ((ch5 > 1500) && (ch3 > 1050)) {
    myphiPID.Compute();
    mythetaPID.Compute();
  }
  Serial.println(phi_PID*(180/PI));
  Serial.println(theta_PID*(180/PI));
  //-----------------------------------

//--------Serial Monitor--------
  PrintSensorData();
  printStates();
  //printPPM();
  //printGPS();
  //printDes();
}

/*
============================================================
🛩 Sensors Int
============================================================
*/

void IMUint() {
  SPI.begin();
  if (!imu.Begin()) {
    Serial.println("Error initializing IMU");
    while (1) {}
  }
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error setting SRD");
    while (1) {}
  }
}

void calibrateIMU(unsigned long duration_ms) {
  Serial.println("Calibrating IMU... keep still");

  unsigned long start_time = millis();
  int count = 0;
  float sum_gyro[3] = {0};
  float sum_acc[3] = {0};

  while (millis() - start_time < duration_ms) {
    if (imu.Read()) {
      sum_gyro[0] += imu.gyro_y_radps();
      sum_gyro[1] -= imu.gyro_x_radps();
      sum_gyro[2] += imu.gyro_z_radps();

      sum_acc[0] += imu.accel_y_mps2();
      sum_acc[1] -= imu.accel_x_mps2();
      sum_acc[2] += imu.accel_z_mps2();

      count++;
    }
  }

  if (count > 0) {
    for (int i = 0; i < 3; i++) {
      gyro_bias[i] = sum_gyro[i] / count;
      acc_bias[i] = 0;
      mag_bias[i] = 0;
    }
  }

  Serial.println("Calibration Complete.");
  delay(500);
}

/*
============================================================
🛩 PWM Int
============================================================*/
void PWMint() {
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);
  pinMode(m5Pin, OUTPUT);
  servo1.attach(servo1Pin, 900, 2100); //Pin, min PWM value, max PWM value
  servo2.attach(servo2Pin, 900, 2100);
  servo3.attach(servo3Pin, 900, 2100);
  servo4.attach(servo4Pin, 900, 2100);
  servo5.attach(servo5Pin, 900, 2100);
  servo6.attach(servo6Pin, 900, 2100);

  //Arm servo channels
  servo1.write(0); //Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  servo2.write(0); //Set these to 90 for servos if you do not want them to briefly max out on startup
  servo3.write(0); //Keep these at 0 if you are using servo outputs for motors
  servo4.write(0);
  servo5.write(0);
  servo6.write(0);

  //Arm OneShot125 motors
  m1_command_PWM = 125; //Command OneShot125 ESC from 125 to 250us pulse length
  m2_command_PWM = 125;
  m3_command_PWM = 125;
  m4_command_PWM = 125;
  m5_command_PWM = 125;
  armMotors(); //Loop over commandMotors() until ESCs happily arm
  Serial.println("Motor Armed");
  delay(500);
}

void armMotors() {
  //DESCRIPTION: Sends many command pulses to the motors, to be used to arm motors in the void setup()
  /*  
   *  Loops over the commandMotors() function 50 times with a delay in between, simulating how the commandMotors()
   *  function is used in the main loop. Ensures motors arm within the void setup() where there are some delays
   *  for other processes that sometimes prevent motors from arming.
   */
  for (int i = 0; i <= 50; i++) {
    commandMotors();
    delay(2);
  }
}

void MagCorrection() {
  BLA::Matrix<3, 3> magmat1 = {cos(theta_k),0,sin(theta_k),
                                0,          1,        0,
                              -sin(theta_k),0,cos(theta_k)};
  BLA::Matrix<3, 3> magmat2 = {1,         0,          0,
                               0,cos(phi_k),-sin(phi_k),
                               0,sin(phi_k),cos(phi_k)};  
  BLA::Matrix<3> magdata = {magx,magy,magz};
  BLA::Matrix<3> magxyz = magmat1*magmat2*magdata;

  psi_m = atan2(-magy, magx);

  //Serial.println(magmat1);
  //Serial.println(magmat2);
  //Serial.println(magdata);
  //Serial.println(magxyz);
}

/*
============================================================
🛩 Sensors Loop
============================================================
*/
void IMUdata() {
  if (imu.Read()) {
    accx = imu.accel_y_mps2() - acc_bias[1];
    accy = -imu.accel_x_mps2() + acc_bias[0];
    accz = imu.accel_z_mps2() - acc_bias[2];

    gyrox = imu.gyro_y_radps() - gyro_bias[1];
    gyroy = -imu.gyro_x_radps() - gyro_bias[0];
    gyroz = imu.gyro_z_radps() - gyro_bias[2];

    magx = imu.mag_y_ut() - mag_bias[0];
    magy = -imu.mag_x_ut() - mag_bias[1];
    magz = imu.mag_z_ut() - mag_bias[2];


  }
}
void BMPdata() {
  pressure = bmp280.getPressure();
  temperature = bmp280.getTemperature();
}

/*
============================================================
🛩 Receiver Loop
============================================================*/
void PPMinput(){
  static unsigned long lastPPMtime = 0;
  const unsigned long ppmInterval = 20;  // ms
  if (millis() - lastPPMtime >= ppmInterval) {
    read_receiver();
    lastPPMtime = millis();
  }
  channelAvailable = ChannelNumber;
  ch1 = ReceiverValue[0]; //roll
  ch2 = ReceiverValue[1]; //pitch
  ch3 = ReceiverValue[2]; //throttle
  ch4 = ReceiverValue[3]; //yaw
  ch5 = ReceiverValue[4];
  ch6 = ReceiverValue[5];
  ch7 = ReceiverValue[6];
  ch8 = ReceiverValue[7];
}

/*
============================================================
🛩 GPS Loop
============================================================*/
void GPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isValid() && gps.location.isUpdated()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    alt = gps.altitude.meters();
    Gvel = gps.speed.mps();
    sat = gps.satellites.value();
  }
}



/*
========================================================================================================================

                                                  State Estimation

========================================================================================================================*/

/*
============================================================
🛩 Phi(Roll) Theta(Pitch) Psi(Yaw)
============================================================*/
void AngleEstimate() {
  //Angle from IMU (rad)
  phi_a = atan2(-accy,sqrt(accx*accx + accz*accz));
  theta_a = atan2(accx,sqrt(accy*accy + accz*accz));
  //Angle from Mag (rad)
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 2*0.01745 * 2*0.01745;//Q
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 0.1*0.01745 * 0.1*0.01745);//R: Overshoot if too high
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

void Phi_Theta_Est() {
  kalman_1d(phi_k, phi_k_U, gyrox, phi_a);
  phi_k=Kalman1DOutput[0]; 
  phi_k_U=Kalman1DOutput[1];
  kalman_1d(theta_k, theta_k_U, gyroy, theta_a);
  theta_k=Kalman1DOutput[0]; 
  theta_k_U=Kalman1DOutput[1];
}



/*
========================================================================================================================

                                                  Controller

========================================================================================================================*/

/*
============================================================
🛩 PID
============================================================*/


/*
============================================================
🛩 Control Mixer Loop
============================================================*/
void controlMixer() {
  if (ch6 < 1300) {
    m1_command_scaled = thro_des + phi_PID + theta_PID - yaw_des;//FL
    m2_command_scaled = thro_des + phi_PID - theta_PID + yaw_des;//BL
    m3_command_scaled = thro_des - phi_PID + theta_PID + yaw_des;//FR
    m4_command_scaled = thro_des - phi_PID - theta_PID - yaw_des;//BR
    m5_command_scaled = 0;

   //0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
    s1_command_scaled = thro_des;
    s2_command_scaled = thro_des;
    s3_command_scaled = thro_des;
    s4_command_scaled = thro_des;
    s5_command_scaled = thro_des;
    s6_command_scaled = 0;
  }
  if ((ch6 > 1300) && (ch6 <1700)) {
    m1_command_scaled = thro_des;// + phi_PID + theta_PID - yaw_des;//FL
    m2_command_scaled = thro_des;// + phi_PID - theta_PID + yaw_des;//BL
    m3_command_scaled = thro_des;// - phi_PID + theta_PID + yaw_des;//FR
    m4_command_scaled = thro_des;// - phi_PID - theta_PID - yaw_des;//BR
    m5_command_scaled = 0;

   //0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
    s1_command_scaled = thro_des;
    s2_command_scaled = thro_des;
    s3_command_scaled = thro_des;
    s4_command_scaled = thro_des;
    s5_command_scaled = thro_des;
    s6_command_scaled = 0;
  }  
}

void armedStatus() {
  //DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.
  if ((ch5 < 1500) && (ch3 < 1050)) {
    armedFly = true;
  }
}

/*
============================================================
🛩 Scale Commands Loop
============================================================*/
void scaleCommands() {

  //Scaled to 125us - 250us for oneshot125 protocol
  m1_command_PWM = m1_command_scaled*125 + 125;
  m2_command_PWM = m2_command_scaled*125 + 125;
  m3_command_PWM = m3_command_scaled*125 + 125;
  m4_command_PWM = m4_command_scaled*125 + 125;
  m5_command_PWM = m5_command_scaled*125 + 125;
  //Constrain commands to motors within oneshot125 bounds
  m1_command_PWM = constrain(m1_command_PWM, 125, 250);
  m2_command_PWM = constrain(m2_command_PWM, 125, 250);
  m3_command_PWM = constrain(m3_command_PWM, 125, 250);
  m4_command_PWM = constrain(m4_command_PWM, 125, 250);
  m5_command_PWM = constrain(m5_command_PWM, 125, 250);

  //Scaled to 0-180 for servo library
  s1_command_PWM = s1_command_scaled*180;
  s2_command_PWM = s2_command_scaled*180;
  s3_command_PWM = s3_command_scaled*180;
  s4_command_PWM = s4_command_scaled*180;
  s5_command_PWM = s5_command_scaled*180;
  s6_command_PWM = s6_command_scaled*180;
  //Constrain commands to servos within servo library bounds
  s1_command_PWM = constrain(s1_command_PWM, 0, 180);
  s2_command_PWM = constrain(s2_command_PWM, 0, 180);
  s3_command_PWM = constrain(s3_command_PWM, 0, 180);
  s4_command_PWM = constrain(s4_command_PWM, 0, 180);
  s5_command_PWM = constrain(s5_command_PWM, 0, 180);
  s6_command_PWM = constrain(s6_command_PWM, 0, 180);

}

/*
============================================================
🛩 Desired State Loop
============================================================*/
void getDesState() {
  thro_des = (ch3 - 1000.0)/1000.0; //Between 0 and 1
  roll_des = (ch1 - 1500.0)/500.0; //Between -1 and 1
  pitch_des = -((ch2 - 1500.0)/500.0); //Between -1 and 1
  yaw_des = (ch4 - 1500.0)/500.0; //Between -1 and 1

  thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
}

/*
============================================================
🛩 Failsafe Loop
============================================================
*/
void failSafe() {
  //DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
   * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of 
   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 
   * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting 
   * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
   */
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  //Triggers for failure criteria
  if (ch1 > maxVal || ch1 < minVal) check1 = 1;
  if (ch2 > maxVal || ch2 < minVal) check2 = 1;
  if (ch3 > maxVal || ch3 < minVal) check3 = 1;
  if (ch4 > maxVal || ch4 < minVal) check4 = 1;
  if (ch5 > maxVal || ch5 < minVal) check5 = 1;
  if (ch6 > maxVal || ch6 < minVal) check6 = 1;

  //If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
    ch1 = channel_1_fs;
    ch2 = channel_2_fs;
    ch3 = channel_3_fs;
    ch4 = channel_4_fs;
    ch5 = channel_5_fs;
    ch6 = channel_6_fs;
  }
}

/*
============================================================
🛩 CommandMotors Loop
============================================================
*/
void commandMotors() {
  //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
  /*
   * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
   * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.
   */
  int wentLow = 0;
  int pulseStart, timer;
  int flagM1 = 0;
  int flagM2 = 0;
  int flagM3 = 0;
  int flagM4 = 0;
  int flagM5 = 0;
  
  //Write all motor pins high
  digitalWrite(m1Pin, HIGH);
  digitalWrite(m2Pin, HIGH);
  digitalWrite(m3Pin, HIGH);
  digitalWrite(m4Pin, HIGH);
  digitalWrite(m5Pin, HIGH);
  pulseStart = micros();

  //Write each motor pin low as correct pulse length is reached
  while (wentLow < 5 ) { //Keep going until final (6th) pulse is finished, then done
    timer = micros();
    if ((m1_command_PWM <= timer - pulseStart) && (flagM1==0)) {
      digitalWrite(m1Pin, LOW);
      wentLow = wentLow + 1;
      flagM1 = 1;
    }
    if ((m2_command_PWM <= timer - pulseStart) && (flagM2==0)) {
      digitalWrite(m2Pin, LOW);
      wentLow = wentLow + 1;
      flagM2 = 1;
    }
    if ((m3_command_PWM <= timer - pulseStart) && (flagM3==0)) {
      digitalWrite(m3Pin, LOW);
      wentLow = wentLow + 1;
      flagM3 = 1;
    }
    if ((m4_command_PWM <= timer - pulseStart) && (flagM4==0)) {
      digitalWrite(m4Pin, LOW);
      wentLow = wentLow + 1;
      flagM4 = 1;
    } 
    if ((m5_command_PWM <= timer - pulseStart) && (flagM5==0)) {
      digitalWrite(m5Pin, LOW);
      wentLow = wentLow + 1;
      flagM5 = 1;
    } 
  }
}

/*
============================================================
🛩 PWM Loop
============================================================
*/
void PWMloop() {
  //commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol
  servo1.write(s1_command_PWM); //Writes PWM value to servo object
  servo2.write(s2_command_PWM);
  servo3.write(s3_command_PWM);
  servo4.write(s4_command_PWM);
  servo5.write(s5_command_PWM);
  servo6.write(s6_command_PWM);
}


/*
============================================================
🛩 Throttle Cut
============================================================
*/
void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
      Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
      minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function
      called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
      the motors to anything other than minimum value. Safety first.

      channel_5_pwm is LOW then throttle cut is OFF and throttle value can change. (ThrottleCut is DEACTIVATED)
      channel_5_pwm is HIGH then throttle cut is ON and throttle value = 120 only. (ThrottleCut is ACTIVATED), (drone is DISARMED)
  */
  if ((ch5 < 1500) || (armedFly == false)) {
    armedFly = false;
    m1_command_PWM = 120;
    m2_command_PWM = 120;
    m3_command_PWM = 120;
    m4_command_PWM = 120;
    m5_command_PWM = 120;



    //Uncomment if using servo PWM variables to control motor ESCs
    s1_command_PWM = 0;
    s2_command_PWM = 0;
    s3_command_PWM = 0;
    s4_command_PWM = 0;
    s5_command_PWM = 0;
    s6_command_PWM = 0;
  }
}


/*
========================================================================================================================

                                                  Serial Monitor

========================================================================================================================
*/

void PrintSensorData() {
  Serial.println("");
  Serial.print("MPU9250:");
  Serial.println();
  Serial.print("IMU\tMAG\tacc_x\tacc_y\tacc_z\tgyro_x\tgyro_y\tgyro_z\tmag_x\tmag_y\tmag_z\ttemp_c\n");
  Serial.print(imu.new_imu_data()); Serial.print("\t");
  Serial.print(imu.new_mag_data()); Serial.print("\t");
  Serial.print(accx); Serial.print("\t");
  Serial.print(accy); Serial.print("\t");
  Serial.print(accz); Serial.print("\t");
  Serial.print(gyrox); Serial.print("\t");
  Serial.print(gyroy); Serial.print("\t");
  Serial.print(gyroz); Serial.print("\t");
  Serial.print(magx); Serial.print("\t");
  Serial.print(magy); Serial.print("\t");
  Serial.print(magz); Serial.print("\t");
  Serial.print(imu.die_temp_c()); Serial.println();
  Serial.println("BMP280:");
  Serial.print("temp_c[°C]\tpressure[Pa]\n");
  Serial.print(temperature); Serial.print("\t"); Serial.print("\t");
  Serial.print(pressure); Serial.print("\t"); Serial.print("\n");
}

void printStates() {
  Serial.println("");
  Serial.println("Phi(roll)\tTheta(pitch)\tPsi(yaw)");
  Serial.print(phi_a/(PI/180)); Serial.print("\t\t");
  Serial.print(theta_a/(PI/180)); Serial.print("\t\t");
  //Serial.print(psi_m/(PI/180)); Serial.print("\t");
  Serial.print("\n");
  Serial.print(phi_k/(PI/180)); Serial.print("\t\t");
  Serial.print(theta_k/(PI/180)); Serial.print("\t\t");
  Serial.print(psi_m/(PI/180)); Serial.println("\t");
  Serial.print("\n");
}

void printPPM() {
  Serial.println("");
  Serial.print("Number of channels: ");
  Serial.println(channelAvailable);
  Serial.print("ch1(roll): ");
  Serial.println(ch1);
  Serial.print("ch2(pitch): ");
  Serial.println(ch2);
  Serial.print("ch3(throttle): ");
  Serial.println(ch3);
  Serial.print("ch4(yaw): ");
  Serial.println(ch4);
  Serial.print("ch5(SWD): ");
  Serial.println(ch5);
  Serial.print("ch6(SWC): ");
  Serial.println(ch6);
  Serial.print("ch7(VRA): ");
  Serial.println(ch7);
  Serial.print("ch8(VRB): ");
  Serial.println(ch8);
  Serial.println();
}

void printDes() {
  Serial.print("roll_des: ");
  Serial.print(roll_des);
}

void printGPS() {
  Serial.print("Lat: "); Serial.println(lat, 6);
  Serial.print("Lon: "); Serial.println(lon, 6);
  Serial.print("Alt: "); Serial.println(alt);
  Serial.print("GrdSpeed: "); Serial.print(Gvel, 6); Serial.println("m/s");
  Serial.print("Satellites: "); Serial.println(sat);
  Serial.println();
}