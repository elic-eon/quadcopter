//mpu6050ver
//#define X_KP_DEFAULT 0.3
//#define X_KI_DEFAULT 0.05
//#define X_KD_DEFAULT 0.09

//#define Y_KP_DEFAULT 0.28
//#define Y_KI_DEFAULT 0.02
//#define Y_KD_DEFAULT 0.09

#define scale_large 0.02
#define scale_mid 0.01
#define scale_small 0.005

#define motion_degree 1

#define MODE_KP 0
#define MODE_KI 1
#define MODE_KD 2

#include <Servo.h>
#include <EEPROM.h>
/**eeprom addrress 0-35 store the value of pid constant **/



#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "PID_v1.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//#include <ADXL345.h>
//#include <L3G4200D.h>
//ADXL345 adxl;
//L3G4200D gyro;

MPU6050 mpu;

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



float data_timer;
float timer;
float timer_interval;
float timer_old;

float looping_timer;

Servo quad[4];
double pwm[4];
double base[4];
char in;
int condition;//1:stop,2:up,3:down,4:stable,5:active physical controller
int physical_enable;
float base_get_from_BT;

int x, y, z;

double X, Y, Z;

double theta_x, theta_y, theta_z;

double x_kp, x_ki, x_kd;
double y_kp, y_ki, y_kd;
double tuning_scale = 0;
int tuning_mode;


double sum_err_x_theta, sum_err_y_theta, sum_err_z_theta;//use in I-control
double angular_v_x, angular_v_y;

double setpoint_x, setpoint_y;

double pid_out_x, pid_out_y;

PID pid_x(&theta_x, &pid_out_x, &setpoint_x, 0.3, 0.05, 0.09, DIRECT);
PID pid_y(&theta_y, &pid_out_y, &setpoint_y, 0.28, 0, 0.09, DIRECT);//0.3 0.03 0.07

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {

  looping_timer = millis();
  theta_x = 0;
  theta_y = 0;
  theta_z = 0;
  sum_err_x_theta = 0;
  sum_err_y_theta = 0;
  sum_err_z_theta = 0;

  EEPROM.get(0, x_kp);
  EEPROM.get(4, x_ki);
  EEPROM.get(8, x_kd);
  EEPROM.get(12, y_kp);
  EEPROM.get(16, y_ki);
  EEPROM.get(20, y_kd);

  pid_x.SetTunings(x_kp, x_ki, x_kd);
  pid_y.SetTunings(y_kp, y_ki, y_kd);

  /*x_kp = X_KP_DEFAULT;
  x_ki = X_KI_DEFAULT;
  x_kd = X_KD_DEFAULT;
  y_kp = Y_KP_DEFAULT;
  y_ki = Y_KI_DEFAULT;
  y_kd = Y_KD_DEFAULT;
  */
  condition = 1;
  physical_enable = 0;
  base_get_from_BT = 0;
  for (int i = 0; i < 4; i++) {
    pwm[i] = 0;
    base[i] = 0;
  }



  //adxl.setAxisOffset(-1, -1, 0);
  //adxl.set_bw(B00001100);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(38400);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity

  //***still need to test now not measure yet 2015/10/25****//

  mpu.setXGyroOffset(+45);
  mpu.setYGyroOffset(-12);
  mpu.setZGyroOffset(+2);
  mpu.setXAccelOffset(606);
  mpu.setYAccelOffset(-1485);
  mpu.setZAccelOffset(+1488);
  

/*
  int qq=mpu.getXGyroOffset();
  int rr=mpu.getYGyroOffset();
  int ss=mpu.getZGyroOffset();

*/
  /*
  int qq=mpu.getXAccelOffset();
  int rr=mpu.getYAccelOffset();
  int ss=mpu.getZAccelOffset();

  Serial.println((int)qq);
 Serial.println((int)rr);
 Serial.println((int)ss);
  */
  /**
   offset[0]=XGyroOffset
   offset[1]=YGyroOffset
   offset[2]=ZGyroOffset
   offset[3]=XAccelOffset
   offset[4]=YAccelOffset
   offset[5]=ZAccelOffset
  */
  /*
  mpu.setXGyroOffset(+45);
  mpu.setYGyroOffset(-12);
  mpu.setZGyroOffset(+2);
  mpu.setZAccelOffset(+1153);
*/
/*
  int16_t raw[6] = {0};
  double offset[6] = {0};
  for (int i = 0; i < 20; i++) {
    mpu.getMotion6(&raw[3], &raw[4], &raw[5] , &raw[0], &raw[1], &raw[2]);
    offset[0] = offset[0] + (double)raw[0];
    offset[1] = offset[1] + (double)raw[1];
    offset[2] = offset[2] + (double)raw[2];
    offset[3] = offset[3] + (double)raw[3];
    offset[4] = offset[4] + (double)raw[4];
    offset[5] = offset[5] + (double)raw[5];
  }
  offset[0]=offset[0]/20;
  offset[1]=offset[1]/20;
  offset[2]=offset[2]/20;

  offset[3]=offset[3]/20;
  offset[4]=offset[4]/20;
  offset[5]=offset[5]/20;

 Serial.println((int)offset[0]);
 Serial.println((int)offset[1]);
 Serial.println((int)offset[2]);
 Serial.println((int)offset[3]);
 Serial.println((int)offset[4]);
 Serial.println((int)offset[5]);
*/

/*
  mpu.setXGyroOffset((int)offset[0]);
  mpu.setYGyroOffset((int)offset[1]);
  mpu.setZGyroOffset((int)offset[2]);
  mpu.setXAccelOffset((int)offset[3]);
  mpu.setYAccelOffset((int)offset[4]);
  mpu.setZAccelOffset((int)offset[5]);
*/
/*
  mpu.setXGyroOffset((int)offset[0]);
  mpu.setYGyroOffset((int)offset[1]);
  mpu.setZGyroOffset((int)offset[2]);
  mpu.setXAccelOffset((int)offset[3]);
  mpu.setYAccelOffset((int)offset[4]);
  mpu.setZAccelOffset((int)offset[5]);
  */
 

    
  Serial.println("Offset setting done!");
  /*
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); */

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
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
  quad[0].attach(4); //attach M1 on 4 5 6 7
  quad[1].attach(5); //attach M2 on 4 5 6 7
  quad[2].attach(6); //attach M3 on 4 5 6 7
  quad[3].attach(7); //attach M4 on 4 5 6 7
  //gyro.writeReg(0x20, 0x5F);
  //gyro.writeReg(0x23, 0x90);
  timer_old = millis();

  /*
    adxl.powerOn();
    adxl.readAccel(&x, &y, &z);
    X = x * 0.00383;
    Y = y * 0.00384;
    Z = z * 0.00388;
    theta_x = (atan(Y / Z) * (57.29)); //error function
    theta_y = (atan(X / Z) * (-57.29));//error function
    theta_z = 0 ;
  */


  /*
  old_theta_x = theta_x;
  old_theta_y = theta_y;
  old_theta_z = theta_z;
  */
  data_timer = millis();

  setpoint_x = 0;
  setpoint_y = 0;

  pid_x.SetMode(AUTOMATIC);
  pid_x.SetOutputLimits(-100, 100);
  pid_x.SetSampleTime(20);

  pid_y.SetMode(AUTOMATIC);
  pid_y.SetOutputLimits(-100, 100);
  pid_y.SetSampleTime(20);



}

void loop() {

  // if programming failed, don't try to do anything
  if (!dmpReady) {
    Serial.print("Programming failed!");
    return;
  }

  if ( physical_enable == 0 && Serial.available() > 0) {
    in = Serial.read();
    switch (in) {
      case 'a'://stop
        condition = 1;
        Serial.println("~~STOP~~");
        break;
      case 'b'://up
        condition = 2;
        Serial.println("~~UP~~");
        break;
      case 'c'://down
        condition = 3;
        Serial.println("~~DOWN~~");
        break;
      case 'd'://stable
        condition = 4;
        Serial.println("~~STABLE~~");
        break;
      /*
       case 'e'://active the physical controller
         physical_enable = 1;
         condition = 5;
         Serial.println("~~Turn to The physical controller~~");
         break;
       */

      case 'f': //left
        if (setpoint_y <= -40) {
          setpoint_y = setpoint_y - motion_degree;
        }
        Serial.println(setpoint_y);
        break;

      case 'h': //right
        if (setpoint_y >= 40) {
          setpoint_y = setpoint_y + motion_degree;
        }
        Serial.println(setpoint_y);
        break;

      case 't'://front
        if (setpoint_x <= -40) {
          setpoint_x = setpoint_x - motion_degree;
        }
        Serial.println(setpoint_x);
        break;

      case 'g'://back
        if (setpoint_x >= 40) {
          setpoint_x = setpoint_x + motion_degree;
        }
        Serial.println(setpoint_x);
        break;

      case 'v'://normal
        setpoint_x = 0;
        setpoint_y = 0;
        Serial.println(setpoint_x);
        Serial.println(setpoint_y);
        break;


      case 'p':
        //condition = 4;
        tuning_mode = MODE_KP;
        Serial.println("change tuning_mode KP");
        break;
      case 'i':
        //condition = 4;
        tuning_mode = MODE_KI;
        Serial.println("change tuning_mode KI");
        break;
      case 'o':
        //condition = 4;
        tuning_mode = MODE_KD;
        Serial.println("change tuning_mode KD");
        break;

      //tuning scale setting
      case 'q':
        //condition = 4;
        tuning_scale = scale_large;
        Serial.println("change the scale to large");
        break;

      case 'r':
        //condition = 4;
        tuning_scale = scale_mid;
        Serial.println("change the scale to mid");
        break;

      case 's':
        //condition = 4;
        tuning_scale = scale_small;
        Serial.println("change the scale to small");
        break;

      //Decrease
      case 'x':
        //condition = 4;
        if (tuning_mode == MODE_KP) {
          x_kp = x_kp * (1 - tuning_scale);
          pid_x.SetTunings(x_kp, x_ki, x_kd);
          //x_kp = x_kp * 0.95;
          Serial.print("DEC x_kp = ");
          Serial.println(x_kp, 6);
        }
        else if (tuning_mode == MODE_KI) {
          x_ki = x_ki * (1 - tuning_scale);
          pid_x.SetTunings(x_kp, x_ki, x_kd);
          //x_ki = x_ki * 0.95;
          Serial.print("DEC x_ki = ");
          Serial.println(x_ki, 6);
        }
        else if (tuning_mode == MODE_KD) {
          x_kd = x_kd * (1 - tuning_scale);
          pid_x.SetTunings(x_kp, x_ki, x_kd);
          //x_kd = x_kd * 0.95;
          Serial.print("DEC x_kd = ");
          Serial.println(x_kd, 6);
        }
        break;

      case 'y':
        //condition = 4;
        if (tuning_mode == MODE_KP) {
          y_kp = y_kp * (1 - tuning_scale);
          pid_y.SetTunings(y_kp, y_ki, y_kd);
          //y_kp = y_kp * 0.95;
          Serial.print("DEC y_kp = ");
          Serial.println(y_kp, 6);
        }
        else if (tuning_mode == MODE_KI) {
          y_ki = y_ki * (1 - tuning_scale);
          pid_y.SetTunings(y_kp, y_ki, y_kd);
          //y_ki = y_ki * 0.95;
          Serial.print("DEC y_ki = ");
          Serial.println(y_ki, 6);
        }
        else if (tuning_mode == MODE_KD) {
          y_kd = y_kd * (1 - tuning_scale);
          pid_y.SetTunings(y_kp, y_ki, y_kd);
          //y_kd = y_kd * 0.95;
          Serial.print("DEC y_kd = ");
          Serial.println(y_kd, 6);
        }
        break;

      //Increase
      case 'X':
        //condition = 4;
        if (tuning_mode == MODE_KP) {
          x_kp = x_kp * (1 + tuning_scale);
          pid_x.SetTunings(x_kp, x_ki, x_kd);
          //x_kp = x_kp * 1.05;
          Serial.print("INC x_kp = ");
          Serial.println(x_kp, 6);
        }
        else if (tuning_mode == MODE_KI) {
          x_ki = x_ki * (1 + tuning_scale);
          pid_x.SetTunings(x_kp, x_ki, x_kd);
          //x_ki = x_ki * 1.05;
          Serial.print("INC x_ki = ");
          Serial.println(x_ki, 6);
        }
        else if (tuning_mode == MODE_KD) {
          x_kd = x_kd * (1 + tuning_scale);
          pid_x.SetTunings(x_kp, x_ki, x_kd);
          //x_kd = x_kd * 1.05;
          Serial.print("INC x_kd = ");
          Serial.println(x_kd, 6);
        }
        break;

      case 'Y':
        //condition = 4;
        if (tuning_mode == MODE_KP) {
          y_kp = y_kp * (1 + tuning_scale);
          pid_y.SetTunings(y_kp, y_ki, y_kd);
          //y_kp = y_kp * 1.05;
          Serial.print("INC y_kp = ");
          Serial.println(y_kp, 6);
        }
        else if (tuning_mode == MODE_KI) {
          y_ki = y_ki * (1 + tuning_scale);
          pid_y.SetTunings(y_kp, y_ki, y_kd);
          //y_ki = y_ki * 1.05;
          Serial.print("INC y_ki = ");
          Serial.println(y_ki, 6);
        }
        else if (tuning_mode == MODE_KD) {
          y_kd = y_kd * (1 + tuning_scale);
          pid_y.SetTunings(y_kp, y_ki, y_kd);
          //y_kd = y_kd * 1.05;
          Serial.print("INC y_kd = ");
          Serial.println(y_kd, 6);
        }
        break;

      case '@':
        Serial.println("Save the pid constant to EEPROM");

        EEPROM.put(0, x_kp);
        EEPROM.put(4, x_ki);
        EEPROM.put(8, x_kd);

        EEPROM.put(12, y_kp);
        EEPROM.put(16, y_ki);
        EEPROM.put(20, y_kd);

        Serial.println("Sucess");
        break;

      case '!':
        Serial.println("The pid constant in EEPROM is");
        double tmp_out;
        EEPROM.get(0, tmp_out);
        Serial.println(tmp_out);

        EEPROM.get(4, tmp_out);
        Serial.println(tmp_out);

        EEPROM.get(8, tmp_out);
        Serial.println(tmp_out);

        EEPROM.get(12, tmp_out);
        Serial.println(tmp_out);

        EEPROM.get(16, tmp_out);
        Serial.println(tmp_out);

        EEPROM.get(20, tmp_out);
        Serial.println(tmp_out);
        break;

      case '#':
        Serial.println("The pid constant now is");
        Serial.println(x_kp);
        Serial.println(x_ki);
        Serial.println(x_kd);

        Serial.println(y_kp);
        Serial.println(y_ki);
        Serial.println(y_kd);
        break;
      case '$':
        Serial.println("Reload pid constant in the eeprom");
        EEPROM.get(0, x_kp);
        EEPROM.get(4, x_ki);
        EEPROM.get(8, x_kd);
        EEPROM.get(12, y_kp);
        EEPROM.get(16, y_ki);
        EEPROM.get(20, y_kd);

        pid_x.SetTunings(x_kp, x_ki, x_kd);
        pid_y.SetTunings(y_kp, y_ki, y_kd);
        break;

    }
  }

  if (physical_enable == 1 && Serial.available() >= 4) { //get the value,the data in the serial buffer ,there should be more than 4-bytes or it will get a wrong value
    /*get the value send by the physical controller and the set it as the base*/
    /*parse float*/
    /*the final character will be an alphabet to force output*/
    base_get_from_BT = Serial.parseFloat();
  }

  timer = millis();
  timer_interval = timer - timer_old;
  timer_old = timer;
  /********************************
  get the sensor value below
  ********************************/
  while (!mpuInterrupt && fifoCount < packetSize) {
    // Serial.println("wait for data1");
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  /* Serial.print(" 2mpuIntStatus: ");
   Serial.println(mpuIntStatus);*/
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
    if (fifoCount >= 1024) {
      Serial.print("ERROR fifoCount:");
      Serial.println(fifoCount);
      Serial.print(" mpuIntStatus: ");
      Serial.print(mpuIntStatus);
    }
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x01) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // Serial.println("wait for data2");
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;



    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


    theta_z = ypr[0] * 180 / M_PI; //yaw

    theta_x = - ypr[2] * 180 / M_PI; //roll

    theta_y = ypr[1] * 180 / M_PI; //pitch


    /*
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
    */
  }

  //gyro.read();

  //adxl.readAccel(&x, &y, &z);
  /*
  X = x * 0.00383;
  Y = y * 0.00384;
  Z = z * 0.00388;
  */
  /*old_theta_x = theta_x;
  old_theta_y = theta_y;
  old_theta_z = theta_z;
  */

  /*
  angular_v_x=(gyro.g.x  - 9.018621) * 0.0179;
  angular_v_y=(gyro.g.y  + 2.5052198) * 0.0175;

  theta_x = (theta_x + (gyro.g.x  - 9.018621) * 0.0179 * (timer_interval / 1000)) * 0.9996 + (atan(Y / Z) * (57.29)) * 0.0004;
  theta_y = (theta_y + (gyro.g.y  + 2.5052198) * 0.0175 * (timer_interval / 1000)) * 0.9996 + (atan(X / Z) * (-57.29)) * 0.0004;

  */
  //theta_z = (theta_z + (gyro.g.z - 46.255112) * 0.01802 * (timer_interval / 1000));
  //theta_x = (theta_x + (gyro.g.x  - 9.018621) * 0.0179 * (timer_interval / 1000)) ;
  //theta_y = (theta_y + (gyro.g.y  + 2.5052198) * 0.0175 * (timer_interval / 1000));



  feedback_start(condition);
  if (physical_enable == 1 && base_get_from_BT > 0.1) {
    if (condition != 1) {
      pid_x.Compute();
      pid_y.Compute();
      error_correct( -pid_out_x , pid_out_y, pid_out_x, -pid_out_y ); //for x-axis and y-axis
    }
  }
  else {
    if (condition != 1) {
      pid_x.Compute();
      pid_y.Compute();
      error_correct(-pid_out_x, pid_out_y, pid_out_x, -pid_out_y );//for x-axis and y-axit
    }
  }


  //condition=4;
  if (millis() - data_timer > 1500) {
    //if (1) {
    data_timer = millis();
    /*Serial.print(atan(Y / Z) * (57.29));
    Serial.print("  ");
    Serial.println(atan(X / Z) * (-57.29)); */
    /*
        Serial.print((float)gyro.g.x);
        Serial.print("  ");
        Serial.print((float)gyro.g.y);
        Serial.print("  ");
        Serial.println((float)gyro.g.z);*/

    Serial.print(theta_x, 4);
    Serial.print("  ");
    Serial.print(theta_y, 4);
    Serial.print("  ");
    Serial.println(theta_z, 4);

    /*Serial.print("X= ");
    Serial.print(X, 4);
    Serial.print("       ");
    //Serial.print("Y= ");
    Serial.print(Y, 4);
    Serial.print("       ");
    //Serial.print("Z= ");
    Serial.print(Z, 4);
    Serial.println("  ");*/

    /*  Serial.println("Time : ");
      Serial.print(time2-time1);
    */
    /*  Serial.print("Condition : ");
      Serial.println(condition);
    */

    Serial.println("PWM% : ");
    Serial.print("M1: ");
    Serial.print(pwm[0]);
    Serial.print(" M2: ");
    Serial.print(pwm[1]);
    Serial.print(" M3: ");
    Serial.print(pwm[2]);
    Serial.print(" M4: ");
    Serial.println(pwm[3]);

    // Serial.println(millis()-looping_timer);
  }
  looping_timer = millis();
  //delay(300);
}

void feedback_start(int mode) { //this function will change the pwm width by feedback control
  switch (mode) {
    case 1:
      speed_setting(0, 0, 0, 0);
      for (int i = 0; i < 4; i++) {
        base[i] = 0;
      }
      break;

    case 2:
      for (int i = 0; i < 4; i++) {
        base[i] = base[i] + 5;
      }
      for (int i = 0; i < 4; i++) {
        if (base[i] < 5 && condition != 1) {
          base[i] = 5;
        }

        if (base[i] >= 60) {
          base[i] = 60;
        }
      }
      //find_sum_p();
      condition = 4;
      break;

    case 3:
      for (int i = 0; i < 4; i++) {
        base[i] = base[i] - 2;
      }
      for (int i = 0; i < 4; i++) {
        if (base[i] < 5 && condition != 1) {
          base[i] = 5;
        }

        if (base[i] >= 60) {
          base[i] = 60;
        }
      }
      //find_sum_p();
      condition = 4;
      break;

    case 4:
      //find_sum_p();

      /*CAUTION!!!!! The part that is commented is still in developing,do not uncomment the  belowing code.*/



      break;

    case 5://in case 5 it should do the same thing in case4,that is "control the height"(How?)

      for (int i = 0; i < 4; i++) {
        base[i] = base_get_from_BT;
      }

      for (int i = 0; i < 4; i++) {
        if (base[i] >= 60) {
          base[i] = 60;
        }
      }
      if (base_get_from_BT == 0) {
        speed_setting(0, 0, 0, 0);
      }

      //find_sum_p();
      condition = 5;
      break;
  }
}




void error_correct(double m1, double m2, double m3, double m4) {
  //Serial.println(m2, 6);
  speed_setting(base[0] + m1, base[1] + m2, base[2] + m3, base[3] + m4);
}

void speed_setting() {//set the speed
  for (int i = 0; i < 4; i++) {
    if (pwm[i] < 5 && ( (physical_enable == 0 && condition != 1) || (physical_enable == 1 && base_get_from_BT > 0.1))) { //(physical_enable==0 && condition != 1) || (physical_enable==1 && base_get_from_BT>0.1)
      pwm[i] = 5;
    }

    if (pwm[i] >= 60) {
      pwm[i] = 60;
    }
  }

  for (int i = 0; i < 4; i++) {
    quad[i].write(pwm[i] + 80);
  }
}

void speed_setting(double a, double b, double c, double d) {
  pwm[0] = a;
  pwm[1] = b;
  pwm[2] = c;
  pwm[3] = d;
  speed_setting();
}



