#define kp 1.2
#define ki 0.0
#define kd 0.0

#include<Servo.h>
#include <Wire.h>
#include <ADXL345.h>
#include <L3G4200D.h>

ADXL345 adxl;
L3G4200D gyro;

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

double sum_err_x_theta, sum_err_y_theta, sum_err_z_theta;//use in I-control
double old_theta_x, old_theta_y, old_theta_z;
double predict_theta_x, predict_theta_y, predict_theta_z;


double sum_p[4];//this array is used to save the values of P-control
double sum_i[4];//this array is used to save the values of I-control
double sum_d[4];//this array is used to save the values of D-control

void setup() {
  looping_timer = millis();
  theta_x = 0;
  theta_y = 0;
  theta_z = 0;
  sum_err_x_theta = 0;
  sum_err_y_theta = 0;
  sum_err_z_theta = 0;

  condition = 1;
  physical_enable = 0;
  base_get_from_BT = 0;
  for (int i = 0; i < 4; i++) {
    pwm[i] = 0;
    base[i] = 0;
    sum_p[i] = 0;
    sum_i[i] = 0;
  }

  quad[0].attach(3); //attach on 3 5 7 9
  quad[1].attach(5); //attach on 3 5 7 9
  quad[2].attach(7); //attach on 3 5 7 9
  quad[3].attach(9); //attach on 3 5 7 9

  adxl.setAxisOffset(-1, -1, 0);
  //adxl.set_bw(B00001100);
  Serial.begin(9600);
  Wire.begin();
  gyro.writeReg(0x20, 0x5F);
  gyro.writeReg(0x23, 0x90);
  timer_old = millis();

  adxl.powerOn();
  adxl.readAccel(&x, &y, &z);
  X = x * 0.00383;
  Y = y * 0.00384;
  Z = z * 0.00388;
  theta_x = (atan(Y / Z) * (57.29));
  theta_y = (atan(X / Z) * (-57.29));
  theta_z = 0 ;

  old_theta_x = theta_x;
  old_theta_y = theta_y;
  old_theta_z = theta_z;

  data_timer = millis();
}

void loop() {

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
      case 'e'://active the physical controller
        physical_enable = 1;
        condition = 5;
        Serial.println("~~Turn to The physical controller~~");
        break;
    }
  }

  if (physical_enable == 1 && Serial.available() >= 4) { //get the value,the data in the serial buffer should more than 4-bytes or it will get a wrong value
    /*get the value send by the physical controller and the set it as the base*/
    /*parse float*/
    /*the final character will be an alphabet to force output*/
    base_get_from_BT = Serial.parseFloat();
  }

  timer = millis();
  timer_interval = timer - timer_old;
  timer_old = timer;
  gyro.read();

  adxl.readAccel(&x, &y, &z);
  X = x * 0.00383;
  Y = y * 0.00384;
  Z = z * 0.00388;

  old_theta_x = theta_x;
  old_theta_y = theta_y;
  old_theta_z = theta_z;


  theta_x = (theta_x + (gyro.g.x  - 9.018621) * 0.0179 * (timer_interval / 1000)) * 0.98 + (atan(Y / Z) * (57.29)) * 0.02;
  theta_y = (theta_y + (gyro.g.y  + 2.5052198) * 0.0175 * (timer_interval / 1000)) * 0.98 + (atan(X / Z) * (-57.29)) * 0.02;
  //theta_z = (theta_z + (gyro.g.z - 46.255112) * 0.01802 * (timer_interval / 1000));

  p_controller_and_feedback_start(condition);
  if (condition != 1 && base_get_from_BT >= 0) {//for security ,you should be careful when modify the segment
    i_controller();
    d_controller();
    sum_error_and_correct();
  }
  //condition=4;
  if (millis() - data_timer > 1500) {
    data_timer = millis();
    /*Serial.print(atan(Y / Z) * (57.29));
    Serial.print("  ");
    Serial.println(atan(X / Z) * (-57.29)); * /

    /*Serial.print((float)gyro.g.x);
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

void p_controller_and_feedback_start(int mode) { //this function will change the pwm width by feedback control
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
      find_sum_p();
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
      find_sum_p();
      condition = 4;
      break;

    case 4:
      find_sum_p();

      /*CAUTION!!!!! The part that is commented is still in developing,do not uncomment the  belowing code.*/

      /*if  ( (angular_v_x<10 && angular_v_x>-10) && (angular_v_y<10 && angular_v_y>-10) && (err_z_accel > 0.1 | err_z_accel < -0.1) ) {
        error_correct(-kp*err_z_accel,-kp*err_z_accel,-kp*err_z_accel,-kp*err_z_accel);
      }*/

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
      find_sum_p();
      condition=5;
      break;
  }
}
void d_controller() {
  predict_theta_x = theta_x + theta_x - old_theta_x;
  predict_theta_y = theta_y + theta_y - old_theta_y;
  predict_theta_z = theta_z + theta_z - old_theta_z;
  sum_d[0] = ( kd *  predict_theta_x ) + ( -kd *  predict_theta_y);
  sum_d[1] = ( -kd *  predict_theta_x ) + ( -kd *  predict_theta_y);
  sum_d[2] = ( -kd *  predict_theta_x ) + ( kd *  predict_theta_y);
  sum_d[3] = ( kd *  predict_theta_x ) + ( kd *  predict_theta_y);

}

void i_controller() {

  sum_err_x_theta = 0.8 * sum_err_x_theta + theta_x;
  sum_err_y_theta = 0.8 * sum_err_y_theta + theta_y;
  sum_err_z_theta = 0.8 * sum_err_z_theta + theta_z;
  find_sum_i();

}

void find_sum_p() {
  sum_p[0] = ( kp * theta_x ) + ( -kp * theta_y);
  sum_p[1] = (- kp * theta_x ) + ( -kp * theta_y);
  sum_p[2] = (- kp * theta_x ) + ( kp * theta_y);
  sum_p[3] = ( kp * theta_x ) + ( kp * theta_y);
}

void find_sum_i() {
  sum_i[0] = (ki * sum_err_x_theta ) + (- ki * sum_err_y_theta );
  sum_i[1] = (-ki * sum_err_x_theta ) + (- ki * sum_err_y_theta );
  sum_i[2] = (-ki * sum_err_x_theta ) + ( ki * sum_err_y_theta );
  sum_i[3] = (ki * sum_err_x_theta ) + ( ki * sum_err_y_theta );
}
void sum_error_and_correct() {
  error_correct(sum_i[0] + sum_p[0] + sum_d[0], sum_i[1] + sum_p[1] + sum_d[1], sum_i[2] + sum_p[2] + sum_d[2], sum_i[3] + sum_p[3] + sum_d[3]);
}

void error_correct(double m1, double m2, double m3, double m4) {
  speed_setting(base[0] + m1, base[1] + m2, base[2] + m3, base[3] + m4);
}

void speed_setting() {//set the speed
  for (int i = 0; i < 4; i++) {
    if (pwm[i] < 5 && condition != 1) {
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

