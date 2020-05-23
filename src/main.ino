#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#include <Servo.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>

// 1 analog pin for cds
// 2 digital pin for sonar
#define BAUD_RATE 115200
#define BT_BAUD_RATE 9600
#define SBUF_SIZE 64
#define ZERO_RUDDER_ANGLE 95
#define LEFT_RUDDER_ANGLE_MAX 160
#define RIGHT_RUDDER_ANGLE_MAX 20
#define PI 3.14159

int CDS_PIN[3] = {A0, A1, A2};
int TRIG_PIN[3] = {31, 35, 39};
int ECHO_PIN[3] = {33, 37, 41};
int R_EN = 7;
int L_EN = 8;
int RPWM = 5;
int LPWM = 6;
int RUDDER_PIN = 9;
int BT_TX_PIN = 50;
int BT_RX_PIN = 51;
// int EBI_TX_PIN = 14;
// int EBI_RX_PIN = 15;

int speed = 35;
long distance[3] = {200, 200, 200}; // right, center, left
int cds_val[3];
int count = 0;
int dist_crit = 120; // critical distance
int brightlim = 400;

float euler = 0;      // euler is EBimu sensor value. range = (-180, 180)
float euler_init = 0; // indicates euler angle when ship starts operating
float yaw = 0;        // yaw is measured w.r.t euler_init. range = (-180, 180)
float yaw_target = 0; // temporal yaw value used when turning

Servo servo;
SoftwareSerial BTSerial(BT_TX_PIN, BT_RX_PIN);
LiquidCrystal_I2C lcd(0x27, 16, 2);

SimpleTimer timer;

int EBimuAsciiParser(float *item, int number_of_item);
void goForward();
void goBackward();
void motorStop();
void setRudderAngle(int angle);
void updateSensorValues();
void setInitEuler();
float getYaw(float ref_angle);
float addAngle(float angle1, float angle2);
void turnRight(float yaw_target);
void turnLeft(float yaw_target);
long getDistance(int idx);
void sendValues(SoftwareSerial s);

void setup() {
  Serial.begin(BAUD_RATE);
  servo.attach(RUDDER_PIN);
  servo.write(91);
  BTSerial.begin(BT_BAUD_RATE);
  Serial3.begin(BAUD_RATE);
  lcd.begin();
  lcd.backlight();

  for (int i = 0; i < 3; i++) {
    pinMode(TRIG_PIN[i], OUTPUT);
    pinMode(ECHO_PIN[i], INPUT);
  }

  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  digitalWrite(RPWM, 0);
  digitalWrite(LPWM, 0);

  // initial yaw angle
  setInitEuler();
  yaw_target = getYaw(euler_init);
  goForward();
  delay(500);

  delay(500);

  timer.setInterval(100, updateSensorValues);
}

void loop() {
  timer.run();

  yaw = getYaw(euler_init);

  // 오른쪽으로 돌아야 하는 상황 가정
  if (/* nextstepturnright == True*/) {
    yaw_target = addAngle(yaw, 90); // target yaw를 현재 yaw + 90으로 지정
    // bool isTurningRight = True;
  }

  if (/*isTurningRight == True*/) {
    turnRight(yaw_target); // 오른쪽으로 쬐끔 돈다
  }

  if (/* nextstepturnleft == True*/) {
    yaw_target = addAngle(yaw, -90);
  }

  if (/*isTurningRight == True*/) {
    turnLeft(yaw_target); // 왼쪽으로 쬐끔 돈다
  }

  switch (count) {
  case 0:
    if (distance[1] < distancelim) {
      turnLeft();
      count++;
    }
    break;
  case 1:
    if (distance[0] > 100 || distance[1] < 120) {
      turnRight();
      count++;
    }
    break;
  case 2:
    if (distance[0] > 100 || distance[1] < 120) {
      turnRight(); //대충 우회전 하라는 뜻
      count++;
    }
    break;
  case 3:
    if (distance[2] > 120 || distance[1] < 120) {
      turnLeft(); //대충 좌횟=전 하라는뜻
      count++;
    }
    break;
  case 4:
    if (distance[1] < distancelim) {
      turnLeft(); //좌회전
      count++;
    }
    break;
  case 5:
    if (distance[0] > 100 || distance[1] < 120) {
      turnRight(); //우회전
      count++;
    }
    break;
  case 6:
    if (distance[0] > 100 || distance[1] < 120) {
      turnRight(); //대충 우회전 하라는 뜻
      count++;
    }
    break;
  case 7:
    if (distance[2] > 120 || distance[1] < 120) {
      turnLeft();
      count++;
    }

    break;
  }
  goForward();
  delay(100);
}

void goForward() {
  analogWrite(RPWM, speed);
  analogWrite(LPWM, 0);
  servo.attach(RUDDER_PIN);
  servo.write(Initial_Angle);
  delay(300);
  servo.detach();
}

void goBackward() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, speed);
  servo.attach(RUDDER_PIN);
  servo.write(Initial_Angle);
  delay(300);
  servo.detach();
}

void motorStop() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  servo.attach(RUDDER_PIN);
  servo.write(91);
  delay(300);
  servo.detach();
}

void setRudderAngle(int angle) {
  //
  int rem = angle % 3;
  int mod_angle;
  if (rem == 1) mod_angle = angle - 1;
  if (rem == 2) mod_angle = angle + 1;
  servo.attach(RUDDER_PIN);
  servo.write(mod_angle);
}

void updateSensorValues() { /*
                            update sensor values (distance, cds, euler)
                            */
  for (int i = 0; i < 3; i++) {
    distance[i] = getDistance(i);
    cds_val[i] = analogRead(CDS_PIN[i]);
  }
  float euler_temp[3];
  EBimuAsciiParser(euler_temp, 3);
  euler = euler_temp[2];
}

void setInitEuler() {
  float temp = 0;
  for (int i = 0; i < 10; i++) {
    temp += getYaw();
    delay(100);
  }
  euler_init = temp * 0.1; // mean of first 10 values
}

float getYaw(float ref_angle) { /*
                                calculate yaw angle w.r.t ref_angle
                                if ref_angle == euler_init,
                                  return ordinary yaw (relative direction w.r.t
                                initial direction) if ref_angle == yaw_temp,
                                  return yaw for rotation (which will be
                                iterated until yaw == -90 or +90)

                                ALWAYS RETURN VALUES BETWEEN (-180, 180)
                                */
  yaw = euler - ref_angle;
  if (yaw > 180)
    yaw -= 360;
  if (yaw < -180)
    yaw += 360;
  return yaw;
}

float addAngle(float angle1, float angle2) {
  float result = angle1 + angle2;
  if (result > 180)
    result -= 360;
  if (result < -180)
    result += 360;
  return result;
}

void turnRight(float yaw_target) // turn until yaw is 90
{
  // yaw = getYaw(euler_init); // turn 시작 전 현재 yaw
  yaw_temp = getYaw(yaw_target); // 목표 yaw 기준 좌표계 재정의
  delta_yaw = addAngle(yaw_target, -yaw_temp);

  int angle = int(
    (ZERO_RUDDER_ANGLE - RIGHT_RUDDER_ANGLE_MAX)
    * cos(delta_yaw * PI / 180)
    + RIGHT_RUDDER_ANGLE_MAX);
  setRudderAngle(angle);
}

void turnLeft(float yaw_target) {
  yaw_temp = getYaw(yaw_target); // 목표 yaw 기준 좌표계 재정의
      delta_yaw = addAngle(yaw_target, -yaw_temp);

    int angle = int(
      - (LEFT_RUDDER_ANGLE_MAX - ZERO_RUDDER_ANGLE)
      * cos(delta_yaw * PI / 180)
      + LEFT_RUDDER_ANGLE_MAX;
    setRudderAngle(angle);
}

long getDistance(int idx) {
  digitalWrite(TRIG_PIN[idx], LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN[idx], HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN[idx], LOW);
  long duration = pulseIn(ECHO_PIN[idx], HIGH);
  long distance = (duration1 / 2) / 29.1;
  if (distance > 200)
    distance = 999;
  return distance;
}

void sendValues(SoftwareSerial s) {
  s.println("dist, cds, euler, yaw, count");
  for (int i = 2; i >= 0; i--) {
    s.print(distance[i]);
    s.print(", ");
  }
  for (int i = 2; i >= 0; i--) {
    s.print(cds_val[i]);
    s.print(", ");
  }
  s.print(euler);
  s.print(", ");
  s.print(yaw);
  s.print(", ");
  s.print(count);
  s.println("");
}

////////////////////////////////////////////////////////////////////
/////////////////////// EBIMU FUNCTION /////////////////////////////

int EBimuAsciiParser(float *item, int number_of_item) {
  int n, i;
  int rbytes;
  char *addr;
  int result = 0;
  char sbuf[SBUF_SIZE];
  signed int sbuf_cnt = 0;

  rbytes = Serial3.available();
  for (n = 0; n < rbytes; n++) {
    sbuf[sbuf_cnt] = Serial3.read();
    if (sbuf[sbuf_cnt] == 0x0a) {
      addr = strtok(sbuf, ",");
      for (i = 0; i < number_of_item; i++) {
        item[i] = atof(addr);
        addr = strtok(NULL, ",");
      }

      result = 1;
    } else if (sbuf[sbuf_cnt] == '*') {
      sbuf_cnt = -1;
    }

    sbuf_cnt++;
    if (sbuf_cnt >= SBUF_SIZE)
      sbuf_cnt = 0;
  }

  return result;
}
/////////////////////// EBIMU FUNCTION /////////////////////////////
////////////////////////////////////////////////////////////////////

// legacy functions
float getRemainder(float dividend, float divisor) {
  float temp = dividend / divisor;
  temp = (temp - floor(temp)) * divisor;
  if (temp > 180) {
    return temp - 360.0;
  } else {
    return temp;
  }
}

void goRight(int angle) {
  analogWrite(RPWM, speed);
  analogWrite(LPWM, 0);
  servo.attach(RUDDER_PIN);
  servo.write(angle);
}

void goLeft(int angle) {
  analogWrite(RPWM, speed);
  analogWrite(LPWM, 0);
  servo.attach(RUDDER_PIN);
  servo.write(angle);
}