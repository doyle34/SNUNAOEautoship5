#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <SimpleTimer.h>

// 1 analog pin for cds
// 2 digital pin for sonar
#define BAUD_RATE 115200
#define BT_BAUD_RATE 9600
#define SBUF_SIZE 64
#define Initial_Angle 95

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
//int EBI_TX_PIN = 14;
//int EBI_RX_PIN = 15;

int speed = 35;
long distance[3] = {200, 200, 200}; // right, center, left
int cds_val[3];
int count = 0;
int dist_crit = 120; // critical distance
int brightlim = 400;

float euler = 0;      // euler is EBimu sensor value. range = (-180, 180)
float euler_init = 0; // indicates euler angle when ship starts operating
float yaw = 0;        // yaw is measured w.r.t euler_init. range = (-180, 180)
float yaw_temp = 0;   // temporal yaw value used when turning

Servo servo;
SoftwareSerial BTSerial(BT_TX_PIN, BT_RX_PIN);
LiquidCrystal_I2C lcd(0x27, 16, 2);

SimpleTimer timer;

////////////////////////////////////////////////////////////////////
/////////////////////// EBIMU FUNCTION /////////////////////////////

int EBimuAsciiParser(float *item, int number_of_item)
{
  int n, i;
  int rbytes;
  char *addr;
  int result = 0;
  char sbuf[SBUF_SIZE];
  signed int sbuf_cnt = 0;

  rbytes = Serial3.available();
  for (n = 0; n < rbytes; n++)
  {
    sbuf[sbuf_cnt] = Serial3.read();
    if (sbuf[sbuf_cnt] == 0x0a)
    {
      addr = strtok(sbuf, ",");
      for (i = 0; i < number_of_item; i++)
      {
        item[i] = atof(addr);
        addr = strtok(NULL, ",");
      }

      result = 1;
    }
    else if (sbuf[sbuf_cnt] == '*')
    {
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

void setup()
{
  Serial.begin(BAUD_RATE);
  servo.attach(RUDDER_PIN);
  servo.write(91);
  BTSerial.begin(BT_BAUD_RATE);
  Serial3.begin(BAUD_RATE);
  lcd.begin();
  lcd.backlight();


  for (int i = 0; i < 3; i++)
  {
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
  gofront();
  delay(500);

  delay(500);

  timer.setInterval(100, updateSensorValues);
}

void loop()
{
  timer.run();

  switch (count)
  {
  case 0:
    if (distance[1] < distancelim)
    {
      turnLeft();
      count++;
    }
    break;
  case 1:
    if (distance[0] > 100 || distance[1] < 120)
    {
      turnRight();
      count++;
    }
    break;
  case 2:
    if (distance[0] > 100 || distance[1] < 120)
    {
      turnRight(); //대충 우회전 하라는 뜻
      count++;
    }
    break;
  case 3:
    if (distance[2] > 120 || distance[1] < 120)
    {
      turnLeft(); //대충 좌횟=전 하라는뜻
      count++;
    }
    break;
  case 4:
    if (distance[1] < distancelim)
    {
      turnLeft(); //좌회전
      count++;
    }
    break;
  case 5:
    if (distance[0] > 100 || distance[1] < 120)
    {
      turnRight(); //우회전
      count++;
    }
    break;
  case 6:
    if (distance[0] > 100 || distance[1] < 120)
    {
      turnRight(); //대충 우회전 하라는 뜻
      count++;
    }
    break;
  case 7:
    if (distance[2] > 120 || distance[1] < 120)
    {
      turnLeft();
      count++;
    }

    break;
  }
  gofront();
  delay(300);
}

void gofront()
{
  analogWrite(RPWM, speed);
  analogWrite(LPWM, 0);
  servo.attach(RUDDER_PIN);
  servo.write(Initial_Angle);
  delay(300);
  servo.detach();
}

void gobackward()
{
  analogWrite(RPWM, 0);
  analogWrite(LPWM, speed);
  servo.attach(RUDDER_PIN);
  servo.write(Initial_Angle);
  delay(300);
  servo.detach();
}

void goright(int angle)
{
  analogWrite(RPWM, speed);
  analogWrite(LPWM, 0);
  if (angle < 20)
  {
    angle = 20;
  }
  servo.attach(RUDDER_PIN);
  ;
  servo.write(angle);
}

void goleft(int angle)
{
  analogWrite(RPWM, speed);
  analogWrite(LPWM, 0);
  servo.attach(RUDDER_PIN);
  servo.write(angle);
}

void motorstop()
{
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  servo.attach(RUDDER_PIN);
  servo.write(91);
  delay(300);
  servo.detach();
}

void updateSensorValues()
{ /*
  update sensor values (distance, cds, euler)
  */
  for (int i = 0; i < 3; i++)
  {
    distance[i] = getDistance(i);
    cds_val[i] = analogRead(CDS_PIN[i]);
  }
  float euler_temp[3];
  EBimuAsciiParser(euler_temp, 3);
  euler = euler_temp[2];
}

void setInitEuler()
{
  float temp = 0;
  for (int i = 0; i < 10; i++)
  {
    temp += getYaw();
    delay(100);
  }
  euler_init = temp * 0.1; // mean of first 10 values
}

float getYaw(float ref_angle)
{ /*
  calculate yaw angle w.r.t ref_angle
  if ref_angle == euler_init,
    return ordinary yaw (relative direction w.r.t initial direction)
  if ref_angle == yaw_temp,
    return yaw for rotation (which will be iterated until yaw == -90 or +90)
  
  ALWAYS RETURN VALUES BETWEEN (-180, 180)
  */
  yaw = euler - ref_angle;
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
  return yaw;
}

float getRemainder(float dividend, float divisor)
{
  float temp = dividend / divisor;
  temp = (temp - floor(temp)) * divisor;
  if (temp > 180)
  {
    return temp - 360.0;
  }
  else
  {
    return temp;
  }
}

void turnRight()
{ // turn until yaw is 90
  Serial.println("TurnRight");
  updateDisp();
  while (abs(getRemainder((yaw - yaw_temp), 360.0) - 90) > 1)
  {
    int angle = int(90 - 90 * cos(((yaw - yaw_temp) * 71) / 4068.0));
    goright(angle);
    updateDisp();
    delay(100);
  }
  yaw_temp = yaw;
  Serial.println("");
}

void turnLeft()
{ // turn untill yaw is 270
  Serial.println("TurnLeft");
  updateDisp();
  Serial.print("yaw_temp: ");
  Serial.print(yaw_temp);
  Serial.print("\t");
  while (abs(getRemainder((yaw - yaw_temp), 360.0) - (-90)) > 1)
  {
    Serial.print("yaw: ");
    Serial.print(yaw);
    Serial.print("\t");
    int angle = int(90 + 90 * cos(((yaw - yaw_temp) * 71) / 4068.0));
    Serial.print("cos(yaw): ");
    Serial.print(cos(((yaw - yaw_temp) * 71) / 4068.0));
    Serial.print("\t");
    Serial.print("angle: ");
    Serial.print(angle);
    Serial.println("");
    goleft(angle);
    updateDisp();
    delay(100);
  }
  yaw_temp = yaw;
}

long getDistance(int idx)
{
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