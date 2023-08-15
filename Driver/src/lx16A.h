
#ifndef LX16A_H
#define LX16A_H

#include <LSServo.h>
LSServo SERVO;

#define ID_SERVO_L 2 // ID сервомоторов устанавливается так что-бы не совпадали на шине
#define ID_SERVO_R 1

//Функция инициализации последовательного порта 2 для сервомоторов
void initServo()
{
  Serial.println(String(millis()) + " Start initServo ...");

  Serial2.begin(115200);
  pinMode(16, INPUT_PULLUP);      // RX
  pinMode(17, OUTPUT_OPEN_DRAIN); // TX
  delay(500);
  SERVO.pSerial = &Serial2;
  delay(500);
  // SERVO.EnableTorque(ID, 1);
  Serial.println(String(millis()) + " End initServo ...");
}
//Управление левым мотором
void runServo_L(int pos_, int time_)
{
  pos_ = 1000 - pos_;
  SERVO.SetPos(ID_SERVO_L, pos_, time_);
}
//Управление правым мотором
void runServo_R(int pos_, int time_)
{
  SERVO.SetPos(ID_SERVO_R, pos_, time_);
}
// Получение позиции правым мотором
int getPosServo_R()
{
  return SERVO.GetPos(ID_SERVO_R);
}
//Получение позиции  левым мотором
int getPosServo_L()
{
  return (1000 - SERVO.GetPos(ID_SERVO_L));
}
// Функция подьема рук вразнобой
void runServo_1(int time_)
{
  flag_servo_position_0 = 0; // Снимаем флаг начального положения рук

  static int flag_servo1 = 0;
  static double time_servo1 = 0;
  static double time_delta = 0;

  if (flag_servo1 == 0)
  {
    runServo_R(1000, time_);
    runServo_L(250, time_);
    time_servo1 = millis();
    flag_servo1 = 1;
  }
  else
  {
    time_delta = millis() - time_servo1;
    if (time_delta > time_ + 500 && flag_servo1 == 1) // Если прошло более
    {
      runServo_R(250, time_);
      runServo_L(1000, time_);
      flag_servo1 = 2;
    }
    if (time_delta > (time_ + 500) * 2 && flag_servo1 == 2) // Если прошло более
    {
      flag_servo1 = 0;
    }
  }
}
// Функция установки рук в начальное положение
void runServo_0(int time_)
{
  if (flag_servo_position_0 == 0) // Если еще не было каманды встать в начальное положение, то выполнячем команду
  {
    runServo_R(250, time_);
    runServo_L(1000, time_);
    flag_servo_position_0 = 1;  // Ставим флаг что мы в начальном положении рук
  }
}
// Начальная проверка сервомоторов
void testServo()
{
  Serial.println(String(millis()) + " Up right handle ...");
  runServo_R(1000, 1000);
  delay(1000);
  Serial.println(String(millis()) + " Up left handle ...");
  runServo_L(1000, 1000);
  delay(1000);
  Serial.println(String(millis()) + " Down right handle ...");
  runServo_R(0, 1000);
  delay(1000);
  Serial.println(String(millis()) + " Down left handle ...");
  runServo_L(0, 1000);
  delay(1000);
  Serial.println(String(millis()) + " End position ...");
  runServo_L(250, 1000);
  runServo_R(250, 1000); // SERVO.GetPos(ID_SERVO_L);
  delay(1000);
  // delayMicroseconds(500);
  // SERVO.ReadPos(ID_SERVO_L, My_pos);
  // delayMicroseconds(1000);
  // printf(" %u %i \n", millis(), My_pos);

  // SERVO.SetPos(ID_SERVO_L, 333, 1000);
  // SERVO.SetPos(ID_SERVO_R, 333, 1000);
  // runServo_L(0, 1000);
  // runServo_R(0, 1000);
  // delay(2000);

  // delay(2000);
  // SERVO.GetPos(ID_SERVO_L);
  // delayMicroseconds(500);
  // SERVO.ReadPos(ID_SERVO_L, My_pos);
  // delayMicroseconds(1000);
  // printf(" %u %i \n", millis(), My_pos);

  // SERVO.SetPos(ID2, 500, 1800);
}

#endif