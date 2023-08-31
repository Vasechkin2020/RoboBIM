
#ifndef LX16A_H
#define LX16A_H

bool flagTest = false;
unsigned long timeTest = millis();

#include <LSServo.h>
LSServo SERVO;

// Номера сервомоторов управляющих платформой
#define ID_SERVO_LP 1 // ID сервомоторов устанавливается так что-бы не совпадали на шине
#define ID_SERVO_RP 2
#define POSITION_MAX_PLATFORM 400
#define POSITION_MIN_PLATFORM 200
// Номера сервомоторов управляющих механизмами
#define ID_SERVO_L 3
#define ID_SERVO_R 4

// Функция инициализации последовательного порта 2 для сервомоторов
void initServo()
{
  Serial.println(String(millis()) + " ====================================== Start initServo ======================================");

  Serial2.begin(115200);
  pinMode(16, INPUT_PULLUP);      // RX
  pinMode(17, OUTPUT_OPEN_DRAIN); // TX
  delay(500);
  SERVO.pSerial = &Serial2;
  delay(500);
  // SERVO.EnableTorque(ID, 1);
  Serial.println(String(millis()) + " End initServo ...");
}
//Проверка на минимум и максимум положений мотора для платформы
uint16_t checkPosLP(uint16_t pos_)
{
  uint16_t max = POSITION_MAX_PLATFORM;
  uint16_t min = POSITION_MIN_PLATFORM;
  uint16_t ret = pos_;
  if (pos_ >= max)
  {
    ret = max;
  }
  if (pos_ <= min)
  {
    ret = min;
  }
  return ret;
}

// Управление левым мотором
void runServo_LP(int pos_)
{
  pos_ = checkPosLP(pos_);
  SERVO.SetPos(ID_SERVO_LP, pos_, 0);
}

// Управление правым мотором
void runServo_RP(int pos_)
{
  pos_ = checkPosLP(pos_);
  pos_ = 1000 - pos_;
  SERVO.SetPos(ID_SERVO_RP, pos_, 0);
}

// // Запрос позиции  правым мотором, ответ можно через 500 микросекунд спрашивать
// void getPosServo_R()
// {
//   SERVO.GetPos(ID_SERVO_RP);
// }
// // Запрос позиции  левым мотором, ответ можно через 500 микросекунд спрашивать
// void getPosServo_L()
// {
//   SERVO.GetPos(ID_SERVO_LP);
// }

// Получение позиции  левым мотором
int16_t readPosServo_L()
{
  int16_t pos;
  SERVO.GetPos(ID_SERVO_LP, pos);
  return pos;
}
// Получение позиции  левым мотором
int16_t readPosServo_R()
{
  int16_t pos;
  SERVO.GetPos(ID_SERVO_RP, pos);
  return pos;
}
// Получение позиции  левым мотором
int16_t readVinServo_L()
{
  uint16_t ret;
  SERVO.GetVin(ID_SERVO_LP, ret);
  return ret;
}

// Функция подьема рук вразнобой
void runServo_1(int time_)
{
  // flag_servo_position_0 = 0; // Снимаем флаг начального положения рук

  // static int flag_servo1 = 0;
  // static double time_servo1 = 0;
  // static double time_delta = 0;

  // if (flag_servo1 == 0)
  // {
  //   runServo_R(1000, time_);
  //   runServo_L(250, time_);
  //   time_servo1 = millis();
  //   flag_servo1 = 1;
  // }
  // else
  // {
  //   time_delta = millis() - time_servo1;
  //   if (time_delta > time_ + 500 && flag_servo1 == 1) // Если прошло более
  //   {
  //     runServo_R(250, time_);
  //     runServo_L(1000, time_);
  //     flag_servo1 = 2;
  //   }
  //   if (time_delta > (time_ + 500) * 2 && flag_servo1 == 2) // Если прошло более
  //   {
  //     flag_servo1 = 0;
  //   }
  // }
}
// Функция установки рук в начальное положение
void runServo_0(int time_)
{
  // if (flag_servo_position_0 == 0) // Если еще не было каманды встать в начальное положение, то выполнячем команду
  // {
  //   runServo_R(250, time_);
  //   runServo_L(1000, time_);
  //   flag_servo_position_0 = 1; // Ставим флаг что мы в начальном положении рук
  // }
}
// Смена ID сервопривода
void changeServoId(uint8_t id_old, uint8_t id_new)
{
  Serial.printf(" Смена ID сервопривода с % i на %i", id_old, id_new);

  u8 id;
  int syntax_check = SERVO.GetID(0xFE, id);
  Serial.print(" Текущий ID = ");
  Serial.println(id);
  Serial.print(" syntax_check = ");
  Serial.println(syntax_check);
  delay(1000);
  SERVO.FlashServoID(id_old, id_new);
  delay(100);
  SERVO.GetID(0xFE, id);
  Serial.print(" Новый ID = ");
  Serial.println(id);
}
// Узнать номер сервомотора, но только когда он один подключен
void getServoId()
{
  uint8_t id;
  int syntax_check = SERVO.GetID(0xFE, id);
  Serial.print(" Текущий ID = ");
  Serial.println(id);
}
void testServo2()
{
  // for (uint16_t i = 0; i < 1000; i = i + 10)
  // {
  //   int16_t pos;
  //   // runServo_L(i, 100);
  //   SERVO.SetPos(ID_SERVO_RP, i, 0);
  //   delay(200);
  //   SERVO.GetPos(ID_SERVO_RP, pos);
  //   // getPosServo_L();
  //   // delayMicroseconds(500);
  //   //  int16_t rrr = readPosServo_L();
  //   //  int16_t vin = readVinServo_L();
  //   Serial.print(" i= ");
  //   Serial.print(i);
  //   Serial.print(" pos= ");
  //   Serial.print(pos);
  //   // delayMicroseconds(500);
  //   // Serial.print(" vin= ");
  //   // Serial.print(vin);
  //   Serial.println("");
  // }
  // runServo_R(0, 1000);

  // Serial.println(" runServo_L(0,1000) ");
  // delay(9999);
}
void setStartPosition(uint16_t leftPos_, uint32_t rightPos_)
{
  Serial.printf(" ============================================ setStartPosition ============================================\n");
  Serial.printf("Установка сервомоторов платформы в начальную позицию. Левый = %i Правый= %i . \n",leftPos_,rightPos_);
  runServo_LP(leftPos_);
  runServo_RP(rightPos_);
}

// Начальная проверка сервомоторов

void testServo()
{
  Serial.println(String(millis()) + " Up right handle ...");
  // runServo_L(200);
  // delay(5000);
  // runServo_L(1000);
  // delay(5000);
  // runServo_L_plat(0);
  // delay(1000);
  // Serial.println(String(millis()) + " Up left handle ...");
  // runServo_L(1000, 1000);
  // delay(1000);
  // Serial.println(String(millis()) + " Down right handle ...");
  // runServo_R(0, 1000);
  // delay(1000);
  // Serial.println(String(millis()) + " Down left handle ...");
  // runServo_L(0, 1000);
  // delay(1000);
  // Serial.println(String(millis()) + " End position ...");
  // runServo_L(250, 1000);
  // runServo_R(250, 1000); // SERVO.GetPos(ID_SERVO_L);
  // delay(1000);
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

void servoTest()
{
  // if (millis() - timeTest > 6000 && flagTest == true)
  // {
  //   timeTest = millis();
  //   flagTest = false;
  //   runServo_L(0, 2000);
  //   //runServo_R(0, 2000);
  //   Serial.println("servoTest 0");
  // }
  // if (millis() - timeTest > 3000 && flagTest == false)
  // {
  //   flagTest = true;
  //   runServo_L(1000, 2000);
  //   //runServo_R(1000, 2000);
  //   Serial.println("servoTest 1000");
  // }
}

#endif