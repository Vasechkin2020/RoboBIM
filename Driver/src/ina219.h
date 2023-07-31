#ifndef INA219_H
#define INA219_H

#include <Adafruit_INA219.h>
Adafruit_INA219 Ina219(0x40); // Переменная для обращения к методам класса
Struct_INA stru_Ina219;       // структура где храним считанные данные

//Считывание значений с INA219
void getIna219()
{
  stru_Ina219.busVoltage_V= Ina219.getBusVoltage_V();
  stru_Ina219.current_mA= Ina219.getCurrent_mA();
  stru_Ina219.power_mW= Ina219.getPower_mW();
  stru_Ina219.shuntVoltage_mV= Ina219.getShuntVoltage_mV();
}
// Начальная инициализация датчика
void init_INA219()
{
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  if (!Ina219.begin())
  {
    Serial.println("Failed to find INA219 chip");
    flag_setup = 1;
    return;
    // while (1)
    // {
    //   delay(10);
    // }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  Ina219.setCalibration_32V_2A();
  // To use a slightly lower 16V, 5A range (higher precision on amps):
  // Ina219.setCalibration_16V_5A();

  Serial.println("Measuring voltage and current with INA219 ...");

  Serial.println("Read value Power from memory ...");
}

// Вывод на печать данных датчика которые считали из своей переменной
void printDataIna219()
{
    Serial.print(" Time: ");
    Serial.print(millis() / 1000.0);
    Serial.print(" busVoltage_V: ");
    Serial.print(stru_Ina219.busVoltage_V);
    Serial.print(" V");
    // Serial.print(" shuntVoltage_mV: ");
    // Serial.print(stru_Ina219.shuntVoltage_mV);
    // Serial.print(" mV");
    Serial.print(" current_mA: ");
    Serial.print(stru_Ina219.current_mA);
    Serial.print(" mA");
    Serial.print(" power_mW: ");
    Serial.print(stru_Ina219.power_mW);
    Serial.print(" mW");
    Serial.println("");
}
#endif