#ifndef DS3231_H
#define DS3231_H

DateTime g_time_Ds3231; // Глобальная переменная структуры времени
float g_temp_Ds3231;    // Глобальная переменная температуры чипа

// Начальная инициализация датчика времени
void init_DS3231()
{
    Serial.println("init_DS3231...");
    //set_TCA9548A(multi_line_DS3231);

    // проверка наличия модуля на линии i2c
    if (!rtc.begin())
    {
        Serial.println("DS3231 not found");
        // for (;;)
        //     ;
        delay(2000);
    }
    // ======== УСТАНОВКА ВРЕМЕНИ АВТОМАТИЧЕСКИ ========
    // rtc.setTime(COMPILE_TIME); // установить время == времени компиляции

    // визуально громоздкий, но более "лёгкий" с точки зрения памяти способ установить время компиляции
    //   rtc.setTime(BUILD_SEC, BUILD_MIN, BUILD_HOUR, BUILD_DAY, BUILD_MONTH, BUILD_YEAR);

    //   if (rtc.lostPower()) {            // выполнится при сбросе батарейки
    //     Serial.println("lost power!");
    // тут можно однократно установить время == времени компиляции

    // ======== УСТАНОВКА ВРЕМЕНИ ВРУЧНУЮ ========
    // установить время вручную можно двумя способами (подставить реальные числа)
    // rtc.setTime(SEC, MIN, HOUR, DAY, MONTH, YEAR);
    // rtc.setHMSDMY(HOUR, MIN, SEC, DAY, MONTH, YEAR);

    // также можно установить время через DateTime
    /*
    DateTime now;
    now.second = 0;
    now.minute = 10;
    now.hour = 50;
    now.date = 2;
    now.month = 9;
    now.year = 2021;

    rtc.setTime(now);  // загружаем в RTC
    */
}

void printTime_DS3231()
{
    Serial.print(" g_time_Ds3231 = ");
    Serial.print(g_time_Ds3231.hour);
    Serial.print(":");
    Serial.print(g_time_Ds3231.minute);
    Serial.print(":");
    Serial.print(g_time_Ds3231.second);
    Serial.print(" ");
    Serial.print(g_time_Ds3231.day);
    Serial.print(" ");
    Serial.print(g_time_Ds3231.date);
    Serial.print("/");
    Serial.print(g_time_Ds3231.month);
    Serial.print("/");
    Serial.println(g_time_Ds3231.year);
}

void getTime_DS3231()
{
    //set_TCA9548A(multi_line_DS3231);
    // получаем все данные в структуру и используем их этот способ быстрее и "легче" вызова отдельных get-функций
    g_time_Ds3231 = rtc.getTime();
}

void getTemperature_DS3231()
{
    //set_TCA9548A(multi_line_DS3231);
    g_temp_Ds3231 = rtc.getTemperatureFloat();
    Serial.print(" g_temp_Ds3231= ");
    Serial.println(g_temp_Ds3231);
}

#endif