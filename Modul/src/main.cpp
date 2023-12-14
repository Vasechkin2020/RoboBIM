
#define MOTOR yes
// #define SPI_protocol yes

#include <Arduino.h>
#include <Wire.h>

#include "config.h" // Основной конфигурационный файл с общими настройками

// Файлы с функциями отдельных сущностей
#include "motor.h"
#include "protokolSPI.h"
#include "code.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println(" ");
    Serial.println(String(micros()) + " Start program ESP32_Body ...");

#ifdef MOTOR
    initMotor(); // Начальная инициализация и настройка шаговых моторов
    setSpeed_L(0.3);
    setSpeed_R(0.3);
    delay(3000);
    stopMotor();
#endif

    printf("initTimer_0 \n");
    initTimer_0(); // Запуск таймера 0

#ifdef SPI_protocol
    printf("initSPI_slave \n");
    initSPI_slave(); // Инициализация SPI_slave
    printf("spi_slave_queue_Send \n");
    spi_slave_queue_Send(); // Configure receiver Первый раз закладываем данные чтобы как только мастер к нам обратиться было чем ответить
#endif

    delay(1000);
    Serial.println(String(millis()) + " End SetUp !!!");
}

int a, b;

void loop()
{
    // digitalWrite(PIN_ANALIZ, 1);
    if (flag_setup == 0)
    {
        Led_Blink(PIN_LED_GREEN, 500); // Мигаем светодиодом каждую 1/2 секунду, что-бы было видно что цикл не завис
    }
    else
    {
        Led_Blink(PIN_LED_GREEN, 100); // Мигаем светодиодом каждую 1/2 секунду, что-бы было видно что цикл не завис
    }

#ifdef MOTOR
    movementTime(); // Отслеживание времени движения
    delayMotor();   // Задержка отключения драйверов моторов после остановки
#endif

    //----------------------------- По факту обмена данными с верхним уровнем --------------------------------------
    if (flag_data) // Если обменялись данными
    {
        flag_newData = 1; // Есть новые данные по шине
        flag_data = false;
        // printf("+\n");
        processing_Data(); // Обработка пришедших данных после состоявшегося обмена

        // printf(" Receive id= %i cheksum= %i All obmen= %i bed_time= %i bed_crc= %i", Data2Driver_receive.id, Data2Driver_receive.cheksum, obmen_all, obmen_bed_time, obmen_bed_crc);
        // printf(" command= %i radius= %f speed= %f motor_video_angle= %f \n", Data2Driver_receive.command_body, Data2Driver_receive.radius, Data2Driver_receive.speed, Data2Driver_receive.motor_video_angle);
        // printf(" \n");

        // calculateOdom_enc();    // Подсчет одометрии по энкодерам
        // calculateOdom_imu();    // Подсчет одометрии по imu
        collect_Data();         // Собираем данные в структуре для отправки
        spi_slave_queue_Send(); // Закладываем данные в буфер для передачи(обмена)
    }

    //----------------------------- 10 миллисекунд --------------------------------------
    if (flag_timer_10millisec)
    {
        flag_timer_10millisec = false;

        // Serial.println(String(millis()) + " flag_newData = " + flag_newData);
        // Serial.println(String(millis()) + " flag_newControlData = "+ flag_newControlData);
        if (flag_newData || flag_newControlData) // Выполняем если есть новые данные или по шине или вручную если не будет команд ни по шине не вручную то робот остановиться
        {
            executeCommand();        // Выполнение пришедших команд
            flag_newData = 0;        // Сброс флага
            flag_newControlData = 0; // Сброс флага
        }
    }

    //----------------------------- 50 миллисекунд --------------------------------------
    if (flag_timer_50millisec)
    {
        flag_timer_50millisec = false;
        // setSpeed_time(0.2, 0.2, 1000);
    }

    //----------------------------- 1 секунда --------------------------------------
    if (flag_timer_1sec) // Вызывается каждую секунду
    {
        flag_timer_1sec = false;

        printf(" %f \n", millis() / 1000.0);

        // long a = micros();
        // long b = micros();

        // Serial.print(b - a);
        // Serial.println(" time");

        // printBody();

        // Serial.println("setSpeed_time");
    }

    if (flag_timer_60sec) // Вызывается каждую МИНУТУ
    {
        flag_timer_60sec = false;

        // Печать времени что программа не зависла, закомментировать в реальной работе
        printf(" %f \n", millis() / 1000.0);
    }
    // digitalWrite(PIN_ANALIZ, 0);
}


//***********************************************************************
