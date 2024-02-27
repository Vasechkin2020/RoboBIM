#ifndef LASER80_H
#define LASER80_H

//*********************** ОБЬЯВЛЕНИЕ ФУНКЦИЙ *****************************************
// Функции применимык к конкретному датчику, так как задается адрес датчика
void singleMeasurement(byte addr_);     // Одиночное измерение примерно 800 милисекунд
void continuousMeasurement(byte addr_); // Непрерывное измерение
void stopMeasurement(byte addr_);       // Прекратить измерение
bool setAddress(byte addr_);            // Установка нового адрес на датчике
bool readCache(byte addr_);                  // Считывание данных из буфера датчика результат измерения
void controlLaser(byte status_, byte addr_); // Управление лазером 1- Включен 0-Выключен

// Функции применяются ко всем датчикам на линии и поэтому калибровку и прочее делать с одиночным датчиком
void broadcastMeasurement();                 // Единое измерние. Команда всем подключенным датчикам произвести измерение.Потом его надо считать с каждого датчика

bool setDistanceModification(byte data_); // Модификация дистанции. Думаю что колибровка измерений. Можно в плюс или в минус
bool setTimeInterval(byte data_);         // Установка инрервала вывода значения при настройке. Не понятно что это. 
bool setStartingPoint(byte data_);        // Устанвка точки откоторой считем расстояние. 1- от носа 0 - от зада
void setRange(byte range_);               // Установление максимального диапзона измерений. Возможно 5,10,30,50,80 метров
void setFrequency(byte freq_);            // Установка частоты измерений, задается в герцах 3,5,10,20 только такие частоты
void setResolution(byte reso_);           // Установка разрешения измерения есди 1- то 1 мм, если 2 то 0,1 мм. Непонятно работает ли нет фактически и на чем сказывается (время измерения?)
bool setTestPowerOn(byte data_);          // Установка нужно ли проводить тест датчика после включения питания. Значение 0 или 1


//************************************************************************************
byte calcCs(byte *data_) // Расчет контрольной суммы. Берется массив всех оправляемых данных без последнего байта и суммируется побайтно, потом в бинарном виде инвертируются 1 в нолик и нолик в единицу и потом прибавляется 1
{
    byte ret = 0;
    // Serial.print(sizeof(data_), DEC);
    for (int i = 0; i < (sizeof(data_)); i++)
    {
        ret += data_[i];
        // Serial.print(data_[i], HEX);
        // Serial.print("-");
    }
    // Serial.println(" |");
    // Serial.print(ret, DEC);
    // Serial.print(" ");
    // Serial.print(ret, HEX);
    // Serial.print(" ");
    // Serial.println(ret, BIN);
    ret = (ret ^ 0b11111111) + 1;
    // Serial.print(ret, DEC);
    // Serial.print(" ");
    // Serial.print(ret, HEX);
    // Serial.print(" ");
    // Serial.println(ret, BIN);
    return ret;
}
// Одиночное измерение примерно 800 милисекунд
void singleMeasurement(byte addr_)
{
    byte buf[4] = {addr_, 0x06, 0x02, 0x00}; // Команда без последнего байта, там будет контрольная сумма, а пока 0х00
    buf[3] = calcCs(buf);
    Serial2.write(buf, 4);
}

// Непрерывное измерение
void continuousMeasurement(byte addr_)
{
    byte buf[4] = {addr_, 0x06, 0x03, 0x00};
    buf[3] = calcCs(buf);
    Serial2.write(buf, 4);
}
// Прекратить измерение
void stopMeasurement(byte addr_)
{
    byte buf[4] = {addr_, 0x04, 0x02, 0x00};
    buf[3] = calcCs(buf);
    Serial2.write(buf, 4);
    delay(10);
    buf[0] = Serial2.read();
    buf[1] = Serial2.read();
    buf[2] = Serial2.read();
    buf[3] = Serial2.read();

    if (buf[0] == addr_ && buf[1] == 0x04 && buf[2] == 0x82 && buf[3] == 0xFA)
    {
        printf("stopMeasurement ok \n");
    }
    else
    {
        printf("stopMeasurement ERROR \n");
    }
}

// Единое измерние. Команда всем подключенным датчикам произвести измерение.Потом его надо считать с каждого датчика
void broadcastMeasurement()
{
    byte buf[4] = {0xFA, 0x06, 0x06, 0xFA};
    Serial2.write(buf, 4);
}
// Считывание данных из буфера датчика результат измерения
bool readCache(byte addr_)
{
    while (Serial2.available()) // Очищаем буфер
        Serial2.read();
    byte buf[4] = {addr_, 0x06, 0x07, 0x00};
    buf[3] = calcCs(buf); // Считаем контрольную сумму и записываем последним байтом
    Serial2.write(buf, 4);
    delay(20);
    byte len = 11;
    byte bufRead[len];
    for (int i = 0; i < len; i++)
    {
        bufRead[i] = Serial2.read();
        printf("%X-", bufRead[i]);
    }

    if (bufRead[0] == addr_ && buf[1] == 0x06 && buf[2] == 0x82 && buf[3] == 0x45 && buf[4] == 0x52 && buf[4] == 0x52) // Проверка на ошибку. Возвращает ADDR 06 82"'E' 'R' 'R' '-' '-' '3X' '3X' ”CS
    {
        printf("readMeasurement ERROR \n");
        return false;
    }
    // String rez = bufRead[3] + bufRead[4] + bufRead[5] + "." + bufRead[6] + bufRead[6] + bufRead[8];

    // printf(" rez = %s \n", rez.c_str());
    // Serial.println("rezultat=" + rez);

    return true;
}
// Установка инрервала вывода значения при настройке. Не понятно что это. 
bool setTimeInterval(byte data_)
{
    if (data_ == 0) // через 1 секунду
    {
        byte buf[5] = {0xFA, 0x04, 0x05, 0x00, 0xFC};
        Serial2.write(buf, 5);
    }
    if (data_ == 1) // через 0 секунду
    {
        byte buf[5] = {0xFA, 0x04, 0x05, 0x01, 0xFD};
        Serial2.write(buf, 5);
    }

}
// Установка нового адрес на датчике Широковещательный команда, смотреть что датчик один в этот момент на шине.
bool setAddress(byte addr_)
{
    byte buf[5] = {0xFA, 0x04, 0x01, addr_, 0x00};
    buf[4] = calcCs(buf);
    Serial2.write(buf, 5);
    delay(10);
    buf[0] = Serial2.read();
    buf[1] = Serial2.read();
    buf[2] = Serial2.read();
    buf[3] = Serial2.read();
    if (buf[0] == 0xFA && buf[1] == 0x04 && buf[2] == 0x81 && buf[3] == 0x81)
    {
        // printf("setAddress ok \n");
        return true;
    }
    else
    {
        // printf("setAddress ERROR \n");
        return false;
    }
}
// Модификация дистанции. Думаю что колибровка измерений. Можно в плюс или в минус
bool setDistanceModification(byte data_)
{
    byte sign_;
    if (data_ > 0) // Если корректировка плюсовая
    {
        sign_ = 0x2b;
    }
    else // если минусовая
    {
        sign_ = 0x2d;
    }

    byte buf[6] = {0xFA, 0x04, 0x06, sign_, data_, 0x00};
    buf[5] = calcCs(buf);
    Serial2.write(buf, 6);
}
// Устанвка точки откоторой считем расстояние. 1- от носа 0 - от зада
bool setStartingPoint(byte data_)
{
    if (data_ == 0) //
    {
        byte buf[5] = {0xFA, 0x04, 0x08, 0x00, 0xFA};
        Serial2.write(buf, 5);
    }
    if (data_ == 1) //
    {
        byte buf[5] = {0xFA, 0x04, 0x08, 0x01, 0xF9};
        Serial2.write(buf, 5);
    }
}

// Установка нужно ли проводить тест датчика после включения питания. Значение 0 или 1
bool setTestPowerOn(byte data_)
{
    if (data_ == 0) //
    {
        byte buf[5] = {0xFA, 0x04, 0x0D, 0x00, 0xF5}; // Нет теста
        Serial2.write(buf, 5);
    }
    if (data_ == 1) //
    {
        byte buf[5] = {0xFA, 0x04, 0x0D, 0x01, 0xF4}; // Тестировать
        Serial2.write(buf, 5);
    }
}
// Установление максимального диапзона измерений. Возможно 5,10,30,50,80 метров
void setRange(byte range_)
{
    if (range_ == 5) //
    {
        byte buf[5] = {0xFA, 0x04, 0x09, 0x05, 0xF4};
        Serial2.write(buf, 5);
    }
    if (range_ == 10) //
    {
        byte buf[5] = {0xFA, 0x04, 0x09, 0x0A, 0xEE};
        Serial2.write(buf, 5);
    }
    if (range_ == 30) //
    {
        byte buf[5] = {0xFA, 0x04, 0x09, 0x1E, 0xDB};
        Serial2.write(buf, 5);
    }
    if (range_ == 50) //
    {
        byte buf[5] = {0xFA, 0x04, 0x09, 0x32, 0xC7};
        Serial2.write(buf, 5);
    }
    if (range_ == 80) //
    {
        byte buf[5] = {0xFA, 0x04, 0x09, 0x50, 0xA9};
        Serial2.write(buf, 5);
    }
}

// Установка разрешения измерения есди 1- то 1 мм, если 2 то 0,1 мм. Непонятно работает ли нет фактически и на чем сказывается (время измерения?)
void setResolution(byte reso_)
{
    if (reso_ == 1) //
    {
        byte buf[5] = {0xFA, 0x04, 0x0C, 0x01, 0xF5};
        Serial2.write(buf, 5);
    }
    if (reso_ == 2) //
    {
        byte buf[5] = {0xFA, 0x04, 0x0C, 0x02, 0xF4};
        Serial2.write(buf, 5);
    }
}
// Установка частоты измерений, задается в герцах 3,5,10,20 только такие частоты
void setFrequency(byte freq_)
{
    if (freq_ == 3) //  примерно 3 Hz
    {
        byte buf[5] = {0xFA, 0x04, 0x0A, 0x00, 0xF8};
        Serial2.write(buf, 5);
    }
    if (freq_ == 5) //  фактически 7 Hz
    {
        byte buf[5] = {0xFA, 0x04, 0x0A, 0x05, 0xF3};
        Serial2.write(buf, 5);
    }
    if (freq_ == 10) //  фактически 8 Hz
    {
        byte buf[5] = {0xFA, 0x04, 0x0A, 0x0A, 0xEE};
        Serial2.write(buf, 5);
    }
    if (freq_ == 20) //  фактически 18 Hz
    {
        byte buf[5] = {0xFA, 0x04, 0x0A, 0x14, 0xE4};
        Serial2.write(buf, 5);
    }
}
// Управление лазером 1- Включен 0-Выключен
void controlLaser(byte status_, byte addr_)
{
    if (status_ == 0) //  Выключить
    {
        byte buf[5] = {addr_, 0x06, 0x05, 0x00, 0x00};
        buf[4] = calcCs(buf);
        Serial2.write(buf, 5);
    }
    if (status_ == 1) //  Включить
    {
        byte buf[5] = {addr_, 0x06, 0x05, 0x01, 0x00};
        buf[4] = calcCs(buf);
        Serial2.write(buf, 5);
    }
}
#endif