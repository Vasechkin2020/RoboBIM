/*
 * icm20948.h
 *
 *  Created on: Dec 26, 2020
 *      Author: mokhwasomssi
 */

#ifndef __ICM20948_H__
#define __ICM20948_H__

#include <stdbool.h>

#include <Arduino.h>
#include "i2c_my.h"

SXyz icm20948_gyro;	 // Данные с гироскопа
SXyz icm20948_accel; // Данные с акселерометра

static float gyro_scale_factor;
static float accel_scale_factor;

#define ICM20948_I2C_ADDRESS 0x69 // Адрес ICM-20948 (0x68 или 0x69, сдвинутый влево)

/* Defines */
#define READ 0x80
#define WRITE 0x00

/* Typedefs */
// Формирование значения для регистра REG_BANK_SEL
// Значение банка (ub) сдвигается на 4 бита влево, так как биты [5:4] в REG_BANK_SEL задают номер банка
// Например, ub_0 = 0x00, ub_1 = 0x10, ub_2 = 0x20, ub_3 = 0x30
typedef enum
{
	ub_0 = 0 << 4,
	ub_1 = 1 << 4,
	ub_2 = 2 << 4,
	ub_3 = 3 << 4
} userbank;

typedef enum
{
	_250dps,
	_500dps,
	_1000dps,
	_2000dps
} gyro_full_scale;

typedef enum
{
	_2g,
	_4g,
	_8g,
	_16g
} accel_full_scale;

typedef struct
{
	float x;
	float y;
	float z;
} axises;

typedef enum
{
	power_down_mode = 0,
	single_measurement_mode = 1,
	continuous_measurement_10hz = 2,
	continuous_measurement_20hz = 4,
	continuous_measurement_50hz = 6,
	continuous_measurement_100hz = 8
} operation_mode;

/* Структура для bias */
typedef struct
{
	float b_x;
	float b_y;
	float b_z;
} StructBias;
// Калибровочные коэффициенты
StructBias aBias = {0, 0, 0}; // для акселерометра
StructBias gBias = {0, 0, 0}; // для гироскопа

/* Структура для scale */
typedef struct
{
	float s_x;
	float s_y;
	float s_z;
} StructScale;

/* Main Functions */

void icm20948_DisableLPMMode();							 // Отключение режима низкого энергопотребления (LPM) для нормальной работы датчика
void icm20948_readData(SXyz *dataAccel, SXyz *dataGyro); // Опрашиваем датчик
void calcBufferICM(SXyz *dataAccel, SXyz *dataGyro);	 // Обработка буфера после считывания данных по шине
void calc_bias_accel_gyro (uint16_t samples); // Функция для получения усредненных данных акселерометра и гироскопа для расчета bias
void calc_average_data(uint16_t samples); // Функция усредненных знаяений акселерометра и гироскопа 

void enable_i2c_mode(void); // Отключение SPI и включение I2C

// Инициализация ICM-20948
// Настраивает основные параметры датчика (сброс, включение I2C, выбор источника тактирования, настройка фильтров и т.д.)
// Вызывается перед началом работы с акселерометром и гироскопом
void icm20948_init();

// Инициализация магнитометра AK09916
// Настраивает магнитометр, подключённый через внутренний I2C-мастер ICM-20948
// Включает I2C-мастер ICM-20948 и устанавливает параметры работы AK09916
void ak09916_init();

// Чтение необработанных данных гироскопа (16-битные значения с АЦП)
// Параметры:
// - data: Указатель на структуру axises (x, y, z), куда записываются необработанные данные для каждой оси (в единицах АЦП)
// Используется для получения сырых данных с гироскопа без преобразования в физические единицы
void icm20948_gyro_read(axises *data);

// Чтение необработанных данных акселерометра (16-битные значения с АЦП)
// Параметры:
// - data: Указатель на структуру axises (x, y, z), куда записываются необработанные данные для каждой оси (в единицах АЦП)
// Используется для получения сырых данных с акселерометра без преобразования
void icm20948_accel_read(axises *data);

// Чтение необработанных данных магнитометра AK09916 (16-битные значения с АЦП)
// Параметры:
// - data: Указатель на структуру axises (x, y, z), куда записываются необработанные данные магнитометра
// Возвращает: true, если данные успешно прочитаны, false в случае ошибки (например, магнитометр не готов)
// Используется для получения сырых данных магнитного поля
bool ak09916_mag_read(axises *data);

// Преобразование необработанных данных гироскопа в угловую скорость (градусы в секунду, dps)
// Параметры:
// - data: Указатель на структуру axises (x, y, z), куда записываются значения в dps
// Преобразует 16-битные значения АЦП в физические единицы на основе текущей шкалы (full scale) гироскопа
void icm20948_gyro_read_dps(axises *data);

// Преобразование необработанных данных акселерометра в ускорение (g)
// Параметры:
// - data: Указатель на структуру axises (x, y, z), куда записываются значения в g (ускорение силы тяжести)
// Преобразует 16-битные значения АЦП в физические единицы на основе текущей шкалы акселерометра
void icm20948_accel_read_g(axises *data);

// Преобразование необработанных данных магнитометра в магнитное поле (микротесла, uT)
// Параметры:
// - data: Указатель на структуру axises (x, y, z), куда записываются значения в uT
// Возвращает: true, если данные успешно прочитаны и преобразованы, false в случае ошибки
// Преобразует сырые данные магнитометра в физические единицы
bool ak09916_mag_read_uT(axises *data);

// Проверка идентификатора ICM-20948
// Читает регистр WHO_AM_I (адрес 0x00, банк 0), который должен вернуть 0xEA
// Возвращает: true, если прочитано 0xEA (подтверждает, что устройство ICM-20948), false в случае ошибки
// Используется для проверки связи с датчиком
void icm20948_who_am_i();

// Проверка идентификатора магнитометра AK09916
// Читает регистр WIA2 (адрес 0x01) через внутренний I2C-мастер ICM-20948, который должен вернуть 0x09
// Возвращает: true, если прочитано 0x09 (подтверждает, что магнитометр AK09916), false в случае ошибки
// Используется для проверки связи с магнитометром
bool ak09916_who_am_i();

// Сброс ICM-20948
// Устанавливает бит DEV_RESET в регистре PWR_MGMT_1 (адрес 0x06, банк 0) для выполнения аппаратного сброса
// После сброса требуется задержка (около 50 мс) для стабилизации
// Используется для возврата датчика в начальное состояние
void icm20948_device_reset();

// Программный сброс магнитометра AK09916
// Отправляет команду сброса через внутренний I2C-мастер ICM-20948 в регистр CNTL3 (адрес 0x32)
// Используется для возврата магнитометра в начальное состояние
void ak09916_soft_reset();

// Вывод ICM-20948 из спящего режима
// Сбрасывает бит SLEEP в регистре PWR_MGMT_1 (адрес 0x06, банк 0) для активации датчика
// Используется для включения акселерометра и гироскопа после спящего режима
void icm20948_wakeup();

// Перевод ICM-20948 в спящий режим
// Устанавливает бит SLEEP в регистре PWR_MGMT_1 (адрес 0x06, банк 0) для снижения энергопотребления
// Используется для отключения акселерометра и гироскопа, когда они не нужны
void icm20948_sleep();

// Включение SPI slave-режима для ICM-20948
// Настраивает ICM-20948 как SPI-ведомое устройство (не используется в вашем случае, так как вы работаете по I2C)
// Используется для специфичных приложений, где ICM-20948 выступает как подчинённое устройство по SPI
void icm20948_spi_slave_enable();

// Сброс внутреннего I2C-мастера ICM-20948
// Устанавливает бит I2C_MST_RST в регистре USER_CTRL (адрес 0x03, банк 0) для сброса I2C-мастера
// Используется для перезапуска внутреннего I2C-мастера, который управляет связью с магнитометром AK09916
void icm20948_i2c_master_reset();

// Включение внутреннего I2C-мастера ICM-20948
// Устанавливает бит I2C_MST_EN в регистре USER_CTRL (адрес 0x03, банк 0) для активации I2C-мастера
// Необходимо для связи с магнитометром AK09916 через внутренний I2C-интерфейс ICM-20948
void icm20948_i2c_master_enable();
void icm20948_i2c_master_disable();

// Настройка частоты тактирования внутреннего I2C-мастера
// Параметры:
// - config: Значение от 0 до 15, определяющее частоту I2C-мастера (см. datasheet, раздел 6.5)
// Используется для настройки скорости связи между ICM-20948 и AK09916 (обычно ~400 кГц)
void icm20948_i2c_master_clk_frq(uint8_t config);

// Выбор источника тактирования для ICM-20948
// Параметры:
// - source: Значение для регистра PWR_MGMT_1 (CLKSEL, биты [2:0]), задающее источник тактирования
// Используется для выбора лучшего источника тактирования (обычно авто-выбор для минимального дрейфа)
void icm20948_clock_source(uint8_t source);

// Включение выравнивания частоты дискретизации (ODR align)
// Устанавливает бит ODR_ALIGN_EN в регистре ODR_ALIGN_EN (адрес 0x09, банк 0)
// Используется для синхронизации частоты вывода данных (ODR) между акселерометром и гироскопом
void icm20948_odr_align_enable();

// Настройка низкочастотного фильтра для гироскопа
// Параметры:
// - config: Значение от 0 до 7, задающее частоту среза фильтра (см. datasheet, раздел 6.8)
// Используется для уменьшения шума в данных гироскопа
void icm20948_gyro_low_pass_filter(uint8_t config);

// Настройка низкочастотного фильтра для акселерометра
// Параметры:
// - config: Значение от 0 до 7, задающее частоту среза фильтра (см. datasheet, раздел 6.8)
// Используется для уменьшения шума в данных акселерометра
void icm20948_accel_low_pass_filter(uint8_t config);

// Установка делителя частоты дискретизации для гироскопа
// Параметры:
// - divider: Значение делителя (Output Data Rate = 1.125 кГц / (1 + divider))
// Используется для настройки частоты вывода данных гироскопа (например, 100 Гц при divider = 10)
void icm20948_gyro_sample_rate_divider(uint8_t divider);

// Установка делителя частоты дискретизации для акселерометра
// Параметры:
// - divider: Значение делителя (Output Data Rate = 1.125 кГц / (1 + divider))
// Используется для настройки частоты вывода данных акселерометра
void icm20948_accel_sample_rate_divider(uint16_t divider);

// Настройка режима работы магнитометра AK09916
// Параметры:
// - mode: Режим работы (например, одиночный замер, непрерывный режим, см. datasheet AK09916)
// Используется для установки режима работы магнитометра (например, периодические измерения)
void ak09916_operation_mode_setting(operation_mode mode);

// Калибровка гироскопа
// Выполняет программную калибровку для устранения смещений (offset) в данных гироскопа
// Должна выполняться перед выбором полной шкалы (full scale) для точных измерений
void icm20948_gyro_calibration();

// Калибровка акселерометра
// Выполняет программную калибровку для устранения смещений в данных акселерометра
// Должна выполняться перед выбором полной шкалы для точных измерений
void icm20948_accel_calibration();

// Выбор полной шкалы для гироскопа
// Параметры:
// - full_scale: Значение шкалы (например, ±250 dps, ±500 dps, см. datasheet, раздел 6.8)
// Используется для установки диапазона измерений гироскопа (влияет на чувствительность)
void icm20948_gyro_full_scale_select(gyro_full_scale full_scale);

// Выбор полной шкалы для акселерометра
// Параметры:
// - full_scale: Значение шкалы (например, ±2g, ±4g, см. datasheet, раздел 6.8)
// Используется для установки диапазона измерений акселерометра (влияет на чувствительность)
void icm20948_accel_full_scale_select(accel_full_scale full_scale);

// Включаем bypass режим для первоначальной настройки магнитометра
void icm20948_bypass_en();
// Выключаем bypass режим для первоначальной настройки магнитометра
void icm20948_bypass_disable();
//***********************************************************************************************************
// void icm20948_init();
// void ak09916_init();

// // 16 bits ADC value. raw data.
// void icm20948_gyro_read(axises *data);
// void icm20948_accel_read(axises *data);
// bool ak09916_mag_read(axises *data);

// // Convert 16 bits ADC value to their unit.
// void icm20948_gyro_read_dps(axises *data);
// void icm20948_accel_read_g(axises *data);
// bool ak09916_mag_read_uT(axises *data);

// /* Sub Functions */
// bool icm20948_who_am_i();
// bool ak09916_who_am_i();

// void icm20948_device_reset();
// void ak09916_soft_reset();

// void icm20948_wakeup();
// void icm20948_sleep();

// void icm20948_spi_slave_enable();

// void icm20948_i2c_master_reset();
// void icm20948_i2c_master_enable();
// void icm20948_i2c_master_clk_frq(uint8_t config); // 0 - 15

// void icm20948_clock_source(uint8_t source);
// void icm20948_odr_align_enable();

// void icm20948_gyro_low_pass_filter(uint8_t config);	 // 0 - 7
// void icm20948_accel_low_pass_filter(uint8_t config); // 0 - 7

// // Output Data Rate = 1.125kHz / (1 + divider)
// void icm20948_gyro_sample_rate_divider(uint8_t divider);
// void icm20948_accel_sample_rate_divider(uint16_t divider);
// void ak09916_operation_mode_setting(operation_mode mode);

// // Calibration before select full scale.
// void icm20948_gyro_calibration();
// void icm20948_accel_calibration();

// void icm20948_gyro_full_scale_select(gyro_full_scale full_scale);
// void icm20948_accel_full_scale_select(accel_full_scale full_scale);

/* ICM-20948 Registers */
#define ICM20948_ID 0xEA
#define REG_BANK_SEL 0x7F

// USER BANK 0
#define B0_WHO_AM_I 0x00
#define B0_USER_CTRL 0x03
#define B0_LP_CONFIG 0x05
#define B0_PWR_MGMT_1 0x06
#define B0_PWR_MGMT_2 0x07
#define B0_INT_PIN_CFG 0x0F
#define B0_INT_ENABLE 0x10
#define B0_INT_ENABLE_1 0x11
#define B0_INT_ENABLE_2 0x12
#define B0_INT_ENABLE_3 0x13
#define B0_I2C_MST_STATUS 0x17
#define B0_INT_STATUS 0x19
#define B0_INT_STATUS_1 0x1A
#define B0_INT_STATUS_2 0x1B
#define B0_INT_STATUS_3 0x1C
#define B0_DELAY_TIMEH 0x28
#define B0_DELAY_TIMEL 0x29
#define B0_ACCEL_XOUT_H 0x2D
#define B0_ACCEL_XOUT_L 0x2E
#define B0_ACCEL_YOUT_H 0x2F
#define B0_ACCEL_YOUT_L 0x30
#define B0_ACCEL_ZOUT_H 0x31
#define B0_ACCEL_ZOUT_L 0x32
#define B0_GYRO_XOUT_H 0x33
#define B0_GYRO_XOUT_L 0x34
#define B0_GYRO_YOUT_H 0x35
#define B0_GYRO_YOUT_L 0x36
#define B0_GYRO_ZOUT_H 0x37
#define B0_GYRO_ZOUT_L 0x38
#define B0_TEMP_OUT_H 0x39
#define B0_TEMP_OUT_L 0x3A
#define B0_EXT_SLV_SENS_DATA_00 0x3B
#define B0_EXT_SLV_SENS_DATA_01 0x3C
#define B0_EXT_SLV_SENS_DATA_02 0x3D
#define B0_EXT_SLV_SENS_DATA_03 0x3E
#define B0_EXT_SLV_SENS_DATA_04 0x3F
#define B0_EXT_SLV_SENS_DATA_05 0x40
#define B0_EXT_SLV_SENS_DATA_06 0x41
#define B0_EXT_SLV_SENS_DATA_07 0x42
#define B0_EXT_SLV_SENS_DATA_08 0x43
#define B0_EXT_SLV_SENS_DATA_09 0x44
#define B0_EXT_SLV_SENS_DATA_10 0x45
#define B0_EXT_SLV_SENS_DATA_11 0x46
#define B0_EXT_SLV_SENS_DATA_12 0x47
#define B0_EXT_SLV_SENS_DATA_13 0x48
#define B0_EXT_SLV_SENS_DATA_14 0x49
#define B0_EXT_SLV_SENS_DATA_15 0x4A
#define B0_EXT_SLV_SENS_DATA_16 0x4B
#define B0_EXT_SLV_SENS_DATA_17 0x4C
#define B0_EXT_SLV_SENS_DATA_18 0x4D
#define B0_EXT_SLV_SENS_DATA_19 0x4E
#define B0_EXT_SLV_SENS_DATA_20 0x4F
#define B0_EXT_SLV_SENS_DATA_21 0x50
#define B0_EXT_SLV_SENS_DATA_22 0x51
#define B0_EXT_SLV_SENS_DATA_23 0x52
#define B0_FIFO_EN_1 0x66
#define B0_FIFO_EN_2 0x67
#define B0_FIFO_RST 0x68
#define B0_FIFO_MODE 0x69
#define B0_FIFO_COUNTH 0X70
#define B0_FIFO_COUNTL 0X71
#define B0_FIFO_R_W 0x72
#define B0_DATA_RDY_STATUS 0x74
#define B0_FIFO_CFG 0x76

// USER BANK 1
#define B1_SELF_TEST_X_GYRO 0x02
#define B1_SELF_TEST_Y_GYRO 0x03
#define B1_SELF_TEST_Z_GYRO 0x04
#define B1_SELF_TEST_X_ACCEL 0x0E
#define B1_SELF_TEST_Y_ACCEL 0x0F
#define B1_SELF_TEST_Z_ACCEL 0x10
#define B1_XA_OFFS_H 0x14
#define B1_XA_OFFS_L 0x15
#define B1_YA_OFFS_H 0x17
#define B1_YA_OFFS_L 0x18
#define B1_ZA_OFFS_H 0x1A
#define B1_ZA_OFFS_L 0x1B
#define B1_TIMEBASE_CORRECTION_PLL 0x28

// USER BANK 2
#define B2_GYRO_SMPLRT_DIV 0x00
#define B2_GYRO_CONFIG_1 0x01
#define B2_GYRO_CONFIG_2 0x02
#define B2_XG_OFFS_USRH 0x03
#define B2_XG_OFFS_USRL 0x04
#define B2_YG_OFFS_USRH 0x05
#define B2_YG_OFFS_USRL 0x06
#define B2_ZG_OFFS_USRH 0x07
#define B2_ZG_OFFS_USRL 0x08
#define B2_ODR_ALIGN_EN 0x09
#define B2_ACCEL_SMPLRT_DIV_1 0x10
#define B2_ACCEL_SMPLRT_DIV_2 0x11
#define B2_ACCEL_INTEL_CTRL 0x12
#define B2_ACCEL_WOM_THR 0x13
#define B2_ACCEL_CONFIG 0x14
#define B2_ACCEL_CONFIG_2 0x15
#define B2_FSYNC_CONFIG 0x52
#define B2_TEMP_CONFIG 0x53
#define B2_MOD_CTRL_USR 0X54

// USER BANK 3
#define B3_I2C_MST_ODR_CONFIG 0x00
#define B3_I2C_MST_CTRL 0x01
#define B3_I2C_MST_DELAY_CTRL 0x02
#define B3_I2C_SLV0_ADDR 0x03
#define B3_I2C_SLV0_REG 0x04
#define B3_I2C_SLV0_CTRL 0x05
#define B3_I2C_SLV0_DO 0x06
#define B3_I2C_SLV1_ADDR 0x07
#define B3_I2C_SLV1_REG 0x08
#define B3_I2C_SLV1_CTRL 0x09
#define B3_I2C_SLV1_DO 0x0A
#define B3_I2C_SLV2_ADDR 0x0B
#define B3_I2C_SLV2_REG 0x0C
#define B3_I2C_SLV2_CTRL 0x0D
#define B3_I2C_SLV2_DO 0x0E
#define B3_I2C_SLV3_ADDR 0x0F
#define B3_I2C_SLV3_REG 0x10
#define B3_I2C_SLV3_CTRL 0x11
#define B3_I2C_SLV3_DO 0x12
#define B3_I2C_SLV4_ADDR 0x13
#define B3_I2C_SLV4_REG 0x14
#define B3_I2C_SLV4_CTRL 0x15
#define B3_I2C_SLV4_DO 0x16
#define B3_I2C_SLV4_DI 0x17

/* AK09916 Registers */
#define AK09916_ID 0x09
#define MAG_SLAVE_ADDR 0x0C

#define MAG_WIA2 0x01
#define MAG_ST1 0x10
#define MAG_HXL 0x11
#define MAG_HXH 0x12
#define MAG_HYL 0x13
#define MAG_HYH 0x14
#define MAG_HZL 0x15
#define MAG_HZH 0x16
#define MAG_ST2 0x18
#define MAG_CNTL2 0x31
#define MAG_CNTL3 0x32
#define MAG_TS1 0x33
#define MAG_TS2 0x34

#endif /* __ICM20948_H__ */

#define multi_line_ICM 2 // На какой линии мультиплексора находится датчик BNO055

extern void set_TCA9548A(uint8_t bus_); // Функция устанавляивающая нужное положение на мультиплексоре

void printByteBinary(byte value)
{
	for (int i = 7; i >= 0; i--)
	{ // Начинаем с 7-го бита (самого старшего)
		// Проверяем, установлен ли i-й бит:
		if ((value >> i) & 0x01)
		{
			Serial.print('1');
		}
		else
		{
			Serial.print('0');
		}
	}
}
// Сброс ICM-20948
void icm20948_device_reset()
{
	WriteByte_I2C(ICM20948_I2C_ADDRESS, REG_BANK_SEL, ub_0);  // Устанавливаем работы с регистрами нулевой страницы
	WriteByte_I2C(ICM20948_I2C_ADDRESS, B0_PWR_MGMT_1, 0x80); // Записываем новое значение регистра B0_PWR_MGMT_1
	delayMicroseconds(500);									  // Задержка для стабилизации после сброса
	printf("+++ icm20948_device_reset \n");
}

// Считываем WHO_AM_I ICM-20948
void icm20948_who_am_i()
{
	printf("+++ icm20948_who_am_i \n");
	WriteByte_I2C(ICM20948_I2C_ADDRESS, REG_BANK_SEL, ub_0); // Устанавливаем работы с регистрами нулевой страницы
	uint8_t WIA_MPU = ReadByte_I2C(ICM20948_I2C_ADDRESS, B0_WHO_AM_I);

	Serial.print(" WIA_ICM 20948: ");
	Serial.print(WIA_MPU, BIN);
	Serial.print(" = ");
	Serial.print(WIA_MPU, DEC);
	Serial.print(" = ");
	Serial.print(WIA_MPU, HEX);

	Serial.print(" printByteBinary: ");
	printByteBinary(WIA_MPU);
	Serial.println("");

	if (WIA_MPU == ICM20948_ID)
		printf("    ICM-20948 connected successfully WHO_AM_I = 0x%02X\n", WIA_MPU);
	else
		printf("    ICM-20948 connection failed: 0x%02X real= 0x%02X\n ", ICM20948_ID, WIA_MPU);
	printf("--- icm20948_who_am_i \n");
}

void icm20948_init()
{
	Serial.println("======================================= START icm20948_init ===================================== ");

	set_TCA9548A(multi_line_ICM);
	delayMicroseconds(100);

	icm20948_device_reset();	 // Сброс ICM-20948
	icm20948_who_am_i();		 // Считываем WHO_AM_I ICM-20948
	icm20948_DisableLPMMode();	 // Отключение режима низкого энергопотребления (LPM) для нормальной работы датчика
	icm20948_odr_align_enable(); // Включение выравнивания частоты дискретизации (ODR align)  Используется для синхронизации частоты вывода данных (ODR) между акселерометром и гироскопом

	//*********************** ГИРОСКОП **************************
	/*Ваша максимальная скорость поворота — 360 градусов за 2 секунды. Давайте определим, как это переводится в частоту (в герцах), чтобы понять, подходит ли выбранная настройка.Угловая скорость:Угловая скорость=360∘2 с=180∘
	Это соответствует 180 градусов в секунду, что укладывается в выбранную чувствительность гироскопа 250 dps (±250°/с).
	Частота вращения в герцах: 	Частота в герцах (Гц) для вращения связана с периодом одного полного оборота (360°). Если полный оборот занимает 2 секунды, то:Частота=1Период=12 с=0,5 Гц\text{Частота} = \frac{1}{\text{Период}} = \frac{1}{2 \, \text{с}} = 0,5 \, \text{Гц}\text{Частота} = \frac{1}{\text{Период}} = \frac{1}{2 \, \text{с}} = 0,5 \, \text{Гц}
	Это означает, что основная частота вашего вращения — 0,5 Гц (один полный оборот каждые 2 секунды).*/
	icm20948_gyro_full_scale_select(_250dps); // Выбираем полный масштаб гироскопа ±250 dps

	// ICM-20948 настройки:Гироскоп: GYRO_DLPFCFG = 5 (3DB BW = 11,6 Гц, NBW = 17,8 Гц), FSR = ±250 dps, ODR ≈ 102,27 Гц (GYRO_SMPLRT_DIV = 10).
	/*Установите GYRO_DLPFCFG = 5:
	3DB BW = 11,6 Гц идеально подходит для захвата сигналов ваших медленных движений (0–5 Гц, с запасом до 10 Гц).
	NBW = 17,8 Гц обеспечивает отличное подавление шума, что повышает точность данных, особенно при высокой чувствительности (250 dps).
	Частота подготовки данных (RATE) при GYRO_SMPLRT_DIV ≤ 10 (например, ~102,27 Гц при GYRO_SMPLRT_DIV = 10) совместима с вашим опросом 100 Гц.*/
	icm20948_gyro_low_pass_filter(4); // Настройка низкочастотного фильтра для гироскопа //  - config: Значение от 0 до 7, задающее частоту среза фильтра (см. datasheet, раздел 6.8) // Используется для уменьшения шума в данных гироскопа

	icm20948_gyro_sample_rate_divider(10); // Установка делителя частоты дискретизации для гироскопа // - divider: Значение делителя (Output Data Rate = 1.125 кГц / (1 + divider))// Используется для настройки частоты вывода данных гироскопа (например, 100 Гц при divider = 10)

	//*********************** АКСЕЛЬРОМЕТР**************************
	// Акселерометр: ACCEL_DLPFCFG = 5 (3DB BW = 11,5 Гц, NBW = 13,1 Гц), FSR = ±2g, ODR ≈ 102,27 Гц (ACCEL_SMPLRT_DIV = 10).
	icm20948_accel_full_scale_select(_2g);	// Выбор полной шкалы для акселерометра  // - full_scale: Значение шкалы (например, ±2g, ±4g, см. datasheet, раздел 6.8) // Используется для установки диапазона измерений акселерометра (влияет на чувствительность)
	icm20948_accel_low_pass_filter(4);		// Настройка низкочастотного фильтра для акселрометра //  - config: Значение от 0 до 7, задающее частоту среза фильтра (см. datasheet, раздел 6.8) // Используется для уменьшения шума в данных гироскопа
	icm20948_accel_sample_rate_divider(10); // Установка делителя частоты дискретизации для акселрометра  //  - divider: Значение делителя (Output Data Rate = 1.125 кГц / (1 + divider)) // Используется для настройки частоты вывода данных гироскопа (например, 100 Гц при divider = 10)

	icm20948_wakeup(); // Вывод ICM-20948 из спящего режима

	Serial.println("======================================= END icm20948_init ===================================== ");
}

// Функция усредненных знаяений акселерометра и гироскопа 
void calc_average_data(uint16_t samples)
{
	SXyz tempA = {0.0f, 0.0f, 0.0f};
	SXyz tempG = {0.0f, 0.0f, 0.0f};

	for (uint16_t i = 0; i < samples; i++)
	{
		SXyz rawA, rawG; // Временные значения
		icm20948_readData(&rawA, &rawG); // Опрашиваем датчик для получения сырых данных
        calcBufferICM(&rawA, &rawG);     // Обработка буфера после считывания данных по шине
		tempA.x += rawA.x;
		tempA.y += rawA.y;
		tempA.z += rawA.z; 
		tempG.x += rawG.x;
		tempG.y += rawG.y;
		tempG.z += rawG.z; 
		delay(10); // Задержка между выборками (настройте по датчику)  у меня 100 Герц
	}
		tempA.x = tempA.x / samples;
		tempA.y = tempA.y / samples;
		tempA.z = tempA.z / samples; 
		tempG.x = tempG.x / samples;
		tempG.y = tempG.y / samples;
		tempG.z = tempG.z / samples; 
		printf( "Accel =%+6.4f %+6.4f %+6.4f | ", tempA.x, tempA.y, tempA.z); // Вывод средних значений  акселерометра
		printf( "Gyro =%+6.4f %+6.4f %+6.4f \n", tempG.x, tempG.y, tempG.z); // Вывод средних значений  гироскопа
}

// Функция для получения усредненных данных акселерометра и гироскопа для расчета bias
void calc_bias_accel_gyro (uint16_t samples)
{
	SXyz tempA = {0.0f, 0.0f, 0.0f};
	SXyz tempG = {0.0f, 0.0f, 0.0f};

	printf("+++ calc_bias_accel_gyro ");
	for (int j = 0; j < 5; j++)
	{
		printf("%d ", 5 - j); // Отсчет времени
		fflush(stdout);		  // Принудительный сброс буфера
		delay(1000);	  // Задержка 1 секунда
	}
	printf("Start... | \n");

	fflush(stdout); // Принудительный сброс буфера для вывода на экран

	for (uint16_t i = 0; i < samples; i++)
	{
		SXyz rawA, rawG; // Временные значения
		icm20948_readData(&rawA, &rawG); // Опрашиваем датчик для получения сырых данных
		tempA.x += rawA.x;
		tempA.y += rawA.y;
		tempA.z += rawA.z;

		tempG.x += rawG.x;
		tempG.y += rawG.y;
		tempG.z += rawG.z;

		delay(10); // Задержка между выборками (настройте по датчику)  у меня 100 Герц
	}

	aBias.b_x = tempA.x / samples;
	aBias.b_y = tempA.y / samples;
	aBias.b_z = accel_scale_factor - (tempA.z / samples); // тут надо свести к accel_scale_factor (16384 для 2g)

	printf("    aBias.b_x = %f  aBias.b_y = %f aBias.b_z = %f \n", aBias.b_x, aBias.b_y, aBias.b_z); // Вывод значений смещений гироскопа

	gBias.b_x = tempG.x / samples;
	gBias.b_y = tempG.y / samples;
	gBias.b_z = tempG.z / samples;
	printf("    gBias.b_x = %f  gBias.b_y = %f gBias.b_z = %f \n", gBias.b_x, gBias.b_y, gBias.b_z); // Вывод значений смещений гироскопа
}


// Вывод ICM-20948 из спящего режима
void icm20948_wakeup()
{
	printf("+++ icm20948_wakeup \n");

	WriteByte_I2C(ICM20948_I2C_ADDRESS, REG_BANK_SEL, ub_0);			 // Устанавливаем работы с регистрами нужной страницы
	uint8_t new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B0_PWR_MGMT_1); // Считываем значение регистра

	Serial.print(" IN B0_PWR_MGMT_1 registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	new_val &= 0xBF;

	Serial.print(" OUT B0_PWR_MGMT_1 registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	WriteByte_I2C(ICM20948_I2C_ADDRESS, B0_PWR_MGMT_1, new_val); // Записываем новое значение регистра
	delayMicroseconds(100);

	new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B0_PWR_MGMT_1); // Считываем значение регистра

	Serial.print(" REZ B0_PWR_MGMT_1 registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	printf("--- icm20948_wakeup \n");
}

// Установка делителя частоты дискретизации для акселерометра
void icm20948_accel_sample_rate_divider(uint16_t divider)
{
	printf("+++ icm20948_accel_sample_rate_divider \n");

	Serial.print(" IN divider = "); // Вывод  делителя
	Serial.print(divider, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(divider);
	Serial.println("");

	uint8_t divider_1 = (uint8_t)(divider >> 8);
	uint8_t divider_2 = (uint8_t)(0x0F & divider);

	Serial.print(" IN divider1 = "); // Вывод  делителя
	Serial.print(divider_1, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(divider_1);
	Serial.println("");

	Serial.print(" IN divider2 = "); // Вывод  делителя
	Serial.print(divider_2, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(divider_2);
	Serial.println("");

	WriteByte_I2C(ICM20948_I2C_ADDRESS, REG_BANK_SEL, ub_2);			   // Устанавливаем работы с регистрами нужной страницы
	WriteByte_I2C(ICM20948_I2C_ADDRESS, B2_ACCEL_SMPLRT_DIV_1, divider_1); // Записываем новое значение регистра
	WriteByte_I2C(ICM20948_I2C_ADDRESS, B2_ACCEL_SMPLRT_DIV_2, divider_2); // Записываем новое значение регистра
	delayMicroseconds(100);

	uint8_t new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B2_ACCEL_SMPLRT_DIV_1); // Считываем значение регистра B2_ACCEL_CONFIG

	Serial.print(" REZ B2_ACCEL_SMPLRT_DIV_1 registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B2_ACCEL_SMPLRT_DIV_2); // Считываем значение регистра B2_ACCEL_CONFIG

	Serial.print(" REZ B2_ACCEL_SMPLRT_DIV_2 registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	printf("--- icm20948_accel_sample_rate_divider \n");
}

// Настройка низкочастотного фильтра для акселерометра
void icm20948_accel_low_pass_filter(uint8_t config)
{
	printf("+++ icm20948_accel_low_pass_filter \n");

	Serial.print(" IN config = "); // Вывод  делителя
	Serial.print(config, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(config);
	Serial.println("");

	WriteByte_I2C(ICM20948_I2C_ADDRESS, REG_BANK_SEL, ub_2);			   // Устанавливаем работы с регистрами нужной страницы
	uint8_t new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B2_ACCEL_CONFIG); // Считываем значение регистра B2_ACCEL_CONFIG

	Serial.print(" IN B2_ACCEL_CONFIG registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	new_val |= config << 3;

	Serial.print(" OUT B2_ACCEL_CONFIG registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	WriteByte_I2C(ICM20948_I2C_ADDRESS, B2_ACCEL_CONFIG, new_val); // Записываем новое значение регистра B2_ACCEL_CONFIG
	delayMicroseconds(100);

	new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B2_ACCEL_CONFIG); // Считываем значение регистра B2_ACCEL_CONFIG

	Serial.print(" REZ B2_ACCEL_CONFIG registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	printf("--- icm20948_accel_low_pass_filter \n");
}

// Выбор полной шкалы для акселерометра
void icm20948_accel_full_scale_select(accel_full_scale full_scale)
{
	printf("+++ icm20948_accel_full_scale_select \n");

	Serial.print(" IN full_scale = "); // Вывод  делителя
	Serial.print(full_scale, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(full_scale);
	Serial.println("");

	WriteByte_I2C(ICM20948_I2C_ADDRESS, REG_BANK_SEL, ub_2);			   // Устанавливаем работы с регистрами нужной страницы
	uint8_t new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B2_ACCEL_CONFIG); // Считываем значение регистра B2_ACCEL_CONFIG

	Serial.print(" IN B2_ACCEL_CONFIG registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	switch (full_scale)
	{
	case _2g:
		new_val |= 0x00;
		accel_scale_factor = 16384;
		break;
	case _4g:
		new_val |= 0x02;
		accel_scale_factor = 8192;
		break;
	case _8g:
		new_val |= 0x04;
		accel_scale_factor = 4096;
		break;
	case _16g:
		new_val |= 0x06;
		accel_scale_factor = 2048;
		break;
	}

	Serial.print(" OUT B2_ACCEL_CONFIG registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	WriteByte_I2C(ICM20948_I2C_ADDRESS, B2_ACCEL_CONFIG, new_val); // Записываем новое значение регистра B2_ACCEL_CONFIG
	delayMicroseconds(100);

	new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B2_ACCEL_CONFIG); // Считываем значение регистра B2_ACCEL_CONFIG

	Serial.print(" REZ B2_ACCEL_CONFIG registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	printf("--- icm20948_accel_full_scale_select \n");
}

// Установка делителя частоты дискретизации для гироскопа
void icm20948_gyro_sample_rate_divider(uint8_t divider)
{
	printf("+++ icm20948_gyro_sample_rate_divider \n");

	Serial.print(" IN divider = "); // Вывод  делителя
	Serial.print(divider, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(divider);
	Serial.println("");

	uint8_t new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B2_GYRO_SMPLRT_DIV); // Считываем значение регистра B2_GYRO_SMPLRT_DIV

	Serial.print(" IN B2_GYRO_SMPLRT_DIV registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	WriteByte_I2C(ICM20948_I2C_ADDRESS, REG_BANK_SEL, ub_2);		  // Устанавливаем работы с регистрами нулевой страницы
	WriteByte_I2C(ICM20948_I2C_ADDRESS, B2_GYRO_SMPLRT_DIV, divider); // Записываем новое значение регистра
	delayMicroseconds(100);

	new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B2_GYRO_SMPLRT_DIV); // Считываем значение регистра B2_GYRO_SMPLRT_DIV

	Serial.print(" REZ B2_GYRO_SMPLRT_DIV registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	printf("--- icm20948_gyro_sample_rate_divider \n");
}

// Включение выравнивания частоты дискретизации (ODR align)
void icm20948_odr_align_enable()
{
	printf("+++ icm20948_odr_align_enable \n");
	WriteByte_I2C(ICM20948_I2C_ADDRESS, REG_BANK_SEL, ub_2);	// Устанавливаем работы с регистрами нулевой страницы
	WriteByte_I2C(ICM20948_I2C_ADDRESS, B2_ODR_ALIGN_EN, 0x01); // Записываем новое значение регистра B0_PWR_MGMT_1
	delayMicroseconds(100);
	uint8_t new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B2_ODR_ALIGN_EN); // Считываем значение регистра B2_ODR_ALIGN_EN после записи // Проверка, что B2_ODR_ALIGN_EN записался корректно

	Serial.print(" REZ B2_ODR_ALIGN_EN registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	printf("--- icm20948_odr_align_enable \n");
}

// Отключение режима низкого энергопотребления (LPM) для нормальной работы датчика
void icm20948_DisableLPMMode()
{
	printf("+++ icm20948_DisableLPMMode \n");
	WriteByte_I2C(ICM20948_I2C_ADDRESS, REG_BANK_SEL, ub_0); // Устанавливаем работы с регистрами нулевой страницы

	// Отключение LP_EN в PWR_MGMT_1 (запись 0x01: LP_EN=0, CLKSEL=001)
	uint8_t data = 0x01;									  // LP_EN=0, CLKSEL=001, остальные биты по умолчанию
	WriteByte_I2C(ICM20948_I2C_ADDRESS, B0_PWR_MGMT_1, data); // Записываем новое значение регистра B0_PWR_MGMT_1

	// Проверка, что PWR_MGMT_1 записался корректно
	uint8_t new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B0_PWR_MGMT_1); // Считываем значение регистра B2_GYRO_CONFIG_1 после записи

	Serial.print(" REZ B0_PWR_MGMT_1 registr = "); // Вывод регистра
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");
	delayMicroseconds(500);
	printf("--- icm20948_DisableLPMMode \n");
}

void icm20948_gyro_full_scale_select(gyro_full_scale full_scale)
{
	uint8_t new_val = 0;
	printf("+++ icm20948_gyro_full_scale_select \n");

	Serial.print(" full_scale = ");
	Serial.print(full_scale, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(full_scale);
	Serial.println("");

	WriteByte_I2C(ICM20948_I2C_ADDRESS, REG_BANK_SEL, ub_2); // Устанавливаем работы с регистрами нужного банка

	new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B2_GYRO_CONFIG_1); // Считываем текущее значение регистра B2_GYRO_CONFIG_1

	Serial.print(" IN B2_GYRO_CONFIG_1 = ");
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	switch (full_scale) // Выбираем полный масштаб гироскопа
	{
	case _250dps: // Значение регистра для ±250 dps
		new_val |= 0x00;
		gyro_scale_factor = 131.0;
		break;
	case _500dps: // Значение регистра для ±500 dps
		new_val |= 0x02;
		gyro_scale_factor = 65.5;
		break;
	case _1000dps:
		new_val |= 0x04;
		gyro_scale_factor = 32.8;
		break;
	case _2000dps:
		new_val |= 0x06;
		gyro_scale_factor = 16.4;
		break;
	}

	Serial.print(" OUT B2_GYRO_CONFIG_1 = ");
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	WriteByte_I2C(ICM20948_I2C_ADDRESS, B2_GYRO_CONFIG_1, new_val); // Записываем новое значение регистра B2_GYRO_CONFIG_1

	new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B2_GYRO_CONFIG_1); // Считываем значение регистра B2_GYRO_CONFIG_1 после записи
	Serial.print(" REZ B2_GYRO_CONFIG_1 = ");
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	printf("    End icm20948_gyro_full_scale_select ****************************************************** \n");
}

void icm20948_gyro_low_pass_filter(uint8_t config)
{
	printf("+++ icm20948_gyro_low_pass_filter \n");

	uint8_t new_val = 0;
	Serial.print(" gyro_low_pass_filter = ");
	Serial.print(config, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(config);
	Serial.println("");

	// uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);

	WriteByte_I2C(ICM20948_I2C_ADDRESS, REG_BANK_SEL, ub_2);		// Устанавливаем работы с регистрами нужного банка
	new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B2_GYRO_CONFIG_1); // Считываем текущее значение регистра B2_GYRO_CONFIG_1

	Serial.print(" IN B2_GYRO_CONFIG_1 = ");
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	new_val |= config << 3;

	Serial.print(" OUT B2_GYRO_CONFIG_1 = ");
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	WriteByte_I2C(ICM20948_I2C_ADDRESS, B2_GYRO_CONFIG_1, new_val); // Записываем новое значение регистра B2_GYRO_CONFIG_1
	delayMicroseconds(100);
	new_val = ReadByte_I2C(ICM20948_I2C_ADDRESS, B2_GYRO_CONFIG_1); // Считываем значение регистра B2_GYRO_CONFIG_1 после записи

	Serial.print(" REZ B2_GYRO_CONFIG_1 = ");
	Serial.print(new_val, HEX);
	Serial.print(" printByteBinary: ");
	printByteBinary(new_val);
	Serial.println("");

	printf("    End icm20948_gyro_low_pass_filter ****************************************************** \n");
}

// Опрашиваем датчик. Возвращает СЫРЫЕ данные без bias and scale
void icm20948_readData(SXyz *dataAccel, SXyz *dataGyro)
{
	uint8_t static buffer[12] = {0}; // буфер для ICM20948
	uint16_t size = 12;				 // Опрашиваем 12 байт из датчика
	set_TCA9548A(multi_line_ICM);
	delayMicroseconds(100);

	Wire.beginTransmission(ICM20948_I2C_ADDRESS); // Start I2C transmission
	Wire.write(B0_ACCEL_XOUT_H);				  /* Make sure to set address auto-increment bit */

	byte reza = Wire.endTransmission(); //  End I2C transmission
	if (reza != 0)						//
	{
		Serial.print("!!! ReadByte_I2C_WriteMistake reza = ");
		Serial.println(reza);
	};

	byte rezb = Wire.requestFrom(ICM20948_I2C_ADDRESS, size); // Request bytes from I2C slave
	if (rezb != size)										  // Вернуть должна столько байтов сколько попросили
	{
		Serial.print("!!! ReadByte_I2C_WriteMistake return byte = ");
		Serial.println(rezb);
	};

	int i = 0;
	for (; i < size && (Wire.available() > 0); i++)
	{
		buffer[i] = Wire.read(); // read one byte of data
	}

	// DEBUG_PRINTF("ICM20948 buffer");
	dataAccel->x = (int16_t)(buffer[0] << 8 | buffer[1]);
	dataAccel->y = (int16_t)(buffer[2] << 8 | buffer[3]);
	dataAccel->z = (int16_t)(buffer[4] << 8 | buffer[5]);

	dataGyro->x = (int16_t)(buffer[6] << 8 | buffer[7]);
	dataGyro->y = (int16_t)(buffer[8] << 8 | buffer[9]);
	dataGyro->z = (int16_t)(buffer[10] << 8 | buffer[11]);

	// for (int i = 0; i < 12; i++)
	// {
	// 	DEBUG_PRINTF(" = 0x%02X", buffer[i]);
	// }
	// DEBUG_PRINTF(" \n");
}

// Функция для расчета буфера ICM20948. Применяем bias and scale.
void calcBufferICM(SXyz *dataAccel, SXyz *dataGyro)
{
	float const ALPHA = 0.5;
	static float g = 9.80665; // Ускорение свободного падения в м/с²

	// DEBUG_PRINTF("Accel accel_scale_factor = %+8.3f | ", accel_scale_factor);
	dataAccel->x = (dataAccel->x - aBias.b_x) / accel_scale_factor * g; // Преобразование в g делением на акселерационный коэффициент Вычитаем bias и умножаем на масштабный коефициент
	dataAccel->y = (dataAccel->y - aBias.b_y) / accel_scale_factor * g; // Преобразование в g делением на акселерационный коэффициент Вычитаем bias и умножаем на масштабный коефициент
	dataAccel->z = (dataAccel->z - aBias.b_z) / accel_scale_factor * g; // Преобразование в g делением на акселерационный коэффициент Вычитаем bias и умножаем на масштабный коефициент

	static SXyz smoothed_data_accel = {0, 0, 9.80665}; // Начальные значения

	smoothed_data_accel.x = ALPHA * dataAccel->x + (1 - ALPHA) * smoothed_data_accel.x; // Экспоненциальное сглаживание везде по всем осям используем один коефициент
	smoothed_data_accel.y = ALPHA * dataAccel->y + (1 - ALPHA) * smoothed_data_accel.y;
	smoothed_data_accel.z = ALPHA * dataAccel->z + (1 - ALPHA) * smoothed_data_accel.z;
	// DEBUG_PRINTF("Norm (g): %.3f",norm);

	// DEBUG_PRINTF("Accel raw = %+8.4f %+8.4f %+8.4f smoothed= %+8.4f %+8.4f %+8.4f | ", data->x, data->y, data->z, smoothed_data.x, smoothed_data.y, smoothed_data.z);
	*dataAccel = smoothed_data_accel;

	// DEBUG_PRINTF("Gyro gyro_scale_factor = %+8.3f | ", gyro_scale_factor);
	dataGyro->x = (dataGyro->x - gBias.b_x) / gyro_scale_factor; // Преобразование сырых данных в физические величины с учетом масштаба
	dataGyro->y = (dataGyro->y - gBias.b_y) / gyro_scale_factor;
	dataGyro->z = (dataGyro->z - gBias.b_z) / gyro_scale_factor;

	static SXyz smoothed_data_gyro = {0, 0, 0}; // Начальные значения

	smoothed_data_gyro.x = ALPHA * dataGyro->x + (1 - ALPHA) * smoothed_data_gyro.x; // Экспоненциальное сглаживание везде по всем осям используем один коефициент
	smoothed_data_gyro.y = ALPHA * dataGyro->y + (1 - ALPHA) * smoothed_data_gyro.y; //
	smoothed_data_gyro.z = ALPHA * dataGyro->z + (1 - ALPHA) * smoothed_data_gyro.z;

	// DEBUG_PRINTF("Gyro raw = %+8.3f %+8.3f %+8.3f smoothed= %+8.3f %+8.3f %+8.3f | ", data->x, data->y, data->z, smoothed_data.x, smoothed_data.y, smoothed_data.z);

	*dataGyro = smoothed_data_gyro;
}