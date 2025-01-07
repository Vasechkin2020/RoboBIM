

#define BNO055_ADDRESS (0x29)  /* 0x28 com3 low 0x29 com3 high     */
#define BNO055_ID (0b10100000) /* pg58                             */

byte BNO055_Offset_Array[22];																										// Массив для хранения офсетов
byte BNO055_Offset_Array_dafault[22]{234, 255, 18, 0, 228, 255, 248, 255, 40, 254, 248, 255, 253, 255, 1, 0, 1, 0, 232, 3, 176, 4}; // Массив для хранения офсетов по умолчанию
byte BNO055_Offset_Array_dafault2[22]{240, 255, 7, 0, 249, 255, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 232, 3, 224, 1};	// Красный датчик
																																	// = 2   = 0   = 212 = 255 = 15   = 0  = 0 = 0 = 0 = 0 = 0 = 0 = 0   = 0   = 255 = 255 = 255 = 255 = 232 = 3 = 224 = 1---
																																	// = 37  = 0   = 198 = 255 = 61   = 0  = 0 = 0 = 0 = 0 = 0 = 0 = 252 = 255 = 0   = 0   = 0   = 0   = 232 = 3 = 224 = 1---
																																	// = 7   = 0   = 191 = 255 = 241 = 255 = 0 = 0 = 0 = 0 = 0 = 0 = 254 = 255 = 254 = 255 = 255 = 255 = 232 = 3 = 224 = 1---
																																	// = 243 = 255 = 234 = 255 = 248 = 255 = 0 = 0 = 0 = 0 = 0 = 0 = 251 = 255 = 3   = 0   = 255 = 255 = 232 = 3 = 224 = 1---
																																	// = 240 = 255 = 7   = 0   = 249 = 255 = 0 = 0 = 0 = 0 = 0 = 0 = 255 = 255 = 255 = 255 = 255 = 255 = 232 = 3 = 224 = 1---
																																	// = 226 = 255 = 235 = 255 = 248 = 255 = 0 = 0 = 0 = 0 = 0 = 0 = 255 = 255 = 255 = 255 = 255 = 255 = 232 = 3 = 224 = 1---

struct BNO055_Info_s
{
	byte SystemStatusCode;
	byte SelfTestStatus;
	byte SystemError;
	byte accel_rev;
	byte mag_rev;
	byte gyro_rev;
	byte bl_rev;
	word sw_rev;
	byte Calibr_sys;
	byte Calibr_accel;
	byte Calibr_gyro;
	byte Calibr_mag;
};

BNO055_Info_s BNO055;

struct BNO055_offsets_t
{
	int16_t accel_offset_x;
	int16_t accel_offset_y;
	int16_t accel_offset_z;
	int16_t mag_offset_x;
	int16_t mag_offset_y;
	int16_t mag_offset_z;
	int16_t gyro_offset_x;
	int16_t gyro_offset_y;
	int16_t gyro_offset_z;

	int16_t accel_radius;
	int16_t mag_radius;
};
enum eBNO055Mode_t
{							  /*HW SENS POWER    SENS SIG         FUSION       */
							  /*  A   M   G       A   M   G       E   Q   L   G*/
  eCONFIGMODE = 0b00000000,	  /*  y   y   y       n   n   n       n   n   n   n*/
  eACCONLY = 0b00000001,	  /*  y   n   n       y   n   n       n   n   n   n*/
  eMAGONLY = 0b00000010,	  /*  n   y   n       n   y   n       n   n   n   n*/
  eGYROONLY = 0b00000011,	  /*  n   n   y       n   n   y       n   n   n   n*/
  eACCMAG = 0b00000100,		  /*  y   y   n       y   y   n       n   n   n   n*/
  eACCGYRO = 0b00000101,	  /*  y   n   y       y   n   y       n   n   n   n*/
  eMAGGYRO = 0b00000110,	  /*  n   y   y       n   y   y       n   n   n   n*/
  eAMG = 0b00000111,		  /*  y   y   y       y   y   y       n   n   n   n*/
  eIMU = 0b00001000,		  /*  y   n   y       y   n   y       y   y   y   y*/
  eCOMPASS = 0b00001001,	  /*  y   y   n       y   y   n       y   y   y   y*/
  eM4G = 0b00001010,		  /*  y   y   n       y   y   y       y   y   y   y*/
  eNDOF_FMC_OFF = 0b00001011, /*  y   y   y       y   y   y       y   y   y   y*/
  eNDOF = 0b00001100,		  /*  y   y   y       y   y   y       y   y   y   y*/
};
enum eBNO055PowerModes_t
{
	eNORMAL_POWER_MODE = 0b00000000,
	eLOW_POWER_MODE = 0b00000001,
	eSUSPEND_POWER_MODE = 0b00000010,
};

enum eBNO055Registers_t
{ /* DEFAULT    TYPE                  */
  /*page0*/
  eBNO055_REGISTER_CHIP_ID = 0x00,			/* 0x00       r                     */
  eBNO055_REGISTER_ACC_ID = 0x01,			/* 0xFB       r                     */
  eBNO055_REGISTER_MAG_ID = 0x02,			/* 0x32       r                     */
  eBNO055_REGISTER_GYR_ID = 0x03,			/* 0x0F       r                     */
  eBNO055_REGISTER_SW_REV_ID_LSB = 0x04,	/*            r                     */
  eBNO055_REGISTER_SW_REV_ID_MSB = 0x05,	/*            r                     */
  eBNO055_REGISTER_BL_REV_ID = 0x06,		/*            r                     */
  eBNO055_REGISTER_PAGE_ID = 0x07,			/* 0x00       rw                    */
  eBNO055_REGISTER_ACC_DATA_X_LSB = 0x08,	/* 0x00       r                     */
  eBNO055_REGISTER_ACC_DATA_X_MSB = 0x09,	/* 0x00       r                     */
  eBNO055_REGISTER_ACC_DATA_Y_LSB = 0x0A,	/* 0x00       r                     */
  eBNO055_REGISTER_ACC_DATA_Y_MSB = 0x0B,	/* 0x00       r                     */
  eBNO055_REGISTER_ACC_DATA_Z_LSB = 0x0C,	/* 0x00       r                     */
  eBNO055_REGISTER_ACC_DATA_Z_MSB = 0x0D,	/* 0x00       r                     */
  eBNO055_REGISTER_MAG_DATA_X_LSB = 0x0E,	/* 0x00       r                     */
  eBNO055_REGISTER_MAG_DATA_X_MSB = 0x0F,	/* 0x00       r                     */
  eBNO055_REGISTER_MAG_DATA_Y_LSB = 0x10,	/* 0x00       r                     */
  eBNO055_REGISTER_MAG_DATA_Y_MSB = 0x11,	/* 0x00       r                     */
  eBNO055_REGISTER_MAG_DATA_Z_LSB = 0x12,	/* 0x00       r                     */
  eBNO055_REGISTER_MAG_DATA_Z_MSB = 0x13,	/* 0x00       r                     */
  eBNO055_REGISTER_GYR_DATA_X_LSB = 0x14,	/* 0x00       r                     */
  eBNO055_REGISTER_GYR_DATA_X_MSB = 0x15,	/* 0x00       r                     */
  eBNO055_REGISTER_GYR_DATA_Y_LSB = 0x16,	/* 0x00       r                     */
  eBNO055_REGISTER_GYR_DATA_Y_MSB = 0x17,	/* 0x00       r                     */
  eBNO055_REGISTER_GYR_DATA_Z_LSB = 0x18,	/* 0x00       r                     */
  eBNO055_REGISTER_GYR_DATA_Z_MSB = 0x19,	/* 0x00       r                     */
  eBNO055_REGISTER_EUL_DATA_X_LSB = 0x1A,	/* 0x00       r                     */
  eBNO055_REGISTER_EUL_DATA_X_MSB = 0x1B,	/* 0x00       r                     */
  eBNO055_REGISTER_EUL_DATA_Y_LSB = 0x1C,	/* 0x00       r                     */
  eBNO055_REGISTER_EUL_DATA_Y_MSB = 0x1D,	/* 0x00       r                     */
  eBNO055_REGISTER_EUL_DATA_Z_LSB = 0x1E,	/* 0x00       r                     */
  eBNO055_REGISTER_EUL_DATA_Z_MSB = 0x1F,	/* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_W_LSB = 0x20,	/* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_W_MSB = 0x21,	/* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_X_LSB = 0x22,	/* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_X_MSB = 0x23,	/* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_Y_LSB = 0x24,	/* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_Y_MSB = 0x25,	/* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_Z_LSB = 0x26,	/* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_Z_MSB = 0x27,	/* 0x00       r                     */
  eBNO055_REGISTER_LIA_DATA_X_LSB = 0x28,	/* 0x00       r                     */
  eBNO055_REGISTER_LIA_DATA_X_MSB = 0x29,	/* 0x00       r                     */
  eBNO055_REGISTER_LIA_DATA_Y_LSB = 0x2A,	/* 0x00       r                     */
  eBNO055_REGISTER_LIA_DATA_Y_MSB = 0x2B,	/* 0x00       r                     */
  eBNO055_REGISTER_LIA_DATA_Z_LSB = 0x2C,	/* 0x00       r                     */
  eBNO055_REGISTER_LIA_DATA_Z_MSB = 0x2D,	/* 0x00       r                     */
  eBNO055_REGISTER_GRV_DATA_X_LSB = 0x2E,	/* 0x00       r                     */
  eBNO055_REGISTER_GRV_DATA_X_MSB = 0x2F,	/* 0x00       r                     */
  eBNO055_REGISTER_GRV_DATA_Y_LSB = 0x30,	/* 0x00       r                     */
  eBNO055_REGISTER_GRV_DATA_Y_MSB = 0x31,	/* 0x00       r                     */
  eBNO055_REGISTER_GRV_DATA_Z_LSB = 0x32,	/* 0x00       r                     */
  eBNO055_REGISTER_GRV_DATA_Z_MSB = 0x33,	/* 0x00       r                     */
  eBNO055_REGISTER_TEMP = 0x34,				/* 0x00       r                     */
  eBNO055_REGISTER_CALIB_STAT = 0x35,		/* 0x00       r                     */
  eBNO055_REGISTER_ST_RESULT = 0x36,		/* xxxx1111   r                     */
  eBNO055_REGISTER_INT_STA = 0x37,			/* 000x00xx   r  pg74               */
  eBNO055_REGISTER_SYS_CLK_STATUS = 0x38,	/* 00000000   r  pg74               */
  eBNO055_REGISTER_SYS_STATUS = 0x39,		/* 00000000   r  pg74               */
  eBNO055_REGISTER_SYS_ERR = 0x3A,			/* 00000000   r  pg75               */
  eBNO055_REGISTER_UNIT_SEL = 0x3B,			/* 0xx0x000   rw pg76               */
  eBNO055_REGISTER_OPR_MODE = 0x3D,			/* x???????   rw pg77               */
  eBNO055_REGISTER_PWR_MODE = 0x3E,			/* xxxxxx??   rw pg78               */
  eBNO055_REGISTER_SYS_TRIGGER = 0x3F,		/* 000xxxx0   w  pg78               */
  eBNO055_REGISTER_TEMP_SOURCE = 0x40,		/* xxxxxx??   rw pg78               */
  eBNO055_REGISTER_AXIS_MAP_CONFIG = 0x41,	/* xx??????   rw pg79               */
  eBNO055_REGISTER_AXIS_MAP_SIGN = 0x42,	/* xxxxx???   rw pg79               */
  eBNO055_REGISTER_SIC_MATRIX = 0x43,		/* xxxxxx??   ?? pg80               */
  eBNO055_REGISTER_ACC_OFFSET_X_LSB = 0x55, /* 0x00       rw                    */
  eBNO055_REGISTER_ACC_OFFSET_X_MSB = 0x56, /* 0x00       rw                    */
  eBNO055_REGISTER_ACC_OFFSET_Y_LSB = 0x57, /* 0x00       rw                    */
  eBNO055_REGISTER_ACC_OFFSET_Y_MSB = 0x58, /* 0x00       rw                    */
  eBNO055_REGISTER_ACC_OFFSET_Z_LSB = 0x59, /* 0x00       rw                    */
  eBNO055_REGISTER_ACC_OFFSET_Z_MSB = 0x5A, /* 0x00       rw                    */
  eBNO055_REGISTER_MAG_OFFSET_X_LSB = 0x5B, /* 0x00       rw                    */
  eBNO055_REGISTER_MAG_OFFSET_X_MSB = 0x5C, /* 0x00       rw                    */
  eBNO055_REGISTER_MAG_OFFSET_Y_LSB = 0x5D, /* 0x00       rw                    */
  eBNO055_REGISTER_MAG_OFFSET_Y_MSB = 0x5E, /* 0x00       rw                    */
  eBNO055_REGISTER_MAG_OFFSET_Z_LSB = 0x5F, /* 0x00       rw                    */
  eBNO055_REGISTER_MAG_OFFSET_Z_MSB = 0x60, /* 0x00       rw                    */
  eBNO055_REGISTER_GYR_OFFSET_X_LSB = 0x61, /* 0x00       rw                    */
  eBNO055_REGISTER_GYR_OFFSET_X_MSB = 0x62, /* 0x00       rw                    */
  eBNO055_REGISTER_GYR_OFFSET_Y_LSB = 0x63, /* 0x00       rw                    */
  eBNO055_REGISTER_GYR_OFFSET_Y_MSB = 0x64, /* 0x00       rw                    */
  eBNO055_REGISTER_GYR_OFFSET_Z_LSB = 0x65, /* 0x00       rw                    */
  eBNO055_REGISTER_GYR_OFFSET_Z_MSB = 0x66, /* 0x00       rw                    */
  eBNO055_REGISTER_ACC_RADIUS_LSB = 0x67,	/* 0x00       rw                    */
  eBNO055_REGISTER_ACC_RADIUS_MSB = 0x68,	/* 0x00       rw                    */
  eBNO055_REGISTER_MAG_RADIUS_LSB = 0x69,	/* 0x00       rw                    */
  eBNO055_REGISTER_MAG_RADIUS_MSB = 0x6A,	/* 0x00       rw                    */

  /*page 1*/

  /*      eBNO055_REGISTER_PAGE_ID             = 0x07,   ??         rw see page0          */
  eBNO055_REGISTER_ACC_CONFIG = 0x08,		/* 00001101   rw pg87               */
  eBNO055_REGISTER_MAG_CONFIG = 0x09,		/* 00001011   rw pg87               */
  eBNO055_REGISTER_GYR_CONFIG = 0x0A,		/* 00111000   rw pg88               */
  eBNO055_REGISTER_GYR_CONFIG_1 = 0x0B,		/* 00000000   rw pg88               */
  eBNO055_REGISTER_ACC_SLEEP_CONFIG = 0x0C, /* ????????   rw pg89               */
  eBNO055_REGISTER_GYR_SLEEP_CONFIG = 0x0D, /* ????????   rw pg90               */
  eBNO055_REGISTER_INT_MSK = 0x0F,			/* 000x00xx   rw pg91               */
  eBNO055_REGISTER_INT_EN = 0x10,			/* 000x00xx   rw pg92               */
  eBNO055_REGISTER_ACC_AM_THRES = 0x11,		/* 00010100   rw pg92               */
  eBNO055_REGISTER_ACC_INT_SETTINGS = 0x12, /* 00000011   rw pg93               */
  eBNO055_REGISTER_ACC_HG_DURATION = 0x13,	/* 00001111   rw pg93               */
  eBNO055_REGISTER_ACC_HG_THRES = 0x14,		/* 11000000   rw pg93               */
  eBNO055_REGISTER_ACC_NM_THRES = 0x15,		/* 00001010   rw pg93               */
  eBNO055_REGISTER_ACC_NM_SET = 0x16,		/* x0001011   rw pg94               */
  eBNO055_REGISTER_GYR_INT_SETTING = 0x17,	/* 00000000   rw pg95               */
  eBNO055_REGISTER_GYR_HR_X_SET = 0x18,		/* 00000001   rw pg95               */
  eBNO055_REGISTER_GYR_DUR_X = 0x19,		/* 00011001   rw pg96               */
  eBNO055_REGISTER_ACC_HR_Y_SET = 0x1A,		/* 00000001   rw pg96               */
  eBNO055_REGISTER_GYR_DUR_Y = 0x1B,		/* 00011001   rw pg96               */
  eBNO055_REGISTER_ACC_HR_Z_SET = 0x1C,		/* 00000001   rw pg97               */
  eBNO055_REGISTER_GYR_DUR_Z = 0x1D,		/* 00011001   rw pg97               */
  eBNO055_REGISTER_GYR_AM_THRES = 0x1E,		/* 00000100   rw pg97               */
  eBNO055_REGISTER_GYR_AM_SET = 0x1F,		/* 00001010   rw pg98               */
};

SXyz BNO055_EulerAngles;
SXyz BNO055_LinAccData;
SXyz BNO055_offsetLinAcc; // Офсет для гироскопа

void BNO055_SetMode(byte mode_); // Установка нужного режима работы
bool BNO055_getCalibration();	 // Чтение калибровочных регистров
void BNO055_readData();			 // Считыванеи данных из датчика
void BNO055_getStatusInfo();	 // Считывание статуса из регистров

void BNO055_SetMode(byte mode_)
{
	WriteByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_OPR_MODE, mode_); // | eFASTEST_MODE);  /* Go to config mode if not there */
	printf("%lu BNO055_SetMode => %i \n", millis(), mode_);
	delay(25);
}

bool BNO055_getCalibration()
{
	// set_TCA9548A(multi_line_BNO);
	uint8_t calData = ReadByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_CALIB_STAT);
	BNO055.Calibr_sys = (calData >> 6) & 0x03;
	Serial.print(" Calibr_sys  : ");
	Serial.print(BNO055.Calibr_sys);

	BNO055.Calibr_gyro = (calData >> 4) & 0x03;
	Serial.print(" Calibr_gyro : ");
	Serial.print(BNO055.Calibr_gyro);

	BNO055.Calibr_accel = (calData >> 2) & 0x03;
	Serial.print(" Calibr_accel: ");
	Serial.print(BNO055.Calibr_accel);

	BNO055.Calibr_mag = calData & 0x03;
	Serial.print(" Calibr_mag  : ");
	Serial.print(BNO055.Calibr_mag);

	// if (BNO055.Calibr_sys < 3 || BNO055.Calibr_gyro < 3 || BNO055.Calibr_accel < 3 || BNO055.Calibr_mag < 3)
	if (BNO055.Calibr_gyro < 3 || BNO055.Calibr_accel < 3)
	{
		Serial.println(" Calibrovka FALSE !!!");
		return false;
	}
	else
	{
		Serial.print(" Calibrovka TRUE !!! = ");
		Serial.println(millis());

		return true;
	}
}

void BNO055_readData()
{
	uint8_t xHigh = 0, xLow = 0, yLow = 0, yHigh = 0, zLow = 0, zHigh = 0;
	uint8_t buffer_bnoTemp[20];
	uint8_t buffer_bno[20];

	set_TCA9548A(multi_line_BNO2);
	delayMicroseconds(500);
	Wire.beginTransmission(BNO055_ADDRESS);
	Wire.write(eBNO055_REGISTER_EUL_DATA_X_LSB); /* Make sure to set address auto-increment bit */
	Wire.endTransmission();
	Wire.requestFrom(BNO055_ADDRESS, (int)20);
	int i = 0;
	for (; i < 20 && (Wire.available() > 0); i++)
	{
		buffer_bnoTemp[i] = Wire.read(); // read one byte of data
	}
	if (i >= 20) // Если считали все данные тогда переносим массив
	{
		for (int y = 0; y < 20; y++)
		{
			buffer_bno[y] = buffer_bnoTemp[y];
		}
		xLow = buffer_bno[0];
		xHigh = buffer_bno[1];
		yLow = buffer_bno[2];
		yHigh = buffer_bno[3];
		zLow = buffer_bno[4];
		zHigh = buffer_bno[5];

		/* Shift values to create properly formed integer (low byte first) */ /* 1 degree = 16 LSB  1 radian = 900 LSB   */
		BNO055_EulerAngles.x = (int16_t)(yLow | (yHigh << 8)) / 16.;
		BNO055_EulerAngles.y = (int16_t)(zLow | (zHigh << 8)) / 16.; // Положение датчика перевернутое
		BNO055_EulerAngles.z = (int16_t)(xLow | (xHigh << 8)) / 16.;

		BNO055_EulerAngles.y = 180 - BNO055_EulerAngles.y; // Испрвления для ориентации датчика
		if (BNO055_EulerAngles.y > 180)
			BNO055_EulerAngles.y = BNO055_EulerAngles.y - 360;

		//printf("%lu roll= %.4f pitch= %.4f yaw= %.4f  /  \n ", millis(), BNO055_EulerAngles.x, BNO055_EulerAngles.y, BNO055_EulerAngles.z);

		xLow = buffer_bno[14];
		xHigh = buffer_bno[15];
		yLow = buffer_bno[16];
		yHigh = buffer_bno[17];
		zLow = buffer_bno[18];
		zHigh = buffer_bno[19];

		// Перевод в m/s2 1m/s2 = 100 LSB, mg = 1LSB
		BNO055_LinAccData.x = (int16_t)(xLow | (xHigh << 8)) / 100.;
		BNO055_LinAccData.y = (int16_t)(yLow | (yHigh << 8)) / 100.;
		BNO055_LinAccData.z = (int16_t)(zLow | (zHigh << 8)) / 100.; // Дальше не используем так как не летаем а ездим по плоскости. И заменяем на угловую скорость полученную из угла Эллера

		// printf("x= % .6f % .6f \n", BNO055_LinAccData.x, BNO055_LinAccData.y);
		bno055.status = 0;
		bno055.angleEuler = BNO055_EulerAngles;
		bno055.linear = BNO055_LinAccData;
	}
	else
	{
		Serial.println("Error read data form BNO055.");
	}
}

void BNO055_getStatusInfo()
{
	Serial.println(" ===================== BNO055_getStatusInfo ===============");
	// set_TCA9548A(multi_line_BNO);

	WriteByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_PAGE_ID, 0); // Устанавливаем работы с регистрами нулевой страницы

	BNO055.SystemStatusCode = ReadByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_SYS_STATUS);
	Serial.print("BNO055.SystemStatusCode= ");
	if (BNO055.SystemStatusCode != 0)
	{
		Serial.println(BNO055.SystemStatusCode);
		/* System Status (see section 4.3.58)
	   ---------------------------------
	   0 = Idle
	   1 = System Error
	   2 = Initializing Peripherals
	   3 = System Iniitalization
	   4 = Executing Self-Test
	   5 = Sensor fusio algorithm running
	   6 = System running without fusion algorithms */
	}
	else
	{
		Serial.println("Ok.");
	}

	BNO055.SelfTestStatus = ReadByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_ST_RESULT);
	Serial.print("BNO055.SelfTestStatus= ");
	if (BNO055.SelfTestStatus != 0b1111)
	{
		Serial.println(BNO055.SelfTestStatus, BIN);
		/* Self Test Results (see section )
	   --------------------------------
	   1 = test passed, 0 = test failed

	   Bit 0 = Accelerometer self test
	   Bit 1 = Magnetometer self test
	   Bit 2 = Gyroscope self test
	   Bit 3 = MCU self test

	   0b1111 = all good! */
	}
	else
	{
		Serial.println("Ok.");
	}

	BNO055.SystemError = ReadByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_SYS_ERR);
	Serial.print("BNO055.SystemError= ");
	if (BNO055.SystemError != 0)
	{
		Serial.println(BNO055.SystemError, HEX);
		/* System Error (see section 4.3.59)
		   ---------------------------------
		   0 = No error
		   1 = Peripheral initialization error
		   2 = System initialization error
		   3 = Self test result failed
		   4 = Register map value out of range
		   5 = Register map address out of range
		   6 = Register map write error
		   7 = BNO low power mode not available for selected operat ion mode
		   8 = Accelerometer power mode not available
		   9 = Fusion algorithm configuration error
		   A = Sensor configuration error */
		delay(5000);
	}
	else
	{
		Serial.println("Ok.");
	}
}

void BNO055_getRevInfo()
{
	Serial.println(" ===================================== BNO055_getRevInfo ===========================================");
	// set_TCA9548A(multi_line_BNO);
	uint8_t a, b;
	WriteByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_PAGE_ID, 0); // Устанавливаем работы с регистрами нулевой страницы

	/* Check the accelerometer revision */
	BNO055.accel_rev = ReadByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_ACC_ID);
	Serial.print("BNO055.accel_rev: ");
	Serial.println(BNO055.accel_rev);

	/* Check the magnetometer revision */
	BNO055.mag_rev = ReadByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_MAG_ID);
	Serial.print("BNO055.mag_rev: ");
	Serial.println(BNO055.mag_rev);

	/* Check the gyroscope revision */
	BNO055.gyro_rev = ReadByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_GYR_ID);
	Serial.print("BNO055.gyro_rev: ");
	Serial.println(BNO055.gyro_rev);

	/* Check the SW revision */
	BNO055.bl_rev = ReadByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_BL_REV_ID);
	Serial.print("BNO055.bl_rev: ");
	Serial.println(BNO055.bl_rev);

	a = ReadByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_SW_REV_ID_LSB);
	b = ReadByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_SW_REV_ID_MSB);
	BNO055.sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
	Serial.print("BNO055.sw_rev: ");
	Serial.println(BNO055.sw_rev);
	Serial.print("a: ");
	Serial.print(a);
	Serial.print(" b: ");
	Serial.println(b);
}

void BNO055_GetID_from_BNO055() // Считывание уникального номера. Не работает.Возвращает нули
{
	Serial.println("- Start BNO055_GetID_from_BNO055-");
	byte BNO055_ID_Array[16];
	WriteByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_PAGE_ID, 0); // Устанавливаем работы с регистрами нулевой страницы
	delay(100);

	BNO055_SetMode(eCONFIGMODE); /* Go to config mode if not there */
	delay(100);

	WriteByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_PAGE_ID, 1); // Устанавливаем работы с регистрами нулевой страницы
	delay(100);

	Wire.beginTransmission(BNO055_ADDRESS);
	Wire.write((uint8_t)50);
	Wire.endTransmission();
	Wire.requestFrom(BNO055_ADDRESS, (int)16); // Считываем последовательно 16 байта начиная с 50 адреса
	for (uint8_t i = 0; i < 16; i++)
	{
		BNO055_ID_Array[i] = Wire.read();
	}
	delay(100);
	WriteByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_PAGE_ID, 0); // Устанавливаем работы с регистрами нулевой страницы
	delay(100);
	// for (uint8_t i = 0; i < 22; i = i + 1)
	//{
	//	//Serial.print(i); Serial.print(" = ");Serial.println(BNO055_Offset_Array[i],BIN);
	// }
	for (uint8_t i = 0; i < 16; i++)
	{
		// Serial.print(i);
		Serial.print(" = ");
		Serial.print(BNO055_ID_Array[i]);
		// Serial.print(BNO055_ID_Array[i + 1] << 8 | BNO055_ID_Array[i]);
	}
	Serial.println("- End BNO055_GetID_from_BNO055-");
}

void BNO055_GetOffset_from_BNO055()
{
	Serial.println("BNO055_GetOffset_from_BNO055");
	// set_TCA9548A(multi_line_BNO);

	BNO055_SetMode(eCONFIGMODE); /* Go to config mode if not there */

	Wire.beginTransmission(BNO055_ADDRESS);
	Wire.write((uint8_t)eBNO055_REGISTER_ACC_OFFSET_X_LSB);
	Wire.endTransmission();
	Wire.requestFrom(BNO055_ADDRESS, (int)22); // Считываем последовательно 22 байта начиная с 55 адреса
	for (uint8_t i = 0; i < 22; i++)
	{
		BNO055_Offset_Array[i] = Wire.read();
	}
	delay(100);

	// for (uint8_t i = 0; i < 22; i = i + 1)
	//{
	//	//Serial.print(i); Serial.print(" = ");Serial.println(BNO055_Offset_Array[i],BIN);
	// }
	for (uint8_t i = 0; i < 22; i++)
	{
		// Serial.print(i);
		Serial.print(" = ");
		// Serial.print(BNO055_Offset_Array[i + 1] << 8 | BNO055_Offset_Array[i]);
		Serial.print(BNO055_Offset_Array[i]);
	}
	Serial.println("---");
}
void BNO055_SetOffset_toBNO055(byte offsetArray_[22])
{
	Serial.println("BNO055_SetOffset_toBNO055");
	BNO055_SetMode(eCONFIGMODE); /* Go to config mode if not there */

	for (uint8_t i = 0; i < 22; i++)
	{
		// Serial.print(i);
		Serial.print(" - ");
		Serial.print(offsetArray_[i]);
	}
	Serial.println(" = ");

	Wire.beginTransmission(BNO055_ADDRESS);
	Wire.write(eBNO055_REGISTER_ACC_OFFSET_X_LSB);
	for (uint8_t i = 0; i < 22; i++)
	{
		Wire.write(offsetArray_[i]);
	}
	byte rezult = Wire.endTransmission();
	Serial.print("Rezult Wire.endTransmission = ");
	Serial.println(rezult);
	delay(100);

	//--------------------------------------------------------
	// Serial.println("/// TEST READ *** ");

	Wire.beginTransmission(BNO055_ADDRESS);
	Wire.write((uint8_t)eBNO055_REGISTER_ACC_OFFSET_X_LSB);
	Wire.endTransmission();
	Wire.requestFrom(BNO055_ADDRESS, (int)22); // Считываем последовательно 22 байта начиная с 55 адреса
	// Serial.print("Wire.available= ");
	Serial.println(Wire.available());
	for (uint8_t i = 0; i < 22; i++)
	{
		BNO055_Offset_Array[i] = Wire.read();
	}
	delay(100);

	for (uint8_t i = 0; i < 22; i++)
	{
		// Serial.print(i);
		Serial.print(" - ");
		Serial.print(BNO055_Offset_Array[i]);
	}
	Serial.println(" = ");

	BNO055_getStatusInfo();
	Serial.println("---");
}

void Calibrovka_BNO055()
{
	set_TCA9548A(multi_line_BNO2);
	BNO055_SetMode(eIMU); // Режим работы гкалибровки где всключены все три устройства аксель гиро и магнит
	delay(500);

	Serial.println("Calibrovka_BNO055...");
	while (BNO055_getCalibration() == false) // Пока не откалибровалась нужно вертеть машинку
	{
		delay(250);
	}
	BNO055_GetOffset_from_BNO055(); // Считываем из датчика
	delay(99999999999);				// Ждем и переписываев вручную данные в массив
}

void Init_BNO055()
{
	Serial.println(String(millis()) + " Init_BNO055");

	// pinMode(PIN_BNO055_Mode, OUTPUT);
	// digitalWrite(PIN_BNO055_Mode, 1);     // Подаем 1 что-бы адрес был всегда один и тоже   /* 0x28 com3 low 0x29 com3 high     */

	// scanI2C();

	int timeOut = 0;
	byte WIA_MPU = ReadByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_CHIP_ID);

	// Serial.println("1"); 	BNO055_getInfo();
	Serial.print(String(millis()) + " WIA_MPU BNO055: ");
	Serial.print(WIA_MPU, BIN);
	if (WIA_MPU == BNO055_ID)
	{
		Serial.println(" eBNO055_REGISTER_SYS_TRIGGER to  BNO055. RESET");
		WriteByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_SYS_TRIGGER, 0b00100000); /* reset the sensor */
		delay(600);
		while (ReadByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_CHIP_ID) != BNO055_ID)
		{
			delay(100);
			Serial.println(String(millis()) + " RESET BNO055.... ");

			if (++timeOut == 1000)
				Serial.println(String(millis()) + "RESET BNO055 NOT ANSWER OVER 1 secunds !!! ");
		}
		WIA_MPU = ReadByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_CHIP_ID);
		Serial.print(String(millis()) + " WIA_MPU BNO055: ");
		Serial.print(WIA_MPU, BIN);
		if (WIA_MPU == BNO055_ID)
		{
			Serial.println(" Successfully connected to  BNO055 after RESET.");
		}

		delay(100);

		BNO055_SetMode(eCONFIGMODE); /* Go to config mode if not there */
		delay(25);
		WriteByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_PAGE_ID, 0); // Устанавливаем работы с регистрами нулевой страницы
		delay(25);
		// Нормальный режим работы по питанию
		WriteByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_PWR_MODE, eNORMAL_POWER_MODE);
		delay(25);

		/* Set the output units */ // ВНИМАНИЕ в даташиде ошибка в пояснении по номерам битов
		// uint8_t unitsel = (0 << 7) | // Orientation = Windows
		//				  (0 << 4) | // Temperature = Celsius
		//				  (0 << 2) | // Euler = Degrees
		//				  (0 << 1) | // Gyro = DPS
		//				  (0 << 0);  // Accelerometer = m/s^2
		// WriteByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_UNIT_SEL, unitsel);
		// delay(25);

		/* Configure axis mapping (see section 3.4) */
		// WriteByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_AXIS_MAP_CONFIG, eREMAP_CONFIG_P5); // P0-P7, Default is P1
		// delay(25);
		// WriteByte_I2C(BNO055_ADDRESS, eBNO055_REGISTER_AXIS_MAP_SIGN, eREMAP_SIGN_P5); // P0-P7, Default is P1
		// delay(10);

		Serial.println(String(millis()) + " BNO055_INFO:");
		BNO055_getStatusInfo();
		BNO055_getRevInfo();
		Serial.println("---------------------------------------");
		// BNO055_GetID_from_BNO055();

		Serial.println(String(millis()) + " END Init BNO055.");
		delay(250);
	}
	else
	{
		Serial.println(String(millis()) + " Failed to Connect to BNO055 !!!!!!!!!!!!!!");
		while (1)
		{
			/* code */
		}
		;
	}
}

void Setup_BNO055()
{
	Serial.println(String(millis()) + " Setup_BNO055");
	set_TCA9548A(multi_line_BNO2); // Выбор линии с датчиком с которым будем работать
	Init_BNO055();
	// Calibrovka_BNO055(); // Калибруем если первый раз и нет еще калибровки для этого датчика.
	BNO055_SetOffset_toBNO055(BNO055_Offset_Array_dafault2); // Установка нужного массива с калибровочными данными
	// Запуск датчика в заданном режиме
	Serial.println(" ================== BNO055_Start =================");
	BNO055_SetMode(eIMU); // Режим работы где он все сам считает	  eIMU
	delay(500);
	BNO055_readData(); // Разовое считывание данных
	Serial.println(" ================== Read Start Angle =================");
	Serial.print(" BNO055_EulerAngles.x = ");
	Serial.print(BNO055_EulerAngles.x);
	Serial.print(" BNO055_EulerAngles.y = ");
	Serial.print(BNO055_EulerAngles.y);
	Serial.print(" BNO055_EulerAngles.z = ");
	Serial.print(BNO055_EulerAngles.z);
	Serial.println(" ================== BNO055_End =================");
	// delay(999999999);
}
