/*
 * pressure_temp.h
 *
 * NOTE: if pressure or temp changes, need to recalibrate
 *
 * USAGE:
 * pressure_temp_Init();
 * pressure_temp_calibrate();
 *
 * PRES_TEMP pres_temp;
 * get_pressure_temp(&pres_temp);
 *
 * printf("pressure: %f\r\n", pres_temp.pressure); //in units of millibar (mbar)
 * printf("temperature: %f\r\n", pres_temp.temperature); //in degrees celcius
 *
 *  Created on: Mar 28, 2025
 *      Author: elainamn
 */

#ifndef SRC_PRESSURE_TEMP_H_
#define SRC_PRESSURE_TEMP_H_



#endif /* SRC_PRESSURE_TEMP_H_ */

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
extern I2C_HandleTypeDef hi2c1;


#define PRESSURE_ADDR (0x76 << 1)

////////////////////////////////////////////////////
//                                                //
//  PROM is the calibration data for the sensor   //
//  needed to calculate pressure and temperature  //
//                                                //
//   PRES_TEMP will hold the values of pressure   //
//   and temperature                              //
//                                                //
////////////////////////////////////////////////////

typedef struct {
	uint16_t c0; //factory
	uint16_t c1; //pressure sensitivity
	uint16_t c2; //pressure offset
	uint16_t c3; //temp coefficient of pressure sensitivity
	uint16_t c4; //temp coefficient of pressure offset
	uint16_t c5; //reference temp
	uint16_t c6; //temp coefficient of temp
} PROM;

PROM prom;


typedef struct {
	float pressure;
	float temperature;
}PRES_TEMP;


////////////////////////////////////////////////////
//                                                //
//        Helper functions for debugging          //
//                                                //
////////////////////////////////////////////////////

void i2c_check (HAL_StatusTypeDef ret, char* message) {
	if (ret != HAL_OK) {
		printf("Pressure Sensor Error: %s\r\n", message);
	} else {
		printf("Success: %s\r\n", message);
	}
}

void print_binary_uint16(uint16_t num) {
    for (int i = 15; i >= 0; i--) {
        printf("%d", (num >> i) & 1);
    }
    printf("\r\n");
}

void print_binary_uint8(uint8_t num) {
    for (int i = 7; i >= 0; i--) {
        printf("%d", (num >> i) & 1);
    }
}

void print_PROM (PROM prom) {
	printf("----PROM---\r\n");
	printf("factory:  "); print_binary_uint16(prom.c0);
	printf("SENS_T1:  %u\r\n", (unsigned int)(prom.c1));
	printf("OFF_T1:   %u\r\n", (unsigned int)(prom.c2));
	printf("TCS:      %u\r\n", (unsigned int)(prom.c3));
	printf("TCO:      %u\r\n", (unsigned int)(prom.c4));
	printf("T_ref:    %u\r\n", (unsigned int)(prom .c5));
	printf("TEMPSENS: %u\r\n\r\n", (unsigned int)(prom.c6));
}

void print_buf(uint8_t buf[3]) {
	printf("----BUF----\r\n");
	printf("buf[0]: "); print_binary_uint8(buf[0]); printf("\r\n");
	printf("buf[1]: "); print_binary_uint8(buf[1]); printf("\r\n");
	printf("buf[2]: "); print_binary_uint8(buf[2]); printf("\r\n");
}

void clear_buf(uint8_t *buf) {
	memset(buf, 0, 3 * sizeof(*buf));
}

void diagnose_pressure(HAL_StatusTypeDef ret) {
	if(ret == HAL_BUSY){
		printf("hal_busy\r\n");
	}
	if(ret == HAL_TIMEOUT){
			printf("hal_timeout\r\n");
	}
	if(ret == HAL_ERROR){
			printf("hal_error\r\n");
	}
	if (ret == HAL_ERROR) {
		printf("\r\n");
		 uint32_t errorCode = HAL_I2C_GetError(&hi2c1);

		 if (errorCode == HAL_I2C_ERROR_NONE) {
			 printf("No Error\r\n");
		 } else if (errorCode & HAL_I2C_ERROR_BERR) {
			 printf("Bus Error\r\n");
		 } else if (errorCode & HAL_I2C_ERROR_ARLO) {
			 printf("Arbitration Lost Error\r\n");
		 } else if (errorCode & HAL_I2C_ERROR_AF) {
			 printf("Acknowledge Failure Error\r\n");
		 } else if (errorCode & HAL_I2C_ERROR_OVR) {
			 printf("Overrun/Underrun Error\r\n");
		 } else if (errorCode & HAL_I2C_ERROR_DMA) {
			 printf("DMA Transfer Error\r\n");
		 } else if (errorCode & HAL_I2C_ERROR_TIMEOUT) {
			 printf("Timeout Error\r\n");
		 } else if (errorCode & HAL_I2C_ERROR_SIZE) {
			 printf("Size Management Error\r\n");
		 } else {
			 printf("Unknown Error: 0x%08lX\r\n", errorCode);
		 }

	  }
}

////////////////////////////////////////////////////
//                                                //
//             Sensor Calibration                 //
//                                                //
////////////////////////////////////////////////////


void pressure_temp_Init(void) {

	HAL_StatusTypeDef ret;
	uint8_t buf[3];
	clear_buf(buf);

	printf("\r\n\r\n-----Begin Pressure Sensor-----\r\n");

	//reset (needed to reconfigure on boot) ---------------------
	HAL_Delay(200);
	buf[0] = 0x1E;//reset command
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	diagnose_pressure(ret);
	i2c_check(ret, "reset command");
	HAL_Delay(200);

}


void pressure_temp_calibrate(void) {

	HAL_StatusTypeDef ret;
	uint8_t buf[3];
	clear_buf(buf);

	printf("Calibrating Pressure Sensor...\r\n");

	//read PROM (calibration data) ------------------------------
	HAL_Delay(100);

	buf[0] = 0xA0; //PROM read command c0
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "command to read PROM data");
	ret = HAL_I2C_Master_Receive(&hi2c1, PRESSURE_ADDR, buf, 2, 1000);
	i2c_check(ret, "receiving PROM data - c1");
	prom.c0 =  ((uint16_t)buf[0] << 8)  | buf[1];  //C0

	buf[0] = 0xA2; //PROM read command c1
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "command to read PROM data");
	ret = HAL_I2C_Master_Receive(&hi2c1, PRESSURE_ADDR, buf, 2, 1000);
	i2c_check(ret, "receiving PROM data - c1");
	prom.c1 =  ((uint16_t)buf[0] << 8)  | buf[1];  //C1

	buf[0] = 0xA4; //PROM read command c2
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "command to read PROM data");
	ret = HAL_I2C_Master_Receive(&hi2c1, PRESSURE_ADDR, buf, 2, 1000);
	i2c_check(ret, "receiving PROM data - c2");
	prom.c2 =   ((uint16_t)buf[0] << 8)  | buf[1];  //C2

	buf[0] = 0xA6; //PROM read command c3
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "command to read PROM data");
	ret = HAL_I2C_Master_Receive(&hi2c1, PRESSURE_ADDR, buf, 2, 1000);
	i2c_check(ret, "receiving PROM data - c3");
	prom.c3 =      ((uint16_t)buf[0] << 8)  | buf[1];  //C3

	buf[0] = 0xA8; //PROM read command c4
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "command to read PROM data");
	ret = HAL_I2C_Master_Receive(&hi2c1, PRESSURE_ADDR, buf, 2, 1000);
	i2c_check(ret, "receiving PROM data - c4");
	prom.c4 =      ((uint16_t)buf[0] << 8)  | buf[1];  //C4

	buf[0] = 0xAA; //PROM read command c5
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "command to read PROM data");
	ret = HAL_I2C_Master_Receive(&hi2c1, PRESSURE_ADDR, buf, 2, 1000);
	i2c_check(ret, "receiving PROM data - c5");
	prom.c5 =    ((uint16_t)buf[0] << 8) | buf[1]; //C5

	buf[0] = 0xAC; //PROM read command c6
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "command to read PROM data");
	ret = HAL_I2C_Master_Receive(&hi2c1, PRESSURE_ADDR, buf, 2, 1000);
	i2c_check(ret, "receiving PROM data - c6");
	prom.c6 = ((uint16_t)buf[0] << 8) | buf[1]; //C6

}


////////////////////////////////////////////////////
//                                                //
// Getting pressure(mbar) & temperature (celcius) //
//                                                //
////////////////////////////////////////////////////

void get_pressure_temp( PRES_TEMP *pres_temp ) {

	HAL_StatusTypeDef ret;
	uint8_t buf[3];
	clear_buf(buf);

	//read digital pressure raw data (D1)  ------------------------
	clear_buf(buf);
	buf[0] = 0x48; // initiate pressure conversion command = 01001000
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "init D1 reading");
	//HAL_Delay(100);


	buf[0] = 0x0; // ADC read sequence command = 00000000
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "command to receive D1 data");

	ret = HAL_I2C_Master_Receive(&hi2c1, PRESSURE_ADDR, buf, 3, 1000); //read
	i2c_check(ret,"receiving D1 data");
	//print_buf(buf);
	uint32_t D1 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];

	//read digital temperature raw data (D2) -----------------------
	buf[0] = 0x58; // initiate temperature conversion command = 01011000
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "init D2 reading");
	//HAL_Delay(100);

	buf[0] = 0x0; // ADC read sequence command = 00000000
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "command to receive D2 data");

	ret = HAL_I2C_Master_Receive(&hi2c1, PRESSURE_ADDR, buf, 3, 1000); //read
	i2c_check(ret, "receiving D2 data");
	uint32_t D2 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
	//print_buf(buf);

	//Calculate temperature -----------------------------------------
	int32_t dT = D2 - (prom.c5 * 256); //difference between actual and reference temperature
	int32_t TEMP = 2000 + ((int64_t)dT*prom.c6) / 8388608; //actual temperature in Celcius

	//Calculate temperature compensated pressure ----------------------
	int64_t OFF = ((int64_t)prom.c2 * 65536) + ((int64_t)prom.c4 * dT)/128; //offset at actual temperature
	int64_t SENS = ((int64_t)prom.c1 * 32768) + ((int64_t)prom.c3 * dT)/256; //sensitivity at actual temperature

	float temp = TEMP / 100.0f;
	float pressure = (((D1*SENS) / 2097152 - OFF) / 8192) / 10.0f; //final temperature compensated pressure

	pres_temp -> temperature = temp;
	pres_temp -> pressure = pressure;

}

uint32_t get_pressure_temp_CAN( PRES_TEMP *pres_temp ) {

	HAL_StatusTypeDef ret;
	uint8_t buf[3];
	clear_buf(buf);

	//read digital pressure raw data (D1)  ------------------------
	clear_buf(buf);
	buf[0] = 0x48; // initiate pressure conversion command = 01001000
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "init D1 reading");
	//HAL_Delay(100);


	buf[0] = 0x0; // ADC read sequence command = 00000000
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "command to receive D1 data");

	ret = HAL_I2C_Master_Receive(&hi2c1, PRESSURE_ADDR, buf, 3, 1000); //read
	i2c_check(ret,"receiving D1 data");
	//print_buf(buf);
	uint32_t D1 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];

	//read digital temperature raw data (D2) -----------------------
	buf[0] = 0x58; // initiate temperature conversion command = 01011000
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "init D2 reading");
	//HAL_Delay(100);

	buf[0] = 0x0; // ADC read sequence command = 00000000
	ret = HAL_I2C_Master_Transmit(&hi2c1, PRESSURE_ADDR, buf, 1, 1000 );
	i2c_check(ret, "command to receive D2 data");

	ret = HAL_I2C_Master_Receive(&hi2c1, PRESSURE_ADDR, buf, 3, 1000); //read
	i2c_check(ret, "receiving D2 data");
	uint32_t D2 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
	//print_buf(buf);

	//Calculate temperature -----------------------------------------
	int32_t dT = D2 - (prom.c5 * 256); //difference between actual and reference temperature
	int32_t TEMP = 2000 + ((int64_t)dT*prom.c6) / 8388608; //actual temperature in Celcius

	//Calculate temperature compensated pressure ----------------------
	int64_t OFF = ((int64_t)prom.c2 * 65536) + ((int64_t)prom.c4 * dT)/128; //offset at actual temperature
	int64_t SENS = ((int64_t)prom.c1 * 32768) + ((int64_t)prom.c3 * dT)/256; //sensitivity at actual temperature

	float temp = TEMP / 100.0f;
	float pressure = (((D1*SENS) / 2097152 - OFF) / 8192) / 10.0f; //final temperature compensated pressure

	uint16_t lsb = temp;
	uint16_t msb = pressure;
	uint32_t presTemp = (msb << 16) | lsb;
	return presTemp;

}
