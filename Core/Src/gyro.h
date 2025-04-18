/*
 * gryo.h
 *
 *  Created on: Apr 9, 2025
 *      Author: elainamn
 */

#ifndef SRC_GRYO_H_
#define SRC_GRYO_H_



#endif /* SRC_GRYO_H_ */

#define GYRO_ADDR (0x28 << 1)

#define OPR_MODE_ADDR 0x3D
//#define GYROONLY_MODE 0x03 //xxxx0011b
#define IMU_FUSION_MODE 0x08;

//#define GYRO_CONFIG_0_ADDR 0xA
//#define DPS 0x4 //100

#define UNIT_SELECT_ADDR 0x3B
#define UNIT_SELECT_DEGREES 0x0 //xxxxx0xxb

//#define GRYO_DATA_X 0x14 //read 2 bytes
//#define GRYO_DATA_Y 0x16 //read 2 bytes
//#define GRYO_DATA_Z 0x18 //read 2 bytes

#define HEADING_ADDR 0x1A

extern I2C_HandleTypeDef hi2c1;


void diagnose_gyro(HAL_StatusTypeDef ret, char* message) {
	if (ret != HAL_OK) {
		printf(message);
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

void gyro_init() {
	uint8_t buf[2];
	HAL_StatusTypeDef ret;

	printf("\r\n\r\n-------GYRO INIT-------\r\n");


	// set operating mode ---------------------------------------
	buf[0] = OPR_MODE_ADDR;
	buf[1] = IMU_FUSION_MODE;
	ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, buf, 2, 1000);
	diagnose_gyro(ret, "error setting op mode");

	// set unit  ---------------------------------------
	buf[0] = UNIT_SELECT_ADDR;
	buf[1] = UNIT_SELECT_DEGREES;
	ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, buf, 2, 1000);

	diagnose_gyro(ret, "error setting unit");

	return;
}

void gyro_test() {

	uint8_t buf[2];
	HAL_StatusTypeDef ret;
	uint16_t heading;

	buf[0] = HEADING_ADDR;
	ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, buf, 1, 1000);
	ret = HAL_I2C_Master_Receive(&hi2c1, GYRO_ADDR, buf, 2, 1000);
	heading = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
	double or_deg = (double)heading/16;

	diagnose_gyro(ret, "ERROR: read heading");
	printf("heading (deg): %f\r\n", or_deg);


	return;
}

uint16_t gyroGetReading() {
	uint8_t buf[2];
	HAL_StatusTypeDef ret;
	uint16_t heading;
	uint16_t heading_calibrated;

	// I2C
	buf[0] = HEADING_ADDR;
	ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, buf, 1, 1000);
	ret = HAL_I2C_Master_Receive(&hi2c1, GYRO_ADDR, buf, 2, 1000);

	// Concatenate the data and calculate the heading
	heading = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
	heading_calibrated = heading/16;
	return heading_calibrated;

}


//void read_gyro_raw() {
//	uint8_t buf[2];
//	HAL_StatusTypeDef ret;
//	uint16_t x_angle, y_angle, z_angle;
//
//	buf[0] = GRYO_DATA_X;
//	ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, buf, 1, 1000);
//	ret = HAL_I2C_Master_Receive(&hi2c1, GYRO_ADDR, buf, 2, 1000);
//	x_angle = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
//	diagnose_gyro(ret, "ERROR: read x data");
//
//	buf[0] = GRYO_DATA_Y;
//	ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, buf, 1, 1000);
//	ret = HAL_I2C_Master_Receive(&hi2c1, GYRO_ADDR, buf, 2, 1000);
//	y_angle = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
//	diagnose_gyro(ret, "ERROR: read y data");
//
//
//	buf[0] = GRYO_DATA_Z;
//	ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, buf, 1, 1000);
//	ret = HAL_I2C_Master_Receive(&hi2c1, GYRO_ADDR, buf, 2, 1000);
//	z_angle = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
//	diagnose_gyro(ret, "ERROR: read z data");
//
//	printf("\r\n");
//	printf("x: %d", (int)x_angle);
//	printf("y: %d", (int)y_angle);
//	printf("z: %d", (int)z_angle);
//
