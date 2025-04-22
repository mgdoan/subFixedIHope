/*
 * can_sub.h
 *
 *  Created on: Apr 17, 2025
 *      Author: mgdoan
 */

#ifndef SRC_CAN_SUB_H_
#define SRC_CAN_SUB_H_



#endif /* SRC_CAN_SUB_H_ */

CAN_HandleTypeDef hcan1;

CAN_TxHeaderTypeDef TxHeader;
CAN_TxHeaderTypeDef sensorHeader;

CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef canfilterconfig;

uint8_t RxData[8];

uint8_t TxData[8];
uint32_t TxMailbox;

uint64_t sensorMailbox;
int datacheck;

uint8_t sensorPacket[8];

uint8_t prevDEC = 0;
uint8_t prevINC = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	//printf("triggered\n");

	if(RxHeader.StdId == 0x128)
	{
		//printf("match\n");
		datacheck = 1;
	}
}

void can_setup(){
	 canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	 canfilterconfig.FilterBank = 5;  // which filter bank to use from the assigned ones
	 canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	 canfilterconfig.FilterIdHigh = 0x128<<5;
	 canfilterconfig.FilterIdLow = 0;
	 canfilterconfig.FilterMaskIdHigh = 0x128<<5;
	 canfilterconfig.FilterMaskIdLow = 0x0000;
	 canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	 canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	 canfilterconfig.SlaveStartFilterBank = 10;  // how many filters to assign to the CAN1 (master can)


	 HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);


	 //using standard ID (don't need extended)
	 TxHeader.IDE = CAN_ID_STD;

	 //ID of transmitter
	 TxHeader.StdId = 0x256;

	 //Set it to send a data frame
	 TxHeader.RTR = CAN_RTR_DATA;

	 //length of message
	 TxHeader.DLC = 8;

	 TxData[0] = 0x69;

	HAL_CAN_Start(&hcan1);

	HAL_StatusTypeDef activate_return = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	printf("notification output: %d\n", activate_return);
}

uint8_t read_control_from_CAN(){
	//  printf("reading = %d\r\n", RxHeader.StdId);
	  static uint8_t control = 0;
	  if(datacheck)
	  {
		  control = RxData[0];
		  datacheck = 0;
	  }
	  //printf("controller command is  = %d\r\n", control);
	  return control;
}

void transmit_sensor_packet(uint16_t gyroReading, uint32_t presTempReading, uint16_t flowReading){
	// Gyro
	uint8_t gyroMSB, gyroLSB;
	gyroMSB = (gyroReading >> 8) & 0xFF;
	gyroLSB = gyroReading & 0xFF;

	printf("gyro reading is  = %d\r\n", gyroReading);
	printf("pulse reading is  = %d\r\n", flowReading);

	TxData[0] = gyroMSB;
	TxData[1] = gyroLSB;

	// Pressure sensor
	uint16_t pres, temp;
	pres = (presTempReading >> 16) & 0xFFFF;
	temp = presTempReading & 0xFFFF;

	printf("pressure reading is  = %d\r\n", pres);
	printf("temp reading is  = %d\r\n", temp);

	uint8_t presLSB, presMSB, tempLSB, tempMSB;
	presMSB = (pres >> 8) & 0xFF;
	presLSB = pres & 0xFF;
	tempMSB = (temp >> 8) & 0xFF;
	tempLSB = temp & 0xFF;

	TxData[2] = presMSB;
	TxData[3] = presLSB;
	TxData[4] = tempMSB;
	TxData[5] = tempLSB;

	// Flow sensor
	uint8_t flowMSB, flowLSB;

	flowMSB = (flowReading >> 8) & 0xFF;
	flowLSB = flowReading & 0xFF;

	TxData[6] = flowMSB;
	TxData[7] = flowLSB;

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
	printf("HAL CAN Transmit Result: %d\r\n", HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox));
}

void controlThrusterStateCAN(uint8_t controllerCommand){
	// turn right
	if(controllerCommand & 0b1){
			printf("turn right (activate left thruster) \r\n");
			set_thruster(0);
			stop_thruster(1);
		 }
	// turn left
	else if((controllerCommand >> 1) & 0b1){
		printf("turn right (activate right thruster) \r\n");
		set_thruster(1);
		stop_thruster(0);
	}
	// go forward
	else if((controllerCommand >> 3) & 0b1){
		printf("forwards \r\n");
		set_thruster(0);
		set_thruster(1);
	}


//	else if((controllerCommand >> 2) & 0b1){ //REVERSE?
//		printf("reverse \r\n");
//		set_thruster_reverse(0);
//		set_thruster_reverse(1);
//	}
	else {
		stop_thruster(0);
		stop_thruster(1);
	}

}

void controlThrusterSpeedCAN(uint8_t controllerCommand) {
    // decode current raw bits
    uint8_t inc = (controllerCommand >> 4) & 0x1;
    uint8_t dec = (controllerCommand >> 5) & 0x1;

    // on rising edge of INCREASE (but not hold both buttons)
    if (inc && !prevInc && !dec) {
        pwm_val_increase();
    }
    // on rising edge of DECREASE
    else if (dec && !prevDec && !inc) {
        pwm_val_decrease();
    }

    // update "previous" for next call
    prevInc = inc;
    prevDec = dec;
}


void controlLinearActuatorCAN(uint8_t controllerCommand) {
	// decrease speed
		if((controllerCommand >> 7) & 0b1 && !((controllerCommand >> 6) & 0b1)){//up
			printf("syringe up\r\n");
			goUp();
		 }
		 if(!((controllerCommand >> 7) & 0b1) && (controllerCommand >> 6) & 0b1){//down
			 printf("syringe down\r\n");
			 goDown();
		 }
		 if(!((controllerCommand >> 7) & 0b1) && !((controllerCommand >> 6)) & 0b1){//down
			 stop();
		 }
}
