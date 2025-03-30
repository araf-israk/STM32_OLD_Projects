/*
 * NRF24L01.c
 *
 *  Created on: Mar 4, 2024
 *      Author: arafi
 */


#include "stm32f4xx_hal.h"
#include "NRF24L01.h"

extern SPI_HandleTypeDef hspi1;
#define NRF24_SPI &hspi1;


#define CE_Pin 			GPIO_PIN_0
#define CE_GPIO_Port 	GPIOB
#define CS_Pin 			GPIO_PIN_1
#define CS_GPIO_Port 	GPIOB

void CS_Select(void){
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}

void CS_UnSelect(void){
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

void CE_Enable(void){
	HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
}

void CE_Disable(void){
	HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
}

void nrf24_WriteReg(uint8_t Reg, uint8_t Data){

	uint8_t buf[2];

	buf[0] = Reg | 1<<5;
	buf[1] = Data;

	CS_Select();

	HAL_SPI_Transmit(&hspi1, buf, 2, 100);

	CS_UnSelect();

}

void nrf24_WriteRegMulti(uint8_t Reg, uint8_t *data, int size){

	uint8_t buf[2];

	buf[0] = Reg | 1<<5;


	CS_Select();

	HAL_SPI_Transmit(&hspi1, buf, 1, 100);
	HAL_SPI_Transmit(&hspi1, data, size, 100);

	CS_UnSelect();

}

uint8_t nrf24_ReadReg(uint8_t Reg){
	uint8_t data = 0;

	CS_Select();

	HAL_SPI_Transmit(&hspi1, &Reg, 1, 100);
	HAL_SPI_Receive(&hspi1, &data, 1, 100);

	CS_UnSelect();

	return data;
}

void nrf24_ReadReg_Multi(uint8_t Reg, uint8_t *data, int size){

	CS_Select();

	HAL_SPI_Transmit(&hspi1, &Reg, 1, 100);
	HAL_SPI_Receive(&hspi1, data, size, 100);

	CS_UnSelect();
}


void nrfsendCmd(uint8_t cmd){
	CS_Select();

	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);

	CS_UnSelect();

}

void nrf24_reset(uint8_t REG)
{
	if (REG == STATUS)
	{
		nrf24_WriteReg(STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS)
	{
		nrf24_WriteReg(FIFO_STATUS, 0x11);
	}

	else {
	nrf24_WriteReg(CONFIG, 0x08);
	nrf24_WriteReg(EN_AA, 0x3F);
	nrf24_WriteReg(EN_RXADDR, 0x03);
	nrf24_WriteReg(SETUP_AW, 0x03);
	nrf24_WriteReg(SETUP_RETR, 0x03);
	nrf24_WriteReg(RF_CH, 0x02);
	nrf24_WriteReg(RF_SETUP, 0x0E);
	nrf24_WriteReg(STATUS, 0x00);
	nrf24_WriteReg(OBSERVE_TX, 0x00);
	nrf24_WriteReg(CD, 0x00);
	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_WriteRegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	nrf24_WriteRegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
	nrf24_WriteReg(RX_ADDR_P2, 0xC3);
	nrf24_WriteReg(RX_ADDR_P3, 0xC4);
	nrf24_WriteReg(RX_ADDR_P4, 0xC5);
	nrf24_WriteReg(RX_ADDR_P5, 0xC6);
	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_WriteRegMulti(TX_ADDR, tx_addr_def, 5);
	nrf24_WriteReg(RX_PW_P0, 0);
	nrf24_WriteReg(RX_PW_P1, 0);
	nrf24_WriteReg(RX_PW_P2, 0);
	nrf24_WriteReg(RX_PW_P3, 0);
	nrf24_WriteReg(RX_PW_P4, 0);
	nrf24_WriteReg(RX_PW_P5, 0);
	nrf24_WriteReg(FIFO_STATUS, 0x11);
	nrf24_WriteReg(DYNPD, 0);
	nrf24_WriteReg(FEATURE, 0);
	}
}

void NRF24_Init(void){
	CE_Disable();

	nrf24_reset(0);

	nrf24_WriteReg(CONFIG, 0);

	nrf24_WriteReg(EN_AA, 0);

	nrf24_WriteReg(EN_RXADDR, 0);

	nrf24_WriteReg(SETUP_AW, 0x03);

	nrf24_WriteReg(SETUP_RETR, 0);

	nrf24_WriteReg(RF_CH, 0);

	nrf24_WriteReg(RF_SETUP, 0x0E);

	CE_Enable();
}


void NRF24_TxMode(uint8_t *Address, uint8_t channel){

	CE_Disable();

	nrf24_WriteReg(RF_CH, channel);

	nrf24_WriteRegMulti(TX_ADDR, Address, 5);

	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1);
	nrf24_WriteReg(CONFIG, config);

	CE_Enable();
}

uint8_t NRF24_Transmit(uint8_t *data){

	uint8_t cmdtosend = 0;

	CS_Select();

	cmdtosend = W_TX_PAYLOAD;
	HAL_SPI_Transmit(&hspi1, &cmdtosend, 1, 100);

	HAL_SPI_Transmit(&hspi1, data, 32, 100);

	CS_UnSelect();

	HAL_Delay(1);

	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

	if((fifostatus&(1<<4)) && (!(fifostatus&(1<<3)))){
		cmdtosend = FLUSH_TX;
		nrfsendCmd(cmdtosend);

		nrf24_reset(FIFO_STATUS);

		return 1;
	}

	return 0;
}


void NRF24_RxMode(uint8_t *Address, uint8_t channel){

	CE_Disable();

	nrf24_WriteReg(RF_CH, channel);

	uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);

	en_rxaddr = en_rxaddr | (1<<1);

	nrf24_WriteReg(EN_RXADDR, en_rxaddr);


	nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5);

	nrf24_WriteReg(RX_PW_P1, 32);


	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1) | (1<<0);
	nrf24_WriteReg(CONFIG, config);

	CE_Enable();

}

uint8_t isDataAvailable(int pipenum){
	uint8_t status = nrf24_ReadReg(STATUS);

	if((status&(1<<6)) && (status&(pipenum<<1))){
		nrf24_WriteReg(STATUS, (1<<6));

		return 1;
	}

	return 0;
}

void NRF24_Receive(uint8_t *data){

	uint8_t cmdtosend = 0;

	CS_Select();

	cmdtosend = R_RX_PAYLOAD;

	HAL_SPI_Transmit(&hspi1, &cmdtosend, 1, 100);

	HAL_SPI_Receive(&hspi1, data, 32, 1000);

	CS_UnSelect();

	HAL_Delay(1);

	cmdtosend = FLUSH_RX;

	nrfsendCmd(cmdtosend);
}
