/*

  ******************************************************************************
  * @file 			( фаил ):   		GC9A01.c
  * @brief 		( описание ):  	
  ******************************************************************************
  * @attention 	( внимание ):	 	author: Golinskiy Konstantin	e-mail: golinskiy.konstantin@gmail.com
  ******************************************************************************
  
*/

#include "GC9A01.h"


extern SPI_HandleTypeDef hspi1;

uint16_t GC9A01_X_Start = GC9A01_XSTART;	
uint16_t GC9A01_Y_Start = GC9A01_YSTART;

uint16_t GC9A01_Width, GC9A01_Height;

#if FRAME_BUFFER
// массив буфер кадра
	uint16_t buff_frame[GC9A01_WIDTH*GC9A01_HEIGHT] = { 0x0000, };
#endif



#define LCD_RST_1 HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_SET)			// LCD_RST = 1 , LCD RESET pin
#define LCD_RST_0 HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_RESET)		// LCD_RST = 0 , LCD RESET pin

#define LCD_CS_1 HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_SET)			// LCD_CS = 1, LCD select pin
#define LCD_CS_0 HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_RESET)			// LCD_CS = 0, LCD select pin

#define LCD_DC_1 HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_SET)			// LCD_DC = 1, LCD Data/Command pin
#define LCD_DC_0 HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_RESET)			// LCD_DC = 0，LCD Data/Command pin

HAL_StatusTypeDef SPI_Write(uint8_t* pbuff, uint16_t size);

//##############################################################################

static void GC9A01_Unselect(void);
static void GC9A01_Select(void);
static void GC9A01_SendCmd(uint8_t Cmd);
static void GC9A01_SendData(uint8_t Data );
static void GC9A01_SendDataMASS(uint8_t* buff, size_t buff_size);
static void GC9A01_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
static void GC9A01_RamWrite(uint16_t *pBuff, uint32_t Len);
static void GC9A01_ColumnSet(uint16_t ColumnStart, uint16_t ColumnEnd);
static void GC9A01_RowSet(uint16_t RowStart, uint16_t RowEnd);
static void SwapInt16Values(int16_t *pValue1, int16_t *pValue2);
static void GC9A01_DrawLine_Slow(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);


//==============================================================================
// Процедура инициализации дисплея
//==============================================================================
//==== данные для инициализации дисплея GC9A01_240X240 ==========
void GC9A01_Init(void){
	
		// Задержка после подачи питания
		// если при старте не всегда запускаеться дисплей увеличиваем время задержки
		HAL_Delay(100);	

		GC9A01_Width = GC9A01_WIDTH;
		GC9A01_Height = GC9A01_HEIGHT;
	
		GC9A01_HardReset(); 

    GC9A01_Select();

		GC9A01_SendCmd(GC9A01_InnerReg2Enable);
		GC9A01_SendCmd(0xEB);        
		GC9A01_SendData(0x14);

		GC9A01_SendCmd(GC9A01_InnerReg1Enable);
		GC9A01_SendCmd(GC9A01_InnerReg2Enable);

		GC9A01_SendCmd(0xEB);        
		GC9A01_SendData(0x14);       

		GC9A01_SendCmd(0x84);       
		GC9A01_SendData(0x40);

		GC9A01_SendCmd(0x85);        
		GC9A01_SendData(0xFF);

		GC9A01_SendCmd(0x86);       
		GC9A01_SendData(0xFF);

		GC9A01_SendCmd(0x87);        
		GC9A01_SendData(0xFF);

		GC9A01_SendCmd(0x88);        
		GC9A01_SendData(0x0A);

		GC9A01_SendCmd(0x89);        
		GC9A01_SendData(0x21);

		GC9A01_SendCmd(0x8A);        
		GC9A01_SendData(0x00);

		GC9A01_SendCmd(0x8B);        
		GC9A01_SendData(0x80);

		GC9A01_SendCmd(0x8C);        
		GC9A01_SendData(0x01);

		GC9A01_SendCmd(0x8D);        
		GC9A01_SendData(0x01);

		GC9A01_SendCmd(0x8E);        
		GC9A01_SendData(0xFF);

		GC9A01_SendCmd(0x8F);        
		GC9A01_SendData(0xFF);

		GC9A01_SendCmd(GC9A01_DisplayFunctionControl);
		GC9A01_SendData(0x00);
		GC9A01_SendData(0x20); // Scan direction S360 -> S1

		// def rotation
		GC9A01_SendCmd(GC9A01_MADCTL);
		GC9A01_SendData(GC9A01_DEF_ROTATION);
		
		// ColorMode
		GC9A01_SendCmd(GC9A01_COLMOD);
		GC9A01_SendData(ColorMode_MCU_16bit & 0x77);

		GC9A01_SendCmd(0x90);        
		GC9A01_SendData(0x08);
		GC9A01_SendData(0x08);
		GC9A01_SendData(0x08);
		GC9A01_SendData(0x08);

		GC9A01_SendCmd(0xBD);        
		GC9A01_SendData(0x06);

		GC9A01_SendCmd(0xBC);        
		GC9A01_SendData(0x00);

		GC9A01_SendCmd(0xFF);        
		GC9A01_SendData(0x60);
		GC9A01_SendData(0x01);
		GC9A01_SendData(0x04);

		GC9A01_SendCmd(GC9A01_PWCTR2);  // Power control 2
		GC9A01_SendData(0x13);       // 5.18 V
		GC9A01_SendCmd(GC9A01_PWCTR3);  // Power control 3
		GC9A01_SendData(0x13);       // VREG2A = -3.82 V, VREG2B = 0.68 V
		GC9A01_SendCmd(GC9A01_PWCTR4);  // Power control 4
		GC9A01_SendData(0x22);       // VREG2A = 5.88 V, VREG2B = -2.88 V

		GC9A01_SendCmd(0xBE);        
		GC9A01_SendData(0x11);

		GC9A01_SendCmd(0xE1);        
		GC9A01_SendData(0x10);
		GC9A01_SendData(0x0E);

		GC9A01_SendCmd(0xDF);        
		GC9A01_SendData(0x21);
		GC9A01_SendData(0x0c);
		GC9A01_SendData(0x02);

		GC9A01_SendCmd(GC9A01_GAMMA1);
		GC9A01_SendData(0x45);
		GC9A01_SendData(0x09);
		GC9A01_SendData(0x08);
		GC9A01_SendData(0x08);
		GC9A01_SendData(0x26);
		GC9A01_SendData(0x2A);

		GC9A01_SendCmd(GC9A01_GAMMA2);
		GC9A01_SendData(0x43);
		GC9A01_SendData(0x70);
		GC9A01_SendData(0x72);
		GC9A01_SendData(0x36);
		GC9A01_SendData(0x37);
		GC9A01_SendData(0x6F);

		GC9A01_SendCmd(GC9A01_GAMMA3);
		GC9A01_SendData(0x45);
		GC9A01_SendData(0x09);
		GC9A01_SendData(0x08);
		GC9A01_SendData(0x08);
		GC9A01_SendData(0x26);
		GC9A01_SendData(0x2A);

		GC9A01_SendCmd(GC9A01_GAMMA4);
		GC9A01_SendData(0x43);
		GC9A01_SendData(0x70);
		GC9A01_SendData(0x72);
		GC9A01_SendData(0x36);
		GC9A01_SendData(0x37);
		GC9A01_SendData(0x6F);

		GC9A01_SendCmd(0xED);        
		GC9A01_SendData(0x1B);
		GC9A01_SendData(0x0B);

		GC9A01_SendCmd(0xAE);        
		GC9A01_SendData(0x77);

		GC9A01_SendCmd(0xCD);        
		GC9A01_SendData(0x63);

		GC9A01_SendCmd(0x70);        
		GC9A01_SendData(0x07);
		GC9A01_SendData(0x07);
		GC9A01_SendData(0x04);
		GC9A01_SendData(0x0E);
		GC9A01_SendData(0x0F);
		GC9A01_SendData(0x09);
		GC9A01_SendData(0x07);
		GC9A01_SendData(0x08);
		GC9A01_SendData(0x03);

		GC9A01_SendCmd(GC9A01_FRAMERATE);       // Frame rate
		GC9A01_SendData(0x34);                  // 4 dot inversion

		GC9A01_SendCmd(0x62);       
		GC9A01_SendData(0x18);
		GC9A01_SendData(0x0D);
		GC9A01_SendData(0x71);
		GC9A01_SendData(0xED);
		GC9A01_SendData(0x70);
		GC9A01_SendData(0x70);
		GC9A01_SendData(0x18);
		GC9A01_SendData(0x0F);
		GC9A01_SendData(0x71);
		GC9A01_SendData(0xEF);
		GC9A01_SendData(0x70);
		GC9A01_SendData(0x70);

		GC9A01_SendCmd(0x63);        
		GC9A01_SendData(0x18);
		GC9A01_SendData(0x11);
		GC9A01_SendData(0x71);
		GC9A01_SendData(0xF1);
		GC9A01_SendData(0x70);
		GC9A01_SendData(0x70);
		GC9A01_SendData(0x18);
		GC9A01_SendData(0x13);
		GC9A01_SendData(0x71);
		GC9A01_SendData(0xF3);
		GC9A01_SendData(0x70);
		GC9A01_SendData(0x70);

		GC9A01_SendCmd(0x64);        
		GC9A01_SendData(0x28);
		GC9A01_SendData(0x29);
		GC9A01_SendData(0xF1);
		GC9A01_SendData(0x01);
		GC9A01_SendData(0xF1);
		GC9A01_SendData(0x00);
		GC9A01_SendData(0x07);

		GC9A01_SendCmd(0x66);        
		GC9A01_SendData(0x3C);
		GC9A01_SendData(0x00);
		GC9A01_SendData(0xCD);
		GC9A01_SendData(0x67);
		GC9A01_SendData(0x45);
		GC9A01_SendData(0x45);
		GC9A01_SendData(0x10);
		GC9A01_SendData(0x00);
		GC9A01_SendData(0x00);
		GC9A01_SendData(0x00);

		GC9A01_SendCmd(0x67);       
		GC9A01_SendData(0x00);
		GC9A01_SendData(0x3C);
		GC9A01_SendData(0x00);
		GC9A01_SendData(0x00);
		GC9A01_SendData(0x00);
		GC9A01_SendData(0x01);
		GC9A01_SendData(0x54);
		GC9A01_SendData(0x10);
		GC9A01_SendData(0x32);
		GC9A01_SendData(0x98);

		GC9A01_SendCmd(0x74);        
		GC9A01_SendData(0x10);
		GC9A01_SendData(0x85);
		GC9A01_SendData(0x80);
		GC9A01_SendData(0x00);
		GC9A01_SendData(0x00);
		GC9A01_SendData(0x4E);
		GC9A01_SendData(0x00);

		GC9A01_SendCmd(0x98);       
		GC9A01_SendData(0x3e);
		GC9A01_SendData(0x07);

		GC9A01_SendCmd(GC9A01_TEON); 		// Tearing effect line on

		// Inversion Mode 1;
		GC9A01_SendCmd(GC9A01_INVON);
		
		// Sleep Mode Exit
		GC9A01_SendCmd(GC9A01_SLPOUT);

		HAL_Delay(120);
		
		// Display Power on
		GC9A01_SendCmd(GC9A01_DISPON);
		
		GC9A01_Unselect();
		
		GC9A01_FillRect(0, 0, GC9A01_Width, GC9A01_Height, GC9A01_BLACK);
		
#if FRAME_BUFFER	// если включен буфер кадра
		GC9A01_Update();
#endif
}
//==============================================================================


//==============================================================================
// Процедура управления SPI
//==============================================================================
static void GC9A01_Select(void) {
	
  #ifdef CS_PORT
	
			//-- если захотим переделать под HAL ------------------	
			#ifdef GC9A01_SPI_HAL
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
			#endif
			//-----------------------------------------------------
			
			//-- если захотим переделать под CMSIS  ---------------
			#ifdef GC9A01_SPI_CMSIS
				CS_GPIO_Port->BSRR = ( CS_Pin << 16 );
			#endif
			//-----------------------------------------------------
	#endif
	
}
//==============================================================================


//==============================================================================
// Процедура управления SPI
//==============================================================================
static void GC9A01_Unselect(void) {
	
  #ifdef CS_PORT
	
			//-- если захотим переделать под HAL ------------------	
			#ifdef GC9A01_SPI_HAL
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
			#endif
			//-----------------------------------------------------
			
			//-- если захотим переделать под CMSIS  ---------------
			#ifdef GC9A01_SPI_CMSIS
					 CS_GPIO_Port->BSRR = CS_Pin;
			#endif
			//-----------------------------------------------------
	
	#endif
	
}
//==============================================================================


//==============================================================================
// Процедура вывода цветного изображения на дисплей
//==============================================================================
void GC9A01_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
	
  if((x >= GC9A01_Width) || (y >= GC9A01_Height)){
		return;
	}
	
  if((x + w - 1) >= GC9A01_Width){
		return;
	}
	
  if((y + h - 1) >= GC9A01_Height){
		return;
	}
	
#if FRAME_BUFFER	// если включен буфер кадра
	for( uint16_t i = 0; i < h; i++ ){
		for( uint16_t j = 0; j < w; j++ ){
			buff_frame[( y + i ) * GC9A01_Width + x + j] = *data;
			data++;
		}
	}
#else	//если попиксельный вывод
	GC9A01_SetWindow(x, y, x+w-1, y+h-1);
	
	GC9A01_Select();
	
  GC9A01_SendDataMASS((uint8_t*)data, sizeof(uint16_t)*w*h);
	
  GC9A01_Unselect();
#endif
  
}
//==============================================================================


//==============================================================================
// Процедура аппаратного сброса дисплея (ножкой RESET)
//==============================================================================
void GC9A01_HardReset(void){

	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);	
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
	HAL_Delay(150);
	
}
//==============================================================================


//==============================================================================
// Процедура отправки команды в дисплей
//==============================================================================
__inline static void GC9A01_SendCmd(uint8_t Cmd){	
		
	//-- если захотим переделать под HAL ------------------	
	#ifdef GC9A01_SPI_HAL
	
		 // pin DC LOW
		 HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
					 
		 HAL_SPI_Transmit(&GC9A01_SPI_HAL, &Cmd, 1, HAL_MAX_DELAY);
		 while(HAL_SPI_GetState(&GC9A01_SPI_HAL) != HAL_SPI_STATE_READY){};
				
		 // pin DC HIGH
		 HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
		 
	#endif
	//-----------------------------------------------------
	
	//-- если захотим переделать под CMSIS  ---------------------------------------------
	#ifdef GC9A01_SPI_CMSIS
		
		// pin DC LOW
		DC_GPIO_Port->BSRR = ( DC_Pin << 16 );
	
		//======  FOR F-SERIES ===========================================================
			
			// Disable SPI	
			//CLEAR_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9A01_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((GC9A01_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9A01_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}
			
			// Ждем, пока не освободится буфер передатчика
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (GC9A01_SPI_CMSIS->SR & SPI_SR_TXE) == RESET ){};	
			
			// заполняем буфер передатчика 1 байт информации--------------
			*((__IO uint8_t *)&GC9A01_SPI_CMSIS->DR) = Cmd;
			
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (GC9A01_SPI_CMSIS->SR & (SPI_SR_TXE | SPI_SR_BSY)) != SPI_SR_TXE ){};
				
			//Ждем, пока SPI освободится от предыдущей передачи
			//while((GC9A01_SPI_CMSIS->SR&SPI_SR_BSY)){};	

			// Disable SPI	
			//CLEAR_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
		//================================================================================
		
/*		//======  FOR H-SERIES ===========================================================

			// Disable SPI	
			//CLEAR_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9A01_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			// Enable SPI
			if((GC9A01_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9A01_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}
			
			SET_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_CSTART);	// GC9A01_SPI_CMSIS->CR1 |= SPI_CR1_CSTART;
			
			// ждем пока SPI будет свободна------------
			//while (!(GC9A01_SPI_CMSIS->SR & SPI_SR_TXP)){};		
		
			// передаем 1 байт информации--------------
			*((__IO uint8_t *)&GC9A01_SPI_CMSIS->TXDR )  = Cmd;
				
			// Ждать завершения передачи---------------
			while (!( GC9A01_SPI_CMSIS -> SR & SPI_SR_TXC )){};
			
			// Disable SPI	
			//CLEAR_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
*/		//================================================================================
		
		// pin DC HIGH
		DC_GPIO_Port->BSRR = DC_Pin;
	
	#endif
	//-----------------------------------------------------------------------------------

}
//==============================================================================


//==============================================================================
// Процедура отправки данных (параметров) в дисплей 1 BYTE
//==============================================================================
__inline static void GC9A01_SendData(uint8_t Data ){
	
	//-- если захотим переделать под HAL ------------------
	#ifdef GC9A01_SPI_HAL
	
		HAL_SPI_Transmit(&GC9A01_SPI_HAL, &Data, 1, HAL_MAX_DELAY);
		while(HAL_SPI_GetState(&GC9A01_SPI_HAL) != HAL_SPI_STATE_READY){};
		
	#endif
	//-----------------------------------------------------
	
	
	//-- если захотим переделать под CMSIS  ---------------------------------------------
	#ifdef GC9A01_SPI_CMSIS
		
		//======  FOR F-SERIES ===========================================================
			
			// Disable SPI	
			//CLEAR_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9A01_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((GC9A01_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9A01_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}

			// Ждем, пока не освободится буфер передатчика
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (GC9A01_SPI_CMSIS->SR & SPI_SR_TXE) == RESET ){};
		
			// передаем 1 байт информации--------------
			*((__IO uint8_t *)&GC9A01_SPI_CMSIS->DR) = Data;

			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (GC9A01_SPI_CMSIS->SR & (SPI_SR_TXE | SPI_SR_BSY)) != SPI_SR_TXE ){};

			// Ждем, пока не освободится буфер передатчика
			//while((GC9A01_SPI_CMSIS->SR&SPI_SR_BSY)){};	
			
			// Disable SPI	
			//CLEAR_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
		//================================================================================
		
/*		//======  FOR H-SERIES ===========================================================

			// Disable SPI	
			//CLEAR_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9A01_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((GC9A01_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9A01_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}

			SET_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_CSTART);	// GC9A01_SPI_CMSIS->CR1 |= SPI_CR1_CSTART;
			
			// ждем пока SPI будет свободна------------
			//while (!(GC9A01_SPI_CMSIS->SR & SPI_SR_TXP)){};		
		
			// передаем 1 байт информации--------------
			*((__IO uint8_t *)&GC9A01_SPI_CMSIS->TXDR )  = Data;
				
			// Ждать завершения передачи---------------
			while (!( GC9A01_SPI_CMSIS -> SR & SPI_SR_TXC )){};
			
			// Disable SPI	
			//CLEAR_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
*/		//================================================================================
		
	#endif
	//-----------------------------------------------------------------------------------

}
//==============================================================================


//==============================================================================
// Процедура отправки данных (параметров) в дисплей MASS
//==============================================================================
__inline static void GC9A01_SendDataMASS(uint8_t* buff, size_t buff_size){
	
	//-- если захотим переделать под HAL ------------------
	#ifdef GC9A01_SPI_HAL
		
		if( buff_size <= 0xFFFF ){
			HAL_SPI_Transmit(&GC9A01_SPI_HAL, buff, buff_size, HAL_MAX_DELAY);
		}
		else{
			while( buff_size > 0xFFFF ){
				HAL_SPI_Transmit(&GC9A01_SPI_HAL, buff, 0xFFFF, HAL_MAX_DELAY);
				buff_size-=0xFFFF;
				buff+=0xFFFF;
			}
			HAL_SPI_Transmit(&GC9A01_SPI_HAL, buff, buff_size, HAL_MAX_DELAY);
		}
		
		while(HAL_SPI_GetState(&GC9A01_SPI_HAL) != HAL_SPI_STATE_READY){};

	#endif
	//-----------------------------------------------------
	
	
	//-- если захотим переделать под CMSIS  ---------------------------------------------
	#ifdef GC9A01_SPI_CMSIS	

		//======  FOR F-SERIES ===========================================================
			
			// Disable SPI	
			//CLEAR_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9A01_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((GC9A01_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9A01_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}
			
			while( buff_size ){
				
			// Ждем, пока не освободится буфер передатчика
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (GC9A01_SPI_CMSIS->SR & SPI_SR_TXE) == RESET ){};
					
				// передаем 1 байт информации--------------
				*((__IO uint8_t *)&GC9A01_SPI_CMSIS->DR) = *buff++;

				buff_size--;
			}
			
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (GC9A01_SPI_CMSIS->SR & (SPI_SR_TXE | SPI_SR_BSY)) != SPI_SR_TXE ){};
				
			// Ждем, пока не освободится буфер передатчика
			// while((GC9A01_SPI_CMSIS->SR&SPI_SR_BSY)){};
				
			// Disable SPI	
			//CLEAR_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
		//================================================================================
		
/*		//======  FOR H-SERIES ===========================================================

			// Disable SPI	
			//CLEAR_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9A01_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((GC9A01_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9A01_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}

			SET_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_CSTART);	// GC9A01_SPI_CMSIS->CR1 |= SPI_CR1_CSTART;
			
			// ждем пока SPI будет свободна------------
			//while (!(GC9A01_SPI_CMSIS->SR & SPI_SR_TXP)){};		
			
			while( buff_size ){
		
				// передаем 1 байт информации--------------
				*((__IO uint8_t *)&GC9A01_SPI_CMSIS->TXDR )  = *buff++;
				
				// Ждать завершения передачи---------------
				while (!( GC9A01_SPI_CMSIS -> SR & SPI_SR_TXC )){};

				buff_size--;

			}
			
			// Disable SPI	
			//CLEAR_BIT(GC9A01_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
*/		//================================================================================
		
	#endif
	//-----------------------------------------------------------------------------------

}
//==============================================================================


//==============================================================================
// Процедура включения режима сна
//==============================================================================
void GC9A01_SleepModeEnter( void ){
	
	GC9A01_Select(); 
	
	GC9A01_SendCmd(GC9A01_SLPIN);
	
	GC9A01_Unselect();
	
	HAL_Delay(150);
}
//==============================================================================


//==============================================================================
// Процедура отключения режима сна
//==============================================================================
void GC9A01_SleepModeExit( void ){
	
	GC9A01_Select(); 
	
	GC9A01_SendCmd(GC9A01_SLPOUT);
	
	GC9A01_Unselect();
	
	HAL_Delay(150);
}
//==============================================================================


//==============================================================================
// Процедура включения/отключения режима частичного заполнения экрана
//==============================================================================
void GC9A01_InversionMode(uint8_t Mode){
	
  GC9A01_Select(); 
	
  if (Mode){
    GC9A01_SendCmd(GC9A01_INVON);
  }
  else{
    GC9A01_SendCmd(GC9A01_INVOFF);
  }
  
  GC9A01_Unselect();
}
//==============================================================================


//==============================================================================
// Процедура закрашивает экран цветом color
//==============================================================================
void GC9A01_FillScreen(uint16_t color){
	
  GC9A01_FillRect(0, 0,  GC9A01_Width, GC9A01_Height, color);
}
//==============================================================================


//==============================================================================
// Процедура очистки экрана - закрашивает экран цветом черный
//==============================================================================
void GC9A01_Clear(void){
	
  GC9A01_FillRect(0, 0,  GC9A01_Width, GC9A01_Height, 0);
}
//==============================================================================


//==============================================================================
// Процедура заполнения прямоугольника цветом color
//==============================================================================
void GC9A01_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){
	
  if ((x >= GC9A01_Width) || (y >= GC9A01_Height)){
	  return;
  }
  
  if ((x + w) > GC9A01_Width){	  
	  w = GC9A01_Width - x;
  }
  
  if ((y + h) > GC9A01_Height){
	  h = GC9A01_Height - y;
  }
  
#if FRAME_BUFFER	// если включен буфер кадра
	for( uint16_t i = 0; i < h; i++ ){
		for( uint16_t j = 0; j < w; j++ ){
			buff_frame[( y + i ) * GC9A01_Width + x + j] = ((color & 0xFF)<<8) | (color >> 8 );
		}
	}
#else	//если попиксельный вывод
	GC9A01_SetWindow(x, y, x + w - 1, y + h - 1);
 	
  GC9A01_RamWrite(&color, (h * w)); 
#endif
	

}
//==============================================================================


//==============================================================================
// Процедура установка границ экрана для заполнения
//==============================================================================
static void GC9A01_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
	
	GC9A01_Select();
	
	GC9A01_ColumnSet(x0, x1);
	GC9A01_RowSet(y0, y1);
	
	// write to RAM
	GC9A01_SendCmd(GC9A01_RAMWR);
	
	GC9A01_Unselect();
	
}
//==============================================================================


//==============================================================================
// Процедура записи данных в дисплей
//==============================================================================
static void GC9A01_RamWrite(uint16_t *pBuff, uint32_t Len){
	
  GC9A01_Select();
	
  uint8_t buff[2];
  buff[0] = *pBuff >> 8;
  buff[1] = *pBuff & 0xFF;
	
  while (Len--){
	  GC9A01_SendDataMASS( (uint8_t*)buff, 2);
  } 
	
  GC9A01_Unselect();
}
//==============================================================================


//==============================================================================
// Процедура установки начального и конечного адресов колонок
//==============================================================================
static void GC9A01_ColumnSet(uint16_t ColumnStart, uint16_t ColumnEnd){
	
  if (ColumnStart > ColumnEnd){
    return;
  }
  
  if (ColumnEnd > GC9A01_Width){
    return;
  }
  
  ColumnStart += GC9A01_X_Start;
  ColumnEnd += GC9A01_X_Start;
  
  GC9A01_SendCmd(GC9A01_CASET);
  GC9A01_SendData(ColumnStart >> 8);  
  GC9A01_SendData(ColumnStart & 0xFF);  
  GC9A01_SendData(ColumnEnd >> 8);  
  GC9A01_SendData(ColumnEnd & 0xFF);  
  
}
//==============================================================================


//==============================================================================
// Процедура установки начального и конечного адресов строк
//==============================================================================
static void GC9A01_RowSet(uint16_t RowStart, uint16_t RowEnd){
	
  if (RowStart > RowEnd){
    return;
  }
  
  if (RowEnd > GC9A01_Height){
    return;
  }
  
  RowStart += GC9A01_Y_Start;
  RowEnd += GC9A01_Y_Start;
 
  GC9A01_SendCmd(GC9A01_RASET);
  GC9A01_SendData(RowStart >> 8);  
  GC9A01_SendData(RowStart & 0xFF);  
  GC9A01_SendData(RowEnd >> 8);  
  GC9A01_SendData(RowEnd & 0xFF);  

}
//==============================================================================


//==============================================================================
// Процедура управления подсветкой (ШИМ)
//==============================================================================
void GC9A01_SetBL(uint8_t Value){
	
//  if (Value > 100)
//    Value = 100;

//	tmr2_PWM_set(ST77xx_PWM_TMR2_Chan, Value);

}
//==============================================================================


//==============================================================================
// Процедура включения/отключения питания дисплея
//==============================================================================
void GC9A01_DisplayPower(uint8_t On){
	
  GC9A01_Select(); 
	
  if (On){
    GC9A01_SendCmd(GC9A01_DISPON);
  }
  else{
    GC9A01_SendCmd(GC9A01_DISPOFF);
  }
  
  GC9A01_Unselect();
}
//==============================================================================


//==============================================================================
// Процедура рисования прямоугольника ( пустотелый )
//==============================================================================
void GC9A01_DrawRectangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
	
  GC9A01_DrawLine(x1, y1, x1, y2, color);
  GC9A01_DrawLine(x2, y1, x2, y2, color);
  GC9A01_DrawLine(x1, y1, x2, y1, color);
  GC9A01_DrawLine(x1, y2, x2, y2, color);
	
}
//==============================================================================


//==============================================================================
// Процедура вспомогательная для --- Процедура рисования прямоугольника ( заполненый )
//==============================================================================
static void SwapInt16Values(int16_t *pValue1, int16_t *pValue2){
	
  int16_t TempValue = *pValue1;
  *pValue1 = *pValue2;
  *pValue2 = TempValue;
}
//==============================================================================


//==============================================================================
// Процедура рисования прямоугольника ( заполненый )
//==============================================================================
void GC9A01_DrawRectangleFilled(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t fillcolor) {
	
  if (x1 > x2){
    SwapInt16Values(&x1, &x2);
  }
  
  if (y1 > y2){
    SwapInt16Values(&y1, &y2);
  }
  
  GC9A01_FillRect(x1, y1, x2 - x1, y2 - y1, fillcolor);
}
//==============================================================================


//==============================================================================
// Процедура вспомогательная для --- Процедура рисования линии
//==============================================================================
static void GC9A01_DrawLine_Slow(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
	
  const int16_t deltaX = abs(x2 - x1);
  const int16_t deltaY = abs(y2 - y1);
  const int16_t signX = x1 < x2 ? 1 : -1;
  const int16_t signY = y1 < y2 ? 1 : -1;

  int16_t error = deltaX - deltaY;

  GC9A01_DrawPixel(x2, y2, color);

  while (x1 != x2 || y1 != y2) {
	  
    GC9A01_DrawPixel(x1, y1, color);
    const int16_t error2 = error * 2;
 
    if (error2 > -deltaY) {
		
      error -= deltaY;
      x1 += signX;
    }
    if (error2 < deltaX){
		
      error += deltaX;
      y1 += signY;
    }
  }
}
//==============================================================================


//==============================================================================
// Процедура рисования линии
//==============================================================================
void GC9A01_DrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {

  if (x1 == x2){

    if (y1 > y2){
      GC9A01_FillRect(x1, y2, 1, y1 - y2 + 1, color);
	}
    else{
      GC9A01_FillRect(x1, y1, 1, y2 - y1 + 1, color);
	}
	
    return;
  }
  
  if (y1 == y2){
    
    if (x1 > x2){
      GC9A01_FillRect(x2, y1, x1 - x2 + 1, 1, color);
	}
    else{
      GC9A01_FillRect(x1, y1, x2 - x1 + 1, 1, color);
	}
	
    return;
  }
  
  GC9A01_DrawLine_Slow(x1, y1, x2, y2, color);
}
//==============================================================================


//==============================================================================
// Процедура рисования треугольника ( пустотелый )
//==============================================================================
void GC9A01_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color){
	/* Draw lines */
	GC9A01_DrawLine(x1, y1, x2, y2, color);
	GC9A01_DrawLine(x2, y2, x3, y3, color);
	GC9A01_DrawLine(x3, y3, x1, y1, color);
}
//==============================================================================


//==============================================================================
// Процедура рисования треугольника ( заполненый )
//==============================================================================
void GC9A01_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color){
	
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
	curpixel = 0;
	
	deltax = abs(x2 - x1);
	deltay = abs(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} 
	else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} 
	else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} 
	else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		GC9A01_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}
//==============================================================================


//==============================================================================
// Процедура окрашивает 1 пиксель дисплея
//==============================================================================
void GC9A01_DrawPixel(int16_t x, int16_t y, uint16_t color){
	
  if ((x < 0) ||(x >= GC9A01_Width) || (y < 0) || (y >= GC9A01_Height)){
    return;
  }
	
#if FRAME_BUFFER	// если включен буфер кадра
	buff_frame[y * GC9A01_Width + x] = ((color & 0xFF)<<8) | (color >> 8 );
#else	//если попиксельный вывод
	GC9A01_SetWindow(x, y, x, y);
  GC9A01_RamWrite(&color, 1);
#endif
}
//==============================================================================


//==============================================================================
// Процедура рисования круг ( заполненый )
//==============================================================================
void GC9A01_DrawCircleFilled(int16_t x0, int16_t y0, int16_t radius, uint16_t fillcolor) {
	
  int x = 0;
  int y = radius;
  int delta = 1 - 2 * radius;
  int error = 0;

  while (y >= 0){
	  
    GC9A01_DrawLine(x0 + x, y0 - y, x0 + x, y0 + y, fillcolor);
    GC9A01_DrawLine(x0 - x, y0 - y, x0 - x, y0 + y, fillcolor);
    error = 2 * (delta + y) - 1;

    if (delta < 0 && error <= 0) {
		
      ++x;
      delta += 2 * x + 1;
      continue;
    }
	
    error = 2 * (delta - x) - 1;
		
    if (delta > 0 && error > 0) {
		
      --y;
      delta += 1 - 2 * y;
      continue;
    }
	
    ++x;
    delta += 2 * (x - y);
    --y;
  }
}
//==============================================================================


//==============================================================================
// Процедура рисования круг ( пустотелый )
//==============================================================================
void GC9A01_DrawCircle(int16_t x0, int16_t y0, int16_t radius, uint16_t color) {
	
  int x = 0;
  int y = radius;
  int delta = 1 - 2 * radius;
  int error = 0;

  while (y >= 0){
	  
    GC9A01_DrawPixel(x0 + x, y0 + y, color);
    GC9A01_DrawPixel(x0 + x, y0 - y, color);
    GC9A01_DrawPixel(x0 - x, y0 + y, color);
    GC9A01_DrawPixel(x0 - x, y0 - y, color);
    error = 2 * (delta + y) - 1;

    if (delta < 0 && error <= 0) {
		
      ++x;
      delta += 2 * x + 1;
      continue;
    }
	
    error = 2 * (delta - x) - 1;
		
    if (delta > 0 && error > 0) {
		
      --y;
      delta += 1 - 2 * y;
      continue;
    }
	
    ++x;
    delta += 2 * (x - y);
    --y;
  }
}
//==============================================================================


//==============================================================================
// Процедура рисования символа ( 1 буква или знак )
//==============================================================================
void GC9A01_DrawChar(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, unsigned char ch){
	
	uint32_t i, b, j;
	
	uint32_t X = x, Y = y;
	
	uint8_t xx, yy;
	
	if( multiplier < 1 ){
		multiplier = 1;
	}

	/* Check available space in LCD */
	if (GC9A01_Width >= ( x + Font->FontWidth) || GC9A01_Height >= ( y + Font->FontHeight)){

	
			/* Go through font */
			for (i = 0; i < Font->FontHeight; i++) {		
				
				if( ch < 127 ){			
					b = Font->data[(ch - 32) * Font->FontHeight + i];
				}
				
				else if( (uint8_t) ch > 191 ){
					// +96 это так как латинские символы и знаки в шрифтах занимают 96 позиций
					// и если в шрифте который содержит сперва латиницу и спец символы и потом 
					// только кирилицу то нужно добавлять 95 если шрифт 
					// содержит только кирилицу то +96 не нужно
					b = Font->data[((ch - 192) + 96) * Font->FontHeight + i];
				}
				
				else if( (uint8_t) ch == 168 ){	// 168 символ по ASCII - Ё
					// 160 эллемент ( символ Ё ) 
					b = Font->data[( 160 ) * Font->FontHeight + i];
				}
				
				else if( (uint8_t) ch == 184 ){	// 184 символ по ASCII - ё
					// 161 эллемент  ( символ ё ) 
					b = Font->data[( 161 ) * Font->FontHeight + i];
				}
				//-------------------------------------------------------------------
				
				//----  Украинская раскладка ----------------------------------------------------
				else if( (uint8_t) ch == 170 ){	// 168 символ по ASCII - Є
					// 162 эллемент ( символ Є )
					b = Font->data[( 162 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 175 ){	// 184 символ по ASCII - Ї
					// 163 эллемент  ( символ Ї )
					b = Font->data[( 163 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 178 ){	// 168 символ по ASCII - І
					// 164 эллемент ( символ І )
					b = Font->data[( 164 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 179 ){	// 184 символ по ASCII - і
					// 165 эллемент  ( символ і )
					b = Font->data[( 165 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 186 ){	// 184 символ по ASCII - є
					// 166 эллемент  ( символ є )
					b = Font->data[( 166 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 191 ){	// 168 символ по ASCII - ї
					// 167 эллемент ( символ ї )
					b = Font->data[( 167 ) * Font->FontHeight + i];
				}
				//-----------------------------------------------------------------------------
			
				for (j = 0; j < Font->FontWidth; j++) {
					
					if ((b << j) & 0x8000) {
						
						for (yy = 0; yy < multiplier; yy++){
							for (xx = 0; xx < multiplier; xx++){
									GC9A01_DrawPixel(X+xx, Y+yy, TextColor);
							}
						}
						
					} 
					else if( TransparentBg ){
						
						for (yy = 0; yy < multiplier; yy++){
							for (xx = 0; xx < multiplier; xx++){
									GC9A01_DrawPixel(X+xx, Y+yy, BgColor);
							}
						}
						
					}
					X = X + multiplier;
				}
				X = x;
				Y = Y + multiplier;
			}
	}
}
//==============================================================================


//==============================================================================
// Процедура рисования строки
//==============================================================================
void GC9A01_print(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, char *str){	
	
	if( multiplier < 1 ){
		multiplier = 1;
	}
	
	unsigned char buff_char;
	
	uint16_t len = strlen(str);
	
	while (len--) {
		
		//---------------------------------------------------------------------
		// проверка на кириллицу UTF-8, если латиница то пропускаем if
		// Расширенные символы ASCII Win-1251 кириллица (код символа 128-255)
		// проверяем первый байт из двух ( так как UTF-8 ето два байта )
		// если он больше либо равен 0xC0 ( первый байт в кириллеце будет равен 0xD0 либо 0xD1 именно в алфавите )
		if ( (uint8_t)*str >= 0xC0 ){	// код 0xC0 соответствует символу кириллица 'A' по ASCII Win-1251
			
			// проверяем какой именно байт первый 0xD0 либо 0xD1---------------------------------------------
			switch ((uint8_t)*str) {
				case 0xD0: {
					// увеличиваем массив так как нам нужен второй байт
					str++;
					// проверяем второй байт там сам символ
					if ((uint8_t)*str >= 0x90 && (uint8_t)*str <= 0xBF){ buff_char = (*str) + 0x30; }	// байт символов А...Я а...п  делаем здвиг на +48
					else if ((uint8_t)*str == 0x81) { buff_char = 0xA8; break; }		// байт символа Ё ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x84) { buff_char = 0xAA; break; }		// байт символа Є ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x86) { buff_char = 0xB2; break; }		// байт символа І ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x87) { buff_char = 0xAF; break; }		// байт символа Ї ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					break;
				}
				case 0xD1: {
					// увеличиваем массив так как нам нужен второй байт
					str++;
					// проверяем второй байт там сам символ
					if ((uint8_t)*str >= 0x80 && (uint8_t)*str <= 0x8F){ buff_char = (*str) + 0x70; }	// байт символов п...я	елаем здвиг на +112
					else if ((uint8_t)*str == 0x91) { buff_char = 0xB8; break; }		// байт символа ё ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x94) { buff_char = 0xBA; break; }		// байт символа є ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x96) { buff_char = 0xB3; break; }		// байт символа і ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x97) { buff_char = 0xBF; break; }		// байт символа ї ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					break;
				}
			}
			//------------------------------------------------------------------------------------------------
			// уменьшаем еще переменную так как израсходывали 2 байта для кириллицы
			len--;
			
			GC9A01_DrawChar(x, y, TextColor, BgColor, TransparentBg, Font, multiplier, buff_char);
		}
		//---------------------------------------------------------------------
		else{
			GC9A01_DrawChar(x, y, TextColor, BgColor, TransparentBg, Font, multiplier, *str);
		}
		
		x = x + (Font->FontWidth * multiplier);
		/* Increase string pointer */
		str++;
	}
}
//==============================================================================


//==============================================================================
// Процедура ротации ( положение ) дисплея
//==============================================================================
// установка ротации дисплея, отзеркаливание по вертикали и горизонтали и режим цветопередаци
// первый параметр ротация значения от 0 до 7 
// второй параметр отзеркаливание по вертикали значения 0-выкл 1-вкл
// третий параметр отзеркаливание по горизонтали значения 0-выкл 1-вкл
// по умолчанию стоит #define GC9A01_DEF_ROTATION  ( 0, 0, 0 )
void GC9A01_rotation(uint8_t Rotation, uint8_t VertMirror, uint8_t HorizMirror){
	
		GC9A01_Select();
		
		uint8_t Value;
		Rotation &= 7;

		GC9A01_SendCmd(GC9A01_MADCTL);

		switch (Rotation) {
		case 0:
			Value = 0;
			break;
		case 1:
			Value = GC9A01_MADCTL_MX;
			break;
		case 2:
			Value = GC9A01_MADCTL_MY;
			break;
		case 3:
			Value = GC9A01_MADCTL_MX | GC9A01_MADCTL_MY;
			break;
		case 4:
			Value = GC9A01_MADCTL_MV;
			break;
		case 5:
			Value = GC9A01_MADCTL_MV | GC9A01_MADCTL_MX;
			break;
		case 6:
			Value = GC9A01_MADCTL_MV | GC9A01_MADCTL_MY;
			break;
		case 7:
			Value = GC9A01_MADCTL_MV | GC9A01_MADCTL_MX | GC9A01_MADCTL_MY;
			break;
		}

		if (VertMirror){
			Value = GC9A01_MADCTL_ML;
		}
		
		if (HorizMirror){
			Value = GC9A01_MADCTL_MH;
		}

		// RGB or BGR
		Value |= GC9A01_DEF_ROTATION;
		
		GC9A01_SendData(Value);
		
		GC9A01_Unselect();
}
//==============================================================================


//==============================================================================
// Процедура рисования иконки монохромной
//==============================================================================
void GC9A01_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color){

    int16_t byteWidth = (w + 7) / 8; 	// Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for(int16_t j=0; j<h; j++, y++){
		
        for(int16_t i=0; i<w; i++){
			
            if(i & 7){
               byte <<= 1;
            }
            else{
               byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }
			
            if(byte & 0x80){
				GC9A01_DrawPixel(x+i, y, color);
			}
        }
    }
}
//==============================================================================


//==============================================================================
// Процедура рисования прямоугольник с закругленніми краями ( заполненый )
//==============================================================================
void GC9A01_DrawFillRoundRect(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t cornerRadius, uint16_t color) {
	
	int16_t max_radius = ((width < height) ? width : height) / 2; // 1/2 minor axis
  if (cornerRadius > max_radius){
    cornerRadius = max_radius;
	}
	
  GC9A01_DrawRectangleFilled(x + cornerRadius, y, x + cornerRadius + width - 2 * cornerRadius, y + height, color);
  // draw four corners
  GC9A01_DrawFillCircleHelper(x + width - cornerRadius - 1, y + cornerRadius, cornerRadius, 1, height - 2 * cornerRadius - 1, color);
  GC9A01_DrawFillCircleHelper(x + cornerRadius, y + cornerRadius, cornerRadius, 2, height - 2 * cornerRadius - 1, color);
}
//==============================================================================

//==============================================================================
// Процедура рисования половины окружности ( правая или левая ) ( заполненый )
//==============================================================================
void GC9A01_DrawFillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color) {

  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;
  int16_t px = x;
  int16_t py = y;

  delta++; // Avoid some +1's in the loop

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    if (x < (y + 1)) {
      if (corners & 1){
        GC9A01_DrawLine(x0 + x, y0 - y, x0 + x, y0 - y - 1 + 2 * y + delta, color);
			}
      if (corners & 2){
        GC9A01_DrawLine(x0 - x, y0 - y, x0 - x, y0 - y - 1 + 2 * y + delta, color);
			}
    }
    if (y != py) {
      if (corners & 1){
        GC9A01_DrawLine(x0 + py, y0 - px, x0 + py, y0 - px - 1 + 2 * px + delta, color);
			}
      if (corners & 2){
        GC9A01_DrawLine(x0 - py, y0 - px, x0 - py, y0 - px - 1 + 2 * px + delta, color);
			}
			py = y;
    }
    px = x;
  }
}
//==============================================================================																		

//==============================================================================
// Процедура рисования четверти окружности (закругление, дуга) ( ширина 1 пиксель)
//==============================================================================
void GC9A01_DrawCircleHelper(int16_t x0, int16_t y0, int16_t radius, int8_t quadrantMask, uint16_t color)
{
    int16_t f = 1 - radius ;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * radius;
    int16_t x = 0;
    int16_t y = radius;

    while (x <= y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
				
        x++;
        ddF_x += 2;
        f += ddF_x;

        if (quadrantMask & 0x4) {
            GC9A01_DrawPixel(x0 + x, y0 + y, color);
            GC9A01_DrawPixel(x0 + y, y0 + x, color);;
        }
        if (quadrantMask & 0x2) {
			GC9A01_DrawPixel(x0 + x, y0 - y, color);
            GC9A01_DrawPixel(x0 + y, y0 - x, color);
        }
        if (quadrantMask & 0x8) {
			GC9A01_DrawPixel(x0 - y, y0 + x, color);
            GC9A01_DrawPixel(x0 - x, y0 + y, color);
        }
        if (quadrantMask & 0x1) {
            GC9A01_DrawPixel(x0 - y, y0 - x, color);
            GC9A01_DrawPixel(x0 - x, y0 - y, color);
        }
    }
}
//==============================================================================		

//==============================================================================
// Процедура рисования прямоугольник с закругленніми краями ( пустотелый )
//==============================================================================
void GC9A01_DrawRoundRect(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t cornerRadius, uint16_t color) {
	
	int16_t max_radius = ((width < height) ? width : height) / 2; // 1/2 minor axis
  if (cornerRadius > max_radius){
    cornerRadius = max_radius;
	}
	
  GC9A01_DrawLine(x + cornerRadius, y, x + cornerRadius + width -1 - 2 * cornerRadius, y, color);         // Top
  GC9A01_DrawLine(x + cornerRadius, y + height - 1, x + cornerRadius + width - 1 - 2 * cornerRadius, y + height - 1, color); // Bottom
  GC9A01_DrawLine(x, y + cornerRadius, x, y + cornerRadius + height - 1 - 2 * cornerRadius, color);         // Left
  GC9A01_DrawLine(x + width - 1, y + cornerRadius, x + width - 1, y + cornerRadius + height - 1 - 2 * cornerRadius, color); // Right
	
  // draw four corners
	GC9A01_DrawCircleHelper(x + cornerRadius, y + cornerRadius, cornerRadius, 1, color);
  GC9A01_DrawCircleHelper(x + width - cornerRadius - 1, y + cornerRadius, cornerRadius, 2, color);
	GC9A01_DrawCircleHelper(x + width - cornerRadius - 1, y + height - cornerRadius - 1, cornerRadius, 4, color);
  GC9A01_DrawCircleHelper(x + cornerRadius, y + height - cornerRadius - 1, cornerRadius, 8, color);
}
//==============================================================================

//==============================================================================
// Процедура рисования линия толстая ( последний параметр толщина )
//==============================================================================
void GC9A01_DrawLineThick(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint8_t thick) {
	const int16_t deltaX = abs(x2 - x1);
	const int16_t deltaY = abs(y2 - y1);
	const int16_t signX = x1 < x2 ? 1 : -1;
	const int16_t signY = y1 < y2 ? 1 : -1;

	int16_t error = deltaX - deltaY;

	if (thick > 1){
		GC9A01_DrawCircleFilled(x2, y2, thick >> 1, color);
	}
	else{
		GC9A01_DrawPixel(x2, y2, color);
	}

	while (x1 != x2 || y1 != y2) {
		if (thick > 1){
			GC9A01_DrawCircleFilled(x1, y1, thick >> 1, color);
		}
		else{
			GC9A01_DrawPixel(x1, y1, color);
		}

		const int16_t error2 = error * 2;
		if (error2 > -deltaY) {
			error -= deltaY;
			x1 += signX;
		}
		if (error2 < deltaX) {
			error += deltaX;
			y1 += signY;
		}
	}
}
//==============================================================================		

//==============================================================================
// Процедура рисования дуга толстая ( часть круга )
//==============================================================================
void GC9A01_DrawArc(int16_t x0, int16_t y0, int16_t radius, int16_t startAngle, int16_t endAngle, uint16_t color, uint8_t thick) {
	
	int16_t xLast = -1, yLast = -1;
	startAngle -= 90;
	endAngle -= 90;

	for (int16_t angle = startAngle; angle <= endAngle; angle += 2) {
		float angleRad = (float) angle * PI / 180;
		int x = cos(angleRad) * radius + x0;
		int y = sin(angleRad) * radius + y0;

		if (xLast == -1 || yLast == -1) {
			xLast = x;
			yLast = y;
			continue;
		}

		if (thick > 1){
			GC9A01_DrawLineThick(xLast, yLast, x, y, color, thick);
		}
		else{
			GC9A01_DrawLine(xLast, yLast, x, y, color);
		}

		xLast = x;
		yLast = y;
	}
}
//==============================================================================	

#if FRAME_BUFFER
	//==============================================================================
	// Процедура вывода буффера кадра на дисплей
	//==============================================================================
	void GC9A01_Update(void){
		
			GC9A01_SetWindow(0, 0, GC9A01_Width-1, GC9A01_Height-1);
		
			GC9A01_Select();
		
			GC9A01_SendDataMASS((uint8_t*)buff_frame, sizeof(uint16_t)*GC9A01_Width*GC9A01_Height);
		
			GC9A01_Unselect();
	}
	//==============================================================================
	
	//==============================================================================
	// Процедура очистка только буфера кадра  ( при етом сам экран не очищаеться )
	//==============================================================================
	void GC9A01_ClearFrameBuffer(void){
		memset((uint8_t*)buff_frame, 0x00, GC9A01_Width*GC9A01_Height*2 );
	}
	//==============================================================================
#endif

//#########################################################################################################################
//#########################################################################################################################

/*

//==============================================================================


//==============================================================================
// Тест поочерёдно выводит на дисплей картинки с SD-флешки
//==============================================================================
void Test_displayImage(const char* fname)
{
  FRESULT res;
  
  FIL file;
  res = f_open(&file, fname, FA_READ);
  if (res != FR_OK)
    return;

  unsigned int bytesRead;
  uint8_t header[34];
  res = f_read(&file, header, sizeof(header), &bytesRead);
  if (res != FR_OK) 
  {
    f_close(&file);
    return;
  }

  if ((header[0] != 0x42) || (header[1] != 0x4D))
  {
    f_close(&file);
    return;
  }

  uint32_t imageOffset = header[10] | (header[11] << 8) | (header[12] << 16) | (header[13] << 24);
  uint32_t imageWidth  = header[18] | (header[19] << 8) | (header[20] << 16) | (header[21] << 24);
  uint32_t imageHeight = header[22] | (header[23] << 8) | (header[24] << 16) | (header[25] << 24);
  uint16_t imagePlanes = header[26] | (header[27] << 8);

  uint16_t imageBitsPerPixel = header[28] | (header[29] << 8);
  uint32_t imageCompression  = header[30] | (header[31] << 8) | (header[32] << 16) | (header[33] << 24);

  if((imagePlanes != 1) || (imageBitsPerPixel != 24) || (imageCompression != 0))
  {
    f_close(&file);
    return;
  }

  res = f_lseek(&file, imageOffset);
  if(res != FR_OK)
  {
    f_close(&file);
    return;
  }

  // Подготавливаем буфер строки картинки для вывода
  uint8_t imageRow[(240 * 3 + 3) & ~3];
  uint16_t PixBuff[240];

  for (uint32_t y = 0; y < imageHeight; y++)
  {
    res = f_read(&file, imageRow, (imageWidth * 3 + 3) & ~3, &bytesRead);
    if (res != FR_OK)
    {
      f_close(&file);
      return;
    }
      
    uint32_t rowIdx = 0;
    for (uint32_t x = 0; x < imageWidth; x++)
    {
      uint8_t b = imageRow[rowIdx++];
      uint8_t g = imageRow[rowIdx++];
      uint8_t r = imageRow[rowIdx++];
      PixBuff[x] = RGB565(r, g, b);
    }

    dispcolor_DrawPartXY(0, imageHeight - y - 1, imageWidth, 1, PixBuff);
  }

  f_close(&file);
}
//==============================================================================


//==============================================================================
// Тест вывода картинок на дисплей
//==============================================================================
void Test240x240_Images(void)
{
  FATFS fatfs;
  DIR DirInfo;
  FILINFO FileInfo;
  FRESULT res;
  
  res = f_mount(&fatfs, "0", 1);
  if (res != FR_OK)
    return;
  
  res = f_chdir("/240x240");
  if (res != FR_OK)
    return;

  res = f_opendir(&DirInfo, "");
  if (res != FR_OK)
    return;
  
  while (1)
  {
    res = f_readdir(&DirInfo, &FileInfo);
    if (res != FR_OK)
      break;
      
    if (FileInfo.fname[0] == 0)
      break;
      
    char *pExt = strstr(FileInfo.fname, ".BMP");
    if (pExt)
    {
      Test_displayImage(FileInfo.fname);
      delay_ms(2000);
    }
  }
}
//==============================================================================


//==============================================================================
// Процедура заполнения прямоугольной области из буфера. Порядок заполнения экрана Y - X
//==============================================================================
void ST77xx_DrawPartYX(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pBuff)
{
  if ((x >= ST77xx_Width) || (y >= ST77xx_Height))
    return;
  
  if ((x + w - 1) >= ST77xx_Width)
    w = ST77xx_Width - x;
  
  if ((y + h - 1) >= ST77xx_Height)
    h = ST77xx_Height - y;

  ST77xx_SetWindow(x, y, x + w - 1, y + h - 1);

  for (uint32_t i = 0; i < (h * w); i++)
    ST77xx_RamWrite(pBuff++, 1);
}
//==============================================================================


//==============================================================================
// Процедура заполнения прямоугольной области из буфера. Порядок заполнения экрана X - Y
//==============================================================================
void ST77xx_DrawPartXY(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pBuff)
{
  if ((x >= ST77xx_Width) || (y >= ST77xx_Height))
    return;
  
  if ((x + w - 1) >= ST77xx_Width)
    w = ST77xx_Width - x;
  
  if ((y + h - 1) >= ST77xx_Height)
    h = ST77xx_Height - y;

  for (uint16_t iy = y; iy < y + h; iy++)
  {
    ST77xx_SetWindow(x, iy, x + w - 1, iy + 1);
    for (x = w; x > 0; x--)
      ST77xx_RamWrite(pBuff++, 1);
  }
}
//==============================================================================

//########################################################################################################

*/



/************************ (C) COPYRIGHT GKP *****END OF FILE****/


	HAL_StatusTypeDef SPI_Write(uint8_t* pbuff, uint16_t size)
	{
		//DMA, use HAL_SPI_Transmit_DMA() function
	    HAL_StatusTypeDef status =  HAL_SPI_Transmit_DMA(&hspi1, pbuff, size);
	    while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){;}
	    return status;

	    //no DMA, use HAL_SPI_Transmit() function
	    //return HAL_SPI_Transmit(&hspi1, pbuff, size, 100);
	}

	void  Write_Cmd_Data (unsigned char CMDP)
	{
	    LCD_CS_0;
	   	LCD_DC_1;

	   	SPI_Write(&CMDP, 1);

	   	LCD_CS_1;
	}

	//=============================================================
	//write command

	void Write_Cmd(unsigned char CMD)
	{
	    LCD_CS_0;
	   	LCD_DC_0;

	   	SPI_Write(&CMD, 1);

	   	LCD_CS_1;
	}



	 void LCD_SetPos(unsigned int Xstart,unsigned int Ystart,unsigned int Xend,unsigned int Yend)
	{
		Write_Cmd(0x2a);
		Write_Cmd_Data(Xstart>>8);
		Write_Cmd_Data(Xstart);
	 	Write_Cmd_Data(Xend>>8);
		Write_Cmd_Data(Xend);

		Write_Cmd(0x2b);
		Write_Cmd_Data(Ystart>>8);
		Write_Cmd_Data(Ystart);
		Write_Cmd_Data(Yend>>8);
		Write_Cmd_Data(Yend);

	  	Write_Cmd(0x2c);//LCD_WriteCMD(GRAMWR);
	}



	void Write_Bytes(unsigned char * pbuff, unsigned short size)
	{
	    LCD_CS_0;
	   	LCD_DC_1;

	   	SPI_Write(pbuff, size);

	   	LCD_CS_1;
	}


	void ClearWindow(unsigned int startX, unsigned int startY, unsigned int endX, unsigned int endY, unsigned int bColor)
	{
	 unsigned int i;

	 //Exchange high 8bit and low 8bit of bColor for DMA batch transmit
	 unsigned char hb = (bColor&0xFFFF) >> 8;
	 unsigned char lb = bColor & 0xFF;
	 unsigned short tempColor = lb * 256 + hb;

	 unsigned int totalSize = (endX-startX) * (endY - startY) * 2; // total clear window data size
	 unsigned int bufSize = 512;  // define bufSize, need less than DMA transmit size

	 unsigned int loopNum = (totalSize - (totalSize % bufSize)) / bufSize; // transmit loop times
	 unsigned int modNum = totalSize % bufSize;  // remainder data bytes


	 //use a tempBuf to initial bColor data, bufSize < DMA transmit size
	 unsigned short tempBuf[bufSize];
	 unsigned char * ptempBuf;

	 //init tempBuf data to tempColor( Exchange high 8bit and low 8bit of bColor )
	 for(i=0; i<bufSize; i++){
		 tempBuf[i] = tempColor;
	 }

	 // Clear window size: from (startX, startY) to (endX, endY)
	 LCD_SetPos(startX,startY,endX-1,endY-1);// (endX-startX) * (endY - startY)

	 // transmit bufSize byte one time, loopNum loops
	 ptempBuf = (unsigned char *)tempBuf;
	 for(i=0; i<loopNum; i++){
		 Write_Bytes(ptempBuf, bufSize);
	 }

	 // transmit remainder data, modNum bytes
	 Write_Bytes(ptempBuf, modNum);

	}


	void ClearScreen2(unsigned int bColor)
	{
		ClearWindow(0,0,GC9A01_TFTWIDTH,GC9A01_TFTHEIGHT,bColor);
	}



