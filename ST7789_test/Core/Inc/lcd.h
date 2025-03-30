#ifndef __LCD_H
#define __LCD_H		
//#include "sys.h"


#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#define BURST_MAX_SIZE 	500
#define OS_SUPPORT 	0
#define SYS_CLK 		100
#define OFFSET_Y 20

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;
typedef const int16_t sc16;
typedef const int8_t sc8;

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;
typedef __I int16_t vsc16;
typedef __I int8_t vsc8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;
typedef const uint16_t uc16;
typedef const uint8_t uc8;

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;
typedef __I uint16_t vuc16;
typedef __I uint8_t vuc8;


extern SPI_HandleTypeDef hspi1;

#define USE_HORIZONTAL 0  //���ú�������������ʾ 0��1Ϊ���� 2��3Ϊ����


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 240
#define LCD_H 280

#else
#define LCD_W 280
#define LCD_H 240
#endif


//-----------------LCD�˿ڶ���----------------
#define SCLK_PORT			GPIOA
#define SCLK_PIN			GPIO_PIN_5

#define SDA_PORT			GPIOA
#define SDA_PIN				GPIO_PIN_7

#define RES_PORT			GPIOB
#define RES_PIN				GPIO_PIN_1

#define DC_PORT				GPIOB
#define DC_PIN				GPIO_PIN_2

#define CS_PORT				GPIOB
#define CS_PIN				GPIO_PIN_0

//#define BLK_PORT			GPIOB
//#define BLK_PIN				GPIO_PIN_0

#define LCD_SCLK_Clr() HAL_GPIO_WritePin(SCLK_PORT,SCLK_PIN,GPIO_PIN_RESET)//SCL=SCLK
#define LCD_SCLK_Set() HAL_GPIO_WritePin(SCLK_PORT,SCLK_PIN,GPIO_PIN_SET)

#define LCD_MOSI_Clr() HAL_GPIO_WritePin(SDA_PORT,SDA_PIN,GPIO_PIN_RESET)//SDA=MOSI
#define LCD_MOSI_Set() HAL_GPIO_WritePin(SDA_PORT,SDA_PIN,GPIO_PIN_SET)

#define LCD_RES_Clr()  HAL_GPIO_WritePin(RES_PORT,RES_PIN,GPIO_PIN_RESET)//RES
#define LCD_RES_Set()  HAL_GPIO_WritePin(RES_PORT,RES_PIN,GPIO_PIN_SET)

#define LCD_DC_Clr()   HAL_GPIO_WritePin(DC_PORT,DC_PIN,GPIO_PIN_RESET)//DC
#define LCD_DC_Set()   HAL_GPIO_WritePin(DC_PORT,DC_PIN,GPIO_PIN_SET)

#define LCD_CS_Clr()   HAL_GPIO_WritePin(CS_PORT,CS_PIN,GPIO_PIN_RESET)//CS
#define LCD_CS_Set()   HAL_GPIO_WritePin(CS_PORT,CS_PIN,GPIO_PIN_SET)

//#define LCD_BLK_Clr()  HAL_GPIO_WritePin(BLK_PORT,BLK_PIN,GPIO_PIN_RESET)//BLK
//#define LCD_BLK_Set()  HAL_GPIO_WritePin(BLK_PORT,BLK_PIN,GPIO_PIN_SET)

void LCD_GPIO_Init(void);//��ʼ��GPIO
void LCD_Writ_Bus(u8 dat);//ģ��SPIʱ��
void LCD_WR_DATA8(u8 dat);//д��һ���ֽ�
void LCD_WR_DATA(u16 dat);//д�������ֽ�
void LCD_WR_REG(u8 dat);//д��һ��ָ��
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//�������꺯��
void LCD_Address_Set1(u16 x1,u16 y1,u16 x2,u16 y2);
void LCD_Init(void);//LCD��ʼ��
void LCD_Set_Light(uint8_t dc);
void LCD_Close_Light(void);
void LCD_ST7789_SleepIn(void);
void LCD_ST7789_SleepOut(void);
void LCD_Open_Light(void);

void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color);//ָ�����������ɫ
void LCD_Color_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 *color);
void LCD_DrawPoint(u16 x,u16 y,u16 color);//��ָ��λ�û�һ����
void LCD_DrawPoint1(u16 x,u16 y,u16 color);
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color);//��ָ��λ�û�һ����
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);//��ָ��λ�û�һ������
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color);//��ָ��λ�û�һ��Բ

void LCD_ShowChinese(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);//��ʾ���ִ�
void LCD_ShowChinese12x12(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);//��ʾ����12x12����
void LCD_ShowChinese16x16(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);//��ʾ����16x16����
void LCD_ShowChinese24x24(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);//��ʾ����24x24����
void LCD_ShowChinese32x32(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);//��ʾ����32x32����

void LCD_ShowChar(u16 x,u16 y,u8 num,u16 fc,u16 bc,u8 sizey,u8 mode);//��ʾһ���ַ�
void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 fc,u16 bc,u8 sizey,u8 mode);//��ʾ�ַ���
u32 mypow(u8 m,u8 n);//����
void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey);//��ʾ��������
void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey);//��ʾ��λС������

void LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[]);//��ʾͼƬ

void ILI9341_Fill_Screen(uint16_t Colour);
//������ɫ
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE           	 0x001F  
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 //��ɫ
#define BRRED 			     0XFC07 //�غ�ɫ
#define GRAY  			     0X8430 //��ɫ
#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
#define LIGHTGREEN     	 0X841F //ǳ��ɫ
#define LGRAY 			     0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ
#define LGRAYBLUE        0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE           0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)

#endif





