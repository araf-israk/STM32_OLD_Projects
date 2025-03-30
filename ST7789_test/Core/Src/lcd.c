#include "lcd.h"
//#include "lcd_init.h"
#include "lcdfont.h"
//#include "delay.h"
//#include "spi.h"
//#include "cmsis_os.h"



//extern osSemaphoreId_t DMA_SemaphoreHandle;
extern DMA_HandleTypeDef hdma_spi1_tx;


extern uint8_t SPI2_TX_completed_flag;
/******************************************************************************
      ����˵������ָ�����������ɫ
      ������ݣ�xsta,ysta   ��ʼ����
                xend,yend   ��ֹ����
								color       Ҫ������ɫ
      ����ֵ��  ��
******************************************************************************/

void LCD_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure = {0};

 	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStructure.Pin = RES_PIN|CS_PIN|DC_PIN;
 	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; 		 //�������
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;//�ٶ�50MHz
 	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);	  //��ʼ��GPIOB
 	HAL_GPIO_WritePin(GPIOB, RES_PIN|CS_PIN|DC_PIN, GPIO_PIN_SET);
}


/******************************************************************************
      ����˵����LCD��������д�뺯��(software SPI)
      ������ݣ�dat  Ҫд��Ĵ�������
      ����ֵ��  ��
******************************************************************************/
void LCD_Writ_Bus(u8 dat)
{
	//hard SPI
//	HAL_SPI_Transmit(&hspi1,&dat,1,1);

	SPI2_TX_completed_flag = 0;
	HAL_SPI_Transmit_DMA(&hspi1, &dat, 1);
	while (SPI2_TX_completed_flag == 0);

	//soft SPI
	/*
	u8 i;
	for(i=0;i<8;i++)
	{
		LCD_SCLK_Clr();
		if(dat&0x80)
		{
		   LCD_MOSI_Set();
		}
		else
		{
		   LCD_MOSI_Clr();
		}
		LCD_SCLK_Set();
		dat<<=1;
	}
	*/
}


void LCD_Writ_Bus1(u8 dat)
{
	//hard SPI
	HAL_SPI_Transmit(&hspi1,&dat,1,1);

//	SPI2_TX_completed_flag = 0;
//	HAL_SPI_Transmit_DMA(&hspi1, &dat, 1);
//	while (SPI2_TX_completed_flag == 0);

	//soft SPI
	/*
	u8 i;
	for(i=0;i<8;i++)
	{
		LCD_SCLK_Clr();
		if(dat&0x80)
		{
		   LCD_MOSI_Set();
		}
		else
		{
		   LCD_MOSI_Clr();
		}
		LCD_SCLK_Set();
		dat<<=1;
	}
	*/
}


/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
	LCD_Writ_Bus(dat);
}



/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
//	LCD_Writ_Bus(dat>>8);
//	LCD_Writ_Bus(dat);
	uint8_t temp[2];
	temp[0]=(dat>>8)&0xff;
	temp[1]=dat&0xff;
//	HAL_SPI_Transmit(&hspi1,temp,2,1);

	SPI2_TX_completed_flag = 0;
	HAL_SPI_Transmit_DMA(&hspi1, &temp[0], 1);
	while (SPI2_TX_completed_flag == 0);

	SPI2_TX_completed_flag = 0;
	HAL_SPI_Transmit_DMA(&hspi1, &temp[1], 1);
	while (SPI2_TX_completed_flag == 0);
//	LCD_Writ_Bus(temp[0]);
//	LCD_Writ_Bus(temp[1]);


}


void LCD_WR_DATA1(u16 dat)
{
//	LCD_Writ_Bus(dat>>8);
//	LCD_Writ_Bus(dat);
	uint8_t temp[2];
	temp[0]=(dat>>8)&0xff;
	temp[1]=dat&0xff;
	HAL_SPI_Transmit(&hspi1,temp,2,1);
////
//	SPI2_TX_completed_flag = 0;
//	HAL_SPI_Transmit_DMA(&hspi1, &temp[0], 1);
//	while (SPI2_TX_completed_flag == 0);
//
//	SPI2_TX_completed_flag = 0;
//	HAL_SPI_Transmit_DMA(&hspi1, &temp[1], 1);
//	while (SPI2_TX_completed_flag == 0);
//	LCD_Writ_Bus(temp[0]);
//	LCD_Writ_Bus(temp[1]);


}

/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
	LCD_DC_Clr();//д����
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//д����
}

void LCD_WR_REG1(u8 dat)
{
	LCD_DC_Clr();//д����
	LCD_Writ_Bus1(dat);
	LCD_DC_Set();//д����
}
/******************************************************************************
      ����˵����������ʼ�ͽ�����ַ
      ������ݣ�x1,x2 �����е���ʼ�ͽ�����ַ
                y1,y2 �����е���ʼ�ͽ�����ַ
      ����ֵ��  ��
******************************************************************************/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
	LCD_WR_REG(0x2a);//�е�ַ����
	LCD_WR_DATA(x1);
	LCD_WR_DATA(x2);
	LCD_WR_REG(0x2b);//�е�ַ����
	LCD_WR_DATA(y1);
	LCD_WR_DATA(y2);
	LCD_WR_REG(0x2c);//������д
}

void LCD_Address_Set1(u16 x1,u16 y1,u16 x2,u16 y2)
{
	LCD_WR_REG1(0x2a);//�е�ַ����
	LCD_WR_DATA1(x1);
	LCD_WR_DATA1(x2);
	LCD_WR_REG1(0x2b);//�е�ַ����
	LCD_WR_DATA1(y1);
	LCD_WR_DATA1(y2);
	LCD_WR_REG1(0x2c);//������д
}

/******************************************************************************
      ����˵����LCD���ڱ���
      ������ݣ�dc,ռ�ձ�,5%~100%
      ����ֵ��  ��
******************************************************************************/
void LCD_Set_Light(uint8_t dc)
{
//	if(dc>=5 && dc<=100)
//		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,dc*PWM_PERIOD/100);
}


/******************************************************************************
      ����˵����LCD�رձ���
      ������ݣ���
      ����ֵ��  ��
******************************************************************************/
void LCD_Close_Light(void)
{
//	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,0);
	//HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
}


/******************************************************************************
      ����˵����LCD��������
      ������ݣ���
      ����ֵ��  ��
******************************************************************************/
void LCD_Open_Light(void)
{
//	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
}

/******************************************************************************
      ����˵����ST7789 SLEEP IN
      ������ݣ���
      ����ֵ��  ��
******************************************************************************/
void LCD_ST7789_SleepIn(void)
{
	LCD_WR_REG(0x10);
//	delay_ms(100);
	HAL_Delay(100);
}


/******************************************************************************
      ����˵����ST7789 SLEEP OUT
      ������ݣ���
      ����ֵ��  ��
******************************************************************************/
void LCD_ST7789_SleepOut(void)
{
	LCD_WR_REG(0x11);
//	delay_ms(100);
}


/******************************************************************************
      ����˵����LCD��ʼ��
      ������ݣ���
      ����ֵ��  ��


******************************************************************************/
void ST7789_SPI_Init(void)
{
	//Peripherials initializations are unnecessary when using code generated by CubeMX


	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);		//CS OFF
}


void ILI9341_Enable(void)
{
	HAL_GPIO_WritePin(RES_PORT, RES_PIN, GPIO_PIN_SET);
}


void LCD_Init(void)
{
//	LCD_GPIO_Init();//��ʼ��GPIO
	ILI9341_Enable();
	ST7789_SPI_Init();
	LCD_CS_Clr();		//chip select

	LCD_RES_Clr();	//��λ
	HAL_Delay(100);
	LCD_RES_Set();
	HAL_Delay(100);

	LCD_WR_REG(0x11);
	HAL_Delay(120);
	LCD_WR_REG(0x36);
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
	else LCD_WR_DATA8(0xA0);

	LCD_WR_REG(0x3A);
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x33);
	LCD_WR_DATA8(0x33);

	LCD_WR_REG(0xB7);
	LCD_WR_DATA8(0x35);

	LCD_WR_REG(0xBB);
	LCD_WR_DATA8(0x19);

	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x2C);

	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x01);

	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x12);

	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x20);

	LCD_WR_REG(0xC6);
	LCD_WR_DATA8(0x0F);

	LCD_WR_REG(0xD0);
	LCD_WR_DATA8(0xA4);
	LCD_WR_DATA8(0xA1);

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0xD0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x2B);
	LCD_WR_DATA8(0x3F);
	LCD_WR_DATA8(0x54);
	LCD_WR_DATA8(0x4C);
	LCD_WR_DATA8(0x18);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x0B);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x23);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0xD0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x2C);
	LCD_WR_DATA8(0x3F);
	LCD_WR_DATA8(0x44);
	LCD_WR_DATA8(0x51);
	LCD_WR_DATA8(0x2F);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x20);
	LCD_WR_DATA8(0x23);

	LCD_WR_REG(0x21);

	LCD_WR_REG(0x29);
}





void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{          
	u16 i,j; 
	LCD_Address_Set(xsta,ysta+OFFSET_Y,xend-1,yend-1+OFFSET_Y);//������ʾ��Χ
	for(i=ysta;i<yend;i++)
	{		
		for(j=xsta;j<xend;j++)
		{
			LCD_WR_DATA(color);
		}
	} 					  	    
}

/******************************************************************************
      ����˵������ָ�����������ɫ
      ������ݣ�xsta,ysta   ��ʼ����
                xend,yend   ��ֹ����
								color       Ҫ������ɫ
      ����ֵ��  ��
******************************************************************************/
void LCD_Color_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 *color_p)
{
	u16 i,j,width,height; 
	width = xend-xsta+1;
	height = yend-ysta+1;
	uint32_t size = width * height;
	
	LCD_Address_Set(xsta,ysta+OFFSET_Y,xend,yend+OFFSET_Y);
	
	hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi1.Instance->CR1|=SPI_CR1_DFF;
	HAL_SPI_Transmit_DMA(&hspi1,(uint8_t*)color_p,size);
	while(__HAL_DMA_GET_COUNTER(&hdma_spi1_tx)!=0);
	
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Instance->CR1&=~SPI_CR1_DFF;
	
}

/******************************************************************************
      ����˵������ָ��λ�û���
      ������ݣ�x,y ��������
                color �����ɫ
      ����ֵ��  ��
******************************************************************************/
void LCD_DrawPoint(u16 x,u16 y,u16 color)
{

	LCD_Address_Set(x,y,x,y);//���ù��λ��
	LCD_WR_DATA(color);
}


void LCD_DrawPoint1(u16 x,u16 y,u16 color)
{
//	 LCD_Address_Set(0,0+OFFSET_Y,LCD_W-1,LCD_H-1+OFFSET_Y);
	LCD_Address_Set1(x,y+OFFSET_Y,x-1,y-1+OFFSET_Y);//���ù��λ��
	LCD_WR_DATA1(color);
} 


/******************************************************************************
      ����˵��������
      ������ݣ�x1,y1   ��ʼ����
                x2,y2   ��ֹ����
                color   �ߵ���ɫ
      ����ֵ��  ��
******************************************************************************/
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //������������ 
	delta_y=y2-y1;
	uRow=x1;//�����������
	uCol=y1;
	if(delta_x>0)incx=1; //���õ������� 
	else if (delta_x==0)incx=0;//��ֱ�� 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//ˮƽ�� 
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//����
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}


/******************************************************************************
      ����˵����������
      ������ݣ�x1,y1   ��ʼ����
                x2,y2   ��ֹ����
                color   ���ε���ɫ
      ����ֵ��  ��
******************************************************************************/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}


/******************************************************************************
      ����˵������Բ
      ������ݣ�x0,y0   Բ������
                r       �뾶
                color   Բ����ɫ
      ����ֵ��  ��
******************************************************************************/
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);             //3           
		LCD_DrawPoint(x0+b,y0-a,color);             //0           
		LCD_DrawPoint(x0-a,y0+b,color);             //1                
		LCD_DrawPoint(x0-a,y0-b,color);             //2             
		LCD_DrawPoint(x0+b,y0+a,color);             //4               
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6 
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r))//�ж�Ҫ���ĵ��Ƿ��Զ
		{
			b--;
		}
	}
}

/******************************************************************************
      ����˵������ʾ���ִ�
      ������ݣ�x,y��ʾ����
                *s Ҫ��ʾ�ĺ��ִ�
                fc �ֵ���ɫ
                bc �ֵı���ɫ
                sizey �ֺ� ��ѡ 16 24 32
                mode:  0�ǵ���ģʽ  1����ģʽ
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowChinese(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	while(*s!=0)
	{
		if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
		else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
		else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
		else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
		else return;
		s+=2;
		x+=sizey;
	}
}

/******************************************************************************
      ����˵������ʾ����12x12����
      ������ݣ�x,y��ʾ����
                *s Ҫ��ʾ�ĺ���
                fc �ֵ���ɫ
                bc �ֵı���ɫ
                sizey �ֺ�
                mode:  0�ǵ���ģʽ  1����ģʽ
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowChinese12x12(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//������Ŀ
	u16 TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	                         
	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//ͳ�ƺ�����Ŀ
	for(k=0;k<HZnum;k++) 
	{
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//�ǵ��ӷ�ʽ
					{
						if(tfont12[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//���ӷ�ʽ
					{
						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}
} 

/******************************************************************************
      ����˵������ʾ����16x16����
      ������ݣ�x,y��ʾ����
                *s Ҫ��ʾ�ĺ���
                fc �ֵ���ɫ
                bc �ֵı���ɫ
                sizey �ֺ�
                mode:  0�ǵ���ģʽ  1����ģʽ
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowChinese16x16(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//������Ŀ
	u16 TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	u16 x0=x;
  TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);	//ͳ�ƺ�����Ŀ
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//�ǵ��ӷ�ʽ
					{
						if(tfont16[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//���ӷ�ʽ
					{
						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}
} 


/******************************************************************************
      ����˵������ʾ����24x24����
      ������ݣ�x,y��ʾ����
                *s Ҫ��ʾ�ĺ���
                fc �ֵ���ɫ
                bc �ֵı���ɫ
                sizey �ֺ�
                mode:  0�ǵ���ģʽ  1����ģʽ
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowChinese24x24(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//������Ŀ
	u16 TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//ͳ�ƺ�����Ŀ
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//�ǵ��ӷ�ʽ
					{
						if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//���ӷ�ʽ
					{
						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}
} 

/******************************************************************************
      ����˵������ʾ����32x32����
      ������ݣ�x,y��ʾ����
                *s Ҫ��ʾ�ĺ���
                fc �ֵ���ɫ
                bc �ֵı���ɫ
                sizey �ֺ�
                mode:  0�ǵ���ģʽ  1����ģʽ
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowChinese32x32(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//������Ŀ
	u16 TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//ͳ�ƺ�����Ŀ
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//�ǵ��ӷ�ʽ
					{
						if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//���ӷ�ʽ
					{
						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}
}


/******************************************************************************
      ����˵������ʾ�����ַ�
      ������ݣ�x,y��ʾ����
                num Ҫ��ʾ���ַ�
                fc �ֵ���ɫ
                bc �ֵı���ɫ
                sizey �ֺ�
                mode:  0�ǵ���ģʽ  1����ģʽ
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowChar(u16 x,u16 y,u8 num,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 temp,sizex,t,m=0;
	u16 i,TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	u16 x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //�õ�ƫ�ƺ��ֵ
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //���ù��λ�� 
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=ascii_1206[num][i];		       //����6x12����
		else if(sizey==16)temp=ascii_1608[num][i];		 //����8x16����
		else if(sizey==24)temp=ascii_2412[num][i];		 //����12x24����
		else if(sizey==32)temp=ascii_3216[num][i];		 //����16x32����
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)//�ǵ���ģʽ
			{
				if(temp&(0x01<<t))LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else//����ģʽ
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//��һ����
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}


/******************************************************************************
      ����˵������ʾ�ַ���
      ������ݣ�x,y��ʾ����
                *p Ҫ��ʾ���ַ���
                fc �ֵ���ɫ
                bc �ֵı���ɫ
                sizey �ֺ�
                mode:  0�ǵ���ģʽ  1����ģʽ
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 fc,u16 bc,u8 sizey,u8 mode)
{         
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}


/******************************************************************************
      ����˵������ʾ����
      ������ݣ�m������nָ��
      ����ֵ��  ��
******************************************************************************/
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;
	return result;
}


/******************************************************************************
      ����˵������ʾ��������
      ������ݣ�x,y��ʾ����
                num Ҫ��ʾ��������
                len Ҫ��ʾ��λ��
                fc �ֵ���ɫ
                bc �ֵı���ɫ
                sizey �ֺ�
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey)
{         	
	u8 t,temp;
	u8 enshow=0;
	u8 sizex=sizey/2;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 


/******************************************************************************
      ����˵������ʾ��λС������
      ������ݣ�x,y��ʾ����
                num Ҫ��ʾС������
                len Ҫ��ʾ��λ��
                fc �ֵ���ɫ
                bc �ֵı���ɫ
                sizey �ֺ�
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey)
{         	
	u8 t,temp,sizex;
	u16 num1;
	sizex=sizey/2;
	num1=num*100;
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}


/******************************************************************************
      ����˵������ʾͼƬ
      ������ݣ�x,y�������
                length ͼƬ����
                width  ͼƬ���
                pic[]  ͼƬ����    
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[])
{
	u16 i,j;
	u32 k=0;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_WR_DATA8(pic[k*2]);
			LCD_WR_DATA8(pic[k*2+1]);
			k++;
		}
	}			
}



void ILI9341_Draw_Colour(uint16_t Colour)
{
//SENDS COLOUR
	unsigned char TempBuffer[2] =
	{ Colour >> 8, Colour };
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

	SPI2_TX_completed_flag = 0;
	HAL_SPI_Transmit_DMA(&hspi1, TempBuffer, 2);
	while (SPI2_TX_completed_flag == 0)

	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
}

//INTERNAL FUNCTION OF LIBRARY
/*Sends block colour information to LCD*/
void ILI9341_Draw_Colour_Burst(uint16_t Colour, uint32_t Size)
{
//SENDS COLOUR
	uint32_t Buffer_Size = 0;
	if ((Size * 2) < BURST_MAX_SIZE)
	{
		Buffer_Size = Size;
	}
	else
	{
		Buffer_Size = BURST_MAX_SIZE;
	}

	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

	unsigned char MSB_color = Colour >> 8;

	unsigned char burst_buffer[Buffer_Size];
	for (uint32_t j = 0; j < Buffer_Size; j += 2)
	{
		burst_buffer[j] = MSB_color;
		burst_buffer[j + 1] = Colour;
	}

	uint32_t Sending_Size = Size * 2;
	uint32_t Sending_in_Block = Sending_Size / Buffer_Size;
	uint32_t Remainder_from_block = Sending_Size % Buffer_Size;

	if (Sending_in_Block != 0)
	{
		for (uint32_t j = 0; j < (Sending_in_Block); j++)
		{
			SPI2_TX_completed_flag = 0;
			HAL_SPI_Transmit_DMA(&hspi1, (unsigned char*) burst_buffer, Buffer_Size);
			while (SPI2_TX_completed_flag == 0);
		}
	}

//REMAINDER!

	if (Remainder_from_block > 0)
	{
		SPI2_TX_completed_flag = 0;
		HAL_SPI_Transmit_DMA(&hspi1, (unsigned char*) burst_buffer, Remainder_from_block);
		while (SPI2_TX_completed_flag == 0);
	}
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
}

//FILL THE ENTIRE SCREEN WITH SELECTED COLOUR (either #define-d ones or custom 16bit)
/*Sets address (entire screen) and Sends Height*Width ammount of colour information to LCD*/
void ILI9341_Fill_Screen(uint16_t Colour)
{
//	LCD_Address_Set(xsta,ysta+OFFSET_Y,xend-1,yend-1+OFFSET_Y);/
	LCD_Address_Set(0, 0+OFFSET_Y, LCD_W-1, LCD_H-1+OFFSET_Y);
	ILI9341_Draw_Colour_Burst(Colour, LCD_W * LCD_H);
}


