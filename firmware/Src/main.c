/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "font.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// these variables are volatile because they are used during the interrupt service routine!
//CS   --PB.4
//SCK  --PB.3
//SDI  --PB.5
//DC   --PB.1
//REST --PB.0 

#define LCD_W 240
#define LCD_H 320
#define LCD_CS_High()       { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);}
#define LCD_CS_Low()  	    { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);}

#define LCD_SCK_High()      { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);}
#define LCD_SCK_Low()  	    { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);}

#define LCD_SDI_High()      { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);}
#define LCD_SDI_Low()      	{ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);}

#define LCD_DC_High()     	{ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);}
#define LCD_DC_Low()    	  { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);}

#define LCD_REST_High()   	{ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);}
#define LCD_REST_Low()    	{ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);}


#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	   0x001F  
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 
#define BRRED 			     0XFC07 
#define GRAY  			     0X8430

#define true 1
#define false 0
int BPM;                   // used to hold the pulse rate
int Signal;                // holds the incoming raw data
int IBI = 600;             // holds the time between beats, must be seeded! 
unsigned char Pulse = false;     // true when pulse wave is high, false when it's low
unsigned char QS = false;        // becomes true when Arduoino finds a beat.
int rate[10];                    // array to hold last ten IBI values
unsigned long sampleCounter = 0;          // used to determine pulse timing
unsigned long lastBeatTime = 0;           // used to find IBI
int P =512;                      // used to find peak in pulse wave, seeded
int T = 512;                     // used to find trough in pulse wave, seeded
int thresh = 512;                // used to find instant moment of heart beat, seeded
int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
int Num;
unsigned char firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
unsigned char secondBeat = false;      // used to seed rate array so we startup with reasonable BPM
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void sendDataToProcessing(char symbol, int dat );
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

uint16_t BACK_COLOR,POINT_COLOR;

long int delay_1us(void)
{
	long int count=2;
	while(count--);
	return 0;
}

void delay_ms(uint16_t nms)
{	 
  uint16_t n,m;	
	n=nms;
	while(n)
	{
	for(m=0;m<1000;m++) delay_1us();
	n--;	
	}  	    
}   
		    								   

void Delay(uint32_t count)
{
	while(count--);
}

void LCD_Writ_Bus(unsigned char com)   
{	
unsigned char uci;

  for(uci=0;uci<8;uci++)
	{
		if(com & 0x80)
		{
			LCD_SDI_High();
	  }
		else
		{
			LCD_SDI_Low();
	  }
		com = com << 1;
		
		LCD_SCK_Low();
		LCD_SCK_High();
	} 	
}
void LCD_WR_DATA8(char da) 
{
  LCD_DC_High();
	LCD_Writ_Bus(da);
}  
void LCD_WR_DATA(int da)
{
  LCD_DC_High();
	LCD_Writ_Bus(da>>8);
	LCD_Writ_Bus(da);
}	  
void LCD_WR_REG(char da)	 
{
  LCD_DC_Low();
	LCD_Writ_Bus(da);
}
void LCD_WR_REG_DATA(int reg,int da)
{
  LCD_WR_REG(reg);
	LCD_WR_DATA(da);
}

void Address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)
{  
   LCD_WR_REG(0x2a);
   LCD_WR_DATA8(x1>>8);
   LCD_WR_DATA8(x1);
   LCD_WR_DATA8(x2>>8);
   LCD_WR_DATA8(x2);
  
   LCD_WR_REG(0x2b);
   LCD_WR_DATA8(y1>>8);
   LCD_WR_DATA8(y1);
   LCD_WR_DATA8(y2>>8);
   LCD_WR_DATA8(y2);

   LCD_WR_REG(0x2C);
}

static GPIO_InitTypeDef  GPIO_InitStruct;
void GPIO_Configuration(void) 
{ 
	__HAL_RCC_GPIOB_CLK_ENABLE();  
  GPIO_InitStruct.Pin   = GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_1|GPIO_PIN_0; 
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;     //GPIO_SPEED_FREQ_VERY_HIGH
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP; 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}
void Lcd_Init(void)
{
	  LCD_REST_Low();
	  delay_ms(20);
	  LCD_REST_High();
	  delay_ms(20);
    LCD_CS_Low();  

		LCD_WR_REG(0xCB);  
    LCD_WR_DATA8(0x39); 
    LCD_WR_DATA8(0x2C); 
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0x34); 
    LCD_WR_DATA8(0x02); 

    LCD_WR_REG(0xCF);  
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0XC1); 
    LCD_WR_DATA8(0X30); 
 
    LCD_WR_REG(0xE8);  
    LCD_WR_DATA8(0x85); 
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0x78); 
 
    LCD_WR_REG(0xEA);  
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0x00); 
 
    LCD_WR_REG(0xED);  
    LCD_WR_DATA8(0x64); 
    LCD_WR_DATA8(0x03); 
    LCD_WR_DATA8(0X12); 
    LCD_WR_DATA8(0X81); 

    LCD_WR_REG(0xF7);  
    LCD_WR_DATA8(0x20); 
  
    LCD_WR_REG(0xC0);     //Power control 
    LCD_WR_DATA8(0x23);   //VRH[5:0] 
 
    LCD_WR_REG(0xC1);     //Power control 
    LCD_WR_DATA8(0x10);   //SAP[2:0];BT[3:0] 
 
    LCD_WR_REG(0xC5);     //VCM control 
    LCD_WR_DATA8(0x3e);   //?????
    LCD_WR_DATA8(0x28); 
 
    LCD_WR_REG(0xC7);     //VCM control2 
    LCD_WR_DATA8(0x86);   //--
 
    LCD_WR_REG(0x36);     // Memory Access Control 
    LCD_WR_DATA8(0x48);   //C8	   //48 68??//28 E8 ??

    LCD_WR_REG(0x3A);    
    LCD_WR_DATA8(0x55); 

    LCD_WR_REG(0xB1);    
    LCD_WR_DATA8(0x00);  
    LCD_WR_DATA8(0x18); 
 
    LCD_WR_REG(0xB6);    // Display Function Control 
    LCD_WR_DATA8(0x08); 
    LCD_WR_DATA8(0x82);
    LCD_WR_DATA8(0x27);  
 
    LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
    LCD_WR_DATA8(0x00); 
 
    LCD_WR_REG(0x26);    //Gamma curve selected 
    LCD_WR_DATA8(0x01); 
 
    LCD_WR_REG(0xE0);    //Set Gamma 
    LCD_WR_DATA8(0x0F); 
    LCD_WR_DATA8(0x31); 
    LCD_WR_DATA8(0x2B); 
    LCD_WR_DATA8(0x0C); 
    LCD_WR_DATA8(0x0E); 
    LCD_WR_DATA8(0x08); 
    LCD_WR_DATA8(0x4E); 
    LCD_WR_DATA8(0xF1); 
    LCD_WR_DATA8(0x37); 
    LCD_WR_DATA8(0x07); 
    LCD_WR_DATA8(0x10); 
    LCD_WR_DATA8(0x03); 
    LCD_WR_DATA8(0x0E); 
    LCD_WR_DATA8(0x09); 
    LCD_WR_DATA8(0x00); 

    LCD_WR_REG(0XE1);    //Set Gamma 
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0x0E); 
    LCD_WR_DATA8(0x14); 
    LCD_WR_DATA8(0x03); 
    LCD_WR_DATA8(0x11); 
    LCD_WR_DATA8(0x07); 
    LCD_WR_DATA8(0x31); 
    LCD_WR_DATA8(0xC1); 
    LCD_WR_DATA8(0x48); 
    LCD_WR_DATA8(0x08); 
    LCD_WR_DATA8(0x0F); 
    LCD_WR_DATA8(0x0C); 
    LCD_WR_DATA8(0x31); 
    LCD_WR_DATA8(0x36); 
    LCD_WR_DATA8(0x0F); 
 
    LCD_WR_REG(0x11);    //Exit Sleep 
    delay_ms(120); 
				
    LCD_WR_REG(0x29);    //Display on 
    LCD_WR_REG(0x2c); 
}

void LCD_Clear(uint16_t Color)
{
	char VH,VL;
	uint16_t i,j;
	VH=Color>>8;
	VL=Color;	
	Address_set(0,0,LCD_W-1,LCD_H-1);
    for(i=0;i<LCD_W;i++)
	  {
	    for (j=0;j<LCD_H;j++)
	   	{
       LCD_WR_DATA8(VH);
			 LCD_WR_DATA8(VL);	
      }
	  }
}

void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	Address_set(x,y,x,y); 
	LCD_WR_DATA(POINT_COLOR); 	    
} 	 

void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{          
	uint16_t i,j; 
	Address_set(xsta,ysta,xend,yend);      
	for(i=ysta;i<=yend;i++)
	{													   	 	
		for(j=xsta;j<=xend;j++)LCD_WR_DATA(color);//?????? 	    
	} 					  	    
}  

void LCD_DrawPoint_big(uint16_t x,uint16_t y)
{
	LCD_Fill(x-1,y-1,x+1,y+1,POINT_COLOR);
} 

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	uint16_t t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 

	delta_x=x2-x1;  
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1;  
	else if(delta_x==0)incx=0; 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0; 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ ) 
	{  
		LCD_DrawPoint(uRow,uCol); 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
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

void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}

void Draw_Circle(uint16_t x0,uint16_t y0,char r)
{
	int a,b;
	int di;
	a=0;  b=r;	  
	di=3-(r<<1);             
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a);             //3           
		LCD_DrawPoint(x0+b,y0-a);             //0           
		LCD_DrawPoint(x0-a,y0+b);             //1       
		LCD_DrawPoint(x0-b,y0-a);             //7           
		LCD_DrawPoint(x0-a,y0-b);             //2             
		LCD_DrawPoint(x0+b,y0+a);             //4               
		LCD_DrawPoint(x0+a,y0-b);             //5
		LCD_DrawPoint(x0+a,y0+b);             //6 
		LCD_DrawPoint(x0-b,y0+a);             
		a++;
		   
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 
		LCD_DrawPoint(x0+a,y0+b);
	}
} 

void LCD_ShowChar(uint16_t x,uint16_t y,char num,char size,char mode)
{  							  
  char temp,t1,t;
	uint16_t y0=y;
	uint16_t colortemp=POINT_COLOR;      			     
	num=num-' ';//???????
	if(!mode) //?????
	{
	    for(t=0;t<size;t++)
	    {   
			if(size==12)temp=asc2_1206[num][t];  //??1206??
			else temp=asc2_1608[num][t];		 //??1608?? 	                          
	    for(t1=0;t1<8;t1++)
			{			    
		    if(temp&0x80) POINT_COLOR=colortemp;
				else  POINT_COLOR=BACK_COLOR;
				LCD_DrawPoint(x,y);	
				temp<<=1;
				y++;
				if(y>=320)  { POINT_COLOR=colortemp; return; } //
				if((y-y0)==size)
				{
					y=y0;
					x++;
					if(x>=240) {POINT_COLOR=colortemp; return;}  //
					break;
				}
			}  	 
	    }    
	}else
	{
	    for(t=0;t<size;t++)
	    {   
			if(size==12)temp=asc2_1206[num][t];  //1206
			else temp=asc2_1608[num][t];		     //1608 	                          
	        for(t1=0;t1<8;t1++)
			{			    
		        if(temp&0x80)LCD_DrawPoint(x,y); 
				temp<<=1;
				y++;
				if(y>=320){POINT_COLOR=colortemp;return;}//
				if((y-y0)==size)
				{
					y=y0;
					x++;
					if(x>=240){POINT_COLOR=colortemp;return;}//
					break;
				}
			}  	 
	    }     
	}
	POINT_COLOR=colortemp;	    	   	 	  
}
uint32_t LCD_Pow(char m,char n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}
void LCD_ShowNum(uint16_t x,uint16_t y,uint32_t num,char len,char size)
{         	
	char t,temp;
	char enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+(size/2)*t,y,' ',size,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,0); 
	}
}
void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,char size,char *p)
{         
	char x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))
    {       
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;
        LCD_ShowChar(x,y,*p,size,1);
        x+=size/2;
        p++;
    }  
}
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
  LCD_WR_REG(0X2A); 
	LCD_WR_DATA8(Xpos>>8); 
	LCD_WR_DATA8(Xpos&0XFF);	 
	LCD_WR_REG(0X2B); 
	LCD_WR_DATA8(Ypos>>8); 
	LCD_WR_DATA8(Ypos&0XFF);
} 

void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(0X2C);  
}
void showhanzi16(unsigned int x,unsigned int y,unsigned char index)	
{  
	unsigned char i,j,k;
	const unsigned char *temp=hanzi16;    
	temp+=index*32;	
	for(j=0;j<16;j++)
	{
		LCD_SetCursor(x,y+j);
		LCD_WriteRAM_Prepare();
		for(k=0;k<2;k++)
		{
			for(i=0;i<8;i++)
			{ 		     
			 	if((*temp&(1<<i))!=0)
				{
					LCD_WR_DATA(POINT_COLOR);
				} 
				else
				{
					LCD_WR_DATA(BACK_COLOR);
				}   
			}
			temp++;
		}
	 }
}
void showhanzi32(unsigned int x,unsigned int y,unsigned char index)	
{  
	unsigned char i,j,k;
	const unsigned char *temp=hanzi32;    
	temp+=index*128;	
	for(j=0;j<32;j++)
	{
		LCD_SetCursor(x,y+j);
		LCD_WriteRAM_Prepare();	//GRAM
		for(k=0;k<4;k++)
		{
			for(i=0;i<8;i++)
			{ 		     
			 	if((*temp&(1<<i))!=0)
				{
					LCD_WR_DATA(POINT_COLOR);
				} 
				else
				{
					LCD_WR_DATA(BACK_COLOR);
				}   
			}
			temp++;
		}
	 }
}	
void xianshi()  //界面显示
{ 
	LCD_Clear(RED);     //WHITE
	BACK_COLOR=RED;
	POINT_COLOR=WHITE;   
	//32*32
	showhanzi32(20,10,0);	    //心
	showhanzi32(60,10,1);	    //率
	showhanzi32(100,10,2);    //检
	showhanzi32(140,10,3);	  //测
	showhanzi32(180,10,4);	  //仪
	//16*16 
	
	showhanzi16(20,80,18);	  //设
	showhanzi16(40,80,19);	  //计
		
	showhanzi16(20,120,2);	  //脉
	showhanzi16(40,120,3);	  //搏
	showhanzi16(60,120,15);	  //数
	showhanzi16(80,120,16);	  //值
	
	showhanzi16(20,140,0);	  //心
	showhanzi16(40,140,1);	  //率
	showhanzi16(60,140,15);	  //数
	showhanzi16(80,140,16);	  //值

	showhanzi16(20,160,11);	  //相
	showhanzi16(40,160,12);	  //邻
	showhanzi16(60,160,0);	  //心
	showhanzi16(80,160,17);	  //拍
	showhanzi16(100,160,4);	  //时
	showhanzi16(120,160,5);	  //间
	
	LCD_ShowString(60,80,160,16,16,": jinglxixi ");
	LCD_ShowString(140,120,40,16,16,": S");
	LCD_ShowString(140,140,20,16,16,": B");
	LCD_ShowString(140,160,20,16,16,": Q");
	LCD_ShowString(20,240,200,16,16,"2.2 SPI TFT  240*320");
	LCD_ShowString(20,260,200,16,16,"     2016.8.26      ");
}

	
//   ADC1:PA0  USART2:  115200 bps  USART2_TX:PA2  USART2_RX:PA3
	
int main(void)
{
	unsigned char m=0;
  unsigned int b=300; 
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
	GPIO_Configuration();
	Lcd_Init();           //tft
	xianshi();
	delay_ms(1000);

	LCD_Clear(WHITE);
	BACK_COLOR=WHITE;
	POINT_COLOR=RED;   // BLACK  YELLOW   ;
	
	showhanzi16(20,0,2);	  //脉
	showhanzi16(40,0,3);	  //搏
	showhanzi16(60,0,6);	  //波
	showhanzi16(80,0,7);	  //形

	POINT_COLOR=RED;
	LCD_ShowString(0,302,200,16,16,"Design : jinglxixi ");
	LCD_DrawLine(0, 300, 239, 300);

	HAL_TIM_Base_Start_IT(&htim3);

	BACK_COLOR=WHITE;
	POINT_COLOR=RED;  //BLUE;
  while (1)
  {
		sendDataToProcessing('S', Signal);     // send Processing the raw Pulse Sensor data
		LCD_ShowString(140,240,80,16,16,"S  :");
		LCD_ShowNum(180,240,Signal,3,16);
		if (Signal>=750) Signal=750;
		LCD_DrawLine(m*4, 300-b*2/5, (m+1)*4, 300-Signal*2/5); 
		b=Signal;
		m++;
		if (m>=60) 
		{
		m=0;
		LCD_Clear(WHITE);
		showhanzi16(20,0,2);	  //脉
	  showhanzi16(40,0,3);	  //搏
	  showhanzi16(60,0,6);	  //波
	  showhanzi16(80,0,7);	  //形
		LCD_DrawLine(0, 300, 239, 300);
		LCD_ShowString(0,302,200,16,16,"Design : jinglxixi ");
		}	
		
		if (QS == true)
		{
			  LCD_ShowString(140,260,200,16,16,"BPM:");
			  LCD_ShowString(140,280,200,16,16,"HRV:");
			  LCD_ShowNum(180,260,BPM,3,16);
			  LCD_ShowNum(180,280,IBI,3,16);
			  sendDataToProcessing('B',BPM);   // send heart rate with a 'B' prefix
				sendDataToProcessing('Q',IBI);   // send time between beats with a 'Q' prefix
				QS = false;                      // reset the Quantified Self flag for next time
		}
		HAL_Delay(20);	//delay for 20ms
  }

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION10b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF); 

  return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	unsigned int runningTotal;
	if(htim->Instance==htim3.Instance)
	{
		Signal=HAL_ADC_GetValue(&hadc1);					 // read the Pulse Senso
		sampleCounter += 2;                         // keep track of the time in mS with this variable
		Num = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise
		HAL_ADC_Start(&hadc1);									//restart ADC conversion

		//  find the peak and trough of the pulse wave
  if(Signal < thresh && Num > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
    if (Signal < T){                        // T is the trough
      T = Signal;                         // keep track of lowest point in pulse wave 
    }
  }

  if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
    P = Signal;                             // P is the peak
  }                                        // keep track of highest point in pulse wave

  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (Num > 250){                                   // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (Num > (IBI/5)*3) ){        
      Pulse = true;                               // set the Pulse flag when we think there is a pulse
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);                // turn on pin 13 LED
      IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
      lastBeatTime = sampleCounter;               // keep track of time for next pulse

      if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
        secondBeat = false;                  // clear secondBeat flag
        for(int i=0; i<=9; i++){             // seed the running total to get a realisitic BPM at startup
          rate[i] = IBI;                      
        }
      }

      if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
        firstBeat = false;                   // clear firstBeat flag
        secondBeat = true;                   // set the second beat flag
        return;                              // IBI value is unreliable so discard it
      }   


      // keep a running total of the last 10 IBI values
      runningTotal = 0;                  // clear the runningTotal variable    

      for(int i=0; i<=8; i++){                // shift data in the rate array
        rate[i] = rate[i+1];                  // and drop the oldest IBI value 
        runningTotal += rate[i];              // add up the 9 oldest IBI values
      }

      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values 
      BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
      QS = true;                              // set Quantified Self flag 
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }                       
  }

  if (Signal < thresh && Pulse == true){   // when the values are going down, the beat is over
     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);            // turn off pin 13 LED
    Pulse = false;                         // reset the Pulse flag so we can do it again
    amp = P - T;                           // get amplitude of the pulse wave
    thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
    P = thresh;                            // reset these for next time
    T = thresh;
  }

  if (Num > 2500){                           // if 2.5 seconds go by without a beat
    thresh = 512;                          // set thresh default
    P = 512;                               // set P default
    T = 512;                               // set T default
    lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
    firstBeat = true;                      // set these to avoid noise
    secondBeat = false;                    // when we get the heartbeat back
  }

	}
}

void sendDataToProcessing(char symbol, int dat )
{
    putchar(symbol);                // symbol prefix tells Processing what type of data is coming
		printf("%d\r\n",dat);						// the data to send culminating in a carriage return
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
