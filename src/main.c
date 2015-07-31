/**
 ******************************************************************************
 * @file    IO_Toggle/main.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    19-September-2011
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" // again, added because ST didn't put it here ?
#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"

#include "usart2.h"
#include "ctrltbl.h"

#include <stdio.h>
#include <limits.h>
#include <math.h>

//TDKセンサのレジスタアドレス
#define kCNTL (0x00)
#define kLED (0x10)
#define kMEM_CONTROL2 (0x2c)
#define kMEM_CONTROL (0x2d)
#define kPRE_CALIB (0x40)
#define kANG_OFST (0x148)
#define kANG_OUTH (0x06)
#define kANG_OUTL (0x08)
#define kSTS8 (0x1b)


#define kPWMMAX (1000)	//PWMのフルスケール値
#define _PC10 ((uint16_t)(1<<10)) // LED1
#define _PC11 ((uint16_t)(1<<11)) // LED2
#define _PB10 ((uint16_t)(1<<10)) // USART3 TX
#define _PB11 ((uint16_t)(1<<11)) // USART3 RX
#define _PB2  ((uint16_t)(1<<2))  // USART3 DIR
#define _PB12 ((uint16_t)(1<<12)) // USART3 TERM

#define _PA8  ((uint16_t)(1<<8))
#define _PA7  ((uint16_t)(1<<7))

#define _PA9  ((uint16_t)(1<<9))
#define _PB14 ((uint16_t)(1<<14))

#define _PA10 ((uint16_t)(1<<10))
#define _PB15 ((uint16_t)(1<<15))

void Delay(__IO uint32_t nCount);
void xTim1Init();
void xSetPwm(const int16_t duty);
void xGPIOInit();
void xSpi2Init();
void xSpi3Init();
void xTim5Init();
void xTim4Init();
float xCurrentRead(void);
void xADCInit();
void xLed(int sw);
void xTim2Init(void);
void xSetTim2CountDir(void);
void xTim3Init(void);

TControlTable gT;
uint16_t g_ad_value[8];
int32_t gTimCount;

long gLineInt;

int pwm=0;

int32_t gCount;
int32_t gSinTable[100];
int32_t gFreq=50;
int32_t gPower=0;
float gPhase=0.0;


int main(void)
{
	char buf[256];


	xGPIOInit();
	GPIOB->BSRRL = _PB14;    // PB14=H (BSRRLに書くとH BSRRHに書くとLになる IR2302の/SHDN
	GPIOB->BSRRL = _PB15;    // PB15=H (BSRRLに書くとH BSRRHに書くとLになる IR2302の/SHDN

	xUSART2_init();
	xTim1Init();	//PWM
	xSetPwm(0);
	//	xADCInit();
	//	xTim2Init();
	//	xEXTILineConfig();	//DIR(PC11)入力で割り込みがかかるようにする。
	//	xSetTim2CountDir();





	xTim3Init();	//タイマ割り込み(10khz)

	int rxnum=0;


	xUSART2_puts("\r\n\r\n\r\n\r\nwelcome to nucleo F401 inverter\r\n");



	//make sin table
	float p=0.0;
	float step=(2.0*3.141592)/100;
	int i;
	for(i=0;i<100;i++){
		gSinTable[i]=sin(p)*1000;
		p+=step;
		sprintf(buf,"%d,%d\r\n",i,gSinTable[i]);
		xUSART2_puts(buf);
	}

	while (1)
	{

		int dispcount=0;

		rxnum=xUSART2_rxnum();

		dispcount++;

		//シリアル入力の処理
		if(rxnum>0){
			char c=xUSART2_getc();
			/*
			if(c=='0'){
				pwm=0;
				xSetPwm(pwm);
			}
			if(c=='1'){
				pwm+=10;
				xSetPwm(pwm);
			}
			if(c=='2'){
				pwm-=10;
				xSetPwm(pwm);
			}*/
			sprintf(buf,"tim,%ld,pwm,%d\r\n",
					gTimCount,
					pwm);
			xUSART2_puts(buf);

			if(c=='1')gPower+=10;
			if(c=='2')gPower-=10;
			if(c=='3')gFreq++;
			if(c=='4')gFreq--;
			if(c=='0')gPower=0;

		}
		sprintf(buf,"freq,%d,power,%d\r\n",gFreq,gPower);
		xUSART2_puts(buf);


		//		Delay(0xFFF);


	}
}
void xLed(int sw)
{
	if(sw){
		//LED点灯
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
	}else{
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	}
}

void xCurrentSensorCalib()
{
	//電流センサの平均値をとり、
	float sum=0;
	int i;
	char buf[256];
	xUSART2_puts("current sensor calib.\r\n");
	for(i=0;i<100;i++){
		sum+=g_ad_value[0];
		//		sprintf(buf,"%d,adval=%d\r\n",i,g_ad_value[0]);
		//		xUSART2_puts(buf);
		Delay(0x10000);
	}
	gT.present_current_sensor_offset=(sum/100.0);
	sprintf(buf,"present_current_sensor_offset=%d\r\n",gT.present_current_sensor_offset);
	xUSART2_puts(buf);
}


float xCurrentRead(void)
{
	//電流値[A]を取得
	float raw=(float)((g_ad_value[0])-gT.present_current_sensor_offset);
	//170ns
	//float current=raw/4096.0*3.3/0.185;	//ACS712出力は185mV/A、AD分解能12ビット=4096、ADレンジ=3.3Vより //9.4us
	float current=raw*0.004354941;	//1.52us
	return current;
}


//PC11を入力にして外部割込みがかかるようにする
void xEXTILineConfig(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	// Enable GPIOC clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// Enable SYSCFG clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// PC11を入力ピンに設定
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// PC11をEXTIにつなぐ
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource11);

	// EXTIの15_10ラインにPC11をつないで、いずれかの変化で割り込みがかかるようにしたい
	// Configure EXTI Line11
	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Enable and set EXTI15_10 Interrupt to the lowest priority
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// GPIO割り込み PC11の変化で割り込みがかかる
void EXTI15_10_IRQHandler(void)
{

	EXTI_ClearITPendingBit(EXTI_Line11);   // 割り込みペンディングビットをクリア
	xSetTim2CountDir();
	gLineInt++;
}
void xSetTim2CountDir(void)
{
	//PC11(DIR入力)に従ってTIM2のカウント方向を変える
	int pc11;
	pc11=(GPIO_ReadInputData(GPIOC)>>11)&0x01;	//PC11を読み取ってTIM2のカウント方向を変化する
	if(pc11==1){
		TIM2->CR1=(TIM2->CR1 | 0x0010);	//downcount;
	}else{
		TIM2->CR1=(TIM2->CR1 & 0xffef);	//upcount
	}

}
/// @brief タイマ3割り込みハンドラ
/// @note 割り込み周期10KHz
void TIM3_IRQHandler (void)
{
	xLed(1);
	// TIM2_FREQUENCY周期での処理
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {
		TIM_ClearITPendingBit (TIM3, TIM_IT_CC1);


		gPhase+=(0.01*gFreq);
		if(gPhase>=100.0)gPhase=0.0;
		xSetPwm(gSinTable[(int)gPhase]*gPower/1000);

	}


	gTimCount++;
	xLed(0);
}

//===============================================================================


void xTim3Init(void)
{
	//TIM3はタイマ割り込み(10KHz)で制御ループを回す
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;            /* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  /* value = 0 - 15*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         /* value = 0 - 15*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);



	TIM_TimeBaseInitTypeDef  TIM3_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM3_OCInitStructure;
	TIM_DeInit (TIM3);
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3, ENABLE);	//tim2クロックは84MHz。
	TIM3_TimeBaseStructure.TIM_Period = (84000000 / 10000) - 1;	//=8400　これで10KHz
	TIM3_TimeBaseStructure.TIM_Prescaler = 0;	//プリスケーラ。0だと1分周。
	TIM3_TimeBaseStructure.TIM_ClockDivision = 0;	//外部クロックサンプルの間隔。使用しないので0。
	TIM3_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit (TIM3, &TIM3_TimeBaseStructure);

	TIM3_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM3_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM3_OCInitStructure.TIM_Pulse = 0;
	TIM3_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init (TIM3, &TIM3_OCInitStructure);
	TIM_OC1PreloadConfig (TIM3, TIM_OCPreload_Enable);
	TIM_ITConfig (TIM3, TIM_IT_CC1, ENABLE);
	TIM_Cmd (TIM3, ENABLE);
}


void xADCInit()
{
	/// @brief AD変換初期化
	/// @note チャンネル設定のみハンド仕様
	/// @note 元のxADCInit
	// A/D変換が終了すると値はDMAにより変数g_ad_valueに転送される
	//PC14 電流センサ
	//PC0 予備


	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;

	// Enable peripheral clocks
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_DMA2 , ENABLE);
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_ADC1, ENABLE);


	//GPIOのピンをADC入力に設定する
	//GPIO_Cの出力ポート設定
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// DMA2 Stream0 channel0 configU3ration
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&g_ad_value;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 1;	//ADCチャンネル数
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init (DMA2_Stream0, &DMA_InitStructure);

	DMA_Cmd (DMA2_Stream0, ENABLE);

	// ADC Common configU3ration
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_2;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInit (&ADC_CommonInitStructure);

	// ADC1 regU3lar channel 12 configU3ration
	ADC_DeInit();
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;	//ADCの本数
	ADC_Init (ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_480Cycles);
	//	ADC_RegularChannelConfig(ADC1, ADC_Channel_0,  2, ADC_SampleTime_480Cycles);


	// Enable ADC1 DMA
	ADC_DMACmd(ADC1, ENABLE);
	// Enable DMA request after last transfer
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
	// Start ADC1 Software Conversion
	ADC_SoftwareStartConv(ADC1);


}

void xGPIOInit()
{
	/* GPIOD Periph clock enable */
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	//GPIO_Aの出力ポート設定
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//GPIO_Bの出力ポート設定
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//GPIO_Cの出力ポート設定
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//GPIO_Dの出力ポート設定
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


}


void Delay(__IO uint32_t nCount)
{
	while(nCount--)
	{
	}
}

// TB6643むけPWM出力
// IN1=PA9(TIM1_CH2)
// IN1=PA10(TIM1_CH3)
// dは(-1000)～1000 0が停止
void xSetPwm(int16_t duty)
{

	if(duty<-kPWMMAX)duty=-kPWMMAX;
	if(duty>kPWMMAX)duty=kPWMMAX;


	if(duty >= 0) {
		// 正転 U=PWM_HiZ(PB15_CH3N=Lo CH3_PA10=PWM) V=GND(PB14_CH2N=Hi PA9_CH2=Lo)
		// PA10=CH3 =PMW+
		// PA9_ CH2 =L

		//                                                                             1514 1312 1110  9 8  7 6  5 4  3 2  1 0   1514 1312 1110  9 8  7 6  5 4  3 2  1 0
		GPIOA->MODER = ((GPIOA->MODER) & 0xffcfffff) | 0x00200000;   // PA10 = AF[10] &1111 1111 1100 1111 1111 1111 1111 1111 | 0000 0000 0010 0000 0000 0000 0000 0000
		GPIOA->AFR[10] = 0x1;    // TIM1

		//                                                                              1514 1312 1110  9 8  7 6  5 4  3 2  1 0   1514 1312 1110  9 8  7 6  5 4  3 2  1 0
		GPIOA->MODER = ((GPIOA->MODER) & 0xfff3ffff) | 0x00040000;   // PA9 = GPIO[01] &1111 1111 1111 0011 1111 1111 1111 1111 | 0000 0000 0000 0100 0000 0000 0000 0000
		//    GPIOA->BSRRL = _PA9;    // PA9=H (BSRRLに書くとH BSRRHに書くとLになる PWMオフ時道通
		GPIOA->BSRRH = _PA9;    // PA9=H (BSRRLに書くとH BSRRHに書くとLになる PWMオフ時HiZ


		TIM1->CCR3 = duty;
	}
	else {
		// 逆転 U=GND(PB15_CH3N=Hi CH3_PA10=Lo) V=PWM-HiZ(PB14_CH2N=Lo PA9_CH2=PWM)
		// PA10=CH3 =L
		// PA9_ CH2 =PWM+

		//                                                                              1514 1312 1110  9 8  7 6  5 4  3 2  1 0   1514 1312 1110  9 8  7 6  5 4  3 2  1 0
		GPIOA->MODER = ((GPIOA->MODER) & 0xffcfffff) | 0x00100000;   // PA10 = GPIO[01] &1111 1111 1100 1111 1111 1111 1111 1111 | 0000 0000 0001 0000 0000 0000 0000 0000
		//   GPIOA->BSRRL = _PA10;    //PA10=H (BSRRLに書くとH BSRRHに書くとLになる
		GPIOA->BSRRH = _PA10;    //PA10=H (BSRRLに書くとH BSRRHに書くとLになる

		//                                                                              1514 1312 1110  9 8  7 6  5 4  3 2  1 0   1514 1312 1110  9 8  7 6  5 4  3 2  1 0
		GPIOA->MODER = ((GPIOA->MODER) & 0xfff3ffff) | 0x00080000;   // PA9 = AF[10] &1111 1111 1111 0011 1111 1111 1111 1111 | 0000 0000 0000 1000 0000 0000 0000 0000
		GPIOA->AFR[9] = 0x1;   // AF=TIM1


		//pwm = -pwm;
		TIM1->CCR2 = -duty;   //kPWMMAXをCCR2に入れた時、100%dutyになる
	}
}


/// @brief TIM1（PWM）の初期化
void xTim1Init()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM1_BDTRInitStructure;
	uint16_t TimerPeriod1 = 0;

	TIM_DeInit (TIM1);
	// TimerPeriod1=PWM周期[tick]。
	// フリーランカウンタの上限値を決める。TIM_FREQUENCYはPWM周波数を表す。50KHzなので50000。
	// すなわちTimerPeriod1=2400[tick]。1tick=1/120MHz[sec]
	// 2400[tick]=2400*(1/120M)=20[us]
	// 1/20[us]=50[KHz]となる。
	//  TimerPeriod1 = (4200) - 1;		//10KHz. 84000000/20000=4200でフルスケール。
	TimerPeriod1 = (kPWMMAX) - 1;		//16.8MHz/1000=16.8KHz. 1000でフルスケール。

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM_TimeBaseStructure.TIM_Prescaler = 5;	//84M/5=16.8MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TimerPeriod1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  // TIMx_CNT < TIMx_CCRyであるときOCがHになるモード。つまりCCRレジスタが0で停止、TimerPeriodでMAX出力。

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;   // 補相出力（反転したもの）の出力を有効化

	TIM_OCInitStructure.TIM_Pulse = 0;    // 初期のCCRレジスタの値を決める。
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;

	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 0;    // 初期の出力
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 0;    // 初期の出力
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;

	TIM1_BDTRInitStructure.TIM_DeadTime = 48;  // 0.4usのデッドバンド。単位は[tick] 1tick=1/120MHz

	TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM1_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRConfig(TIM1, &TIM1_BDTRInitStructure);

	TIM_Cmd(TIM1, ENABLE);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	//GPIOの出力をTIM1に切り替える
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);


}


