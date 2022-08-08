/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

// This Function Provides Delay In Milliseconds Using DWT



/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum{tachefond,green,red,blue};

enum{alpha,blue2,green2,red2,freq,ifs,tachefond2};

enum{blanc,bleu,vert,rouge,violet, jaune, cyan,};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GAUCHE 0
#define DROITE 1
#define DelaiNextState 250
#define FALSE 0
#define TRUE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 *  Project     Segmented LED Display - ASCII Library
 *  @author     David Madison
 *  @link       github.com/dmadison/Segmented-LED-Display-ASCII
 *  @license    MIT - Copyright (c) 2017 David Madison
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */
const uint8_t SevenSegmentASCII[96] = {0x00, /* (space) */0x86, /* ! */0x22, /* " */0x7E, /* # */0x6D, /* $ */
		0xD2, /* % */
		0x46, /* & */
		0x20, /* ' */
		0x29, /* ( */
		0x0B, /* ) */
		0x21, /* * */
		0x70, /* + */
		0x10, /* , */
		0x40, /* - */
		0x80, /* . */
		0x52, /* / */
		0x3F, /* 0 */
		0x06, /* 1 */
		0x5B, /* 2 */
		0x4F, /* 3 */
		0x66, /* 4 */
		0x6D, /* 5 */
		0x7D, /* 6 */
		0x07, /* 7 */
		0x7F, /* 8 */
		0x6F, /* 9 */
		0x09, /* : */
		0x0D, /* ; */
		0x61, /* < */
		0x48, /* = */
		0x43, /* > */
		0xD3, /* ? */
		0x5F, /* @ */
		0x77, /* A */
		0x7C, /* B */
		0x39, /* C */
		0x5E, /* D */
		0x79, /* E */
		0x71, /* F */
		0x3D, /* G */
		0x76, /* H */
		0x30, /* I */
		0x1E, /* J */
		0x75, /* K */
		0x38, /* L */
		0x15, /* M */
		0x37, /* N */
		0x3F, /* O */
		0x73, /* P */
		0x6B, /* Q */
		0x33, /* R */
		0x6D, /* S */
		0x78, /* T */
		0x3E, /* U */
		0x3E, /* V */
		0x2A, /* W */
		0x76, /* X */
		0x6E, /* Y */
		0x5B, /* Z */
		0x39, /* [ */
		0x64, /* \ */
		0x0F, /* ] */
		0x23, /* ^ */
		0x08, /* _ */
		0x02, /* ` */
		0x5F, /* a */
		0x7C, /* b */
		0x58, /* c */
		0x5E, /* d */
		0x7B, /* e */
		0x71, /* f */
		0x6F, /* g */
		0x74, /* h */
		0x10, /* i */
		0x0C, /* j */
		0x75, /* k */
		0x30, /* l */
		0x14, /* m */
		0x54, /* n */
		0x5C, /* o */
		0x73, /* p */
		0x67, /* q */
		0x50, /* r */
		0x6D, /* s */
		0x78, /* t */
		0x1C, /* u */
		0x1C, /* v */
		0x14, /* w */
		0x76, /* x */
		0x6E, /* y */
		0x5B, /* z */
		0x46, /* { */
		0x30, /* | */
		0x70, /* } */
		0x01, /* ~ */
		0x00, /* (del) */
};

void all_on_SEG(void)
{
	HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SEG_DP_GPIO_Port, SEG_DP_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(DIGIT1_GPIO_Port, DIGIT1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIGIT2_GPIO_Port, DIGIT2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIGIT3_GPIO_Port, DIGIT3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIGIT4_GPIO_Port, DIGIT4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIGIT5_GPIO_Port, DIGIT5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIGIT6_GPIO_Port, DIGIT6_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIGIT7_GPIO_Port, DIGIT7_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIGIT8_GPIO_Port, DIGIT8_Pin, GPIO_PIN_SET);
}

void initSEG(void)
{
	uint16_t pinportA = SEG_A_Pin;
	uint16_t pinportC = SEG_B_Pin|SEG_C_Pin|SEG_D_Pin;
	uint16_t pinportB = SEG_F_Pin|SEG_G_Pin|SEG_DP_Pin|DIGIT1_Pin|DIGIT2_Pin|DIGIT3_Pin|DIGIT4_Pin|DIGIT5_Pin|DIGIT6_Pin|DIGIT7_Pin|DIGIT8_Pin;
	uint16_t pinportD = SEG_E_Pin;

	HAL_GPIO_WritePin(SEG_A_GPIO_Port, pinportA, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(SEG_B_GPIO_Port,pinportC , GPIO_PIN_RESET);

	HAL_GPIO_WritePin(SEG_E_GPIO_Port, pinportD, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(SEG_F_GPIO_Port,pinportB , GPIO_PIN_RESET);

}


void uprintchar(int shiftseg1,int shiftseg2,int shiftseg3,int shiftseg4,int shiftseg5,int shiftseg6,int shiftseg7,int shiftseg8,uint16_t Pin)
{

	uint16_t pinsegBCD=(SEG_B_Pin&(shiftseg2<<9))|(SEG_C_Pin&shiftseg3<<9)|(SEG_D_Pin&shiftseg4<<9);
	uint16_t pinsegFGDPdig=(SEG_F_Pin&(shiftseg6>>2))|(SEG_G_Pin&shiftseg7>>2)|(SEG_DP_Pin&shiftseg8>>2)|Pin;
	uint16_t pinsegA=(SEG_A_Pin&(shiftseg1<<15));
	uint16_t pinsegE=(SEG_E_Pin&(shiftseg5>>2));
	initSEG();
	GPIOC->ODR = pinsegBCD;
	GPIOA->ODR = pinsegA;
	GPIOD->ODR = pinsegE;
	GPIOB->ODR = pinsegFGDPdig;
}

void printchar(int chara,uint16_t Pin)
{
	int segmA =(0x01&(SevenSegmentASCII[chara-32]));
	int segmB =(0x02&(SevenSegmentASCII[chara-32]))>>1;
	int segmC =(0x04&(SevenSegmentASCII[chara-32]))>>2;
	int segmD =(0x08&(SevenSegmentASCII[chara-32]))>>3;
	int segmE =(0x10&(SevenSegmentASCII[chara-32]))>>4;
	int segmF =(0x20&(SevenSegmentASCII[chara-32]))>>5;
	int segmG =(0x40&(SevenSegmentASCII[chara-32]))>>6;
	int segmDP =(0x80&(SevenSegmentASCII[chara-32]))>>7;
	initSEG();
	HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, segmA);
	HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, segmB);
	HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, segmC);
	HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, segmD);
	HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, segmE);
	HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, segmF);
	HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, segmG);
	HAL_GPIO_WritePin(SEG_DP_GPIO_Port, SEG_DP_Pin, segmDP);
	HAL_GPIO_WritePin(DIGIT1_GPIO_Port, Pin, GPIO_PIN_SET);
}


void printFigures(int number,uint16_t Pin, int leading_zero)
{
	printchar(number+48,Pin);

	if(!leading_zero)
	{
		if(number==0)
		{
			HAL_GPIO_WritePin(GPIOB, Pin, GPIO_PIN_RESET);
		}
	}
}

void affichageDig(int position_compteur,int delai,int position, int limpos,int limneg,int appel)
{
	initSEG();
	static int j=0;
	int digit[4];
	int pos=position*6;

	if(appel&&j==0)
		j=1;
	if(appel)
		j--;
	if(!appel&&j==4)
		j=0;
	switch (j){

	case 0:
	{
		if(position_compteur==limpos+1)
			position_compteur=0;

		if(position_compteur<0)
		{
			printchar('-', DIGIT1_Pin);
			if(position_compteur==-(limneg+1))
				position_compteur=0;
		}
		digit[0]=abs(position_compteur)%10;
		digit[1]=((abs(position_compteur)-digit[0])/10)%10;
		digit[2]=((abs(position_compteur)-digit[1]*10-digit[0])/100)%10;
		digit[3]=((abs(position_compteur)-digit[2]*100-digit[0]-digit[1]*10)/1000)%10;
		printFigures(digit[0], DIGIT4_Pin<<pos,1);
		HAL_Delay(delai);
		break;
	}
	case 1:
	{
		if(position_compteur==limpos+1)
			position_compteur=0;


		if(position_compteur<0)
		{
			printchar('-', DIGIT1_Pin);
			if(position_compteur==-(limneg+1))
				position_compteur=0;
		}
		digit[0]=abs(position_compteur)%10;
		digit[1]=((abs(position_compteur)-digit[0])/10)%10;
		digit[2]=((abs(position_compteur)-digit[1]*10-digit[0])/100)%10;
		digit[3]=((abs(position_compteur)-digit[2]*100-digit[0]-digit[1]*10)/1000)%10;
		printFigures(digit[1], DIGIT3_Pin<<pos,1);
		HAL_Delay(delai);
		break;
	}
	case 2:
	{
		if(position_compteur==limpos+1)
			position_compteur=0;

		if(position_compteur<0)
		{
			printchar('-', DIGIT1_Pin);
			if(position_compteur==-(limneg+1))
				position_compteur=0;
		}
		digit[0]=abs(position_compteur)%10;
		digit[1]=((abs(position_compteur)-digit[0])/10)%10;
		digit[2]=((abs(position_compteur)-digit[1]*10-digit[0])/100)%10;
		digit[3]=((abs(position_compteur)-digit[2]*100-digit[0]-digit[1]*10)/1000)%10;
		printFigures(digit[2], DIGIT2_Pin<<pos, 1);
		HAL_Delay(delai);
		break;
	}
	case 3:
	{
		if(position_compteur==limpos+1)
			position_compteur=0;

		if(position_compteur<0)
		{
			printchar('-', DIGIT1_Pin);
			if(position_compteur==-(limneg+1))
				position_compteur=0;
		}
		else
		{
			digit[0]=abs(position_compteur)%10;
			digit[1]=((abs(position_compteur)-digit[0])/10)%10;
			digit[2]=((abs(position_compteur)-digit[1]*10-digit[0])/100)%10;
			digit[3]=((abs(position_compteur)-digit[2]*100-digit[0]-digit[1]*10)/1000)%10;
			printFigures(digit[3], DIGIT1_Pin<<pos, 1);
		}
		HAL_Delay(delai);
		break;
	}
	}
	j++;
}
void affichageChar(char * word,int delai, int position, int appel)
{
	static int i=0;
	int pos=position*6;
	if(appel&&i==0)
		i=1;
	if(appel)
		i--;
	if(!appel&&i==4)
		i=0;
	switch (i){
	case 0:
	{
		printchar(word[i],DIGIT1_Pin<<pos);
		HAL_Delay(delai);
	}
	break;
	case 1:
	{
		printchar(word[i],DIGIT2_Pin<<pos);
		HAL_Delay(delai);
	}
	break;
	case 2:
	{
		printchar(word[i],DIGIT3_Pin<<pos);
		HAL_Delay(delai);

	}
	break;
	case 3:
	{
		printchar(word[i],DIGIT4_Pin<<pos);
		HAL_Delay(delai);
	}
	break;
	}
	i++;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	int delai=1;
	int SW1; int SW2;
	int lastState=HAL_GPIO_ReadPin(ROT_A_2_GPIO_Port, ROT_A_2_Pin);
	int lastState1=HAL_GPIO_ReadPin(ROT_A_1_GPIO_Port, ROT_A_1_Pin);
	int lastBstate=HAL_GPIO_ReadPin(ROT_B_1_GPIO_Port, ROT_B_1_Pin);
	int lastBbstate=HAL_GPIO_ReadPin(ROT_B_1_GPIO_Port, ROT_B_1_Pin);
	int lastBstate2=HAL_GPIO_ReadPin(ROT_B_2_GPIO_Port, ROT_B_2_Pin);

	int duty_cycler=5;int duty_cycleg=5;int duty_cycleb=5;
	int duty_cycler2=5;int duty_cycleg2=5;int duty_cycleb2=5;

	int frequency=32000000;
	int32_t autoreload = 1000;
	int32_t compare=(duty_cycleb2*autoreload)/100;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty_cycler2);__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_cycleg2);__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty_cycleb2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_cycler);__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_cycleg);__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_cycleb);
	int pulser=1000;int pulseg=1000;int pulseb=1000;

	static int etat=green;
	static int etat2=green2;
	int comptsw=0;
	static int alphaa=50;
	static int couleur=blanc;
	static int appel1=0;
	static int appel2=1;
	static int compt=0;
	int shortpress=FALSE;
	int longpress=FALSE;
	int shortpress2=FALSE;
	int longpress2=FALSE;
	int flag=0;
	int sauv=0;
	int time=0;
	HAL_GPIO_WritePin(GPIOC, PC3_Pin|PC4_Pin|PC5_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//Get State of switches
		SW1= HAL_GPIO_ReadPin(ROT_SW_1_GPIO_Port, ROT_SW_1_Pin);
		SW2= HAL_GPIO_ReadPin(ROT_SW_2_GPIO_Port, ROT_SW_2_Pin);

		//Encoder right
		switch(etat)
		{
		case tachefond:
			appel2=0;
			compt++;
			if(compt>1000)
				affichageChar(" BOW", delai,DROITE,appel1);
			if(compt>2000)
				compt=0;
			if(compt<1000)
				affichageChar("RAIN", delai,DROITE,1);

			switch(couleur)
			{
				case blanc:
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulser/10);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulseg/10);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulseb/10);
					pulseg--;
					if(pulseg==0)
						couleur=violet;
					break;
				case violet:
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulser/10);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulseg/10);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulseb/10);
					pulser--;
					if(!pulser)
						couleur=bleu;
					break;
				case bleu:
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulser/10);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulseg/10);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulseb/10);
					pulseg++;
					if(pulseg==1000)
						couleur=cyan;
					break;
				case cyan:
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulser/10);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulseg/10);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulseb/10);
					pulseb--;
					if(!pulseb)
						couleur=vert;
					break;
				case vert:
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulser/10);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulseg/10);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulseb/10);
					pulser++;
					if(pulser==1000)
						couleur=jaune;
					break;
				case jaune:
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulser/10);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulseg/10);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulseb/10);
					pulseg--;
					if(!pulseg)
						couleur=rouge;
					break;
				case rouge:
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulser/10);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulseg/10);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulseb/10);
					pulseg++;
					pulseb++;
					if(pulseg==1000)
						couleur=blanc;
					break;
			}
			if(SW1==0)
			{
				initSEG();
				HAL_Delay(DelaiNextState);
				etat=green;
			}
			break;
			case green:

				HAL_GPIO_WritePin(DIAL_B_GPIO_Port, DIAL_B_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(DIAL_R_GPIO_Port, DIAL_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(DIAL_G_GPIO_Port, DIAL_G_Pin, GPIO_PIN_RESET);

				compt++;
				appel2=1;
				if(HAL_GPIO_ReadPin(ROT_A_1_GPIO_Port, ROT_A_1_Pin)!=lastState1)
				{
					if(lastState1==HAL_GPIO_ReadPin(ROT_B_1_GPIO_Port, ROT_B_1_Pin))
					{
							if (duty_cycleg<=0)
								duty_cycleg=0;
							else
								duty_cycleg-=1;
					}
					else
					{

						if(lastState1!=HAL_GPIO_ReadPin(ROT_B_1_GPIO_Port, ROT_B_1_Pin))
						{
							if (duty_cycleg>=100)
								duty_cycleg=100;
							else
								duty_cycleg+=1;
						}
					}
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_cycleg);
				}
				lastState1=HAL_GPIO_ReadPin(ROT_A_1_GPIO_Port, ROT_A_1_Pin);
				if(compt>1000)
					affichageDig(duty_cycleg, delai,DROITE,100,0,appel1);
				if(compt>2000)
					compt=0;
				if(compt<1000)
					affichageChar("GREE", delai,DROITE,appel1);

				SW1= HAL_GPIO_ReadPin(ROT_SW_1_GPIO_Port, ROT_SW_1_Pin);
				if(!SW1 && !longpress && !shortpress)
				{
					flag=1;
					sauv=HAL_GetTick();
					while(SW1==0 && flag==1)
					{
						time=HAL_GetTick();
						if(time-sauv>30)
						{
							shortpress=TRUE;
							longpress=FALSE;
							if(time-sauv>1000)
							{
								shortpress=FALSE;
								longpress=TRUE;
								flag=0;
							}
						}
						if((time-sauv)<600 && (time-sauv)>200)
							affichageChar("MODE", delai,DROITE,appel1);
						else
							affichageDig((time-sauv), delai,DROITE,100,0,appel1);
						SW1=HAL_GPIO_ReadPin(ROT_SW_1_GPIO_Port, ROT_SW_1_Pin);
					}
				}
				if(shortpress && SW1==1)
				{
					HAL_Delay(DelaiNextState);
					longpress=FALSE;
					shortpress=FALSE;
					etat=red;
				}
				if(longpress && SW1==1)
				{
					HAL_Delay(DelaiNextState);
					longpress=FALSE;
					shortpress=FALSE;
					etat=tachefond;
				}
				break;


			case red:
				HAL_GPIO_WritePin(DIAL_B_GPIO_Port, DIAL_B_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(DIAL_G_GPIO_Port, DIAL_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(DIAL_R_GPIO_Port, DIAL_R_Pin, GPIO_PIN_RESET);
				compt++;
				if(compt>1000)
					affichageDig(duty_cycler, delai,DROITE,100,0,appel1);
				if(compt>2000)
					compt=0;
				if(compt<1000)
					affichageChar(" RED", delai,DROITE,appel1);

				if(HAL_GPIO_ReadPin(ROT_A_1_GPIO_Port, ROT_A_1_Pin)!=lastState1)
				{
					if(lastState1==HAL_GPIO_ReadPin(ROT_B_1_GPIO_Port, ROT_B_1_Pin))
					{
						if (duty_cycler<=0)
							duty_cycler=0;
						else
							duty_cycler-=1;
					}
					else
					{
						if (duty_cycler>=100)
							duty_cycler=100;
						else
							duty_cycler+=1;
					}
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_cycler);
				}
				lastState1=HAL_GPIO_ReadPin(ROT_A_1_GPIO_Port, ROT_A_1_Pin);
				lastBstate=HAL_GPIO_ReadPin(ROT_B_1_GPIO_Port, ROT_B_1_Pin);

				SW1= HAL_GPIO_ReadPin(ROT_SW_1_GPIO_Port, ROT_SW_1_Pin);
				sauv=0;
				time=0;
				if(!SW1 && !longpress && !shortpress)
				{
					flag=1;
					sauv=HAL_GetTick();
					while(SW1==0 && flag==1)
					{
						time=HAL_GetTick();
						if(time-sauv>30)
						{
							shortpress=TRUE;
							longpress=FALSE;
							if(time-sauv>1000)
							{
								shortpress=FALSE;
								longpress=TRUE;
								flag=0;
							}
						}
						if((time-sauv)<600 && (time-sauv)>200)
							affichageChar("MODE", delai,DROITE,appel1);
						else
							affichageDig((time-sauv), delai,DROITE,100,0,appel1);
						SW1=HAL_GPIO_ReadPin(ROT_SW_1_GPIO_Port, ROT_SW_1_Pin);
					}
				}
				if(shortpress && SW1==1)
				{
					HAL_Delay(DelaiNextState);
					longpress=FALSE;
					shortpress=FALSE;
					etat=blue;
				}
				if(longpress && SW1==1)
				{
					HAL_Delay(DelaiNextState);
					longpress=FALSE;
					shortpress=FALSE;
					etat=tachefond;
				}
				break;


			case blue:
				HAL_GPIO_WritePin(DIAL_G_GPIO_Port, DIAL_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(DIAL_R_GPIO_Port, DIAL_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(DIAL_B_GPIO_Port, DIAL_B_Pin, GPIO_PIN_RESET);
				compt++;
				if(compt>1000)
					affichageDig(duty_cycleb, delai,DROITE,100,0,appel1);
				if(compt>2000)
				{
					compt=0;
				}
				if(compt<1000)
				{
					affichageChar("BLUE", delai,DROITE,appel1);
				}

				if(HAL_GPIO_ReadPin(ROT_A_1_GPIO_Port, ROT_A_1_Pin)!=lastState1)
				{
					if(lastState1==HAL_GPIO_ReadPin(ROT_B_1_GPIO_Port, ROT_B_1_Pin))
					{
						if (duty_cycleb<=0)
							duty_cycleb=0;
						else
							duty_cycleb-=1;
					}
					else
					{
						if (duty_cycleb>=100)
							duty_cycleb=100;
						else
							duty_cycleb+=1;
					}
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_cycleb);
				}
				lastState1=HAL_GPIO_ReadPin(ROT_A_1_GPIO_Port, ROT_A_1_Pin);
				lastBstate=HAL_GPIO_ReadPin(ROT_B_1_GPIO_Port, ROT_B_1_Pin);

				SW1= HAL_GPIO_ReadPin(ROT_SW_1_GPIO_Port, ROT_SW_1_Pin);
				sauv=0;
				time=0;
				if(!SW1 && !longpress && !shortpress)
				{
					flag=1;
					sauv=HAL_GetTick();
					while(SW1==0 && flag==1)
					{
						time=HAL_GetTick();
						if(time-sauv>30)
						{
							shortpress=TRUE;
							longpress=FALSE;
							if(time-sauv>1000)
							{
								shortpress=FALSE;
								longpress=TRUE;
								flag=0;
							}
						}
						if((time-sauv)<600 && (time-sauv)>200)
							affichageChar("MODE", delai,DROITE,appel1);
						else
							affichageDig((time-sauv), delai,DROITE,100,0,appel1);
						SW1=HAL_GPIO_ReadPin(ROT_SW_1_GPIO_Port, ROT_SW_1_Pin);
					}
				}
				if(shortpress && SW1==1)
				{
					HAL_Delay(DelaiNextState);
					longpress=FALSE;
					shortpress=FALSE;
					etat=green;
				}
				if(longpress && SW1==1)
				{
					HAL_Delay(DelaiNextState);
					longpress=FALSE;
					shortpress=FALSE;
					etat=tachefond;
				}
				break;
		}
		// Encoder Switch left
			switch (etat2)
			{
				case tachefond2:
					appel2=0;
					compt++;
					if(compt>1000)
						affichageChar(" BOW", delai,GAUCHE,appel1);
					if(compt>2000)
						compt=0;
					if(compt<1000)
						affichageChar("RAIN", delai,GAUCHE,1);

					switch(couleur)
					{
						case blanc:
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulser/10);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulseg/10);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulseb/10);
							pulseg--;
							if(pulseg==0)
								couleur=violet;
							break;
						case violet:
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulser/10);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulseg/10);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulseb/10);
							pulser--;
							if(!pulser)
								couleur=bleu;
							break;
						case bleu:
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulser/10);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulseg/10);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulseb/10);
							pulseg++;
							if(pulseg==1000)
								couleur=cyan;
							break;
						case cyan:
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulser/10);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulseg/10);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulseb/10);
							pulseb--;
							if(!pulseb)
								couleur=vert;
							break;
						case vert:
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulser/10);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulseg/10);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulseb/10);
							pulser++;
							if(pulser==1000)
								couleur=jaune;
							break;
						case jaune:
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulser/10);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulseg/10);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulseb/10);
							pulseg--;
							if(!pulseg)
								couleur=rouge;
							break;
						case rouge:
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulser/10);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulseg/10);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulseb/10);
							pulseg++;
							pulseb++;
							if(pulseg==1000)
								couleur=blanc;
							break;
					}

					if(SW1==0)
					{
						initSEG();
						HAL_Delay(DelaiNextState);
						etat=green2;
					}
					break;
			case green2:
				compt++;
				if(compt>1000)
					affichageDig(duty_cycleg2, delai,GAUCHE,100,0,appel2);
				if(compt>2000)
					compt=0;
				if(compt<1000)
					affichageChar("GREE", delai,GAUCHE,appel2);

				if(HAL_GPIO_ReadPin(ROT_A_2_GPIO_Port, ROT_A_2_Pin)!=lastState)
				{
					if(lastState==HAL_GPIO_ReadPin(ROT_B_2_GPIO_Port, ROT_B_2_Pin))
					{
						if (duty_cycleg2<=0)
							duty_cycleg2=0;
						else
							duty_cycleg2--;
					}
					else
					{
						if (duty_cycleg2>=100)
							duty_cycleg2=100; //122
						else
							duty_cycleg2++;
					}
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_cycleg2);
				}
				lastState=HAL_GPIO_ReadPin(ROT_A_2_GPIO_Port, ROT_A_2_Pin);
				lastBstate2=HAL_GPIO_ReadPin(ROT_B_2_GPIO_Port, ROT_B_2_Pin);

				if(!SW2 && !longpress2 && !shortpress2)
				{
					flag=1;
					sauv=HAL_GetTick();
					while(SW2==0 && flag==1)
					{
						time=HAL_GetTick();
						if(time-sauv>30)
						{
							shortpress2=TRUE;
							longpress2=FALSE;
							if(time-sauv>1000)
							{
								shortpress2=FALSE;
								longpress2=TRUE;
								flag=0;
							}
						}
						if((time-sauv)<600 && (time-sauv)>200)
							affichageChar(" RAZ", delai,GAUCHE,appel1);
						else
							affichageDig((time-sauv), delai,GAUCHE,100,0,appel1);
						SW2=HAL_GPIO_ReadPin(ROT_SW_2_GPIO_Port, ROT_SW_2_Pin);
					}
				}
				if(shortpress2 && SW2==1)
				{
					HAL_Delay(DelaiNextState);
					longpress2=FALSE;
					shortpress2=FALSE;
					etat2=red2;
				}
				if(longpress2 && SW2==1)
				{
					duty_cycleg2=0;duty_cycler2=0;duty_cycleb2=0;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_cycleg2);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty_cycleb2);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty_cycler2);
					longpress2=FALSE;
					shortpress2=FALSE;
					etat2=green2;
				}
				break;
			case red2:
				compt++;
				if(compt>1000)
					affichageDig(duty_cycler2, delai,GAUCHE,100,0,appel2);
				if(compt>2000)
					compt=0;
				if(compt<1000)
					affichageChar(" RED", delai,GAUCHE,appel2);

				if(HAL_GPIO_ReadPin(ROT_A_2_GPIO_Port, ROT_A_2_Pin)!=lastState)
				{
					if(lastState==HAL_GPIO_ReadPin(ROT_B_2_GPIO_Port, ROT_B_2_Pin))
					{
						if (duty_cycler2<=0)
							duty_cycler2=0;
						else
							duty_cycler2--;
					}
					else
					{
						if (duty_cycler2>=100)
							duty_cycler2=100; //77
						else
							duty_cycler2++;
					}
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty_cycler2);
				}
				lastState=HAL_GPIO_ReadPin(ROT_A_2_GPIO_Port, ROT_A_2_Pin);
				lastBstate2=HAL_GPIO_ReadPin(ROT_B_2_GPIO_Port, ROT_B_2_Pin);

				if(!SW2 && !longpress2 && !shortpress2)
				{
					flag=1;
					sauv=HAL_GetTick();
					while(SW2==0 && flag==1)
					{
						time=HAL_GetTick();
						if(time-sauv>30)
						{
							shortpress2=TRUE;
							longpress2=FALSE;
							if(time-sauv>1000)
							{
								shortpress2=FALSE;
								longpress2=TRUE;
								flag=0;
							}
						}
						if((time-sauv)<600 && (time-sauv)>200)
							affichageChar(" RAZ", delai,GAUCHE,appel1);
						else
							affichageDig((time-sauv), delai,GAUCHE,100,0,appel1);
						SW2=HAL_GPIO_ReadPin(ROT_SW_2_GPIO_Port, ROT_SW_2_Pin);
					}
				}
				if(shortpress2 && SW2)
				{
					HAL_Delay(DelaiNextState);
					longpress2=FALSE;
					shortpress2=FALSE;
					etat2=blue2;
				}
				if(longpress2 && SW2)
				{
					duty_cycleg2=0;duty_cycler2=0;duty_cycleb2=0;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_cycleg2);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty_cycleb2);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty_cycler2);
					longpress2=FALSE;
					shortpress2=FALSE;
					etat2=red2;
				}
				break;

			case blue2:
				compt++;
				if(compt>1000)
					affichageDig(duty_cycleb2, delai,GAUCHE,100,0,appel2);
				if(compt>2000)
					compt=0;
				if(compt<1000)
					affichageChar("BLUE", delai,GAUCHE,appel2);

				if(HAL_GPIO_ReadPin(ROT_A_2_GPIO_Port, ROT_A_2_Pin)!=lastState)
				{
					if(lastState==HAL_GPIO_ReadPin(ROT_B_2_GPIO_Port, ROT_B_2_Pin))
					{
						if (duty_cycleb2<=0)
							duty_cycleb2=0;
						else
							duty_cycleb2--;
					}
					else
					{
						if (duty_cycleb2>=100)
							duty_cycleb2=100; //138
						else
							duty_cycleb2++;
					}
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty_cycleb2);
				}
				lastState=HAL_GPIO_ReadPin(ROT_A_2_GPIO_Port, ROT_A_2_Pin);
				lastBstate2=HAL_GPIO_ReadPin(ROT_B_2_GPIO_Port, ROT_B_2_Pin);

				if(!SW2 && !longpress2 && !shortpress2)
				{
					flag=1;
					sauv=HAL_GetTick();
					while(!SW2 && flag)
					{
						time=HAL_GetTick();
						if(time-sauv>30)
						{
							shortpress2=TRUE;
							longpress2=FALSE;
							if(time-sauv>1000)
							{
								shortpress2=FALSE;
								longpress2=TRUE;
								flag=0;
							}
						}
						if((time-sauv)<600 && (time-sauv)>200)
							affichageChar(" RAZ", delai,GAUCHE,appel1);
						else
							affichageDig((time-sauv), delai,GAUCHE,100,0,appel1);
						SW2=HAL_GPIO_ReadPin(ROT_SW_2_GPIO_Port, ROT_SW_2_Pin);
					}
				}
				if(shortpress2 && SW2)
				{
					HAL_Delay(DelaiNextState);
					longpress2=FALSE;
					shortpress2=FALSE;
					etat2=alpha;
				}
				if(longpress2 && SW2)
				{
					duty_cycleg2=0;duty_cycler2=0;duty_cycleb2=0;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_cycleg2);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty_cycleb2);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty_cycler2);
					longpress2=FALSE;
					shortpress2=FALSE;
					etat2=blue2;
				}
				break;
			case alpha:
				compt++;
				if(compt>1000)
					affichageDig(alphaa,delai,GAUCHE,1000,1,appel2);
				if(compt>2000)
					compt=0;
				if(compt<1000)
					affichageChar("DUTY", delai,GAUCHE,appel2);

				if(HAL_GPIO_ReadPin(ROT_A_2_GPIO_Port, ROT_A_2_Pin)!=lastState)
				{
					if(lastState==HAL_GPIO_ReadPin(ROT_B_2_GPIO_Port, ROT_B_2_Pin))
					{
						if (alphaa<=0)
							alphaa=0;
						else
							alphaa--;
					}
					else
					{
						if (alphaa>=100)
							alphaa=100;
						else
							alphaa++;
					}
				}
				lastState=HAL_GPIO_ReadPin(ROT_A_2_GPIO_Port, ROT_A_2_Pin);
				lastBstate2=HAL_GPIO_ReadPin(ROT_B_2_GPIO_Port, ROT_B_2_Pin);

				comptsw++;
				if(comptsw>5*alphaa)
					HAL_GPIO_WritePin(GPIOC, PC3_Pin|PC4_Pin|PC5_Pin, GPIO_PIN_SET);
				if(comptsw>=500)
					comptsw=0;
				if(comptsw<5*alphaa)
					HAL_GPIO_WritePin(GPIOC, PC3_Pin|PC4_Pin|PC5_Pin, GPIO_PIN_RESET);

				if(!SW2 && !longpress2 && !shortpress2)
				{
					flag=1;
					sauv=HAL_GetTick();
					while(SW2==0 && flag==1)
					{
						time=HAL_GetTick();
						if(time-sauv>30)
						{
							shortpress2=TRUE;
							longpress2=FALSE;
							if(time-sauv>1000)
							{
								shortpress2=FALSE;
								longpress2=TRUE;
								flag=0;
							}
						}
						if((time-sauv)<600 && (time-sauv)>200)
							affichageChar(" RAZ", delai,GAUCHE,appel1);
						else
							affichageDig((time-sauv), delai,GAUCHE,100,0,appel1);
						SW2=HAL_GPIO_ReadPin(ROT_SW_2_GPIO_Port, ROT_SW_2_Pin);
					}
				}
				if(shortpress2 && SW2==1)
				{
					HAL_Delay(DelaiNextState);
					longpress2=FALSE;
					shortpress2=FALSE;
					etat2=freq;
				}
				if(longpress2 && SW2==1)
				{
					alphaa=0;
					longpress2=FALSE;
					shortpress2=FALSE;
					etat2=alpha;
				}
				break;
			case freq:
				lastState=HAL_GPIO_ReadPin(ROT_A_2_GPIO_Port, ROT_A_2_Pin);
				compt++;
				if(compt>1000)
				{
					if (frequency>1000)
						affichageChar(" 10k", delai, GAUCHE,appel2);
					else
						affichageDig(frequency,delai,GAUCHE,1000,1,appel2);
				}
				if(compt>2000)
					compt=0;
				if(compt<1000)
					affichageChar("FREQ", delai,GAUCHE,appel2);
				//Rotary Switch for frequency
				comptsw++;
				if(comptsw>10*alphaa/frequency)
					HAL_GPIO_WritePin(GPIOC, PC3_Pin|PC4_Pin|PC5_Pin, GPIO_PIN_SET);
				if(comptsw>=1000/frequency)
					comptsw=0;
				if(comptsw<10*alphaa/frequency)
					HAL_GPIO_WritePin(GPIOC, PC3_Pin|PC4_Pin|PC5_Pin, GPIO_PIN_RESET);
				if(HAL_GPIO_ReadPin(ROT_A_2_GPIO_Port, ROT_A_2_Pin)!=lastState)
				{
					if(lastState==HAL_GPIO_ReadPin(ROT_B_2_GPIO_Port, ROT_B_2_Pin))
					{
							frequency/=10;
							if (frequency<=1)
								frequency=1;
							/*autoreload=32000000/((TIM2->PSC)*(frequency));
							compare=duty_cycleb2*autoreload/100;
							__HAL_TIM_SET_AUTORELOAD(&htim2,autoreload);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, compare);
							compare=duty_cycleg2*autoreload/100;
							__HAL_TIM_SET_AUTORELOAD(&htim2,autoreload);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, compare);
							compare=duty_cycler2*autoreload/100;
							__HAL_TIM_SET_AUTORELOAD(&htim2,autoreload);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, compare);*/
					}
					else
					{
							frequency*=10;

							/*autoreload=32000000/((TIM2->PSC)*(frequency));
							compare=duty_cycleb2*autoreload/100;
							__HAL_TIM_SET_AUTORELOAD(&htim2,autoreload);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, compare);
							compare=duty_cycleg2*autoreload/100;
							__HAL_TIM_SET_AUTORELOAD(&htim2,autoreload);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, compare);
							compare=duty_cycler2*autoreload/100;
							__HAL_TIM_SET_AUTORELOAD(&htim2,autoreload);
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, compare);*/

							if (frequency>100)
								frequency=1000;

					}
					lastState=HAL_GPIO_ReadPin(ROT_A_2_GPIO_Port, ROT_A_2_Pin);
					lastBstate2=HAL_GPIO_ReadPin(ROT_B_2_GPIO_Port, ROT_B_2_Pin);
				}
				if(!SW2 && !longpress2 && !shortpress2)
				{
					flag=1;
					sauv=HAL_GetTick();
					while(SW2==0 && flag==1)
					{
						time=HAL_GetTick();
						if(time-sauv>30)
						{
							shortpress2=TRUE;
							longpress2=FALSE;
							if(time-sauv>1000)
							{
								shortpress2=FALSE;
								longpress2=TRUE;
								flag=0;
							}
						}
						if((time-sauv)<600 && (time-sauv)>200)
							affichageChar(" RAZ", delai,GAUCHE,appel1);
						else
							affichageDig((time-sauv), delai,GAUCHE,100,0,appel1);
						SW2=HAL_GPIO_ReadPin(ROT_SW_2_GPIO_Port, ROT_SW_2_Pin);
					}
				}
				if(shortpress2 && SW2==1)
				{
					HAL_Delay(DelaiNextState);
					longpress2=FALSE;
					shortpress2=FALSE;
					etat2=green2;
				}
				if(longpress2 && SW2==1)
				{
					duty_cycleg2=0;duty_cycler2=0;duty_cycleb2=0;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_cycleg2);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty_cycleb2);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_cycler2);
					longpress2=FALSE;
					shortpress2=FALSE;
					etat2=freq;
				}
			break;
			}

		}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 600;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PC3_Pin|PC4_Pin|PC5_Pin|SEG_B_Pin
                          |SEG_C_Pin|SEG_D_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIGIT8_Pin|DIGIT7_Pin|DIGIT6_Pin|DIGIT5_Pin
                          |SEG_F_Pin|SEG_G_Pin|SEG_DP_Pin|DIGIT4_Pin
                          |DIGIT3_Pin|DIGIT2_Pin|DIGIT1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ROT_A_2_Pin ROT_B_2_Pin ROT_SW_2_Pin */
  GPIO_InitStruct.Pin = ROT_A_2_Pin|ROT_B_2_Pin|ROT_SW_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3_Pin PC4_Pin PC5_Pin */
  GPIO_InitStruct.Pin = PC3_Pin|PC4_Pin|PC5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ROT_SW_1_Pin ROT_B_1_Pin ROT_A_1_Pin */
  GPIO_InitStruct.Pin = ROT_SW_1_Pin|ROT_B_1_Pin|ROT_A_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGIT8_Pin DIGIT7_Pin DIGIT6_Pin DIGIT5_Pin
                           SEG_F_Pin SEG_G_Pin SEG_DP_Pin DIGIT4_Pin
                           DIGIT3_Pin DIGIT2_Pin DIGIT1_Pin */
  GPIO_InitStruct.Pin = DIGIT8_Pin|DIGIT7_Pin|DIGIT6_Pin|DIGIT5_Pin
                          |SEG_F_Pin|SEG_G_Pin|SEG_DP_Pin|DIGIT4_Pin
                          |DIGIT3_Pin|DIGIT2_Pin|DIGIT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SEG_A_Pin */
  GPIO_InitStruct.Pin = SEG_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SEG_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_B_Pin SEG_C_Pin SEG_D_Pin */
  GPIO_InitStruct.Pin = SEG_B_Pin|SEG_C_Pin|SEG_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SEG_E_Pin */
  GPIO_InitStruct.Pin = SEG_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SEG_E_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

