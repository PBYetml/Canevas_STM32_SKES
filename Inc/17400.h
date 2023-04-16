/*--------------------------------------------------------*/
//	17400.h
/*--------------------------------------------------------*/
//	Description :		defines généraux pour carte 17400
//
//  Cible :
//   Carte 17400C KitArmM0
//
//	Auteurs / date : 	SCA / 18.12.2020
//
//	Version		:	V1.0
//	Compilateur	:	Keil 5.33
//
// Revu / modifié: 
// -
/*--------------------------------------------------------*/

#ifndef __17400_H
#define __17400_H


/*--------------------------------------------------------*/
// Définition des signaux
/*--------------------------------------------------------*/

//4 boutons
#define BTN0_PORT GPIOC
#define BTN0_BIT GPIO_PIN_0
#define BTN1_PORT GPIOC
#define BTN1_BIT GPIO_PIN_1
#define BTN2_PORT GPIOC
#define BTN2_BIT GPIO_PIN_2
#define BTN3_PORT GPIOC
#define BTN3_BIT GPIO_PIN_3

//4 leds
#define LED0_PORT GPIOC
#define LED0_BIT GPIO_PIN_4
#define LED1_PORT GPIOC
#define LED1_BIT GPIO_PIN_5
#define LED2_PORT GPIOC
#define LED2_BIT GPIO_PIN_6
#define LED3_PORT GPIOC
#define LED3_BIT GPIO_PIN_7

#endif //__17400_H
