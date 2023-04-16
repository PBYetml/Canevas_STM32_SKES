/*--------------------------------------------------------*/
//	stm32delays.c
/*--------------------------------------------------------*/
//	Description :	Fonctions de delay afin
//			de pouvoir utiliser la même syntaxe
//			que sous CCS.
//  
//  Versions :
//	 no version/versions outils/auteur/date/description
//   -v1 / Keil 5.24 / SCA / 12.2020 
//	  Version d'origine
//   -v2 /Keil 5.33, compilateur C 5.06 / SCA / 10.2022
//     Ajustement des délais (us et ms)
//     Les valeurs minimales n'étaient pas garanties.
//
/*--------------------------------------------------------*/

#include "stm32delays.h"


#define SYSTICK_FREQ 48000000

//réglage optimisation 0 pour les fonctions de ce fichier
// voir https://developer.arm.com/documentation/ka003013/latest
#pragma push /* Save existing optimization level */
#pragma O0   /* Optimization level now O0 */
	
/*--------------------------------------------------------*/
// Fonction delay500ns
// Approximative, mais testé >= 500ns
// Testé sur STM32F051R8T7 @ 48 MHz :
//  691 ns optimisation 3
//  628 ns optimisation 0 
/*--------------------------------------------------------*/
__attribute__((noinline, section(".ramfunc")))
void delay500ns(void)
{
	delay_cycle();
}

/*--------------------------------------------------------*/
// Fonction delay_us
// Attention, la valeur passée en param doit être < 1000
//  sinon, utiliser fonction delay_ms()
// Testé sur STM32F051R8T7 @ 48 MHz :
// environ 100 à 300 ns de surplus, optimisation 0 ou 3.
/*--------------------------------------------------------*/
//Relevé de durées sans soustraction overhead:
//2,32 us au lieu de 1 us opt. 0 / min 2,19 us opt. 3
//3,46 us au lieu de 2 us opt.0 / min 3,35 us opt. 3
//11,3 us au lieu de 10 us opt.0 / min 11 us opt. 3
//102 us au lieu de 100 us opt. 0 / min 101 us opt. 3
#define DELAY_US_TICK_OVERHEAD 48 

__attribute__((noinline, section(".ramfunc")))
void delay_us(uint32_t us)
{
	uint32_t startVal, endVal, currentVal;
	uint32_t nTick;
	
	startVal = SysTick->VAL;	// Où en est-on lors de l'entrée dans la fonction ?
	nTick = (SYSTICK_FREQ/1000000)*us + 1;	//nbre de ticks que systick doit compter
	nTick -= DELAY_US_TICK_OVERHEAD;
	
	if (startVal >= nTick)	//SysTick décompte normalement ?
	{
		endVal = startVal-nTick;
		do{
			currentVal = SysTick->VAL;
		}
		while ((currentVal <= startVal) && (currentVal > endVal) ); //(original)
		//on n'a pas fait le tour ET valeur finale pas atteinte
	}
	else	//SysTick se recycle durant le delay
	{
		endVal = (startVal + SysTick->LOAD + 1) - nTick;
		do{
			currentVal = SysTick->VAL;
		}
		while ((currentVal <= startVal) || (currentVal > endVal));	//(original)
		//on n'a pas fait le tour OU valeur finale pas atteinte
	}		
}

/*--------------------------------------------------------*/
// Fonction delay_ms
// Basé sur Systick timer. L'attente d'au moins le nb de ms
//  est garantie. Il peut y avoir un surplus de 0.9999 etc ms.
// Délai testé sur STM32F051R8T7 @ 48 MHz, en optimisation 0 et 3
/*--------------------------------------------------------*/
__attribute__((noinline, section(".ramfunc")))
void delay_ms(uint32_t ms)
{
	//Attention lire doc si appel HAL_Delay depuis interruption
	if (ms>0)
		HAL_Delay(ms);	//fonction HAL basée par défaut sur le SysTick timer
}

 #pragma pop /* Restore original optimization level */
