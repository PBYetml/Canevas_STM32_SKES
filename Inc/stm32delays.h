/*--------------------------------------------------------*/
//	stm32delays.h
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

#ifndef __STM32DELAYS_H
#define __STM32DELAYS_H

#include "stm32f0xx.h"	//nécessaire pour __NOP


/*--------------------------------------------------------*/
// Définition des fonctions prototypes
/*--------------------------------------------------------*/

//attention : le nop ne garantit pas un délai en architecture ARM
// (dû au pipeline du CPU)
// son utilité est plutôt du padding mémoire pour aligner le code.
#define delay_cycle() __nop()	

void delay500ns(void); // nécessaire pour lcd
void delay_us(uint32_t); // 32 bits
void delay_ms(uint32_t); // 32 bits

#endif
