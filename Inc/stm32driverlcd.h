/*--------------------------------------------------------*/
//	stm32driverlcd.h
/*--------------------------------------------------------*/
//	Description :		Driver pour LCD sur carte ARM (STM32) 17400
//   Code basé sur driver LCD pour carte PIC32
//
//  Cible :
//   Modèles LCD NHD-C0220AA-FSW-FTW et NHD-C0220BA-FSW-FTW-3V3
//   Carte 17400 (versions A-B-C-D) KitArmM0
//
//  Versions :
//	 no version/versions outils/auteur/date/description
//   -v1 / Keil 5.24 / SCA / 12.2020 
//	  Version d'origine (kits v A-B-C avec NHD-C0220AA-FSW-FTW
//   -v2 / Keil 5.33, compilateur C 5.06 / SCA / 10.2022
//    Adaptation pour kit version D avec LCD NHD-C0220BA-FSW-FTW-3V3
//    (l'ancien LCD est devenu obsolète)
//    La version du kit (et donc le modèle de LCD) est reconnue
//		à l'init du LCD.
//   -v2.01 / Keil 5.33, compilateur C 5.06 / SCA / 02.2023
//    Ajout activation clk GPIO C et D dans l'init du lcd
//    Sinon, si on ne configure pas graphiquement les GPIO
//		dans STM32CubeMX, les GPIO ne sont pas utilisables
//
/*--------------------------------------------------------*/

#ifndef __STM32DRIVERLCD_H
#define __STM32DRIVERLCD_H

#include <stdarg.h>
#include <stdint.h>


/*--------------------------------------------------------*/
// Définition des fonctions prototypes
/*--------------------------------------------------------*/
//uint8_t lcd_read_byte( void );
//void lcd_send_nibble( uint8_t n );
//void lcd_send_byte( uint8_t address, uint8_t n );
void lcd_init(void);
void lcd_gotoxy( uint8_t x, uint8_t y);
void lcd_putc( uint8_t c);
void lcd_put_string_ram( char *ptr_char );
//void lcd_put_string_rom( const char *ptr_char );
//char lcd_getc( uint8_t x, uint8_t y);
void lcd_bl_on( void ); 
void lcd_bl_off( void );
void printf_lcd( const char *format,  ...);
void lcd_clearLine(uint8_t lineNr);
void lcd_clearScreen(void);

#endif
