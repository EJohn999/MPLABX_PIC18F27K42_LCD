/*
 * File:   main.c
 * Author: John
 *
 * Created on 17 de febrero de 2021, 12:40 PM
 */

#include <xc.h>
#include "LCD_PIC18F.h"
#include <stdio.h>
#define _XTAL_FREQ 8000000

// CONFIG1L
#pragma config FEXTOSC = HS     // External Oscillator Selection (HS (crystal oscillator) above 8 MHz; PFM set to high power)
#pragma config RSTOSC = EXTOSC  // Reset Oscillator Selection (EXTOSC operating per FEXTOSC bits (device manufacturing default))

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = OFF     // PRLOCKED One-Way Set Enable bit (PRLOCK bit can be set and cleared repeatedly)
#pragma config CSWEN = OFF      // Clock Switch Enable bit (The NOSC and NDIV bits cannot be changed by user software)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)

// CONFIG2L
#pragma config MCLRE = INTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin function is port defined function; If LVP =1, RE3 pin fuction is MCLR)
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON      // Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit (IVTLOCK bit can be cleared and set only once)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (ULPBOR disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF      // WDT operating mode (WDT enabled regardless of sleep)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG4H
#pragma config WRTB = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTC = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)


#define SYNC   LATCbits.LATC5
#define DOUT   LATCbits.LATC4
#define CLK   LATCbits.LATC3
#define LED   LATAbits.LATA1

#define P_MENOS   PORTAbits.RA5
#define P_OK PORTAbits.RA4
#define P_MAS   PORTAbits.RA3

unsigned int current=0;

void CONFIG_PIC(){
    OSCCON1=0b01110000; //EXTOSC y NDIV=1
    
    PORTA =0;LATA=0;ANSELA=0;TRISA=0B00111100;
    PORTB =0;LATB=0;ANSELB=0;TRISB=0;//LCD pines
    PORTC =0;LATC=0;ANSELC=0;TRISC=0;
}
void verificar_uA(){
    unsigned short vR=0;
    NOP();
    //<------
    //  leer ADC
    //  verificar si V/R =uA     V=(5*ADC)/4096 & R=8200 --> 
    //  V/R -> (5000000*ADC)/(4096 *8200);
    //
    // si es menor suma y si es mayor resta;
    //corrijo DAC y verifico --->
    
    
}

void LCD_mAdato(unsigned int uA){// Solo muestro el valor uA en LCD
    char buffer[4];
    lcd_clear();          //Limpiamos pantalla LCD
    lcd_gotoxy(1,2);      //Ubicamos cursor en fila 1, columna 2
    lcd_putc("Modificando..."); //mostramos una cadena de caracteres en la pantalla LCD
    lcd_gotoxy(2,6);      //Ubicamos cursor en fila 2, columna 1
    sprintf(buffer,"%d",uA); // actualizo dato en LCD
    lcd_putc(buffer);
    lcd_gotoxy(2,10); 
    lcd_putc("uA."); //mostramos una cadena de caracteres en la pantalla LCD
}
void LCD_new(unsigned int uA){// llega calor en uA, actualizo DAC, corrijo con ADC y muestro en LCD   
    
    char buffer[4];
    unsigned long DAC=0;
    // V= (DAC*5)/65536  R=8200  I=uA  despejo DAC y escalo Voltage debido a I->uA
    DAC=(8200*65536*uA)/5000000; // (R*MaxDAC*I)/V 
    
    //Actualizo valor via SPI(DAC);
    
    //verificar_uA();
    
    lcd_clear();          //Limpiamos pantalla LCD
    lcd_gotoxy(1,2);      //Ubicamos cursor en fila 1, columna 2
    lcd_putc("Actualizado OK"); //mostramos una cadena de caracteres en la pantalla LCD
    __delay_ms(1000);
    lcd_clear();
    lcd_gotoxy(1,2);      //Ubicamos cursor en fila 2, columna 1
    lcd_putc("  Corriente:"); //mostramos una cadena de caracteres en la pantalla LCD
    lcd_gotoxy(2,6);      //Ubicamos cursor en fila 2, columna 1
    sprintf(buffer,"%d",uA); // actualizo dato en LCD
    lcd_putc(buffer);
    lcd_gotoxy(2,10);      //Ubicamos cursor en fila 2, columna 1
    lcd_putc("uA."); //mostramos una cadena de caracteres en la pantalla LCD
}


void main()
{  

    CONFIG_PIC();
    lcd_init();
    __delay_ms(200);
    LCD_new(current);
    
    while(1){
        if(P_MENOS==0){
            __delay_ms(20);
            if(P_MENOS==1)// Verifico que no sea un falso
                break;
            while(P_MENOS==0){// Espero a que deje de precionarse
            current=current-10;
            __delay_ms(200);
        }
            if(current>500){ current=500;}// se desborada y es mayor a 0 cerca del maximo
            LCD_mAdato(current);
        }
        
        if(P_MAS==0){
            __delay_ms(20);
            if(P_MAS==1)// Verifico que no sea un falso
                break;
            while(P_MAS==0){// Espero a que deje de precionarse
            current=current+10; 
            __delay_ms(200);
            }
            if(current>500){ current=0;}
            LCD_mAdato(current);
        }
        
        if(P_OK==0){
            __delay_ms(20);
            if(P_OK==1)// Verifico que no sea un falso
                break;
            while(P_OK==0);// Espero a que deje de precionarse
            LCD_new(current);
        }
    }
    
    
    
}
 

/*
 * while(1){
        lcd_clear();          //Limpiamos pantalla LCD
        lcd_gotoxy(1,1);      //Ubicamos cursor en fila 2, columna 1
        lcd_putc("*PIC18F27K22**"); //mostramos una cadena de caracteres en la pantalla LCD
        lcd_gotoxy(2,1);      //Ubicamos cursor en fila 2, columna 1
        lcd_write_char('J');  //mostramos caracter 
        __delay_ms(500);      //Retardo de 500 ms
        lcd_write_char('O');  //Mostramos caracter
        __delay_ms(500);      //Retardo de 500 ms
        lcd_write_char('H');  //mostramos caracter 
        __delay_ms(500);      //Retardo de 500 ms
        lcd_write_char('N');  //Mostramos caracter
        __delay_ms(500);      //Retardo de 500 ms
        lcd_clear();          //Limpiamos pantalla LCD
        lcd_gotoxy(1,1);      //Ubicamos el cursor en fila 1, columna 1
        lcd_putc(buffer_lcd); //Mostramos el valor de buffer_lcd
        sprintf(buffer_lcd,"Float: %03.2f", decimal);//Cargamos variable decimal en buffer_lcd.
        lcd_gotoxy(1,1);      //Ubicamos el cursor en fila 1, columna 1
        lcd_putc(buffer_lcd); //Mostramos el valor de buffer_lcd
        sprintf(buffer_lcd,"Int: %d",entero); //Cargamos variable decimal en buffer_lcd.
        
        lcd_gotoxy(2,1);      //Ubicamos el cursor en fila 1 columna 1
        lcd_putc(buffer_lcd); //Mostramos el valor de buffer_lcd
        __delay_ms(2000);     //Retardo de 2s   
 }
 */