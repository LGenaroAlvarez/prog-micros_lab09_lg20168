/*
 * File:   lab09_main-20168.c
 * Author: luisg
 *
 * Created on April 25, 2022, 6:37 PM
 */
// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include<stdio.h>
#include <stdint.h>

//DEFINICION DE FRECUENCIA PARA DELAY
#define _XTAL_FREQ 500000

//DEFINICIONES GENERALES
#define POT_MIN 0
#define POT_MAX 255
#define PWM_MIN 31
#define PWM_MAX 63

//VARIABLES GLOBALES
unsigned short CCPR = 0;
unsigned short CCPR_b = 0;

//PROTO FUNCIONES
void setup(void);

unsigned short interpol(uint8_t val, uint8_t pot_min, uint8_t pot_max, 
        unsigned short pwm_min, unsigned short pwm_max);

//CONFIGURACION PRINCIPAL
void setup(void){
    ANSEL = 0b00000011;         // PORTA AN0 y AN1 COMO ANALOGICO, RESTO COMO DIGITALES
    ANSELH = 0;                 // DEMAS PUERTOS COMO DIGITALES
    
    TRISA = 0b00000011;         // PORTA AN0 y AN1 COMO ENTRADA, RESTO COMO SALIDA
    PORTA = 0;                  // LIMPIEZA DEL PORTA

    //OSCCONFIC
    OSCCONbits.IRCF = 0b0011;   // FRECUENCIA DE OSCILADOR INTERNO (500KHz)
    OSCCONbits.SCS  = 1;        // RELOJ INTERNO
    
    //ADC CONFIG
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    
    ADCON1bits.VCFG0 = 0;       // USO DE VDD COMO VOLTAJE DE REFERENCIA INTERNO
    ADCON1bits.VCFG1 = 1;       // USO DE VSS COMO VOLTAJE DE REFERENCIA INTERNO
    
    ADCON0bits.CHS = 0b0000;    // SELECCION DE PORTA PIN0 (AN0) COMO ENTRADA DE ADC
    ADCON1bits.ADFM = 0;        // FORMATO DE BITS JUSTIFICADOS A LA IZQUIERDA
    ADCON0bits.ADON = 1;        // HABILITACION DE MODULO DE ADC
    __delay_us(40);
    
    //PWM CONFIG
    TRISCbits.TRISC2 = 1;       // CCP1 COMO ENTRADA (SALIDA DESABILITADA)
    TRISCbits.TRISC1 = 1;       // CCP2 COMO ENTRADA (SALIDA DESABILITADA)
    PR2 = 156;                  // PERIODO DE TMR2 EN 2mS
    
    //CCP CONFIG
    CCP1CON = 0;
    CCP2CON = 0;
    CCP1CONbits.P1M = 0;        //
    CCP1CONbits.CCP1M = 0b1100; //
    CCP2CONbits.CCP2M = 0b1100; //
    
    CCPR1L = 125>>2;            //
    CCP1CONbits.DC1B = 125 & 0b11;  //
    CCPR2L = 125>>2;            //
    CCP2CONbits.DC2B0 = 125 & 0b01; //
    CCP2CONbits.DC2B1 = 125 & 0b10; //
    
    
    T2CONbits.T2CKPS = 0b11;
    PIR1bits.TMR2IF = 0;
    T2CONbits.TMR2ON = 1;
    while(!PIR1bits.TMR2IF);
    PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC1 = 0;
    
    //CONFIG DE INTERRUPCIONES
    INTCONbits.GIE = 1;         // HABILITAR INTERRUPCIONES GLOBALES
    INTCONbits.PEIE = 1;        // HABILITAR INTERRUPCIONES EN PERIFERICOS
    PIE1bits.ADIE = 1;          // HABILITAR INTERRUPCION DE ADC
    PIR1bits.ADIF = 0;          // LIMPIEZA DE BANDERA DE INTERRUPCION DE ADC
    return;
}
   
unsigned short interpol(uint8_t val, uint8_t pot_min, uint8_t pot_max, 
        unsigned short pwm_min, unsigned short pwm_max){
    
    return (unsigned short)(pwm_min+((float)(pwm_max-pwm_min)/(pot_max-pot_min))
            *(val-pot_min));
    
}

//INTERRUPCIONES
void __interrupt() isr(void){
    if (PIR1bits.ADIF){         // REVISAR INTERRUPCION DE ADC
        if (ADCON0bits.CHS == 0){   // REVISAR SI ESTA ACTIVADO AN0
            CCPR = interpol(ADRESH, POT_MIN, POT_MAX, PWM_MIN, PWM_MAX);
            CCPR1L = (uint8_t)(CCPR>>2);
            CCP1CONbits.DC1B = CCPR & 0b11;
        }
        
        else if (ADCON0bits.CHS == 1){
            CCPR_b = interpol(ADRESH, POT_MIN, POT_MAX, PWM_MIN, PWM_MAX);
            CCPR2L = (uint8_t)(CCPR_b>>2);
            CCP2CONbits.DC2B0 = 125 & 0b01;
            CCP2CONbits.DC2B1 = 125 & 0b10;
        }
        PIR1bits.ADIF = 0;          // LIMPIEZA DE BANDERA DE INTERRUPCION ADC
    }
    return;
}

void main(void) {
    //EJECUCION CONFIG
    setup();  
    
    while (1){
        if (ADCON0bits.GO == 0){
            if (ADCON0bits.CHS == 0){
                ADCON0bits.CHS = 0b0001;
            }
            else if (ADCON0bits.CHS == 1){
                ADCON0bits.CHS = 0b0000;
            }
            __delay_us(40);
            ADCON0bits.GO = 1;
        }
    }
    return;
}
