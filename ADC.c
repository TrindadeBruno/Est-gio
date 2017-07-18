/* 
 * File:   Main.c
 * Author: 1dTracker
 *
 * Created on 13 July 2017, 15:34
 */

#include <libpic30.h>
#include <p30F4011.h>
#include <math.h>
#include <limits.h>
#include <uart.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define FCY 29491200L // 30MPI / 4
#define FOSC (FCY*4)
#define PI 3.14159265358979323846

// Configuration settings
#pragma config FCKSMEN=CSW_FSCM_OFF
#pragma config FOS=PRI               //fonte é o cristal
#pragma config FPR=XT_PLL16          //oscilador a 16x cristal
#pragma config WDT=WDT_OFF           //watchdog timer off
#pragma config MCLRE=MCLR_EN         //turn MCLR pin ON and
#pragma config FPWRT=PWRT_OFF

unsigned int data[4] = {0,0,0,0};
int a=0;
char str[11];
int Temperatura;
int Inclinacao;

void init_uart(void) {
    
   unsigned int UMODEvalue, U2STAvalue;  //auxiliary UART config variables (parâmetros de configuração da porta série)
   
   /*Serial port config*/ 
    
    UMODEvalue = UART_EN & UART_IDLE_CON & UART_NO_PAR_8BIT;
    U2STAvalue = UART_INT_TX & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_RX_TX;
    OpenUART2 (UMODEvalue, U2STAvalue, 15); 
    U2STAbits.URXISEL = 1;
    _U2RXIE = 1;
    U2MODEbits.LPBACK = 0;
    __C30_UART = 2;
 
    return;
 }


void init_ADC(void) {
    
    //ADCON1
    ADCON1bits.ADON = 0;
	ADCON1bits.ADSIDL = 0;	//Continue in iddle mode
	ADCON1bits.FORM = 0;	//Output format - integer
	ADCON1bits.SSRC = 7;	//Auto-convert
    ADCON1bits.SIMSAM = 0;   // Samples individually
	ADCON1bits.ASAM = 0;	//auto-start sample

	//ADCON2
	ADCON2bits.VCFG = 0;	//Reference - AVdd; AVss
	ADCON2bits.CSCNA = 0;	//Scan inputs
    ADCON2bits.CHPS = 0;    //Converts CH0
	ADCON2bits.SMPI = 0;	//interrupts at each 2 sample/convert sequence
	ADCON2bits.BUFM = 0;	//buffer 16-word buffer
	ADCON2bits.ALTS = 0;	//use mux A

	//ADCON3
	ADCON3bits.SAMC = 0b11111;	//Auto sample time = fastest possible with sequential sampling
	ADCON3bits.ADRC = 0;	//Conversion clock system
	ADCON3bits.ADCS = 0b111111;	//Conversion Clock select
	
	//INPUT
    ADCHSbits.CH0NA = 0;	// MUX A negative input in vref-
    ADCHSbits.CH0SA = 0;   // Channel 0 positive input is AN0      

	//Input Scan Select Register
	ADCSSL = 0; //não necessário
    
//    //Interrupt Configuration
//	IEC0bits.ADIE = 1;		//Enable ADC conversion complete interrupt
//	IFS0bits.ADIF = 0;		//Clear ADC conversion complete flag
//	IPC2bits.ADIP = 4;		//Priority for ADC interrupt

    TRISB = 0b000000000;
    ADPCFG = 0b111111111;
    
	ADCON1bits.ADON = 1;	//Enable ADC
	__delay32(FCY);	//ADC off-on stabilization delay
	
 
	return;
}

void init_Timer3(void)

{
    T3CON = 0;            // Clear Timer 3 configuration
    TMR3 = 0x0000; // Sets timer value to zero
    T3CONbits.TCKPS = 3;  // Set timer 3 prescaler (0=1:1, 1=1:8, 2=1:64, 3=1:256)
    PR3 = 58594;          // Set Timer 3 period (max value is 65535)
    _T3IP = 1;            // Set Timer 3 interrupt priority
    _T3IF = 0;            // Clear Timer 3 interrupt flag
    _T3IE = 1;            // Enable Timer 3 interrupt
    
    
    T3CONbits.TON = 1;    // Turn on Timer 3    
    

    
    return;
}

void ADC_Analogic(int analogic) {

    TRISB |= (1 << analogic);            // ADC_CHANNEL defined as input
    ADPCFG &= ~(1 << analogic);          // ADC_CHANNEL defined as analog
}



int Read_ADC (int analogic) {

    int aux;
    ADCHSbits.CH0SA = analogic;
    ADCON1bits.SAMP = 1;
    while (ADCON1bits.SAMP);
    while (!ADCON1bits.DONE);
    aux = ADCBUF0;
    return aux;
}


void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {

    a=1;
    IFS0bits.T3IF = 0; //Clears interrupt flag 
    return;
    }

int main (void)
{  
    
    init_uart();
    init_Timer3();
    init_ADC(); 
   
    ADC_Analogic(0);
    ADC_Analogic(1);
    
    while(1)        
    {
 
      if (a==1)
      {
            
       
        data[0]=Read_ADC(0);         
        data[1]=Read_ADC(1);
        
        Temperatura = (100/1023) * data[0];
        Temperatura = Temperatura - 273.15;
        
        Inclinacao = ((((305*PI)/180)/1023) * data[1]);   
        
        sprintf(str, " %d , %d", Temperatura , Inclinacao);
        
        a=0;
        printf ("\n\r%d, %d",data[0],data[1] );
        
         }    
    }
}

