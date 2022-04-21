#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "SPI.h"

// #include <proc/p32mx170f256b.h>

#include <stdio.h>
#include <math.h>

#define pi  3.14159265

// from uart code
// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = FRCPLL // use fast frc oscillator with pll
#pragma config FSOSCEN = OFF // disable secondary oscillator
#pragma config IESO = OFF // disable switching clocks
#pragma config POSCMOD = OFF // primary osc disabled
#pragma config OSCIOFNC = OFF // disable clock output
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // disable clock switch and FSCM
#pragma config WDTPS = PS1048576 // use largest wdt value
#pragma config WINDIS = OFF // use non-window mode wdt
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz fast rc internal oscillator
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations

// end from uart code




void initSPI() ;
unsigned char spi_io(unsigned char o) ;


int main(){
   
    __builtin_disable_interrupts(); // disable interrupts while initializing things

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB4 = 1;
    LATAbits.LATA4 = 0;
   
   
    initSPI() ;
   
    __builtin_enable_interrupts() ;
 
       
    unsigned short v = 0;
    unsigned short v2 = 0;//voltage
    unsigned char c = 0; //channel
    unsigned char c2 = 0;
    unsigned short p = 0;
    unsigned short p2 = 0;
   
    double wave1[100];
    double wave2[100];
   
    // 8 bit 0 to 256
   
    int i = 0  ;
   

        //make waves
        //y=Asin(2?fx+B)
       
        for (i=0 ; i <100 ; i++ ) {
            wave1[i]= 127*sin(2*pi*2*i*.01) + 127 ; //sine wave
           
           
           
           //triangle wave
           
            if (i<50){
               wave2[i]= 5.1*i ;  
            }
            else {
               wave2[i]=255 - (i* 5.1) + 255 ;  
            }
         
           
        }



   
    while (1) {
       
        
       
       
       
        for (i=0 ; i <100 ; i++ ) {
           
       
        // sine wave
        c = 0 ; 
        v = wave1[i];
        p = c <<15 ;
        p = p | (0b111 << 12) ;
        p = p | (v<<4);
       
        //triangle wave
        c2 = 1;
        v2 = wave2[i];
        p2 = c2 <<15 ;
        p2 = p2 | (0b111 << 12) ;
        p2 = p2 | (v2<<4);
       
        //sine wave
        LATAbits.LATA0 = 0 ; // CS low
        spi_io(p >> 8) ;
        spi_io(p) ;
        LATAbits.LATA0 = 1 ; //CS high
       
        //triangle wave
        LATAbits.LATA0 = 0 ; // CS low
        spi_io(p2 >> 8) ;
        spi_io(p2) ;
        LATAbits.LATA0 = 1 ; //CS high
       
    _CP0_SET_COUNT(0);
   
    //while (_CP0_GET_COUNT()< (12*1000000) ){  
    while (_CP0_GET_COUNT()< (24*1000000)* 0.01) {   //1Hz

                }
       
    }
    }
   
   
}


// initialize SPI1
void initSPI() {
    // Pin B14 has to be SCK1
   
    // Turn of analog pins
    ANSELA =0 ;
    //...
    // Make an output pin for CS
    //cs is slave select?
    TRISAbits.TRISA0 = 0;
    LATAbits.LATA0 = 1;
    //...
    //...
    // Set SDO1
    RPA1Rbits.RPA1R = 0b0011;
    //miso?
   
    //...
    // Set SDI1
    SDI1Rbits.SDI1R= 0b0001 ;
    //mosi?
    //...

    // setup SPI1
    SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 1000; // 1000 for 24kHz, 1 for 12MHz; // baud rate to 10 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0; // clear the overflow bit
    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1; // master operation
    SPI1CONbits.ON = 1; // turn on spi
}


// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}










