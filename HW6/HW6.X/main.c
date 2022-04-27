#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro


#include <stdio.h>
#include "i2c_master_noint.h" 

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


//UART functions
void readUART1(char * string, int maxLength);
void writeUART1(const char * string);


void blinkled();

unsigned char writeadd=  0b01000000 ; //write
unsigned char readadd= 0b01000001 ; //read 
unsigned char address=  0b01000000 ; 
//unsigned char address=  0b0100000 ; 
unsigned char IODIR = 0x00 ; 
unsigned char GPIO = 0x09; 
unsigned char OLAT = 0x0A ;

    //IODIR = 00h 
    //GPIO = 09h

void mcpWrite (unsigned char add , unsigned char reg , unsigned char val) ;
unsigned char mcpRead( unsigned char add, unsigned char reg ) ; 

int main() {

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
     
    //UART Specific stuff 
    U1RXRbits.U1RXR = 0b0001; // Set B6 to U1RX
    RPB7Rbits.RPB7R = 0b0001; // Set B7 to U1TX

    
    //UART CODE from NU32
    // turn on UART3 without an interrupt
    U1MODEbits.BRGH = 0; // set baud to NU32_DESIRED_BAUD
    U1BRG = ((48000000 / 230400) / 16) - 1;

    // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
    U1MODEbits.PDSEL = 0;
    U1MODEbits.STSEL = 0;

    // configure TX & RX pins as output & input pins
    U1STAbits.UTXEN = 1;
    U1STAbits.URXEN = 1;
    

    // enable the uart
    U1MODEbits.ON = 1;
    
    char m[100]; //array
    
   //end uart specific stuff  
     
    i2c_master_setup();  
    
    mcpWrite(address, IODIR , 0b01111111 ) ;
    mcpWrite(address , OLAT , 0b10000000) ;
     
     
     __builtin_enable_interrupts();
     
     unsigned char state ; 
     unsigned char st; // returned received value from read function
    
     while (1) {
         //turn on GP7 
         //delay 
         //turn off GP7 
         st = mcpRead(address,GPIO );  // returned received value from read function
         state = st & 1  ;
         if (state ==0) {
             mcpWrite(address, OLAT , 0b010000000 ) ; //turn on led
         }
         else {
             mcpWrite(address, OLAT , 0b000000000 ) ; //turn off 
         }
         
         
         LATAINV = 0b10000; 
         _CP0_SET_COUNT(0);
                while (_CP0_GET_COUNT()< (12*1000000) ){   
                }
     }
     
    
     
           
    
    
  
}


void mcpWrite (unsigned char add , unsigned char reg , unsigned char val){
    i2c_master_start();
    i2c_master_send(add);
    i2c_master_send(reg);
    i2c_master_send(val);
    i2c_master_stop();
   
//for write 
// start bit
//address write 
// send data register 
//send data value 
//stop bit

    
}


unsigned char mcpRead( unsigned char add, unsigned char reg ){
    i2c_master_start();
    i2c_master_send(add); //write address 
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send(readadd); //read address 
    unsigned char r = i2c_master_recv(); 
    i2c_master_ack(1);
    i2c_master_stop();
    return r ; 
}

//blinkled 

void blinkled(){
    char m[100]; //array

    while (1) {
        // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
        // remember the core timer runs at half the sysclk
        
        int count = 4;
        
        if (PORTBbits.RB4 == 0) {
            int light = 1;
            while (count > 0) {
                if (light) {
                    LATAbits.LATA4 = 1; 
                } else {
                    LATAbits.LATA4 = 0; 
                }
                _CP0_SET_COUNT(0);
                while (_CP0_GET_COUNT()< (12*1000000) ){   
                }
                if (light) {
                    light = 0;
                } else {
                    light = 1;
                }
                count--;
            } 
            
        }
        else {
            LATAbits.LATA4 = 0 ;
        }
    }
}



// uart functions 
void readUART1(char * message, int maxLength) {
  char data = 0;
  int complete = 0, num_bytes = 0;
  // loop until you get a '\r' or '\n'
  while (!complete) {
    if (U1STAbits.URXDA) { // if data is available
      data = U1RXREG;      // read the data
      if ((data == '\n') || (data == '\r')) {
        complete = 1;
      } else {
        message[num_bytes] = data;
        ++num_bytes;
        // roll over if the array is too small
        if (num_bytes >= maxLength) {
          num_bytes = 0;
        }
      }
    }
  }
  // end the string
  message[num_bytes] = '\0';
}

// Write a character array using UART1
void writeUART1(const char * string) {
  while (*string != '\0') {
    while (U1STAbits.UTXBF) {
      ; // wait until tx buffer isn't full
    }
    U1TXREG = *string;
    ++string;
  }
}