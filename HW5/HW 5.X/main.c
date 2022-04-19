// initialize SPI1
void initSPI() {
    // Pin B14 has to be SCK1
    
    // Turn of analog pins
    ANSELA =0 ; 
    //...
    // Make an output pin for CS
    //cs is slave select?
    TRISAbits.TRISA0 = 0; 
    LATAbiits.LATA0 = 1; 
    //...
    //...
    // Set SDO1
    RPAA1Rbits.RPA1R = 0b0011;
    //miso?
   
    //...
    // Set SDI1
    SDII1Rbits.SDI1R=0b0001 ; 
    //mosi?
    //...

    // setup SPI1
    SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 500; // 1000 for 24kHz, 1 for 12MHz; // baud rate to 10 MHz [SPI1BRG = (48000000/(2*desired))-1]
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