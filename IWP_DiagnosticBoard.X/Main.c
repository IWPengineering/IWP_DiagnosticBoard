/*
 * File:   Main.c
 * Author: SandraSnozzi NicholasSum DrFish
 *
 * Created on November 27, 2017, 3:52 PM
 */

/// PIC18LF27K40 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config FEXTOSC = OFF    // External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_1MHZ// Power-up default value for COSC bits (HFINTOSC with HFFRQ = 4 MHz and CDIV = 4:1)

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // Master Clear Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (Power up timer disabled)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG2H
//#pragma config BORV = VBOR_190  // Brown Out Reset Voltage selection bits (Brown-out Reset Voltage (VBOR) set to 1.90V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG4H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config SCANE = ON       // Scanner Enable bit (Scanner module is available for use, SCANMD bit can control the module)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
#pragma config CP = OFF         // UserNVM Program Memory Code Protection bit (UserNVM code protection disabled)
#pragma config CPD = OFF        // DataNVM Memory Code Protection bit (DataNVM code protection disabled)

// CONFIG5H

// CONFIG6L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG6H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <pic18lf27k40.h>
//////#include <pic18.h>
#include "Pin_Manager.h"

//int __attribute__ ((space(eedata))) eeData; // Global variable located in EEPROM

// ****************************************************
//            Constants
// *****************************************************

    const int xAxis = 6; // analog pin connected to x axis of accelerometer
    const int yAxis = 7; // analog pin connected to y axis of accelerometer
    const int signedNumAdjustADC = 511; // Used to divide the total range of the output of the 10 bit ADC into positive and negative range.
    const int upstrokeInterval = 10; // The number of milliseconds to delay before reading the upstroke
    const float PI = 3.141592;
    const float angleThresholdSmall = 0.1; //number close to zero to determine if handle is moving w/o interpreting accelerometer noise as movement.
    const float handleMovementThreshold = 5.0; // When the handle has moved this many degrees from rest, we start calculating priming 
    float angle1 = 0;
    float angle2 = 0;
    float angle3 = 0;
    float angle4 = 0;
    float angle5 = 0;
    float angle6 = 0;
    float angle7 = 0;
    float angle8 = 0;
    float angle9 = 0;
    float angle10 = 0;
    const int pulseWidthThreshold = 20; 
    union EEData {
        int I_data;
        short long L_data;
        float F_data;
    };
    union EEData EEPromData; 
// ****************************************************************************
// *** Global Variables *******************************************************
// ****************************************************************************
    int waterPresenceSensorPin = 5;
    
// *****************************************************
//              Function Prototype
// *****************************************************
int readAdc(int pin);
void initAdc(void);
void initialization(void);
float getHandleAngle();
int readWaterSensor(void);
void delayMs(int ms);
char BcdToDec(char val);
void initSPI(void);
//void writeSPI(char, int, int);
void writeSPI(unsigned int output_data, unsigned short long EEaddress);
void writeEEPromFloat(float output_data, unsigned short long EEaddress);
char readSPI(unsigned int dummy, unsigned short long EEaddress);
float readEEPromFloat(unsigned short long EEaddress);
//int EEProm_Read_Int(int addr);
//void EEProm_Read_Float(int ee_addr, void *obj_p);
//void ClearWatchDogTimer(void);
//void EEProm_Write_Float(unsigned int ee_addr, void *obj_p);

/*********************************************************************
 * Function: initialization()
 * Input: None
 * Output: None
 * Overview: configures chip to work in our system (when power is turned on, these are set once)
 * Note: Pic Dependent
 * TestDate: 06-03-14
 ********************************************************************/

void initialization(void) {
    
    ////------------Sets up all ports as digital inputs-----------------------
    //IO port control
    ANSELA = 0; // Make PORTA digital I/O
    TRISA = 0xFFFF; // Make PORTA all inputs
    ANSELB = 0; // All port B pins are digital. Individual ADC are set in the readADC function
    TRISB = 0xFFFF; // Sets all of port B to input
    ANSELC = 0;
    TRISC = 0xFFFF; 

    //TMR 0 configurations
    T0CON1bits.T0CS = 3;
    T0CON0bits.T016BIT = 1;  // 16 bits 
    T0CON1bits.T0CKPS = 8; //Prescaler 1:256, Since Fosc is 4Mhz this means we clock at 15.624khz
    T0CON0bits.T0EN = 1;    // Turns TMR0 ON
  
    
    // Timer control
    // For WPS
    TMR1CLKbits.CS = 3;  // Selects the timer source as the High Frequency Internal Oscillator (4Mhz)
    T1CONbits.CKPS = 3;  // Prescales the clock input by 1:8 so we are now at 500khz timer clock (a lot faster)
    T1CONbits.RD16 = 1;  // Enables 16bit read/write of TMR1 (NOT USED IN IWP FOR WPS)
    T1CONbits.ON = 1;    // Turns TMR1 ON
    T1GCONbits.GE = 0; 
    
    // For getHandleAngle()
    TMR3CLKbits.CS = 3; // Timer2 Timer Clock Source Selection bits-High Frequency Internal Oscillator (4Mhz)
    T3CONbits.RD16 = 1; // Enables register read/write of Timer in one 16-bit operation 
    T3CONbits.CKPS = 3; // 1:8 Prescaler 500kHz timer clock
    T3CONbits.ON = 1; // Turns on Timer3
    
    initAdc();  //Initialize ADC
    initSPI();  //Initialize SPI communication with the serial EEPROM
    
//    UART config
//    U1MODE = 0x8000;  
/****    
    U1MODEbits.BRGH = 0;  // Use the standard BRG speed
    U1BRG = 25;           // set baud to 9600, assumes FCY=4Mhz (FNOSC = FRC)
    U1MODEbits.PDSEL = 0; // 8 bit data, no parity
    U1MODEbits.STSEL = 0; // 1 stop bit
    
                           // The Tx and Rx PINS are enabled as the default 
    U1STA = 0;            // clear Status and Control Register 
    U1STAbits.UTXEN = 0;  // disable transmit
                          // no need to enable receive.  The default is that a 
                          // receive interrupt will be generated when any character
                          // is received and transferred to the receive buffer
    U1STAbits.URXISEL =0; // generate an interrupt each time a character is received
    IFS0bits.U1RXIF = 0;  // clear the Rx interrupt flag
    _U1RXIF = 0;  
   // IEC0bits.U1RXIE = 1;  // enable Rx interrupts
   // _U1RXIE = 1;
    U1MODEbits.UARTEN = 1; // Turn on the UART
    ReceiveTextMsg[0] = 0;  // Start with an empty string
    ReceiveTextMsgFlag = 0;
           

     
    pinDirectionIO(18, 1); // Explicitly say RX Control Board is an input 
    pinDirectionIO(17, 1); // Explicitly say TX Control Board is an input 
    //H2O sensor configurations
//    pinDirectionIO(waterPresenceSensorOnOffPin, 0); //makes water presence sensor pin an output.
//    digitalPinSet(waterPresenceSensorOnOffPin, 0); //turns off the water presence sensor.  
    
            
    angle2 = getHandleAngle(); // Deduce handle movement 
    angle3 = getHandleAngle();
    angle4 = getHandleAngle();
    angle5 = getHandleAngle();
    angle6 = getHandleAngle();
    angle7 = getHandleAngle();
    angle8 = getHandleAngle();
    angle9 = getHandleAngle();
    angle10 = getHandleAngle();
 ****/
    
    
}

/*********************************************************************
 * Function: void initSPI(void)
 * Input: None
 * Output: None
 * Overview: configures the SPI interface to the EEPROM
 * Note: Pic Dependent
 * TestDate: not tested
 ********************************************************************/
void initSPI(void){
    TRISCbits.TRISC5 = 0; // make Master Data Out an output (Slave in))
    TRISCbits.TRISC3 = 0; // make SCLK an output
    TRISCbits.TRISC0 = 0; // make CS output
    //SSP1STAT = default values 
    
    SSP1CON1bits.CKP = 0; //Clock idle is low
    SSP1STATbits.CKE = 1; //Clock select bit is high
    SSP1CON1bits.SSPM = 0; //SCLK = Fosc/4; (250khz)
    SSP1CON3bits.BOEN = 0; //Don't overwrite the data buffer with new stuff if its not been read yet
    SSP1DATPPS = 20; // This is Master Data input to C4
    RC3PPS = 15; // This is data clock to C3
    RC5PPS = 16; // This is data output to C5
    SSP1CON1bits.SSPEN = 1;  // Enable SPI
    PORTCbits.RC0 = 1;
    
    
    
    
}
/*********************************************************************
 * Function: writeEEPromFloat(float output_data, long EEaddress)
 * Input: output_data = float value (3 bytes) to be saved
 *        EEaddress = address in EEPROM to write to (Max = 1FFD)
 *                    NOTE:  The float will be written to three sequential 
 *                           addresses starting with EEaddress
 *        
 * Output: None
 * Overview: This function calls writeSPI three times to save a float to 
 *           the external EEPROM
 * Note: Pic Dependent
 * TestDate: Not tested
 ********************************************************************/
void writeEEPromFloat(float output_data, unsigned short long EEaddress){

    EEPromData.F_data = output_data;
    unsigned int Data2 = (EEPromData.L_data >> 16)& 0xff; // Data MSB
    unsigned int Data1 = (EEPromData.L_data >> 8)& 0xff;
    unsigned int Data0 = EEPromData.L_data & 0xff;        // Data LSB

    // Write to external EEPROM
    writeSPI(Data0, EEaddress); // LSB
    writeSPI(Data1, EEaddress + 1); 
    writeSPI(Data2, EEaddress + 2); //MSB
}
/*********************************************************************
 * Function: float readEEPromFloat(long EEaddress)
 * Input: EEaddress = address in EEPROM to read (Max = 1FFD)
 *                    NOTE:  The float will be read from three sequential 
 *                           addresses starting with EEaddress
 *        
 * Output: float reconstructed from the three bytes saved in EEPROM
 * Overview: This function calls readSPI three times to read a float from 
 *           the external EEPROM
 * Note: Pic Dependent
 * TestDate: Not tested
 ********************************************************************/
float readEEPromFloat(unsigned short long EEaddress){
    char Data2 = 0;// Data MSB
    char Data1 = 0;
    char Data0 = 0;
    
    // Write to external EEPROM
    Data0 = readSPI(0, EEaddress); // LSB
    Data1 = readSPI(0, EEaddress + 1); 
    Data2 = readSPI(0, EEaddress + 2);
    
    EEPromData.L_data = Data2;   
    EEPromData.L_data = EEPromData.L_data << 16; 
    EEPromData.L_data = EEPromData.L_data | (Data1 << 8);
    EEPromData.L_data = EEPromData.L_data | Data0;
   
    return EEPromData.F_data; //Return the information as a float
}

/*********************************************************************
 * Function: void writeSPI(int output_data, long EEaddress)
 * Input: output_data = byte to be sent
 *        EEaddress = address in EEPROM to write to (Max = 1FFF)
 *        
 * Output: None
 * Overview: This function first makes sure the EEPROM is enabled for a Write
 *           and that it is finished writing whatever may have been sent earlier
 *           Then sends the address and data to the EEPROM to perform the write.
 *           A single byte write takes about 600usec.  However, if you call the 
 *           function again right away, it will take longer since the EEPROM 
 *           will still be in the process of writing
 * ** Programming Note:  For some reason the PIC messed up the LSByte of the long
 *                       address if the output_data was a char rather than an int
 * Note: Pic Dependent
 * TestDate: 10/24/2018 RKF
 ********************************************************************/
void writeSPI(unsigned int output_data, unsigned short long EEaddress){
    unsigned int local_data = 0;
    char AdrsMsb = 0;
    char AdrsMid = 0;
    char AdrsLsb = 0;
    
    // Parse the address into three bytes
    AdrsMsb = (EEaddress >> 16)& 0xff;
    AdrsMid = (EEaddress >> 8)& 0xff;
    AdrsLsb = EEaddress & 0xff;   
    
    PORTCbits.RC0 = 1; 
    SSP1CON1bits.WCOL = 0; //Clear collision bit in case there was a previous problem
    PORTCbits.RC0 = 0; 
    
    SSP1BUF = 6;  // send Write enable command
    while(SSP1STATbits.BF == 0){  } //wait for the command to be sent
    PORTCbits.RC0 = 1;
    local_data = SSP1BUF; // we don't care about this, just getting ready for the next write
    
    // See if the EEPROM is ready for another data byte
    PORTCbits.RC0 = 0; 
    SSP1BUF = 5;  // read status register command
    while(SSP1STATbits.BF == 0){  } //wait for the command to be sent
    local_data = SSP1BUF;
    local_data = 1;
    while (local_data == 1) { // Keep reading the contents of the Status Register until it says it is ready for a new write
        SSP1BUF = 0;  // garbage
        while(SSP1STATbits.BF == 0){  } //wait for the command to be sent
        local_data = SSP1BUF;
        local_data = local_data & 1; //The LSB is LOW when the EEPROM is ready for more data
    }
    PORTCbits.RC0 = 1;  
    // Here is where we actually do the WRITING (Address MSB-Middle-LSB, DATA, )
    PORTCbits.RC0 = 0;
    
    SSP1BUF = 2;  // Write Command
    while(SSP1STATbits.BF == 0){  } //wait for the command to be sent
    local_data = SSP1BUF;
  
    SSP1BUF = AdrsMsb;  // Msb address
    while(SSP1STATbits.BF == 0){  } //wait for the command to be sent
    local_data = SSP1BUF;

    SSP1BUF = AdrsMid;  // middle address
    while(SSP1STATbits.BF == 0){  } //wait for the command to be sent
    local_data = SSP1BUF;
    
    SSP1BUF = AdrsLsb;  // lsb address
    while(SSP1STATbits.BF == 0){  } //wait for the command to be sent
    local_data = SSP1BUF;
    
    SSP1BUF = output_data;  // data
    while(SSP1STATbits.BF == 0){  } //wait for the command to be sent
    local_data = SSP1BUF;
    
    PORTCbits.RC0 = 1;  // CS high  
}

/*********************************************************************
 * Function: char readSPI(int dummy, long EEaddress)
 * Input: dummy = For some reason the LSByte of the address is read as
 *                0x48 if you don't have an int sent to the function first.
 *                Just send any number it is not used
 *        EEaddress = address you want to read (max is 1FFF)   
 * Output: Byte read from the specified EEPROM address
 * Overview: This function reads a single byte from the external EEPROM 
 *           connected with the SPI interface and returns it.
 * ** Programming Note:  For some reason the PIC messed up the LSByte of the long
 *                       address if the output_data was a char rather than an int
 * Note: Pic Dependent
 * TestDate: 10/24/2018 RKF
 ********************************************************************/
char readSPI(unsigned int dummy, unsigned short long EEaddress){
    char local_data = 0;
    char AdrsMsb = 0;
    char AdrsMid = 0;
    char AdrsLsb = 0;
    
    // Parse the address into three bytes
    AdrsMsb = (EEaddress >> 16)& 0xff;
    AdrsMid = (EEaddress >> 8)& 0xff;
    AdrsLsb = EEaddress & 0xff;  
    
    PORTCbits.RC0 = 1; 
    SSP1CON1bits.WCOL = 0; //Clear collision bit in case there was a previous problem
    
    // Here is where we actually read the data (Address MSB-Middle-LSB, data )
    PORTCbits.RC0 = 0; 
    
    SSP1BUF = 3;  // send READ command
    while(SSP1STATbits.BF == 0){  } //wait for the command to be sent
    local_data = SSP1BUF; // we don't care about this, just getting ready for the next write
  
    SSP1BUF = AdrsMsb;  // Msb address
    while(SSP1STATbits.BF == 0){  } //wait for the command to be sent
    local_data = SSP1BUF;

    SSP1BUF = AdrsMid;  // middle address
    while(SSP1STATbits.BF == 0){  } //wait for the command to be sent
    local_data = SSP1BUF;
    
    SSP1BUF = AdrsLsb;  // lsb address
    while(SSP1STATbits.BF == 0){  } //wait for the command to be sent
    local_data = SSP1BUF;
    
    SSP1BUF = 00;  // garbage byte so we can read the response
    while(SSP1STATbits.BF == 0){  } //wait for the command to be sent
    PORTCbits.RC0 = 1;  // CS high
    
    local_data = SSP1BUF;
    return local_data;
        
    
}
    
/*********************************************************************
 * Function: delayMs()
 * Input: milliseconds
 * Output: None
 * Overview: Delays the specified number of milliseconds. Handles
 *           any amount of time, more accurate calculation. 
 * Note: Assumes that TMR1 is clocked by 500khz
 *       
 * TestDate: 4/4/2018
 ********************************************************************/
void delayMs(int ms) { // Actually using the timer
    int debug = 0;
    int end_count;
    
    while(ms > 4000) {
        TMR0 = 0;
        while(TMR0 < 62500) {
            ms = ms - 4000;
        }
    }
    
    end_count = ms * 15.625;
    TMR0 = 0;
    while(TMR0<end_count){
        debug = TMR0;
    }
    //T1CONbits.ON = 1;
    
    /******
    while(ms > 60){ 
        TMR3 = 0;
        while(TMR3 < 30000){} //wait 60ms
        ms = ms - 60;
    }
    // now we fit within the timer's range
    end_count = ms*500; // assumes TC1 is clocked by 500khz
    TMR3 = 0;
    //while(TMR1 < 5) {
    while(TMR3<end_count){
        debug = TMR3;
    }
     *****/
    
}


// * Function: EEProm_Write_Float(unsigned int ee_addr, void *obj_p)
// * Input: ee_addr - the location to write to relative to the start of EEPROM
// *                  it is assumed that you are referring to the # of the float 
// *                  you want to write.  The first is 0, the next is 1 etc.
// *        *obj_p - the address of the variable which contains the float
// *                  to be written to EEPROM
// * Output: none
// * Overview: A single float is written to EEPROM start + offset.  This is done by
// *           writing the contents of the float address provided, one int at a time
// * Note: Library
// * TestDate: 12-26-2016
// ********************************************************************/
//void EEProm_Write_Float(unsigned int ee_addr, void *obj_p) {
//
//        unsigned int *p = obj_p;
//        unsigned int offset;
//        NVMCON = 0x4004;
//        ee_addr = ee_addr*4;  // floats use 4 address locations
//
//        // Write the first half of the float
//         // Set up a pointer to the EEPROM location to be erased
//        TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
//        offset = __builtin_tbloffset(&eeData) + (ee_addr & 0x01ff); // Initialize lower word of address
//        __builtin_tblwtl(offset, *p); // Write EEPROM data to write latch
//         asm volatile ("disi #5"); // Disable Interrupts For 5 Instructions
//        __builtin_write_NVM(); // Issue Unlock Sequence & Start Write Cycle
//        while(NVMCONbits.WR==1); // Optional: Poll WR bit to wait for
//        // first half of float write sequence to complete
//
//        // Write the second half of the float
//        p++;
//        ee_addr = ee_addr + 2;
//        TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
//        offset = __builtin_tbloffset(&eeData) + (ee_addr & 0x01ff); // Initialize lower word of address
//        __builtin_tblwtl(offset, *p); // Write EEPROM data to write latch
//         asm volatile ("disi #5"); // Disable Interrupts For 5 Instructions
//        __builtin_write_NVM(); // Issue Unlock Sequence & Start Write Cycle
//        while(NVMCONbits.WR==1); // Optional: Poll WR bit to wait for
//        // second half of float write sequence to complete
//    
//}
//
//
///*********************************************************************
// * Function: int EEProm_Read_Int(int addr);
// * Input: addr - the location to read from relative to the start of EEPROM
// * Output: int value read from EEPROM
// * Overview: A single int is read from EEPROM start + offset and is returned
// * Note: Library
// * TestDate: 12-26-2016
// ********************************************************************/
//int EEProm_Read_Int(int addr){
//    int data; // Data read from EEPROM
//    unsigned int offset;
// 
//    // Set up a pointer to the EEPROM location to be erased
//    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
//    offset = __builtin_tbloffset(&eeData) + (2* addr & 0x01ff); // Initialize lower word of address
//    data = __builtin_tblrdl(offset); // Write EEPROM data to write latch
//    return data;
//}
//
///*********************************************************************
// * Function: EEProm_Read_Float(unsigned int ee_addr, void *obj_p)
// * Input: ee_addr - the location to read from relative to the start of EEPROM
// *        *obj_p - the address of the variable to be updated (assumed to be a float)
// * Output: none
// * Overview: A single float is read from EEPROM start + offset.  This is done by
// *           updating the contents of the float address provided, one int at a time
// * Note: Library
// * TestDate: 12-26-2016
// ********************************************************************/
//void EEProm_Read_Float(int ee_addr, void *obj_p){
//    unsigned int *p = obj_p; // point to the float to be updated
//    unsigned int offset; 
//    ee_addr == ee_addr*4; // floats use 4 address locations
//    // Read and update the first half of the float
//    // Set up a pointer to the EEPROM location to be erased
//    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
//    offset = __builtin_tbloffset(&eeData) + (ee_addr & 0x01ff); // Initialize lower word of address 
//    *p = __builtin_tblrdl(offset); // Write EEPROM data to write latch 
//    // First half read is complete
//
//    p++; 
//    ee_addr = ee_addr+2; 
//
//    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
//    offset = __builtin_tbloffset(&eeData) + (ee_Addr & 0x01ff); // Initialize lower word of address 
//    *p = __builtin_tblrdl(offset); // Write EEPROM data to write latch
//    // second half read is complete
//}
//

/*********************************************************************
 * Function: readWaterSensor
 * Input: None
 * Output: 1 if water is present and 0 if it is not
 * Overview: The output of the 555 times is monitored to see what its
 *           output frequency is.  When there is no water, the frequency
 *           is less than 400hz.  If the frequency is greater than this
 *           we assume water is present.  We only measure the duration of
 *           the high part of the pulse. * 
 *           This routine assumes that Timer1 is clocked at 15.625khz
 * Note: Pic Dependent
 * TestDate: changed and not tested as of 1-31-2018
 ********************************************************************/
int readWaterSensor(void) // RA3 is one water sensor
{
    int WaterPresent = 0;  //assume there is no water
   // turn WPS on and off in the Main loop 
    delayMs(5);  //make sure the 555 has had time to turn on 
    //make sure you start at the beginning of the positive pulse
    TMR0 = 0;
    if (digitalPinStatus(waterPresenceSensorPin) == 1) {
        while ((digitalPinStatus(waterPresenceSensorPin))&&(TMR0 <= pulseWidthThreshold)) { //quit if the line is high for too long
        }; 
    }
    //wait for rising edge
    TMR0 = 0;
    while ((digitalPinStatus(waterPresenceSensorPin) == 0)&&(TMR0 <= pulseWidthThreshold)) { //quit if the line is low for too long
    }; 
    //Now measure the high part of the signal
    TMR0 = 0;
    while ((digitalPinStatus(waterPresenceSensorPin))&&(TMR0 <= pulseWidthThreshold)) { //quit if the line is high for too long
    };
    int varToSee = digitalPinStatus(waterPresenceSensorPin); // For debugging purposes 
    if(TMR0 <= pulseWidthThreshold){
        WaterPresent = 1;
    }
    return WaterPresent;
}

//This function converts a BCD to DEC
//Input: BCD Value
//Returns: Hex Value

char BcdToDec(char val) {
    return ((val / 16 * 10) + (val % 16));
}

    /* global variables. Might change*/
int depthSensorPin = 2;
int Pin4 = 4;
int rxPin = 6;
int xAxisAccelerometerPin = 24;
int yAxisAccelerometerPin = 23;
int batteryLevelPin = 11;
int TimeSinceLastHourCheck = 0;  // we check this when we have gone around the no pumping loop enough times that 1 minute has gone by
int hour = 0; // Hour of day


/*********************************************************************
 * Function: initAdc()
 * Input: None
 * Output: None
 * Overview: Initializes Analog to Digital Converter
 * Note: Pic Dependent
 * TestDate: NA
 ********************************************************************/
void initAdc(void) {
    
    ADCON0bits.ADON = 0; // Ensure the ADC is disabled before configuration
    
    TRISAbits.TRISA4 = 1; //Make this an input
    ANSELAbits.ANSELA4 = 1; //Makes RA4(pin 6) analog  X-axis
    TRISAbits.TRISA5 = 1; //Make this an input
    ANSELAbits.ANSELA5 = 1;  //Makes RA5(pin 7) analog  Y-axis
    
    //Choose reference voltages
    ADREFbits.ADPREF = 0;  //Vref+ = Vdd
    ADREFbits.ADNREF = 0;  //Vref- = GND
    
    //Choose clock source  (1 conversion = 24us)
    ADCON0bits.ADCS = 0;  //Using internal F_osc 
    ADCLK = 3;  //ADC Clock frequency = FOSC/(2*(n+1)) = ADC clock is 500khz (TAD = 2us)

    //The interrupt flag is ADIF bit in the PIRx
    
    //Result format
    ADCON0bits.ADFM = 1;  //Right Justified - low bits are in ADRESL and upper 2 bits will be in ADREFH
   
}
 
    
int readAdc(int pin) //check with accelerometer
{ 
    switch (pin) {
        
        case 6:
            ADPCHbits.ADPCH = 4; //connect pin 6 to A/D converter
          
            //ANSBbits.ANSB13 = 1; // AN11 is analog
            //TRISBbits.TRISB13 = 1; // AN11 is an input
            //AD1CHSbits.CH0SA = 11; //Connect AN11 as the S/H input (sample and hold)
            break;
        case 7:
            ADPCHbits.ADPCH = 5; //connect pin 7 to A/D converter
        
            //PORTBbits.RB12 = 1; // AN12 is analog ***I changed this to ANSBbits.ANSBxx 03-31-2015
            //TRISBbits.TRISB12 = 1; // AN12 is an input
            //AD1CHSbits.CH0SA = 12; // Connect AN12 as the S/H input
            break;
    
    }
    
    ADCON0bits.ADON = 1; // Turn on ADC   
    //ADCON0bits.ADCONT = 1;   // Enables continuous sampling
    ADCON0bits.ADGO = 1;
    while (ADCON0bits.ADGO) { //ADGO = 0 means that conversion is finished
    }
    unsigned int adcValue = ADRES;  //From ADC result register
    
    ADCON0bits.ADON = 0; // Turn off ADC   
    return adcValue;
}


///*********************************************************************
// * Function: getHandleAngle()
// * Input: None
// * Output: Float
// * Overview: Returns the average angle of the pump. The accelerometer
//should be oriented on the pump handle so that when the
//pump handle (the side the user is using) is down (water
//present), the angle is positive. When the pump handle
//(the side the user is using) is up (no water present),
//the angle is negative.Gets a snapshot of the current sensor values.
// * Note: Library
// * NOTE2: It turns out that averaging the handle angles would be the most accurate way to report pumping
// * TestDate: TBD
// ********************************************************************/
float getHandleAngle() {

    signed int xValue = readAdc(xAxis) - signedNumAdjustADC; 
    signed int yValue = readAdc(yAxis) - signedNumAdjustADC; 
    float angle = atan2(yValue, xValue) * (180 / PI); //returns angle in degrees 06-20-2014
    // Calculate and return the angle of the pump handle
    if (angle > 20) {
        angle = 20.0;
    } else if (angle < -30) {
        angle = -30.0;
    }
    angle10 = angle9;
    angle9 = angle8;
    angle8 = angle7;
    angle7 = angle6;
    angle6 = angle5;
    angle5 = angle4;
    angle4 = angle3;
    angle3 = angle2;
    angle2 = angle1;
    angle1 = angle;

    float averageAngle = (angle1 + angle2 + angle3 + angle4 + angle5 + angle6 + angle7 + angle8 + angle9 + angle10) / 10.0;

    return averageAngle;
}

void ClearWatchDogTimer(void){
     ClrWdt();
}

void main(void) {
    initialization();
    
    // This is just to Debug the SPI Communication with the EEPROM
    char EEPROMdata = 0;
    
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 0;
    
    while(1){
        float EEPromFloatData = 0;
        
        writeSPI(0, 0x100);
        writeSPI(0xaa, 0x101);
        writeSPI(0x55, 0x102);
        writeSPI(0x11, 0x103);
        writeSPI(0x33, 0x104);
   
        EEPROMdata = 0x70;   //0111 0000        
 // Now let's try to read things back
        EEPROMdata = readSPI(0, 0x103);
        EEPROMdata = readSPI(0, 0x103);
        EEPROMdata = readSPI(0, 0x101);
        EEPROMdata = readSPI(0, 0x100);
        EEPROMdata = readSPI(0, 0x102);
        EEPROMdata = readSPI(0, 0x104);
// Check the Float Functions
        writeEEPromFloat(12.345, 0x1000);
        writeEEPromFloat(12.34567, 0x1200);
        writeEEPromFloat(1234.56, 0x1100);
        
        EEPromFloatData = readEEPromFloat(0x1200);
        EEPromFloatData = readEEPromFloat(0x1000);
        EEPromFloatData = readEEPromFloat(0x1100);
        
    }
    
    // This is just to Debug the SPI Communication with the EEPROM
    
    float adcVoltage = getHandleAngle();
    
 
    //int __attribute__ ((space(eedata))) eeData; // Global variable located in EEPROM
    // Initialize
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 0;
    // Clear any data
    int handleMovement = 0; // Either 1 or no 0 if the handle moving upward
    float angleCurrent = 0; // Stores the current angle of the pump handle
	float anglePrevious = 0; // Stores the last recorded angle of the pump handle
	float angleDelta = 0; // Stores the difference between the current and previous angles
    int readWater = 0; // Was there water? 
    // Noon message section 
    
    // Diagnostic message section 
    
    // Are we pumping?
    PORTBbits.RB3 = 0;
    PORTBbits.RB4 = 0;
    while (1)
	{   

        anglePrevious = getHandleAngle();                            // Get the angle of the pump handle to measure against.  
                                                                     // This is our reference when looking for sufficient movement to say the handle is actually moving.  
                                                                     // the "moving" threshold is defined by handleMovementThreshold in IWPUtilities
        
        handleMovement = 0;                                          // Set the handle movement to 0 (handle is not moving)
		while (handleMovement == 0)
		{ 
            ClearWatchDogTimer();     // We stay in this loop if no one is pumping so we need to clear the WDT  
            TimeSinceLastHourCheck++;
            if(TimeSinceLastHourCheck > 5000){ // If no one is pumping this works out to be about every minute
                //hour = BcdToDec(getHourI2C());    //I2C stuff
                TimeSinceLastHourCheck = 0;
            }// Set the handle movement to 0 (handle is not moving) 
            delayMs(upstrokeInterval);                            // Delay for a short time
			float newAngle = getHandleAngle();
			float deltaAngle = newAngle - anglePrevious;        
			if(deltaAngle < 0) {
				deltaAngle *= -1;
			}

            if(deltaAngle > handleMovementThreshold){            // The total movement of the handle from rest has been exceeded
				handleMovement = 1;  
                PORTBbits.RB3 = 1;
                if (readWater == 0) {
                    PORTBbits.RB4 = 0;
                }  
            }else{
                  PORTBbits.RB3 = 0;
                  if (readWater == 0) {
                    PORTBbits.RB4 = 0;
                   }
                  //handleMovement = 1; 
                  
            } 
        }
    
        // Do we have water? 
        if (readWaterSensor()) {
            readWater = 1; 
            PORTBbits.RB4 = 1;
        }
        else {
            readWater = 0; 
            PORTBbits.RB4 = 0; // Test if pin RB3 works 
        }
    }
    
    return;
}


