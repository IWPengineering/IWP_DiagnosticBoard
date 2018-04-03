/*
 * File:   Main.c
 * Author: SandraSnozzi NicholasSum DrFish
 *
 * Created on November 27, 2017, 3:52 PM
 */

/// PIC18LF26K40 Configuration Bit Settings

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
#include <p18LF26K40.h>
#include <pic18.h>
#include "Pin_Manager.h"

//int __attribute__ ((space(eedata))) eeData; // Global variable located in EEPROM

// ****************************************************
//            Constants
// *****************************************************

    const int xAxis = 6; // analog pin connected to x axis of accelerometer ** MAY NEED TO CHANGE
    const int yAxis = 7; // analog pin connected to y axis of accelerometer ** MAY NEED TO CHANGE
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
//void ClearWatchDogTimer(void);
//void EEProm_Write_Float(unsigned int ee_addr, void *obj_p);
float getHandleAngle();
//int EEProm_Read_Int(int addr);
//void EEProm_Read_Float(int ee_addr, void *obj_p);
int readWaterSensor(void);
void delayMs(int ms);
char BcdToDec(char val);
//
//
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

    //DEBUGGING PURPOSES TMR 0
    T0CON1bits.T0CS = 3;
    T0CON0bits.T016BIT = 1;  // 16 bits 
    T0CON1bits.T0CKPS = 8; //Prescaler 1:256
    T0CON0bits.T0EN = 1;    // Turns TMR1 ON
  
    
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
    
    // UART config
//    U1MODE = 0x8000;  
/*    U1MODEbits.BRGH = 0;  // Use the standard BRG speed
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

    initAdc();    
    
            
    angle2 = getHandleAngle(); // Deduce handle movement 
    angle3 = getHandleAngle();
    angle4 = getHandleAngle();
    angle5 = getHandleAngle();
    angle6 = getHandleAngle();
    angle7 = getHandleAngle();
    angle8 = getHandleAngle();
    angle9 = getHandleAngle();
    angle10 = getHandleAngle();
 */
    
    initAdc();   
}

    
/*********************************************************************
 * Function: delayMs()
 * Input: milliseconds
 * Output: None
 * Overview: Delays the specified number of milliseconds. Handles
 *           any amount of time, more accurate calculation. 
 * Note: Assumes that TMR1 is clocked by 500khz
 *       
 * TestDate: not tested
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
    ANSELAbits.ANSELA4 = 1;  //Makes RAr(pin 7) analog  Y-axis
    
    //Choose reference voltages
    ADREFbits.ADPREF = 0;  //Vref+ = Vdd
    ADREFbits.ADNREF = 0;  //Vref- = GND
    
    //Choose clock source  (1 conversion = 24us)
    ADCON0bits.ADCS = 0;  //Using internal F_osc 
    ADCLK = 0;  //ADC Clock frequency = FOSC/(2*(n+1))

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
    ADCON0bits.ADCONT = 1;   // Enables continuous sampling
    
//    while (ADCON0bits.ADGO) { //ADGO = 0 means that conversion is finished
//    }
    unsigned int adcValue = ADRES;  //From ADC result register
    return adcValue;
    ADCON0bits.ADON = 0; // Turn off ADC   
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
    
    float adcVoltage = getHandleAngle();
    
    
//        TRISBbits.TRISB4 = 0;
//    int counter = 0;
//    int n = 0;
//    while(n < 3) {
//    while(counter < 20000) {                      //Turn on led test
//        PORTBbits.RB4 = 1;
//        counter = counter + 1;
//        //delayMs(4000);                //Added counter instead of using delay which uses the clock. LED is turning on and off.
//    }
//    counter = 0;
//    while(counter < 20000){
//        PORTBbits.RB4 = 0;
//        counter = counter + 1;
//    }
//    n = n + 1;
//    }
//    
//    PORTBbits.RB4 = 0;

        //delayMs(2000);

//    TRISBbits.TRISB4 = 0;
//    int counter = 0;
//    while(counter < 3) {                      //Turn on led test
//        PORTBbits.RB4 = 1;
//        delayMs(8000);
//        PORTBbits.RB4 = 0;
//        delayMs(4000);
//        counter = counter + 1;
//    }
//    
//    PORTBbits.RB4 = 0;
    
 
    //int __attribute__ ((space(eedata))) eeData; // Global variable located in EEPROM
    // Initialize
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 0;
    // Clear any data
    int handleMovement = 0; // Either 1 or no 0 if the handle moving upward
    float angleCurrent = 0; // Stores the current angle of the pump handle
	float anglePrevious = 0; // Stores the last recorded angle of the pump handle
	float angleDelta = 0; // Stores the difference between the current and previous angles
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
            int sandy = 1; 
            delayMs(upstrokeInterval);                            // Delay for a short time
			float newAngle = getHandleAngle();
			float deltaAngle = newAngle - anglePrevious;        
			if(deltaAngle < 0) {
				deltaAngle *= -1;
			}

            if(deltaAngle > handleMovementThreshold){            // The total movement of the handle from rest has been exceeded
				handleMovement = 1;  
                PORTBbits.RB3 = 1;
                //PORTBbits.RB4 = 0;
            }else{
                  PORTBbits.RB3 = 0;
                  PORTBbits.RB4 = 0;
                  //handleMovement = 1; 
                  
			} 
        }
        //PORTBbits.RB4 = 1;
        // Do we have water? 
        //TRISBbits.TRISB3 = 0;
        if (readWaterSensor()) {
            PORTBbits.RB4 = 1;
        }
        else {
            PORTBbits.RB4 = 0; // Test if pin RB3 works 
        }
    }
     

    
//    ClearWatchDogTimer(); // Changed from ClearWatchDogTime() which one is correct? )
//    
//    int i = 0; 
//    
//    if((angleDelta > (-1 * angleThresholdSmall)) && (angleDelta < angleThresholdSmall)){   //Determines if the handle is at rest
//		i++; //increase i while handle is stationary
//	}
//	else {
//		i = 0;
//	}
//    
//    int leakCondition = 3;  // Assume that we are going to be able to calculate a valid leak rate
//      
//    if(!readWaterSensor()){  // If there is already no water when we get here, something strange is happening, don't calculate leak
//        leakCondition = 4;
//    }
//
//    if(never_primed == 1){
//        leakCondition = 4;   // there was never any water
//    }
//    i = 0;  
//    
//    anglePrevious = getHandleAngle(); // Keep track of how many milliseconds have passed
//    int volumeLoopCounter = 15; // 150 ms                           //number of zero movement cycles before loop ends
//    long leakDurationCounter = volumeLoopCounter;   // The volume loop has 150 milliseconds of delay 
//       // if no water or no handle movement before entry.
//    while (readWaterSensor()){
//       angleCurrent = getHandleAngle();                   //Get the current angle of the pump handle
//       angleDelta = angleCurrent - anglePrevious;    //Calculate the change in angle of the pump   handle
//       anglePrevious = angleCurrent;            // Update the previous angle for the next calculation
//											                                                                    
//       // intentional pump and break out of the loop (2 is in radians)
//       // If the handle starts moving we will abandon calculating a new leak rate
//       //  Moving is the same criterion as stopping in volume calculation loop
//       if ((angleDelta > (-1 * angleThresholdSmall)) && (angleDelta < angleThresholdSmall)){    
//        //Determines if the handle is at rest
//            i=0; //Handle Not moving
//       }
//       else{
//            i++; //Handle Moving
//       }             
//
//       if (i >= volumeLoopCounter){ //Has the handle been moving for 150ms?
//            leakCondition = 1;
//            break;
//       }
//
//      if (leakDurationCounter >= leakRateTimeOut){                              // (was 100 - 10/8/2015 KK)
//            leakCondition = 2; //Jump to condition for no leak if while is broken out of on condition of
//           //exceeding the leakRateTimeOut wait.
//            break;
//      }
//      delayMs(upstrokeInterval);
//      leakDurationCounter++;
//    }
//     
//    digitalPinSet(waterPresenceSensorOnOffPin, 0); //turns off the water presence sensor.
//        
//        
//    if (upStrokeExtract < 900){  // If someone has not pumped at least 10 liters we don't want to measure leak rate
//
//    // this is to prevent a splash from a slug of water hitting the WPS and being interpreted as leak
//    // when it is just receding back down the pipe when momentum goes away.
//    //  upStrokeExtractis in degrees at this point
//       leakCondition = 5;
//    }
//
//    switch (leakCondition){
//        case 1:
//            leakRate = leakRatePrevious; // They started pumping again so can't calculate a new leak rate, use the last one when calculating volume pumped
//            break;
//        case 2:                          // Waited the max time and water was still there so leak rate = 0
//            leakRate = 0;
//            leakRatePrevious = leakRate;  
//            break;
//        case 3:                         // The pump did prime but water leaked out in less than our max time to wait.  So calculate a new value
//            leakRate = leakSensorVolume / ((leakDurationCounter * upstrokeInterval) / 1000.0); // liters/sec
//            leakRatePrevious = leakRate;    
//            break;           
//		
//        case 4:
//            leakRate = leakRatePrevious;  // there was no water at the start of this so we can't calculate a new leak
////            leakRate = 0;  // there was never any water so we can't calculate a new leak rate, let previous value stay the previous value
//            break;
//        
//        case 5:
//            leakRate = leakRatePrevious; // They started pumping again so can't calculate a new leak rate, use the last one when calculating volume pumped
//            break;
//      }
//       if ((leakRate * 3600) > leakRateLong) {
//            leakRateLong = leakRate * 3600;                                              //reports in L/hr
//            EEProm_Write_Float(0,&leakRateLong);                                        // Save to EEProm
//       }
//
///******************************************* Prime time calc******************************/
//    int i = 0; 
//    timeOutStatus = 0;                               // prepares timeoutstatus for new event
//    anglePrevious = getHandleAngle();    // Get the angle of the pump handle to measure against
//    upStrokePrime = 0;
//    never_primed = 0;
//    hour = BcdToDec(getHourI2C()); //Update the time so we know where to save this pumping event
//    timeSinceLastHourCheck = 0;
//
//    digitalPinSet(waterPresenceSensorOnOffPin, 1); //turns on the water presence sensor.
//    while ((timeOutStatus < waterPrimeTimeOut) && !readWaterSensor()) {
//        angleCurrent = getHandleAngle();                      //gets the latest 10-average angle
//        angleDelta = angleCurrent - anglePrevious;            //determines the amount of handle movement from last reading
//        anglePrevious = angleCurrent;        //Prepares anglePrevious for the next loop
//        if(angleDelta > 0){                                   //Determines direction of handle movement
//            upStrokePrime += angleDelta;    //If the valve is moving upward, the movement is added to an
//                           //accumlation var (even if it was smaller than angleThresholdSmall)
//        }
//                // If they have stopped, pumping we should give up too
//        if((angleDelta > (-1 * angleThresholdSmall)) && (angleDelta < angleThresholdSmall)){   //Determines if the handle is at rest
//                    i++; // we want to stop if the user stops pumping              
//        }
//        else{
//                    i=0;   // they are still trying
//        } 
//        
//        if(i == 100) {  // They quit trying for at least 1 second
//            never_primed = 1;
//            break;
//        }
//        
//        timeOutStatus++; // we will wait for up to waterPrimeTimeOut of pumping
//        delayMs(upstrokeInterval); 
//    }
//    
//    if(timeOutStatus >= waterPrimeTimeOut){
//        never_primed = 1;          
//    }
//    
//    upStrokePrimeMeters = upStrokePrime * upstrokeToMeters;	      // Convert to meters
//    if (upStrokePrimeMeters > longestPrime){                      // Updates the longestPrime
//        longestPrime = upStrokePrimeMeters;
//        EEProm_Write_Float(1,&longestPrime);                      // Save to EEProm
//    }
//
///********************Vol calc************/
//    volumeEvent = (MKII * upStrokeExtract);     //[L/rad][rad]=[L] 
//        
//    volumeEvent -= (leakRate * ((extractionDurationCounter * upstrokeInterval) / 1000.0)); //[L/s][s]=[L]
//    if(volumeEvent < 0) {
//        volumeEvent = 0; // we can't pump negative volume
//    }
//
//       
//    switch (hour / 2) {
//		case 0:
//			volume02 = volume02 + volumeEvent;
//  			break;
//		case 1:
//			volume24 = volume24 + volumeEvent;
//			break;
//		case 2:
//			volume46 = volume46 + volumeEvent;
//			break;
//		case 3:
//			volume68 = volume68 + volumeEvent;
//			break;
//		case 4:
//			volume810 = volume810 + volumeEvent;
//			break;
//		case 5:
//			volume1012 = volume1012 + volumeEvent;
//			break;
//		case 6:
//			volume1214 = volume1214 + volumeEvent;
//			break;
//		case 7:
//			volume1416 = volume1416 + volumeEvent;
//			break;
//		case 8:
//			volume1618 = volume1618 + volumeEvent;
//			break;
//		case 9:
//			volume1820 = volume1820 + volumeEvent;
//			break;
//		case 10:
//			volume2022 = volume2022 + volumeEvent;
//			break;
//		case 11:
//			volume2224 = volume2224 + volumeEvent;
//			break;
//	}
//
//
//// Is it time to record volume from previous time bin to EEProm?
//    if(hour/2 != active_volume_bin){
//        SaveVolumeToEEProm();
//    }             
    return;
}


