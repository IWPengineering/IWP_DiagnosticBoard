/*
 * File:   Main.c
 * Author: ns1307
 *
 * Created on November 27, 2017, 3:52 PM
 */

// CONFIG1L
#pragma config FEXTOSC = LP     // External Oscillator mode Selection bits (LP (crystal oscillator) optimized for 32.768 kHz; PFM set to low power)
#pragma config RSTOSC = LFINTOSC// Power-up default value for COSC bits (Low-Frequency Oscillator)

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
#pragma config BORV = VBOR_2P45 // Brown Out Reset Voltage selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = ON        // WDT operating mode (WDT enabled regardless of sleep)

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


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <xc.h>
#include <string.h>
#include <p18F26K40.h>

int __attribute__ ((space(eedata))) eeData; // Global variable located in EEPROM

// ****************************************************
//            Constants
// *****************************************************

    const int xAxis = 11; // analog pin connected to x axis of accelerometer ** MAY NEED TO CHANGE
    const int yAxis = 12; // analog pin connected to y axis of accelerometer ** MAY NEED TO CHANGE
    const int signedNumAdjustADC = 511; // Used to divide the total range of the output of the 10 bit ADC into positive and negative range.
    const int upstrokeInterval = 10; // The number of milliseconds to delay before reading the upstroke
    const float PI = 3.141592;
    const float angleThresholdSmall = 0.1; //number close to zero to determine if handle is moving w/o interpreting accelerometer noise as movement.
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
// *****************************************************
//              Function Prototype
// *****************************************************
    
//void ClearWatchDogTimer(void);
//void EEProm_Write_Float(unsigned int ee_addr, void *obj_p);
//float getHandleAngle();
//int EEProm_Read_Int(int addr);
//void EEProm_Read_Float(int ee_addr, void *obj_p);
//int readWaterSensor(void);
//void delayMs(int ms);
//char BcdToDec(char val);
//
//
///*********************************************************************
// * Function: ClearWatchDogTimer()
// * Input: none
// * Output: none
// * Overview: You can use just ClrWdt() as a command.  However, I read in a forum 
// *           http://www.microchip.com/forums/m122062.aspx
// *           that since ClrWdt() expands to an asm command, the presence of the 
// *          asm will stop the compiler from optimizing any routine that it is a 
// *          part of.  Since I want to call this in Main, that would be a problem
// * Note: Library
// * TestDate: 1-2-2017
// ********************************************************************/
//void ClearWatchDogTimer(void){
//     ClrWdt();
//}
//
///*********************************************************************
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
//float getHandleAngle() {
//
//        signed int xValue = readAdc(xAxis) - signedNumAdjustADC; 
//        signed int yValue = readAdc(yAxis) - signedNumAdjustADC; 
//        float angle = atan2(yValue, xValue) * (180 / PI); //returns angle in degrees 
//        // Calculate and return the angle of the pump handle
//        if (angle > 20) {
//            angle = 20.0;
//        } else if (angle < -30) {
//            angle = -30.0;
//        }
//        angle10 = angle9;
//        angle9 = angle8;
//        angle8 = angle7;
//        angle7 = angle6;
//        angle6 = angle5;
//        angle5 = angle4;
//        angle4 = angle3;
//        angle3 = angle2;
//        angle2 = angle1;
//        angle1 = angle;
//
//        float averageAngle = (angle1 + angle2 + angle3 + angle4 + angle5 + angle6 + angle7 + angle8 + angle9 + angle10) / 10.0;
//
//        return averageAngle;
//        //return angle;
//}
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
///*********************************************************************
// * Function: readWaterSensor
// * Input: None
// * Output: pulseWidth
// * Overview: RB5 is one water sensor, start at beginning of positive pulse
// * Note: Pic Dependent
// * TestDate: Not tested as of 03-05-2015
// ********************************************************************/ 
//int readWaterSensor(void) { // RB5 is one water sensor
//    // turn on and off in the Main loop so the 555 has time to stabelize 
//   // digitalPinSet(waterPresenceSensorOnOffPin, 1); //turns on the water presence sensor.
//   
//    delayMs(5);  //debug
//    if (digitalPinStatus(waterPresenceSensorPin) == 1) {
//        while (digitalPinStatus(waterPresenceSensorPin)) {
//        }; //make sure you start at the beginning of the positive pulse
//    }
//    while (digitalPinStatus(waterPresenceSensorPin) == 0) {
//    }; //wait for rising edge
//    int prevICTime = TMR1; //get time at start of positive pulse
//    while (digitalPinStatus(waterPresenceSensorPin)) {
//    };
//    int currentICTime = TMR1; //get time at end of positive pulse
//    long pulseWidth = 0;
//    if (currentICTime >= prevICTime) {
//        pulseWidth = (currentICTime - prevICTime);
//    } else {
//        pulseWidth = (currentICTime - prevICTime + 0x100000000);
//    }
//    
//    // digitalPinSet(waterPresenceSensorOnOffPin, 0); //turns off the water presence sensor.
//
//    //Check if this value is right
//    return (pulseWidth <= pulseWidthThreshold);
//}
//
///*********************************************************************
// * Function: delayMs()
// * Input: milliseconds
// * Output: None
// * Overview: Delays the specified number of milliseconds
// * Note: Depends on Clock speed. Pic Dependent
// * TestDate: 05-20-14
// ********************************************************************/
//void delayMs(int ms) {
//    int myIndex;
//    while (ms > 0) {
//        myIndex = 0;
//        while (myIndex < 667) {
//            myIndex++;
//        }
//        ms--;
//    }
//}
//
////This function converts a BCD to DEC
////Input: BCD Value
////Returns: Hex Value
//
//char BcdToDec(char val) {
//    return ((val / 16 * 10) + (val % 16));
//}

void main(void) {
    int __attribute__ ((space(eedata))) eeData; // Global variable located in EEPROM
    
   
    int handleMovement = 0; // Either 1 or no 0 if the handle moving upward
	int timeOutStatus = 0; // Used to keep track of the water prime timeout
    
	float angleCurrent = 0; // Stores the current angle of the pump handle
	float anglePrevious = 0; // Stores the last recorded angle of the pump handle
	float angleDelta = 0; // Stores the difference between the current and previous angles
	float upStrokePrime = 0; // Stores the sum of the upstrokes for calculating the prime
	float upStrokeExtract = 0; // Stores the sum of the upstrokes for calculating volume
	float volumeEvent = 0; // Stores the volume extracted
	float leakRatePrevious = 0; // Stores the previous Leak Rate incase if someone stats to pump before leakage can be measured
	float upStrokePrimeMeters = 0; // Stores the upstroke in meters
	float leakRate = 0; // Rate at which water is leaking from the rising main
    long leakRateTimeOut = 3000; // Equivalent to 3 seconds (in "upstrokeInterval" millisecond intervals); 

    int currentDay;
    char never_primed = 0;  //set to 1 if the priming loop is exited without detecting water
    
    float angle1 = 0;
    float angle2 = 0;
    float angle3 = 0;
    float angle4 = 0;
    float angle5 = 0;
    float angle6 = 0;
    float angle7 = 0;
    float angle8 = 0;
    float angle9 = 0;
    float angle10 = 0; // *MAY NEED TO DELETE  
    
    const float handleMovementThreshold = 5.0;
    
    int readAdc(int channel) //check with accelerometer
{
    switch (channel) {
        case 0:
            specifyAnalogPin(depthSensorPin, 1); //make depthSensor Analog
            pinDirectionIO(depthSensorPin, 1);
            pinSampleSelectRegister(depthSensorPin);
            break;
        case 2: //Currently unused, may be used in the future.
            specifyAnalogPin(Pin4, 1); // makes Pin4 analog
            pinDirectionIO(Pin4, 1); // Pin4 in an input
            pinSampleSelectRegister(Pin4); // Connect Pin4 as the S/H input

            //ANSBbits.ANSB0 = 1; // AN2 is analog
            //TRISBbits.TRISB0 = 1; // AN2 is an input
            //AD1CHSbits.CH0SA = 2; // Connect AN2 as the S/H input
            break;
        case 4:
            specifyAnalogPin(rxPin, 1); // make rx analog
            pinDirectionIO(rxPin, 1); // makes rxPin an input
            pinSampleSelectRegister(rxPin); // Connect rxPin as the S/H input
            //ANSBbits.ANSB2 = 1; // AN4 is analog
            //TRISBbits.TRISB2 = 1; // AN4 is an input
            //AD1CHSbits.CH0SA = 4; // Connect AN4 as the S/H input
            break;
        case 11:
            specifyAnalogPin(xAxisAccelerometerPin, 1); // makes xAxis analog
            pinDirectionIO(xAxisAccelerometerPin, 1); // makes xAxis an input
            pinSampleSelectRegister(xAxisAccelerometerPin); // Connect xAxis as the S/H input
            //ANSBbits.ANSB13 = 1; // AN11 is analog
            //TRISBbits.TRISB13 = 1; // AN11 is an input
            //AD1CHSbits.CH0SA = 11; //Connect AN11 as the S/H input (sample and hold)
            break;
        case 12:
            specifyAnalogPin(yAxisAccelerometerPin, 1); // makes yAxis analog
            pinDirectionIO(yAxisAccelerometerPin, 1); // makes yAxis an input
            pinSampleSelectRegister(yAxisAccelerometerPin); // Connect yAxis as the S/H input
            //PORTBbits.RB12 = 1; // AN12 is analog ***I changed this to ANSBbits.ANSBxx 03-31-2015
            //TRISBbits.TRISB12 = 1; // AN12 is an input
            //AD1CHSbits.CH0SA = 12; // Connect AN12 as the S/H input
            break;
        case 15:
            specifyAnalogPin(batteryLevelPin, 1); // makes batteryLevelPin analog
            pinDirectionIO(batteryLevelPin, 1); // makes batteryLevelPin an input
            pinSampleSelectRegister(batteryLevelPin); // Connect batteryLevelPin
            break;
    }
    AD1CON1bits.ADON = 1; // Turn on ADC
    AD1CON1bits.SAMP = 1;
    while (!AD1CON1bits.DONE) {
    }
    unsigned int adcValue = ADC1BUF0;
    return adcValue;
    
    ***********************************
    float getHandleAngle() {                //This is from Utilities

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
    
    ******************************************************************************************8
    float newAngle = getHandleAngle();     //This is from main
			float deltaAngle = newAngle - anglePrevious;
			if(deltaAngle < 0) {
				deltaAngle *= -1;
			}
            if(deltaAngle > handleMovementThreshold){            // The total movement of the handle from rest has been exceeded
				//handleMovement = 1;      //Instead of handleMovement, we light up LED.
            }
    
            
               
                
    // Initialize
    
    // Clear any data
    
    // Noon message section 
    
    // Diagnostic message section 
    
    // Are we pumping?
    
    // Do we have water? 
    // if (readWaterSensor()) {
    //      light up LED 
    // }
    
    
    
    
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
