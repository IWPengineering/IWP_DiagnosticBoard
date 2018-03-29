/*
 * File:   Pin_Manager.c
 * Author: sandrabrittanysnozzi
 *
 * Created on January 18, 2018, 2:16 PM 
 */


#include "xc.h"
#include "Pin_Manager.h"


/////////////////////////////////////////////////////////////////////
////                                                             ////
////                PIN MANAGEMENT FUNCTIONS                     ////
////                                                             ////
/////////////////////////////////////////////////////////////////////
void pinDirectionIO(int pin, int io) { // 1 is an input, 0 is an output
	// Pin 1 can't change direction
//    if (pin == 2) {
//        TRISAbits.TRISA0 = io;
//    }
//    else if (pin == 3) {
//        TRISAbits.TRISA1 = io;
//    }
//    else if (pin == 4) {
//        TRISAbits.TRISA2 = io;
//    }
//    else if (pin == 5) { // WPS_OUT
//        TRISAbits.TRISA3 = io;
//    }
//    else if (pin == 6) { // ADXL_335_X
//        TRISAbits.TRISA4 = io;
//    }
//    else if (pin == 7) { // ADXL_335_Y
//        TRISAbits.TRISA5 = io; 
//    }
//    // Pin8 -  VSS - do nothing
//    else if (pin == 9) {
//        TRISAbits.TRISA7 = io;
//    }
//    else if (pin == 10) { // Push button 
//        TRISAbits.TRISA6 = io;
//    }
//    else if (pin == 11) { // EEPROM (CS with bar) NEED TO CHECK
//        TRISCbits.TRISC0 = io;
//    }
//    else if (pin == 12) { // PIC_PIN4
//        TRISCbits.TRISC1 = io;
//    }
//    else if (pin == 13) { // PIC_PIN5
//        TRISCbits.TRISC2 = io;
//    }
//    else if (pin == 14) { // EEPROM SCK NEED TO CHECK
//        TRISCbits.TRISC3 = io; 
//    }
//    else if (pin == 15) { // EEPROM SO NEED TO CHECK 
//        TRISCbits.TRISC4 = io;
//    }
//    else if (pin == 16) { // EEPROM SI NEED TO CHECK 
//        TRISCbits.TRISC5 = io;
//    }
//    else if (pin == 17) { // TX (input from control board)
//        TRISCbits.TRISC6 = io; 
//    }
//    else if (pin == 18) { // RX (output to control board)
//        TRISCbits.TRISC7 = io;
//    }
//    // Pin 19 VSS
//    // Pin 20 VDD
//    else if (pin == 21) { // PICKIT_5
//        TRISBbits.TRISB0 = io;
//    }
//    else if (pin == 22) { // PICKIT_4
//        TRISBbits.TRISB1 = io;
//    }
//    else if (pin == 23) { // PIC_PIN12
//        TRISBbits.TRISB2 = io; 
//    }
//    else if (pin == 24) { // LED4
//        TRISBbits.TRISB3 = io;
//    }
//    else if (pin == 25) { // LED5
//        TRISBbits.TRISB4 = io;
//    }
//    else if (pin == 26) { // LED1
//        TRISBbits.TRISB5 = io;
//    }
//    else if (pin == 27) { // LED2
//        TRISBbits.TRISB6 = io;
//    }
//    else if (pin == 28) { // LED3
//        TRISBbits.TRISB7 = io;
//    }
// NEED TO FIX CHECK DATASHEET 
}

void digitalPinSet(int pin, int set) // 1 for high, 0 for low
{
//    // Pin 1 can't change direction
//    if (pin == 2) {
//        TRISAbits.TRISA0 = set;
//    }
//    else if (pin == 3) {
//        TRISAbits.TRISA1 = set;
//    }
//    else if (pin == 4) {
//        TRISAbits.TRISA2 = set;
//    }
//    else if (pin == 5) { // WPS_OUT
//        TRISAbits.TRISA3 = set;
//    }
//    else if (pin == 6) { // ADXL_335_X
//        TRISAbits.TRISA4 = set;
//    }
//    else if (pin == 7) { // ADXL_335_Y
//        TRISAbits.TRISA5 = set; 
//    }
//    // Pin8 -  VSS - do nothing
//    else if (pin == 9) {
//        TRISAbits.TRISA7 = set;
//    }
//    else if (pin == 10) { // Push button 
//        TRISAbits.TRISA6 = set;
//    }
//    else if (pin == 11) { // EEPROM (CS with bar) NEED TO CHECK
//        TRISCbits.TRISC0 = set;
//    }
//    else if (pin == 12) { // PIC_PIN4
//        TRISCbits.TRISC1 = set;
//    }
//    else if (pin == 13) { // PIC_PIN5
//        TRISCbits.TRISC2 = set;
//    }
//    else if (pin == 14) { // EEPROM SCK NEED TO CHECK
//        TRISCbits.TRISC3 = set; 
//    }
//    else if (pin == 15) { // EEPROM SO NEED TO CHECK 
//        TRISCbits.TRISC4 = set;
//    }
//    else if (pin == 16) { // EEPROM SI NEED TO CHECK 
//        TRISCbits.TRISC5 = set;
//    }
//    else if (pin == 17) { // TX (input from control board)
//        TRISCbits.TRISC6 = set; 
//    }
//    else if (pin == 18) { // RX (output to control board)
//        TRISCbits.TRISC7 = set;
//    }
//    // Pin 19 VSS
//    // Pin 20 VDD
//    else if (pin == 21) { // PICKIT_5
//        TRISBbits.TRISB0 = set;
//    }
//    else if (pin == 22) { // PICKIT_4
//        TRISBbits.TRISB1 = set;
//    }
//    else if (pin == 23) { // PIC_PIN12
//        TRISBbits.TRISB2 = set; 
//    }
//    else if (pin == 24) { // LED4
//        TRISBbits.TRISB3 = set;
//    }
//    else if (pin == 25) { // LED5
//        TRISBbits.TRISB4 = set;
//    }
//    else if (pin == 26) { // LED1
//        TRISBbits.TRISB5 = set;
//    }
//    else if (pin == 27) { // LED2
//        TRISBbits.TRISB6 = set;
//    }
//    else if (pin == 28) { // LED3
//        TRISBbits.TRISB7 = set;
//    }
// NEED TO FIX CHECK DATASHEET
}

//TODO: Should be based off of the RB values, not the AN
void specifyAnalogPin(int pin, int analogOrDigital) // analogOrDigital = 1 if analog, 0 is digital
{
//    if (pin == 2) { 
//        ANSELAbits.ANSA0 = analogOrDigital; 
//    }
//    else if (pin == 3) { // ADXL_335_X
//        ANSELAbits.ANSA1 = analogOrDigital;
//    }
//    else if (pin == 4) { // ADXL_335_Y
//        ANSELAbits.ANSA2 = analogOrDigital;
//    }
//    else if (pin == 5) {
//        ANSELAbits.ANSA3 = analogOrDigital;
//    }
//    else if (pin == 6) { // Push button
//        ANSELAbits.ANSA4 = analogOrDigital;
//    }
////    else if (pin == 7) { // CHECK THIS ON DATASHEET 
////        ANSELAbits.ANSA5 = analogOrDigital;
////    }
//    else if (pin == 9) { // PIC_PIN4
//        ANSELAbits.ANSA7 = analogOrDigital;
//    }
//    else if (pin == 10) { // PIC_PIN5
//        ANSELAbits.ANSA6 = analogOrDigital;
//    }
////    else if (pin == 11) { // CHECK THIS ON DATASHEET
////        ANSELCbits.ANSC0 = analogOrDigital;
////    }
////    else if (pin == 12) { // CHECK THIS ON DATASHEET
////        ANSELCbits.ANSC1 = analogOrDigital;
////    }
////    else if (pin == 13) { // CHECK THIS ON DATASHEET
////        ANSELCbits.ANSC2 = analogOrDigital;
////    }
//    else if (pin == 14) { // TX
//        ANSELCbits.ANSC3 = analogOrDigital;
//    }
//    else if (pin == 15) { // RX
//        ANSELCbits.ANSC4 = analogOrDigital;
//    }
//    else if (pin == 16) { // PICKIT_5
//        ANSELCbits.ANSC5 = analogOrDigital;
//    }
//    else if (pin == 17) { // PICKIT_4
//        ANSELCbits.ANSC6 = analogOrDigital;
//    }
//    else if (pin == 18) {
//        ANSELCbits.ANSC7 = analogOrDigital;
//    }
//    else if (pin == 21) {
//        ANSELBbits.ANSB0 = analogOrDigital;
//    }
//    else if (pin == 22) {
//        ANSELBbits.ANSB1 = analogOrDigital;
//    }
//    else if (pin == 23) {
//        ANSELBbits.ANSB2 = analogOrDigital;
//    }
//    else if (pin == 24) {
//        ANSELBbits.ANSB3 = analogOrDigital;
//    }
//    else if (pin == 25) {
//        ANSELBbits.ANSB4 = analogOrDigital;
//    }
//    else if (pin == 26) {
//        ANSELBbits.ANSB5 = analogOrDigital;       
//    }
}


void pinSampleSelectRegister(int pin) { //  A/D Sample Select Register (CHECK WHEN USED)  
                        //(this is only used in the readADC() function)
//    if (pin == 6) { // ADXL_335_X NEED TO CHECK
//        AD1CHSbits.CH0SA = 4; // ANA4
//    }
//    else if (pin == 7) {
//        AD1CHSbits.CH0SA = 5; // ANA5
//    }
//    else if (pin == 17) {
//        AD1CHSbits.CH0SA = 6; // ANC6 NEED TO CHECK 
//    }
    // NEED TO FIX CHECK DATASHEET 
   
}

int digitalPinStatus(int pin)
{
	int pinValue;
	if (pin == 2)
	{
		pinValue = PORTAbits.RA0;
	}
	else if (pin == 3)
	{
		pinValue = PORTAbits.RA1;
	}
	else if (pin == 4)
	{
		pinValue = PORTAbits.RA2;
	}
	else if (pin == 5)
	{
		pinValue = PORTAbits.RA3;
	}
	else if (pin == 6)
	{
		pinValue = PORTAbits.RA4;
	}
	else if (pin == 7)
	{
		pinValue = PORTAbits.RA5;
	}
	// Pin8 - Always VSS for PIC24FV32KA302 - Do nothing
	else if (pin == 9)
	{
		pinValue = PORTAbits.RA7;
	}
	else if (pin == 10)
	{
		pinValue = PORTAbits.RA6;
	}
	else if (pin == 11)
	{
		pinValue = PORTCbits.RC0;
	}
	else if (pin == 12)
	{
		pinValue = PORTCbits.RC1;
	}
	else if (pin == 13)
	{
		pinValue = PORTCbits.RC2;
	}
	else if (pin == 14)
	{
		pinValue = PORTCbits.RC3;
	}
	else if (pin == 15)
	{
		pinValue = PORTCbits.RC4;
	} 
	else if (pin == 16)
	{
		pinValue = PORTCbits.RC5;
	}
	else if (pin == 17)
	{
		pinValue = PORTCbits.RC6;
	}
	else if (pin == 18)
	{
		pinValue = PORTCbits.RC7;
	}
	// Pin 19 - Always Vss for PIC18LF26K40 - Do nothing
	else if (pin == 21)
	{
		pinValue = PORTBbits.RB0;
	}
	else if (pin == 22)
	{
		pinValue = PORTBbits.RB1;
	}
	else if (pin == 23)
	{
		pinValue = PORTBbits.RB2;
	}
	else if (pin == 24)
	{
		pinValue = PORTBbits.RB3;
	}
	else if (pin == 25)
	{
		pinValue = PORTBbits.RB4;
	}
	else if (pin == 26)
	{
		pinValue = PORTBbits.RB5;
	}
  // Pins 27 and 28 are for the Pickit 
	return pinValue;
    //return 1; // CHANGE
}


