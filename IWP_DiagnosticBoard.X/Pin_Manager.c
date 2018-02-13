/*
 * File:   Pin_Manager.c
 * Author: sandrabrittanysnozzi
 *
 * Created on January 18, 2018, 2:16 PM 
 */


#include "xc.h"
//#include "Pin_Manager2.h"


/////////////////////////////////////////////////////////////////////
////                                                             ////
////                PIN MANAGEMENT FUNCTIONS                     ////
////                                                             ////
/////////////////////////////////////////////////////////////////////
void pinDirectionIO(int pin, int io) { // 1 is an input, 0 is an output
	// Pin 1 can't change direction
    if (pin == 2) {
        LATAbits.LATA0 = io; 
    }
    
    else if (pin == 3) {
        LATAbits.LATA1 = io;
    }
    
    else if (pin == 4) {
        LATAbits.LATA2 = io;
    }
   
    else if (pin == 5) { // WPS_OUT
        LATAbits.LATA3 = io;
    }
    else if (pin == 6) { // ADXL_335_X
        LATAbits.LATA4 = io;
    }
    else if (pin == 7) { // ADXL_335_Y
        LATAbits.LATA5 = io; 
    }
    // Pin8 -  VSS - do nothing
    else if (pin == 9) {
        LATAbits.LATA7 = io;
    }
    else if (pin == 10) { // Push button 
        LATAbits.LATA6 = io;
    }
    else if (pin == 11) { // EEPROM (CS with bar) NEED TO CHECK
        LATCbits.LATC0 = io;
    }
    else if (pin == 12) { // PIC_PIN4
        LATCbits.LATC1 = io;
    }
    else if (pin == 13) { // PIC_PIN5
        LATCbits.LATC2 = io;
    }
    else if (pin == 14) { // EEPROM SCK NEED TO CHECK
        LATCbits.LATC3 = io; 
    }
    else if (pin == 15) { // EEPROM SO NEED TO CHECK 
        LATCbits.LATC4 = io;
    }
    else if (pin == 16) { // EEPROM SI NEED TO CHECK 
        LATCbits.LATC5 = io;
    }
    else if (pin == 17) { // TX (input from control board)
        LATCbits.LATC6 = io; 
    }
    else if (pin == 18) { // RX (output to control board)
        LATCbits.LATC7 = io;
    }
    // Pin 19 VSS
    // Pin 20 VDD
    else if (pin == 21) { // PICKIT_5
        LATBbits.LATB0 = io; 
    }
    else if (pin == 22) { // PICKIT_4
        LATBbits.LATB1 = io;
    }
    else if (pin == 23) { // PIC_PIN12
        LATBbits.LATB2 = io; 
    }
    else if (pin == 24) { // LED4
        LATBbits.LATB3 = io;
    }
    else if (pin == 25) { // LED5
        LATBbits.LATB4 = io;
    }
    else if (pin == 26) { // LED1
        LATBbits.LATB5 = io;
    }
    else if (pin == 27) { // LED2
        LATBbits.LATB6 = io;
    }
    else if (pin == 28) { // LED3
        LATBbits.LATB7 = io;
    }
// NEED TO FIX CHECK DATASHEET 
}

void digitalPinSet(int pin, int set) // 1 for high, 0 for low
{
    // Pin 1 can't change direction
    if (pin == 2) {
        TRISAbits.TRISA0 = set;
    }
    else if (pin == 3) {
        TRISAbits.TRISA1 = set;
    }
    else if (pin == 4) {
        TRISAbits.TRISA2 = set;
    }
    else if (pin == 5) { // WPS_OUT
        TRISAbits.TRISA3 = set;
    }
    else if (pin == 6) { // ADXL_335_X
        TRISAbits.TRISA4 = set;
    }
    else if (pin == 7) { // ADXL_335_Y
        TRISAbits.TRISA5 = set; 
    }
    // Pin8 -  VSS - do nothing
    else if (pin == 9) {
        TRISAbits.TRISA7 = set;
    }
    else if (pin == 10) { // Push button 
        TRISAbits.TRISA6 = set;
    }
    else if (pin == 11) { // EEPROM (CS with bar) NEED TO CHECK
        TRISCbits.TRISC0 = set;
    }
    else if (pin == 12) { // PIC_PIN4
        TRISCbits.TRISC1 = set;
    }
    else if (pin == 13) { // PIC_PIN5
        TRISCbits.TRISC2 = set;
    }
    else if (pin == 14) { // EEPROM SCK NEED TO CHECK
        TRISCbits.TRISC3 = set; 
    }
    else if (pin == 15) { // EEPROM SO NEED TO CHECK 
        TRISCbits.TRISC4 = set;
    }
    else if (pin == 16) { // EEPROM SI NEED TO CHECK 
        TRISCbits.TRISC5 = set;
    }
    else if (pin == 17) { // TX (input from control board)
        TRISCbits.TRISC6 = set; 
    }
    else if (pin == 18) { // RX (output to control board)
        TRISCbits.TRISC7 = set;
    }
    // Pin 19 VSS
    // Pin 20 VDD
    else if (pin == 21) { // PICKIT_5
        TRISBbits.TRISB0 = set;
    }
    else if (pin == 22) { // PICKIT_4
        TRISBbits.TRISB1 = set;
    }
    else if (pin == 23) { // PIC_PIN12
        TRISBbits.TRISB2 = set; 
    }
    else if (pin == 24) { // LED4
        TRISBbits.TRISB3 = set;
    }
    else if (pin == 25) { // LED5
        TRISBbits.TRISB4 = set;
    }
    else if (pin == 26) { // LED1
        TRISBbits.TRISB5 = set;
    }
    else if (pin == 27) { // LED2
        TRISBbits.TRISB6 = set;
    }
    else if (pin == 28) { // LED3
        TRISBbits.TRISB7 = set;
    }
// NEED TO FIX CHECK DATASHEET
}

//TODO: Should be based off of the RB values, not the AN
void specifyAnalogPin(int pin, int analogOrDigital) // analogOrDigital = 1 if analog, 0 is digital
{
    if (pin == 5) { // WPS_OUT
        ANSELAbits.ANSELA3 = analogOrDigital; 
    }
    else if (pin == 6) { // ADXL_335_X
        ANSELAbits.ANSELA4 = analogOrDigital;
    }
    else if (pin == 7) { // ADXL_335_Y
        ANSELAbits.ANSELA5 = analogOrDigital;
    }
    else if (pin == 9) {
        ANSELAbits.ANSELA7 = analogOrDigital;
    }
    else if (pin == 10) { // Push button
        ANSELAbits.ANSELA6 = analogOrDigital;
    }
//    else if (pin == 11) { // CHECK THIS ON DATASHEET 
//        ANSCbits.ANSELC0 = analogOrDigital;
//    }
    else if (pin == 12) { // PIC_PIN4
        ANSELCbits.ANSELC1 = analogOrDigital;
    }
    else if (pin == 13) { // PIC_PIN5
        ANSELCbits.ANSELC2 = analogOrDigital;
    }
//    else if (pin == 14) { // CHECK THIS ON DATASHEET
//        ANSELCbits.ANSELC3 = analogOrDigital;
//    }
//    else if (pin == 15) { // CHECK THIS ON DATASHEET
//        ANSELCbits.ANSELC4 = analogOrDigital;
//    }
//    else if (pin == 16) { // CHECK THIS ON DATASHEET
//        ANSELCbits.ANSELC5 = analogOrDigital;
//    }
    else if (pin == 17) { // TX
        ANSELCbits.ANSELC6 = analogOrDigital;
    }
    else if (pin == 18) { // RX
        ANSELCbits.ANSELC7 = analogOrDigital;
    }
    else if (pin == 21) { // PICKIT_5
        ANSELBbits.ANSELB0 = analogOrDigital;
    }
    else if (pin == 22) { // PICKIT_4
        ANSELBbits.ANSELB1 = analogOrDigital;
    }
    else if (pin == 23) {
        ANSELBbits.ANSELB2 = analogOrDigital;
    }
    else if (pin == 24) {
        ANSELBbits.ANSELB3 = analogOrDigital;
    }
    else if (pin == 25) {
        ANSELBbits.ANSELB4 = analogOrDigital;
    }
    else if (pin == 26) {
        ANSELBbits.ANSELB5 = analogOrDigital;
    }
    else if (pin == 27) {
        ANSELBbits.ANSELB6 = analogOrDigital;
    }
    else if (pin == 28) {
        ANSELBbits.ANSELB7 = analogOrDigital;
    }
    // NEED TO FIX CHECK DATASHEET

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
//	int pinValue;
////	if (pin == 1)
////	{
////		pinValue = PORTAbits.RA5;
////	}
//    if (pin == 2) {
//        pinValue = PORTAbits.RA0;
//    }
//    else if (pin == 3) {
//        pinValue = PORTAbits.RA1;
//    }
//	else if (pin == 4)
//	{
//		pinValue = PORTAbits.RA2;
//	}
//	else if (pin == 5)
//	{
//		pinValue = PORTAbits.RA3;
//	}
//	else if (pin == 6)
//	{
//		pinValue = PORTAbits.RA4;
//	}
//	else if (pin == 7)
//	{
//		pinValue = PORTAbits.RA5;
//	}
//    
//	// Pin8 - Always VSS for PIC24FV32KA302 - Do nothing
//	else if (pin == 9)
//	{
//		pinValue = PORTAbits.RA7;
//	}
//	else if (pin == 10)
//	{
//		pinValue = PORTAbits.RA6;
//	}
//	else if (pin == 11)
//	{
//		pinValue = PORTCbits.RC0;
//	}
//	else if (pin == 12)
//	{
//		pinValue = PORTCbits.RC1;
//	}
//    
//    else if (pin == 13) {
//        pinValue = PORTCbits.RC2;
//    }
//	else if (pin == 14)
//	{
//		pinValue = PORTCbits.RC3;
//	}
//	else if (pin == 15)
//	{
//		pinValue = PORTBbits.RB6;
//	}
//	else if (pin == 16)
//	{
//		pinValue = PORTBbits.RB7;
//	} //Usually reserved for TX
//	else if (pin == 17)
//	{
//		pinValue = PORTCbits.RC6;
//	}//Usually reserved for I2C
//	else if (pin == 18)
//	{
//		pinValue = PORTCbits.RC7;
//	}//Usually Reserved for I2C
//	else if (pin == 19)
//	{
//		pinValue = PORTAbits.RA7;
//	}
//	// Pin 20 - Always vCap for PIC24FV32KA302 - Do nothing
//	else if (pin == 21)
//	{
//		pinValue = PORTBbits.RB10;
//	}
//	else if (pin == 22)
//	{
//		pinValue = PORTBbits.RB11;
//	}
//	else if (pin == 23)
//	{
//		pinValue = PORTBbits.RB12;
//	}
//	else if (pin == 24)
//	{
//		pinValue = PORTBbits.RB13;
//	}
//	else if (pin == 25)
//	{
//		pinValue = PORTBbits.RB14;
//	}
//	else if (pin == 26)
//	{
//		pinValue = PORTBbits.RB15;
//	}
//	//return pinValue;
    return 1; // CHANGE
//	// Pin 27 - Always VSS for PIC24FV32KA302 - Do nothing
//	// Pin 28 - Always VDD for PIC24FV32KA302 - Do nothing
}



