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
    if (pin == 2) {
        TRISAbits.TRISA0 = io;
    }
    else if (pin == 3) {
        TRISAbits.TRISA1 = io;
    }
    else if (pin == 4) {
        TRISAbits.TRISA2 = io;
    }
    else if (pin == 5) { // WPS_OUT
        TRISAbits.TRISA3 = io;
    }
    else if (pin == 6) { // ADXL_335_X
        TRISAbits.TRISA4 = io;
    }
    else if (pin == 7) { // ADXL_335_Y
        TRISAbits.TRISA5 = io; 
    }
    // Pin8 -  VSS - do nothing
    else if (pin == 9) {
        TRISAbits.TRISA7 = io;
    }
    else if (pin == 10) { // Push button 
        TRISAbits.TRISA6 = io;
    }
    else if (pin == 11) { // EEPROM (CS with bar) NEED TO CHECK
        TRISCbits.TRISC0 = io;
    }
    else if (pin == 12) { // PIC_PIN4
        TRISCbits.TRISC1 = io;
    }
    else if (pin == 13) { // PIC_PIN5
        TRISCbits.TRISC2 = io;
    }
    else if (pin == 14) { // EEPROM SCK NEED TO CHECK
        TRISCbits.TRISC3 = io; 
    }
    else if (pin == 15) { // EEPROM SO NEED TO CHECK 
        TRISCbits.TRISC4 = io;
    }
    else if (pin == 16) { // EEPROM SI NEED TO CHECK 
        TRISCbits.TRISC5 = io;
    }
    else if (pin == 17) { // TX (input from control board)
        TRISCbits.TRISC6 = io; 
    }
    else if (pin == 18) { // RX (output to control board)
        TRISCbits.TRISC7 = io;
    }
    // Pin 19 VSS
    // Pin 20 VDD
    else if (pin == 21) { // PICKIT_5
        TRISBbits.TRISB0 = io;
    }
    else if (pin == 22) { // PICKIT_4
        TRISBbits.TRISB1 = io;
    }
    else if (pin == 23) { // PIC_PIN12
        TRISBbits.TRISB2 = io; 
    }
    else if (pin == 24) { // LED4
        TRISBbits.TRISB3 = io;
    }
    else if (pin == 25) { // LED5
        TRISBbits.TRISB4 = io;
    }
    else if (pin == 26) { // LED1
        TRISBbits.TRISB5 = io;
    }
    else if (pin == 27) { // LED2
        TRISBbits.TRISB6 = io;
    }
    else if (pin == 28) { // LED3
        TRISBbits.TRISB7 = io;
    }

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
//	if (pin == 1)
//	{
//		PORTAbits.RA5 = set;
//	}
//	else if (pin == 2)
//	{
//		PORTAbits.RA0 = set;
//	}
//	else if (pin == 3)
//	{
//		PORTAbits.RA1 = set;
//	}
//	else if (pin == 4)
//	{
//		PORTBbits.RB0 = set;
//	}
//	else if (pin == 5)
//	{
//		PORTBbits.RB1 = set;
//	}
//	else if (pin == 6)
//	{
//		PORTBbits.RB2 = set;
//	}
//	else if (pin == 7)
//	{
//		PORTBbits.RB3 = set;
//	}
//	// Pin8 - Always VSS for PIC24FV32KA302 - Do nothing
//	else if (pin == 9)
//	{
//		PORTAbits.RA2 = set;
//	}
//	else if (pin == 10)
//	{
//		PORTAbits.RA3 = set;
//	}
//	else if (pin == 11)
//	{
//		PORTBbits.RB4 = set;
//	}
//	else if (pin == 12)
//	{
//		PORTAbits.RA4 = set;
//	}
//	//Pin 13 - Always VDD for PIC24FV32KA302 - Do nothing
//	else if (pin == 14)
//	{
//		PORTBbits.RB5 = set;
//	}
//	else if (pin == 15)
//	{
//		PORTBbits.RB6 = set;
//	}
//	else if (pin == 16)
//	{
//		PORTBbits.RB7 = set;
//	} //Usually reserved for TX
//	else if (pin == 17)
//	{
//		PORTBbits.RB8 = set;
//	}//Usually reserved for I2C
//	else if (pin == 18)
//	{
//		PORTBbits.RB9 = set;
//	}//Usually Reserved for I2C
//	else if (pin == 19)
//	{
//		PORTAbits.RA7 = set;
//	}
//	// Pin 20 - Always vCap for PIC24FV32KA302 - Do nothing
//	else if (pin == 21)
//	{
//		PORTBbits.RB10 = set;
//	}
//	else if (pin == 22)
//	{
//		PORTBbits.RB11 = set;
//	}
//	else if (pin == 23)
//	{
//		PORTBbits.RB12 = set;
//	}
//	else if (pin == 24)
//	{
//		PORTBbits.RB13 = set;
//	}
//	else if (pin == 25)
//	{
//		PORTBbits.RB14 = set;
//	}
//	else if (pin == 26)
//	{
//		PORTBbits.RB15 = set;
//	}
	// Pin 27 - Always VSS for PIC24FV32KA302 - Do nothing
	// Pin 28 - Always VDD for PIC24FV32KA302 - Do nothing
}

//TODO: Should be based off of the RB values, not the AN
void specifyAnalogPin(int pin, int analogOrDigital) // analogOrDigital = 1 if analog, 0 is digital
{
//	if (pin == 4)
//	{
//		ANSBbits.ANSB0 = analogOrDigital;
//	}
//	else if (pin == 5)
//	{
//		ANSBbits.ANSB1 = analogOrDigital;
//	}
//	else if (pin == 6)
//	{
//		ANSBbits.ANSB2 = analogOrDigital;
//	}
//	else if (pin == 7)
//	{
//		ANSBbits.ANSB3 = analogOrDigital;
//		//TODO: Jacqui needs to find out why pin 7 isn't in the library
//	}
//	else if (pin == 11)
//	{
//		ANSBbits.ANSB4 = analogOrDigital;
//	}
//	else if (pin == 23)
//	{
//		ANSBbits.ANSB12 = analogOrDigital;
//	}
//	else if (pin == 24)
//	{
//		ANSBbits.ANSB13 = analogOrDigital;
//	}
//	else if (pin == 25)
//	{
//		ANSBbits.ANSB14 = analogOrDigital;
//	}
//	else if (pin == 26)
//	{
//		ANSBbits.ANSB15 = analogOrDigital;
//	}
}


void pinSampleSelectRegister(int pin) { //  A/D Sample Select Regiser 
                        //(this is only used in the readADC() function)
//    if (pin == 4)
//	{
//		AD1CHSbits.CH0SA = 2; //AN2
//	}
//	else if (pin == 5)
//	{
//		AD1CHSbits.CH0SA = 3; //AN3
//	}
//	else if (pin == 6)
//	{
//		AD1CHSbits.CH0SA = 4;
//	}
//	else if (pin == 7)
//	{
//		AD1CHSbits.CH0SA = 5;
//	}
//	else if (pin == 11)
//	{
//		AD1CHSbits.CH0SA = 15;
//	}
//	else if (pin == 23)
//	{
//		AD1CHSbits.CH0SA = 12;
//	}
//	else if (pin == 24)
//	{
//		AD1CHSbits.CH0SA = 11;
//	}
//	else if (pin == 25)
//	{
//		AD1CHSbits.CH0SA = 10;
//	}
//	else if (pin == 26)
//	{
//		AD1CHSbits.CH0SA = 9;
//	}
}

int digitalPinStatus(int pin)
{
//	int pinValue;
//	if (pin == 1)
//	{
//		pinValue = PORTAbits.RA5;
//	}
//	else if (pin == 2)
//	{
//		pinValue = PORTAbits.RA0;
//	}
//	else if (pin == 3)
//	{
//		pinValue = PORTAbits.RA1;
//	}
//	else if (pin == 4)
//	{
//		pinValue = PORTBbits.RB0;
//	}
//	else if (pin == 5)
//	{
//		pinValue = PORTBbits.RB1;
//	}
//	else if (pin == 6)
//	{
//		pinValue = PORTBbits.RB2;
//	}
//	else if (pin == 7)
//	{
//		pinValue = PORTBbits.RB3;
//	}
//	// Pin8 - Always VSS for PIC24FV32KA302 - Do nothing
//	else if (pin == 9)
//	{
//		pinValue = PORTAbits.RA2;
//	}
//	else if (pin == 10)
//	{
//		pinValue = PORTAbits.RA3;
//	}
//	else if (pin == 11)
//	{
//		pinValue = PORTBbits.RB4;
//	}
//	else if (pin == 12)
//	{
//		pinValue = PORTAbits.RA4;
//	}
//	//Pin 13 - Always VDD for PIC24FV32KA302 - Do nothing
//	else if (pin == 14)
//	{
//		pinValue = PORTBbits.RB5;
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
//		pinValue = PORTBbits.RB8;
//	}//Usually reserved for I2C
//	else if (pin == 18)
//	{
//		pinValue = PORTBbits.RB9;
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
	//return pinValue;
    return 1; // CHANGE
	// Pin 27 - Always VSS for PIC24FV32KA302 - Do nothing
	// Pin 28 - Always VDD for PIC24FV32KA302 - Do nothing
}


//DEBUG DEBUG DEBUG DEBUG
//void debugHighLow(int pin){
//    specifyAnalogPin(pin, 0); // makes digital
//    pinDirectionIO(pin, 0); // makes output
//    if(digitalPinStatus(pin) == 0) {
//        digitalPinSet(pin, 1); // makes high
//    }
//    else{
//        digitalPinSet(pin, 0); //makes low
//    }
//}
