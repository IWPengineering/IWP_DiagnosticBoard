/*
 * File:   Main.c
 * Author: ns1307
 *
 * Created on November 27, 2017, 3:52 PM
 */


#include <xc.h>



/*********************************************************************
 * Function: ClearWatchDogTimer()
 * Input: none
 * Output: none
 * Overview: You can use just ClrWdt() as a command.  However, I read in a forum 
 *           http://www.microchip.com/forums/m122062.aspx
 *           that since ClrWdt() expands to an asm command, the presence of the 
  *          asm will stop the compiler from optimizing any routine that it is a 
  *          part of.  Since I want to call this in Main, that would be a problem
 * Note: Library
 * TestDate: 1-2-2017
 ********************************************************************/
 void ClearWatchDogTimer(void){
     ClrWdt();
 }

void main(void) {
    ClearWatchDogTime();
    return;
}
