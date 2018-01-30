/* 
 * File:   Pin_Manager.h
 * Author: sandrabrittanysnozzi
 *
 * Created on January 18, 2018, 2:17 PM
 */

#ifndef PIN_MANAGER_H
#define	PIN_MANAGER_H
#include <xc.h> // include processor files - each processor file is guarded.

void digitalPinSet(int pin, int io);
void specifyAnalogPin(int pin, int analogOrDigital);
int digitalPinStatus(int pin);
void pinDirectionIO(int pin, int io);
void pinSampleSelectRegister(int pin);

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* PIN_MANAGER_H */

