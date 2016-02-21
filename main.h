/* 
 * File:   main.h
 * Author: deanmiller
 *
 * Created on December 4, 2015, 7:06 PM
 */

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define NUM_AXIS 3
    
    //Compiler settings
#define HEADLESS
//#define ENV_TEST
    
typedef enum {READY, G1, STOPPING, G4, M1} state_T;
typedef enum {STOP_M1, STOP_G4, STOP_EOF} stop_T;

#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

