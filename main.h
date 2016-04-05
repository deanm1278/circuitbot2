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

/*
 * 360 degrees / 1.8 degrees per step = 200 steps per revolution
 * 200 steps / .0625 microsteps per step = 3200 microsteps
 * 3200 steps per rev / 1.5875 mm per rev (lead screw) = 2015.74803 steps per mm
 */
#define STEPS_PER_MM 2015.74803

//#define ENV_TEST

#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

