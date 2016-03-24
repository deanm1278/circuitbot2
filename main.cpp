/* 
 * File:   main.cpp
 * Author: dean
 *
 * Created on November 24, 2015, 2:36 PM
 */

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iterator>

#include "main.h"
#include "ConfigFile.h"
#include "gcParser.h"
#include "motion_planner.h"

#define HEADLESS

#ifndef HEADLESS
#include "libservodrv.h"
#endif

#define BUF_THRESH 799

typedef enum {READY, G1, STOPPING, G4, M1} state_T;
typedef enum {STOP_M1, STOP_G4, STOP_EOF} stop_T;

using namespace std;

string sourceFile; //the gcode source file

state_T state; //the current state of the machine
stop_T stop; //stores impending stop states

vector<cmd_t> cmds; //stores gcode commands the parser gives us
settings_t set; //stores machine settings from the config file

gcParser parse; //gcode parser
motion_planner *planner;

float current_position[NUM_AXIS];
float destination[NUM_AXIS];
float feedrate;

void readConfig(void){ //use as template for our config file
  ConfigFile cf("config.txt");

  set.Linkage_1                     = (float)cf.Value("MACHINE", "LINK_1_LENGTH");
  set.Linkage_2                     = (float)cf.Value("MACHINE", "LINK_2_LENGTH");
  set.L1_2                          = set.Linkage_1 * set.Linkage_1;
  set.L2_2                          = set.Linkage_2 * set.Linkage_2;
  
  set.SCARA_offset_x                = (float)cf.Value("SETTINGS", "OFFSET_X");
  set.SCARA_offset_y                = (float)cf.Value("SETTINGS", "OFFSET_Y");
  set.axis_scaling[X_AXIS]          = (float)cf.Value("SETTINGS", "AXIS_SCALE_X");
  set.axis_scaling[Y_AXIS]          = (float)cf.Value("SETTINGS", "AXIS_SCALE_Y");
  
  set.Sm                            = (float)cf.Value("MACHINE", "JOUNCE_MAX");
  set.Jm                            = (float)cf.Value("MACHINE", "JERK_MAX");
  set.Am                            = (float)cf.Value("MACHINE", "ACCELERATION_MAX");
  set.M                             = (float)cf.Value("MACHINE", "MASS");
  set.T                             = (float)cf.Value("MACHINE", "INTERPOLATION_PERIOD");
  set.Vm                            = (float)cf.Value("MACHINE", "VELOCITY_MAX");
  set.Fmax                          = (float)cf.Value("MACHINE", "IMPULSE_MAX");
}

int parseInput(int argc, char**argv){
    //get options
   int bflag = 0;
   int sflag = 0;
   int ix;
   int c;

   opterr = 0;

   while ((c = getopt (argc, argv, "bs")) != -1)
     switch (c)
       {
       case 'b':
         bflag = 1;
         break;
       case 's':
         sflag = 1;
         break;
       case '?':
         if (isprint (optopt))
           fprintf (stderr, "Unknown option `-%c'.\n", optopt);
         else
           fprintf (stderr,
                    "Unknown option character `\\x%x'.\n",
                    optopt);
         return 1;
       default:
         abort ();
       }

   //printf ("bflag = %d, sflag = %d\n", bflag, sflag);
   
   ix = optind;
   if(argv[ix] != NULL){ //argv ends with null
    sourceFile = string(argv[ix]);
   }
   else{ sourceFile = "gctest.gcode"; }
   
   return 0;
      
}

bool processCommand(cmd_t c){
    //Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G4  - Dwell S<seconds> or P<milliseconds>
// G28 - Home all Axis
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given (in raw SCARA coordinates)

// M Codes
// M0   - Unconditional stop - Wait for user interaction
// M1   - Same as M0
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
    int num = (int) c.number;
    switch(c.letter){
        case 'G':
            switch(num){
                case 0:         // G0  -> G1
                case 1:{         // G1  - Coordinated Movement X Y Z
                    state = G1;
                    if ( c.params.find('X') != c.params.end() ) {
                        destination[X_AXIS] = c.params['X'];
                      }
                    else{ destination[X_AXIS] = current_position[X_AXIS]; }
                    if ( c.params.find('Y') != c.params.end() ) {
                        destination[Y_AXIS] = c.params['Y'];
                      }
                    else{ destination[Y_AXIS] = current_position[Y_AXIS]; }
                    if ( c.params.find('Z') != c.params.end() ) {
                        destination[Z_AXIS] = c.params['Z'];
                      }
                    else{ destination[Z_AXIS] = current_position[Z_AXIS]; }
                    if ( c.params.find('F') != c.params.end() ){ feedrate = c.params['F']; }
                    
                    vector<float> v(destination, destination + sizeof destination / sizeof destination[0]);
                    if( planner->plan_buffer(v, feedrate) ){
                        copy(begin(destination), end(destination), current_position);
                        break;
                    }
                    else { return 0; } //the plan buffer was full and we could not process the command
                }
                case 4:         // G4  - Dwell S<seconds> or P<milliseconds>
                    state = STOPPING;
                    stop = STOP_G4;
                    break;
                case 28:        // G28 - Home all Axis
                    break;
                case 90:        // G90 - Use Absolute Coordinates
                    break;
                case 91:        // G91 - Use Relative Coordinates
                    break;
                case 92:        // G92 - Set current position to cordinates given (in raw SCARA coordinates)
                    break;
                default:
                    //unimplemented function. do nothing
                    break;
            }
            break;
        case 'M':
            switch(num){
                case 0:         // M0   - Unconditional stop - Wait for user interaction  
                case 1:         // M1   - Same as M0
                    state = STOPPING;
                    stop = STOP_M1;
                    break;
                case 17:        // M17  - Enable/Power all stepper motors
                    break;
                case 18:        // M18  - Disable all stepper motors; same as M84
                    break;
                default:
                    //unimplemented function. do nothing
                    break;
            }
            break;
        case NO_LETTER:
            if(state == G1){
                    if ( c.params.find('X') != c.params.end() ) {
                        destination[X_AXIS] = c.params['X'];
                      }
                    else{ destination[X_AXIS] = current_position[X_AXIS]; }
                    if ( c.params.find('Y') != c.params.end() ) {
                        destination[Y_AXIS] = c.params['Y'];
                      }
                    else{ destination[Y_AXIS] = current_position[Y_AXIS]; }
                    if ( c.params.find('Z') != c.params.end() ) {
                        destination[Z_AXIS] = c.params['Z'];
                      }
                    else{ destination[Z_AXIS] = current_position[Z_AXIS]; }
                    if ( c.params.find('F') != c.params.end() ){ feedrate = c.params['F']; }
                    
                    vector<float> v(destination, destination + sizeof destination / sizeof destination[0]);
                    if( planner->plan_buffer(v, feedrate) ){
                        copy(begin(destination), end(destination), current_position);
                        break;
                    }
                    else { return 0; } //the plan buffer was full and we could not process the command
            }
            break;
        default:
            //should never get here
            break;
    }
    return 1;
}

 int main (int argc, char **argv)
 {
   readConfig(); //read config file
   parseInput(argc, argv);
   parse = gcParser();
   planner = new motion_planner(set);
#ifndef HEADLESS

   //we will use the servodrv hardware api=
   int drv = servodrv_open();
   if(drv > -1){
	   cout << "servodrv opened successfully" << endl;
   }
   else{
	   cout << "failed to open servodrv!" << endl;
	   return 1;
   }
#endif
   
   //do something based on the input arguments (set config values, test device available, etc.)
   
   
    if(sourceFile != ""){
        //open the source file and begin reading lines
        ifstream infile(sourceFile.c_str());

        string line;
        bool eof;

        eof = false;
        /*
         * Main loop. This should be the delegate between the different modules. No module should do things like
         * wait in an empty while loop for something or lock the application up for any reason. Modules should be 
         * kept as standalone as possible
         * 
         * - Gcode parser deals with gcode file and parsing
         * - Motion planner deals with velocity profile generation and interpolation of movements
         * 
         */
               
         while (!(planner->empty() && eof))
         {
             if(planner->data_ready() || state == STOPPING){
            	//check how many spaces are left in the buffer
#ifndef HEADLESS
            	int avail = servodrv_avail(drv);
#else
                int avail = BUF_THRESH + 1;
#endif
                int num;
                 
            	if(avail > BUF_THRESH){
            		//allocate buffer for elements
            		uint16_t to_write[avail * NUM_AXIS];
                        //ask the planner for that many elements
                        num = planner->interpolate((uint32_t)avail, to_write);

#ifndef HEADLESS
                        //write to the hardware
                        servodrv_write(drv, to_write, num * sizeof(uint16_t) * NUM_AXIS);
#endif
                        cout << "wrote " << num << endl;
            	}
             }

             //try to read in a new set of commands
             if( !eof && cmds.empty() && state != STOPPING){
                while( 1 ){ //read till we get a valid block or hit EOF
                    if( !getline(infile, line) ){
                        eof = true;
                        state = STOPPING;
                        stop = STOP_EOF;
                        break;
                    }
                    else if( parse.parseBlock(line, cmds) ){
                        break;
                    }
                }
             }

             //if there are unprocessed commands in the cmds buffer, try to process them
             if(!cmds.empty() && state != STOPPING){
                vector<cmd_t>::iterator v = cmds.begin();
                while( v != cmds.end()) {
                   cmd_t c = *v;
                   if ( processCommand(c) ){
                       v = cmds.erase(v);
                   }
                   else{
                       break;
                   }
                }
             }

             //--- PROCESS STOP STATES ---//
             else if(state == STOPPING){
                 if( planner->empty() ){
                        switch(stop){
                                case STOP_M1:
                                    state = M1;
                                    break;
                                case STOP_G4:
                                    state = G4;
                                    break;
                        }

                 }

             }

             if(state == M1){
                 cout << "Machine stopped, press enter to continue...";

                 cin.ignore().get(); //Pause Command for Linux Terminal
                 state = READY;
             }
         }
       }
#ifndef HEADLESS
    //close the driver
    servodrv_close(drv);
#endif
    
   return 0;
}
