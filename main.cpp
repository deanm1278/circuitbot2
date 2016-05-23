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
#include <sstream>
#include <vector>
#include <cmath>
#include <iterator>
#include <algorithm>
#include "boost/circular_buffer.hpp"

#include "main.h"
#include "ConfigFile.h"
#include "gcParser.h"
#include "motion_planner.h"
#include "calibrate.h"

#define HEADLESS

#ifndef HEADLESS
#include "libservodrv.h"
#endif

#define BUF_THRESH 799
#define INTERP_MAX 50

typedef enum {READY, G1, STOPPING, G4, M1, JOGGING} state_T;
typedef enum {STOP_M1, STOP_G4, STOP_EOF} stop_T;

using namespace std;

state_T state; //the current state of the machine
stop_T stop; //stores impending stop states

float conversion;
float scale;

boost::circular_buffer<cmd_t> cmds; //stores gcode commands the parser gives us
settings_t settings; //stores machine settings from the config file

gcParser parse; //gcode parser

motion_planner *planner;

calibrate *cal;

int drv; //File descriptor for servodrv

int camera;
int board_height;
int board_width;

float current_position[NUM_AXIS];
float destination[NUM_AXIS];
float feedrate;

void readConfig(void){ //use as template for our config file
  ConfigFile cf("config.txt");

  settings.Linkage_1                     = (float)cf.Value("MACHINE", "LINK_1_LENGTH");
  settings.Linkage_2                     = (float)cf.Value("MACHINE", "LINK_2_LENGTH");
  settings.L1_2                          = settings.Linkage_1 * settings.Linkage_1;
  settings.L2_2                          = settings.Linkage_2 * settings.Linkage_2;
  
  settings.SCARA_offset_x                = (float)cf.Value("SETTINGS", "OFFSET_X");
  settings.SCARA_offset_y                = (float)cf.Value("SETTINGS", "OFFSET_Y");
  settings.axis_scaling[X_AXIS]          = (float)cf.Value("SETTINGS", "AXIS_SCALE_X");
  settings.axis_scaling[Y_AXIS]          = (float)cf.Value("SETTINGS", "AXIS_SCALE_Y");
  
  settings.Sm                            = (float)cf.Value("MACHINE", "JOUNCE_MAX");
  settings.Jm                            = (float)cf.Value("MACHINE", "JERK_MAX");
  settings.Am                            = (float)cf.Value("MACHINE", "ACCELERATION_MAX");
  settings.M                             = (float)cf.Value("MACHINE", "MASS");
  settings.T                             = (float)cf.Value("MACHINE", "INTERPOLATION_PERIOD");
  settings.Vm                            = (float)cf.Value("MACHINE", "VELOCITY_MAX");
  settings.Fmax                          = (float)cf.Value("MACHINE", "IMPULSE_MAX");
  settings.steps_per_mm                  = (float)cf.Value("MACHINE", "STEPS_PER_MM");
  settings.ticks_per_radian              = (float)cf.Value("MACHINE", "TICKS_PER_RADIAN");
  
  camera	                        = (int)cf.Value("CAMERA", "CAMERA");
  board_height						= (int)cf.Value("CAMERA", "BOARD_HEIGHT");
  board_width						= (int)cf.Value("CAMERA", "BOARD_WIDTH");
}

int processCommand(cmd_t c){
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
                        destination[X_AXIS] = c.params['X'] * conversion * scale;
                      }
                    else{ destination[X_AXIS] = current_position[X_AXIS]; }
                    if ( c.params.find('Y') != c.params.end() ) {
                        destination[Y_AXIS] = c.params['Y'] * conversion * scale;
                      }
                    else{ destination[Y_AXIS] = current_position[Y_AXIS]; }
                    if ( c.params.find('Z') != c.params.end() ) {
                        destination[Z_AXIS] = c.params['Z'] * conversion * scale;
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
                case 20:        // G20 - Use inches
                    conversion = 25.4;
                    break;
                case 21:        // G21 - Use millimeters
                    conversion = 1.0;
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
                    cout << "error: unrecognizd command" << endl;
                    return -1;
                    break;
            }
            break;
        case 'M':
            switch(num){
                case 0:         // M0   - Unconditional stop - Wait for user interaction  
                case 1:         // M1   - Same as M0
                    state = STOPPING;
                    stop = STOP_M1;
                    planner->min_buf_len = 1;
                    break;
                case 17:        // M17  - Enable/Power all stepper motors
                    break;
                case 18:        // M18  - Disable all stepper motors; same as M84
                    break;
                default:
                    //unimplemented function. do nothing
                    cout << "error: unrecognizd command" << endl;
                    return -1;
                    break;
            }
            break;
        case NO_LETTER:
            if(state == G1){
                    if ( c.params.find('X') != c.params.end() ) {
                        destination[X_AXIS] = c.params['X'] * conversion * scale;
                      }
                    else{ destination[X_AXIS] = current_position[X_AXIS]; }
                    if ( c.params.find('Y') != c.params.end() ) {
                        destination[Y_AXIS] = c.params['Y'] * conversion * scale;
                      }
                    else{ destination[Y_AXIS] = current_position[Y_AXIS]; }
                    if ( c.params.find('Z') != c.params.end() ) {
                        destination[Z_AXIS] = c.params['Z'] * conversion * scale;
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
            else{
                cout << "error: unrecognizd command" << endl;
                return -1;
            }
            break;
        default:
            //should never get here
            cout << "error: unrecognizd command" << endl;
            return -1;
            break;
    }
    return 1;
}

/*
 * Main loop. This should be the delegate between the different modules. No module should do things like
 * wait in an empty while loop for something or lock the application up for any reason. Modules should be
 * kept as standalone as possible
 *
 * - Gcode parser deals with gcode file and parsing
 * - Motion planner deals with velocity profile generation and interpolation of movements
 * - Servodrv driver deals with output of data to the motion control hardware
 *
 * return 0 unless EOF is reached and the planner is empty.
 * Takes a stream buffer as input
 */
int motion_loop(istream& infile){

	if(planner->data_ready()){
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
			num = planner->interpolate(min(avail, INTERP_MAX), to_write);

			if(num > 0){
#ifndef HEADLESS
				//write to the hardware
				servodrv_write(drv, to_write, num * sizeof(uint16_t) * NUM_AXIS);
#endif
				cout << "wrote " << num << endl;
			}
		}
	 }

        //try to read in a new set of commands
        if( !infile.eof() && cmds.empty() ){
               string line;
               while( 1 ){ //read till we get a valid block or hit EOF
                       if( !getline(infile, line) ){
                               break;
                       }
                       else if( parse.parseBlock(line, cmds) > -1){
                               break;
                       }
                       else{
                           return 1;
                       }
               }
        }
       //we are at end of file. set min buf len (lookahead size) to 1 to flush out all remaining items
       if(infile.eof()){
           planner->min_buf_len = 1;
       }

	 //if there are unprocessed commands in the cmds buffer, try to process them
	 if(!cmds.empty() && state != STOPPING){
		boost::circular_buffer<cmd_t>::iterator iter = cmds.begin();
		while( iter != cmds.end()) {
		   cmd_t c = *iter;
                   iter++;
		   if ( processCommand(c) ){
                       cmds.pop_front();
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

	 if(infile.eof() && planner->empty()) return 1;
	 else return 0;
}

//send a single gcode command
int send(string input){
    istringstream is(input);
    while(!motion_loop(is));
    return 0;
}

//run a gcode file
int run(string sourceFile){
    ifstream infile(sourceFile.c_str());
    while (!motion_loop(infile));
    return 0;
}

//run calibration routine
int run_calibration(){
	float error[2];
	cal = new calibrate(camera, board_height, board_width);

	cal->find_error(error);
	return 0;
}

vector<string> &parse_input(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    getline(ss, item, delim);
    elems.push_back(item);
    
    getline(ss, item);
    elems.push_back(item);
    return elems;
}

 int main (int argc, char **argv)
 {       
   readConfig(); //read config file
   
   parse = gcParser();
   planner = new motion_planner(settings);
   cmds = boost::circular_buffer<cmd_t>(10);
   conversion = 1.0; //default to mm
   scale = 1;
   
#ifndef HEADLESS
   //we will use the servodrv hardware api
   drv = servodrv_open();
   if(drv > -1){
	   cout << "servodrv opened successfully" << endl;
   }
   else{
	   cout << "failed to open servodrv!" << endl;
	   return 1;
   }
#endif
   
   bool running = true;
   string input;
   while(running){
       cout << "circuitbot> ";
       getline(cin, input);
       
       //split off the command and then do the relevant thing
        vector<string> parts;
        parse_input(input, ' ', parts);

        string c = parts.at(0);

        if (c == "send" || c == "s") {
            send(parts.at(1));
        }
        else if (c == "run") {
            run(parts.at(1));
        }
        else if(c == "calibrate"){
        	run_calibration();
        }
        else if (c == "help") {
            cout << "Circuitbot CNC program:" << endl;
            cout << "Copyright (C) 2016 Dean Miller" << endl;
            cout << "GNU GPL license. This is free software. Do what you want, but you may not sell it in any form." << endl;
            cout << "There is NO WARRANTY, to the extent permitted by law" << endl << endl;
            cout << "Commands: " << endl;
            cout << "send [gcode line] - sends a single line of gcode" << endl;
            cout << "run [filename] - run a gcode program from a file" << endl;
            cout << "set [field] [value] - set the specified setting to the specified value" << endl;
            cout << "calibrate - calibrate the machine. Camera must be positioned over the calibration target." << endl;
            cout << "quit - quit the program" << endl;
        } else if (c == "quit" || c == "q") {
            running = false;
        } else {
            cout << "unrecognized command. Type 'help' for available commands" << endl;
        }
       
       cout << endl;
   }
#ifndef HEADLESS
    //close the driver
    servodrv_close(drv);
#endif
    
   return 0;
}
