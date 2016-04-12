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

#include "main.h"
#include "ConfigFile.h"
#include "gcParser.h"
#include "motion_planner.h"
#include "VC0706.h"
#include "calibrate.h"

#define HEADLESS

#ifndef HEADLESS
#include "libservodrv.h"
#endif

#define BUF_THRESH 799

typedef enum {READY, G1, STOPPING, G4, M1, JOGGING} state_T;
typedef enum {STOP_M1, STOP_G4, STOP_EOF} stop_T;
typedef enum {UNSET, JOG, CALIBRATE, RUN} cbot_run_mode;

using namespace std;

string sourceFile; //the gcode source file

state_T state; //the current state of the machine
stop_T stop; //stores impending stop states

cbot_run_mode mode; //machine mode

vector<cmd_t> cmds; //stores gcode commands the parser gives us
settings_t set; //stores machine settings from the config file

gcParser parse; //gcode parser

motion_planner *planner;

VC0706 *camera; //camera for calibration
string camera_port;

calibrate *calibrate; //image processing class for claibration

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
  
  camera_port                		= (string)cf.Value("MACHINE", "CAMERA_PORT");
}

int parseInput(int argc, char**argv){
    //get options
   int ix;
   int c;

   opterr = 0;

   while ((c = getopt (argc, argv, "jcr")) != -1)
     switch (c)
       {
       case 'j': //jog mode
           if(!mode)
                mode = JOG;
           else{
               fprintf(stderr, "invalid arguments specified\n");
               return 1;
           }
         break;
       case 'c': //calibrate mode
           if(!mode)
                mode = CALIBRATE;
           else{
               fprintf(stderr, "invalid arguments specified\n");
               return 1;
           }
         break;
       case 'r': //run mode
           if(!mode)
                mode = RUN;
           else{
               fprintf(stderr, "invalid arguments specified\n");
               return 1;
           }
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

//non blocking key read
int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
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

	if(planner->data_ready() || state == STOPPING || mode == JOG){
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
	 if( !infile.eof() && cmds.empty() && state != STOPPING){
		string line;
		while( 1 ){ //read till we get a valid block or hit EOF
			if( !getline(infile, line) ){
				if(mode != JOG){
					state = STOPPING;
					stop = STOP_EOF;
				}
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

	 if(infile.eof() && planner->empty()) return 1;
	 else return 0;
}

 int main (int argc, char **argv)
 {       
   readConfig(); //read config file
   
   if(parseInput(argc, argv))
       return 1;
   
   parse = gcParser();
   planner = new motion_planner(set);
   
#ifndef HEADLESS
   //we will use the servodrv hardware api
   int drv = servodrv_open();
   if(drv > -1){
	   cout << "servodrv opened successfully" << endl;
   }
   else{
	   cout << "failed to open servodrv!" << endl;
	   return 1;
   }
#endif
   
   switch(mode){  
   /* Auto-home the machine using the camera and image processing modules.
    * control loop will be:
    * Camera image --> image processing module --> gcode --> planner --> motion hardware
    */
       case RUN:
        if(sourceFile != ""){
            //open the source file and begin reading lines
            ifstream infile(sourceFile.c_str());

             while (!motion_loop(infile));
           }
           break;
       case JOG:
       {
           cout << "we are in jog mode!" << endl;
           ofstream jogfile("jogfile.gcode");
           ifstream infile("jogfile.gcode");

           float zPos = 0.0;
           float xPos = 0.0;
           float yPos = 0.0;

           float jogstep = 0.5;

           while(1){
        	   int c = getch();   // call your non-blocking input function
        	   if(c){
        		   switch(c){
        		   case 53: //page up, z axis up
        			   zPos += jogstep;
        			   jogfile << "G0 Z" << zPos << endl;
        			   break;
        		   case 54: //page down, z axis down
        			   zPos -= jogstep;
        			   jogfile << "G0 Z" << zPos << endl;
        			   break;
        		   case 65: //up arrow, y axis up
        			   yPos += jogstep;
        			   jogfile << "G0 Y" << yPos << endl;
        			   break;
        		   case 66: //down arrow, y axis down
        			   yPos -= jogstep;
        			   jogfile << "G0 Y" << yPos << endl;
        			   break;
        		   case 67: //right arrow, x axis up
        			   xPos += jogstep;
        			   jogfile << "G0 X" << xPos << endl;
        			   break;
        		   case 68: //left arrow, x axis down
        			   xPos -= jogstep;
        			   jogfile << "G0 X" << xPos << endl;
        			   break;
        		   default:
        			   //do nothing
        			   break;
        		   }
        	   }
        	   infile.sync();
        	   motion_loop(infile);
        	   infile.clear();
           }
       }
           break;
       case CALIBRATE:
           cout << "we are in calibrate mode" << endl;

            try {
                //open the camera hardware
                camera = new VC0706(camera_port);
                if(!camera->reset()){
                	cout << "camera reset failed!" << endl;
                	return 1;
                }
                char *version = camera->getVersion();
                if(version){
                	cout << version << endl;
                }
                else{
                	cout << "failed to get camera version!" << endl;
                }
                camera->setImageSize(VC0706_640x480);

            } catch(boost::system::system_error& e)
            {
                cout<<"Error: "<<e.what()<<endl;
                return 1;
            }
            /* TODO: uart is waaay slow lets throw some spi on this
            usleep(999999);

            if (! camera->takePicture())
                cout << "failed to take pic" << endl;
              else
                cout << "picture taken!" << endl;

            uint16_t jpglen = camera->frameLength();
            uint8_t buffer[jpglen];
            camera->readPicture(buffer, jpglen);

            ofstream jpg ("output.jpg",std::ofstream::binary);
            jpg.write((const char *)buffer, jpglen);
            jpg.close();
            */

           break;
   }
#ifndef HEADLESS
    //close the driver
    servodrv_close(drv);
#endif
    
   return 0;
}
