/* 
 * File:   ctrlr.cpp
 * Author: deanmiller
 * 
 * Created on November 24, 2015, 9:13 PM
 */

#include <vector>
#include <cstring>
#include <iostream>
#include "ctrlr.h"
#include "main.h"
#include "bus/SPIDevice.h"
#include "BBBio_lib/BBBiolib.h"

using namespace exploringBB;

ctrlr::ctrlr() : dev(1,0){ //use second SPI bus (both loaded)
    iolib_init();
    iolib_setdir(RDY_HEADER,RDY_PIN, BBBIO_DIR_IN);
    iolib_setdir(STOPPED_HEADER, STOPPED_PIN, BBBIO_DIR_IN);
    
    std::cout << "Starting SPI bus device..." << std::endl;
    dev.setSpeed(SPI_SPEED);      // Have access to SPI Device object
    dev.setMode(SPIDevice::MODE0);
    dev.writeRegister(BLINK_REG, 0x03); //blink 3 times to signal that we have connected
}

bool ctrlr::write(std::vector<float> &buf, unsigned char cmd){
    int b_len = buf.size() * sizeof(float);
    unsigned char mb[b_len + 1]; // +1 for the command
    mb[0] = cmd;
    memcpy(&mb[1],&buf[0],b_len); //copy starting after the command. All the floats need to be unsigned chars
    
    return 0;
};


bool ctrlr::write(unsigned int val, unsigned char cmd){
    int b_len = sizeof(int);
    unsigned char mb[b_len + 1]; //+1 for command
    mb[0] = cmd;
    memcpy(&mb[1],&val,b_len);
    
    
    return 0;
}

bool ctrlr::read(unsigned char* buf, unsigned int length){
    
    return 0;
}

bool ctrlr::ready(){
    //read the RDY pin
    return is_high(RDY_HEADER,RDY_PIN);
}

bool ctrlr::stopped(){
    //read the STOPPED pin
    return is_high(STOPPED_HEADER, STOPPED_PIN);
}

ctrlr::~ctrlr() {
    iolib_free();
    
}

