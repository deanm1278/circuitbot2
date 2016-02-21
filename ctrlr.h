/* 
 * File:   ctrlr.h
 * Author: deanmiller
 *
 * Created on November 24, 2015, 9:13 PM
 */

#ifndef CTRLR_H
#define	CTRLR_H

#include <vector>
#include "bus/SPIDevice.h"

#define RDY_HEADER      8
#define RDY_PIN         11

#define STOPPED_HEADER  8
#define STOPPED_PIN     12

#define SPI_SPEED       4000000

#define BLINK_REG       0x02

class ctrlr{
    
private:
        
        bool RDY; //gets set once the hardware indicates it's ready to receive another block
    
public:
    ctrlr();
    bool write(std::vector<float> &buf, unsigned char cmd);
    bool write(unsigned char* buf);
    bool write(unsigned int, unsigned char cmd);
    
    bool read(unsigned char* buf, unsigned int length);
    bool ready();
    bool stopped();
    virtual ~ctrlr();
    
private:
    exploringBB::SPIDevice dev;
};

#endif	/* CTRLR_H */

