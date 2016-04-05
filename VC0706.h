/*
 * VC0706.h
 *
 *  Created on: Apr 4, 2016
 *      Author: Dean Miller
 *
 *      Derived from the adafruit VC0706 library found here:
 *      https://github.com/adafruit/Adafruit-VC0706-Serial-Camera-Library/blob/master/Adafruit_VC0706.h
 */

#include <boost/asio.hpp>

#ifndef VC0706_H_
#define VC0706_H_

#define VC0706_RESET  0x26
#define VC0706_GEN_VERSION 0x11
#define VC0706_SET_PORT 0x24
#define VC0706_READ_FBUF 0x32
#define VC0706_GET_FBUF_LEN 0x34
#define VC0706_FBUF_CTRL 0x36
#define VC0706_DOWNSIZE_CTRL 0x54
#define VC0706_DOWNSIZE_STATUS 0x55
#define VC0706_READ_DATA 0x30
#define VC0706_WRITE_DATA 0x31
#define VC0706_COMM_MOTION_CTRL 0x37
#define VC0706_COMM_MOTION_STATUS 0x38
#define VC0706_COMM_MOTION_DETECTED 0x39
#define VC0706_MOTION_CTRL 0x42
#define VC0706_MOTION_STATUS 0x43
#define VC0706_TVOUT_CTRL 0x44
#define VC0706_OSD_ADD_CHAR 0x45

#define VC0706_STOPCURRENTFRAME 0x0
#define VC0706_STOPNEXTFRAME 0x1
#define VC0706_RESUMEFRAME 0x3
#define VC0706_STEPFRAME 0x2

#define VC0706_640x480 0x00
#define VC0706_320x240 0x11
#define VC0706_160x120 0x22

#define VC0706_MOTIONCONTROL 0x0
#define VC0706_UARTMOTION 0x01
#define VC0706_ACTIVATEMOTION 0x01

#define VC0706_SET_ZOOM 0x52
#define VC0706_GET_ZOOM 0x53

#define CAMERABUFFSIZ 100
#define CAMERADELAY 10

class VC0706 {
public:
	VC0706(std::string port, unsigned int baud_rate);
	bool begin();
	bool reset();
	bool runCommand(uint8_t cmd, uint8_t *args, uint8_t argn,
				   uint8_t resplen, bool flushflag);
    void sendCommand(uint8_t cmd, uint8_t args[], uint8_t argn);
    uint8_t readResponse(uint8_t numbytes, uint8_t cmd);
	virtual ~VC0706();
private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
    uint8_t  serialNum;
    uint8_t  camerabuff[CAMERABUFFSIZ+1];
    uint8_t  bufferLen;
    uint16_t frameptr;
};

#endif /* VC0706_H_ */
