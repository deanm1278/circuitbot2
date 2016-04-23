/*
 * VC0706.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: mostly ladyada, modified by deanm1278
 *
 *      Derived from the adafruit VC0706 library found here:
 *      https://github.com/adafruit/Adafruit-VC0706-Serial-Camera-Library/blob/master/VC0706.h
 */

#include <boost/asio.hpp>
#include <boost/optional.hpp>
#include "VC0706.h"


//#define DEBUG

VC0706::VC0706(std::string port, unsigned int baud_rate)
: io(), serial(io,port){

	serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	frameptr  = 0;
	bufferLen = 0;
	serialNum = 0;
}

bool VC0706::reset() {

  uint8_t args[] = {0x0};

  return runCommand(VC0706_RESET, args, 1, 5);
}

bool VC0706::motionDetected() {
  if (readResponse(4, 200) != 4) {
    return false;
  }
  if (! verifyResponse(VC0706_COMM_MOTION_DETECTED))
    return false;

  return true;
}


bool VC0706::setMotionStatus(uint8_t x, uint8_t d1, uint8_t d2) {
  uint8_t args[] = {0x03, x, d1, d2};

  return runCommand(VC0706_MOTION_CTRL, args, sizeof(args), 5);
}


uint8_t VC0706::getMotionStatus(uint8_t x) {
  uint8_t args[] = {0x01, x};

  return runCommand(VC0706_MOTION_STATUS, args, sizeof(args), 5);
}


bool VC0706::setMotionDetect(bool flag) {
  if (! setMotionStatus(VC0706_MOTIONCONTROL, 
			VC0706_UARTMOTION, VC0706_ACTIVATEMOTION))
    return false;

  uint8_t args[] = {0x01, flag};
  
  return runCommand(VC0706_COMM_MOTION_CTRL, args, sizeof(args), 5);
}



bool VC0706::getMotionDetect() {
  uint8_t args[] = {0x0};

  if (! runCommand(VC0706_COMM_MOTION_STATUS, args, 1, 6))
    return false;

  return camerabuff[5];
}

uint8_t VC0706::getImageSize() {
  uint8_t args[] = {0x4, 0x4, 0x1, 0x00, 0x19};
  if (! runCommand(VC0706_READ_DATA, args, sizeof(args), 6))
    return -1;

  return camerabuff[5];
}

bool VC0706::setImageSize(uint8_t x) {
  uint8_t args[] = {0x05, 0x04, 0x01, 0x00, 0x19, x};

  return runCommand(VC0706_WRITE_DATA, args, sizeof(args), 5);
}

/****************** downsize image control */

uint8_t VC0706::getDownsize() {
  uint8_t args[] = {0x0};
  if (! runCommand(VC0706_DOWNSIZE_STATUS, args, 1, 6)) 
    return -1;

   return camerabuff[5];
}

bool VC0706::setDownsize(uint8_t newsize) {
  uint8_t args[] = {0x01, newsize};

  return runCommand(VC0706_DOWNSIZE_CTRL, args, 2, 5);
}

/***************** other high level commands */

char * VC0706::getVersion() {

  uint8_t args[] = {0x01};
  
  sendCommand(VC0706_GEN_VERSION, args, 1);
  // get reply
  if (!readResponse(CAMERABUFFSIZ, 200))
    return 0;
  camerabuff[bufferLen] = 0;  // end it!
  return (char *)camerabuff;  // return it!
}


/***************** baud rate commands */

char* VC0706::setBaud9600() {
  uint8_t args[] = {0x03, 0x01, 0xAE, 0xC8};

  sendCommand(VC0706_SET_PORT, args, sizeof(args));
  // get reply
  if (!readResponse(CAMERABUFFSIZ, 200)) 
    return 0;
  camerabuff[bufferLen] = 0;  // end it!
  return (char *)camerabuff;  // return it!

}

char* VC0706::setBaud19200() {
  uint8_t args[] = {0x03, 0x01, 0x56, 0xE4};

  sendCommand(VC0706_SET_PORT, args, sizeof(args));
  // get reply
  if (!readResponse(CAMERABUFFSIZ, 200)) 
    return 0;
  camerabuff[bufferLen] = 0;  // end it!
  return (char *)camerabuff;  // return it!
}

char* VC0706::setBaud38400() {
  uint8_t args[] = {0x03, 0x01, 0x2A, 0xF2};

  sendCommand(VC0706_SET_PORT, args, sizeof(args));
  // get reply
  if (!readResponse(CAMERABUFFSIZ, 200)) 
    return 0;
  camerabuff[bufferLen] = 0;  // end it!
  return (char *)camerabuff;  // return it!
}

char* VC0706::setBaud57600() {
  uint8_t args[] = {0x03, 0x01, 0x1C, 0x1C};

  sendCommand(VC0706_SET_PORT, args, sizeof(args));
  // get reply
  if (!readResponse(CAMERABUFFSIZ, 200)) 
    return 0;
  camerabuff[bufferLen] = 0;  // end it!
  return (char *)camerabuff;  // return it!
}

char* VC0706::setBaud115200() {
  uint8_t args[] = {0x03, 0x01, 0x0D, 0xA6};

  sendCommand(VC0706_SET_PORT, args, sizeof(args));
  // get reply
  if (!readResponse(CAMERABUFFSIZ, 200)) 
    return 0;
  camerabuff[bufferLen] = 0;  // end it!
  return (char *)camerabuff;  // return it!
}

/****************** high level photo comamnds */

void VC0706::OSD(uint8_t x, uint8_t y, char *str) {
  if (strlen(str) > 14) { str[13] = 0; }

  uint8_t args[17] = {static_cast<uint8_t>(strlen(str)), static_cast<uint8_t>(strlen(str)-1), static_cast<uint8_t>((y & 0xF) | ((x & 0x3) << 4))};

  for (uint8_t i=0; i<strlen(str); i++) {
    char c = str[i];
    if ((c >= '0') && (c <= '9')) {
      str[i] -= '0';
    } else if ((c >= 'A') && (c <= 'Z')) {
      str[i] -= 'A';
      str[i] += 10;
    } else if ((c >= 'a') && (c <= 'z')) {
      str[i] -= 'a';
      str[i] += 36;
    }

    args[3+i] = str[i];
  }

   runCommand(VC0706_OSD_ADD_CHAR, args, strlen(str)+3, 5);
   printBuff();
}

bool VC0706::setCompression(uint8_t c) {
  uint8_t args[] = {0x5, 0x1, 0x1, 0x12, 0x04, c};
  return runCommand(VC0706_WRITE_DATA, args, sizeof(args), 5);
}

uint8_t VC0706::getCompression(void) {
  uint8_t args[] = {0x4, 0x1, 0x1, 0x12, 0x04};
  runCommand(VC0706_READ_DATA, args, sizeof(args), 6);
  printBuff();
  return camerabuff[5];
}

bool VC0706::setPTZ(uint16_t wz, uint16_t hz, uint16_t pan, uint16_t tilt) {
  uint8_t args[] = {0x08, static_cast<uint8_t>(wz >> 8), static_cast<uint8_t>(wz), 
		    static_cast<uint8_t>(hz >> 8), static_cast<uint8_t>(wz), 
		    static_cast<uint8_t>(pan>>8), static_cast<uint8_t>(pan), 
		    static_cast<uint8_t>(tilt>>8), static_cast<uint8_t>(tilt)};

  return (! runCommand(VC0706_SET_ZOOM, args, sizeof(args), 5));
}


bool VC0706::getPTZ(uint16_t &w, uint16_t &h, uint16_t &wz, uint16_t &hz, uint16_t &pan, uint16_t &tilt) {
  uint8_t args[] = {0x0};
  
  if (! runCommand(VC0706_GET_ZOOM, args, sizeof(args), 16))
    return false;
  printBuff();

  w = camerabuff[5];
  w <<= 8;
  w |= camerabuff[6];

  h = camerabuff[7];
  h <<= 8;
  h |= camerabuff[8];

  wz = camerabuff[9];
  wz <<= 8;
  wz |= camerabuff[10];

  hz = camerabuff[11];
  hz <<= 8;
  hz |= camerabuff[12];

  pan = camerabuff[13];
  pan <<= 8;
  pan |= camerabuff[14];

  tilt = camerabuff[15];
  tilt <<= 8;
  tilt |= camerabuff[16];

  return true;
}


bool VC0706::takePicture() {
  frameptr = 0;
  return cameraFrameBuffCtrl(VC0706_STOPCURRENTFRAME); 
}

bool VC0706::resumeVideo() {
  return cameraFrameBuffCtrl(VC0706_RESUMEFRAME); 
}

bool VC0706::TVon() {
  uint8_t args[] = {0x1, 0x1};
  return runCommand(VC0706_TVOUT_CTRL, args, sizeof(args), 5);
}
bool VC0706::TVoff() {
  uint8_t args[] = {0x1, 0x0};
  return runCommand(VC0706_TVOUT_CTRL, args, sizeof(args), 5);
}

bool VC0706::cameraFrameBuffCtrl(uint8_t command) {
  uint8_t args[] = {0x1, command};
  return runCommand(VC0706_FBUF_CTRL, args, sizeof(args), 5);
}

uint32_t VC0706::frameLength(void) {
  uint8_t args[] = {0x01, 0x00};
  if (!runCommand(VC0706_GET_FBUF_LEN, args, sizeof(args), 9))
    return 0;

  uint32_t len;
  len = camerabuff[5];
  len <<= 8;
  len |= camerabuff[6];
  len <<= 8;
  len |= camerabuff[7];
  len <<= 8;
  len |= camerabuff[8];

  return len;
}


uint8_t VC0706::available(void) {
  return bufferLen;
}

//TODO: This obviously wont work.
bool VC0706::readPicture(uint8_t *buffer, uint8_t len) {
  uint8_t args[] = {0x0C, 0x0, 0x0F,
                    0, 0, static_cast<uint8_t>(frameptr >> 8), static_cast<uint8_t>(frameptr & 0xFF), 
                    0, 0, 0, len,
                    CAMERADELAY >> 8, CAMERADELAY & 0xFF};

  if (! runCommand(VC0706_READ_FBUF, args, sizeof(args), 5, false))
    return 0;

  if (readResponse(len, 200, buffer) == 0)
      return 0;

  frameptr += len;

  return 0;
}

// ********** LOW LEVEL COMMANDS ********** //

bool VC0706::runCommand(uint8_t cmd, uint8_t *args, uint8_t argn,
		   uint8_t resplen, bool flushflag) {

	// flush out anything in the buffer?
	  if (flushflag) {
		readResponse(100, 10);
	  }

	  sendCommand(cmd, args, argn);
	  if (readResponse(resplen, 200) != resplen)
		return false;
	  if (! verifyResponse(cmd))
            return false;
          return true;

}
bool VC0706::runCommand(uint8_t cmd, uint8_t *args, uint8_t argn,
                               uint8_t resplen){
    return runCommand(cmd, args, argn, resplen, 1);
}

void VC0706::sendCommand(uint8_t cmd, uint8_t args[] = 0, uint8_t argn = 0) {
	//for some reason I don't understand we need to flush this buffer
	std::flush( std::cout );

	char toWrite[3 + argn];
	toWrite[0] = VC0706_RECEIVE_SIGN;
	toWrite[1] = serialNum;
	toWrite[2] = cmd;
	memcpy(args, &toWrite[3], argn * sizeof(uint8_t));

	boost::asio::write(serial, boost::asio::buffer(toWrite, sizeof(uint8_t) * (3 + argn)));
}

uint16_t VC0706::readResponse(uint16_t numbytes, uint8_t timeout, uint8_t *buffer) {
    //reset buffer length
    bufferLen = 0;
    
    //set the timer
    boost::optional<boost::system::error_code> timer_result;
    boost::asio::deadline_timer timer(serial.get_io_service());
    timer.expires_from_now(boost::posix_time::milliseconds(timeout));
    timer.async_wait([&timer_result] (const boost::system::error_code& error) { timer_result.reset(error); });
    
    //read bytes into the camera buffer, set buffer length
    boost::optional<boost::system::error_code> read_result;
    boost::asio::async_read(serial, boost::asio::buffer(buffer, numbytes), [&read_result, this] (const boost::system::error_code& error, size_t transferred) { read_result.reset(error); bufferLen = transferred; });

    //read till timeout or read limit reached
    serial.get_io_service().reset();
    while (serial.get_io_service().run_one())
    { 
        if (read_result)
            timer.cancel();
        else if (timer_result){
            serial.cancel();
        }
    }
    return bufferLen;
}

uint8_t VC0706::readResponse(uint16_t numbytes, uint8_t timeout) {
	return static_cast<uint8_t>(readResponse(numbytes, timeout, camerabuff));
}

bool VC0706::verifyResponse(uint8_t command) {
  if ((camerabuff[0] != VC0706_RETURN_SIGN) ||
      (camerabuff[1] != serialNum) ||
      (camerabuff[2] != command) ||
      (camerabuff[3] != VC0706_SUCCESS_CODE))
      return false;
  return true;
  
}

void VC0706::printBuff() {
  for (uint8_t i = 0; i< bufferLen; i++) {
      std::cout << " 0x" << std::hex << camerabuff[i];
  }
  std::cout << std::endl;
}

VC0706::~VC0706() {
	// TODO Auto-generated destructor stub
}

