/*
 * VC0706.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: debian
 */

#include <boost/asio.hpp>
#include <string.h>
#include "VC0706.h"

VC0706::VC0706(std::string port, unsigned int baud_rate)
: io(), serial(io,port){
	serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	frameptr  = 0;
	bufferLen = 0;
	serialNum = 0;
}

bool VC0706::runCommand(uint8_t cmd, uint8_t *args, uint8_t argn,
		   uint8_t resplen, bool flushflag) {
	// flush out anything in the buffer?
	  if (flushflag) {
		readResponse(100, 10);
	  }

	  sendCommand(cmd, args, argn);
	  if (readResponse(resplen, 200) != resplen)
		return false;
	  return true;

}


void VC0706::sendCommand(uint8_t cmd, uint8_t args[] = 0, uint8_t argn = 0) {
	uint8_t c = 0x56;
	boost::asio::write(serial, boost::asio::buffer(&c, sizeof(uint8_t)));
	boost::asio::write(serial, boost::asio::buffer(&serialNum, sizeof(uint8_t)));
	boost::asio::write(serial, boost::asio::buffer(&cmd, sizeof(uint8_t)));

	boost::asio::write(serial, boost::asio::buffer(args, sizeof(uint8_t) * argn));
}

uint8_t VC0706::readResponse(uint8_t numbytes, uint8_t cmd) {
	size_t len = 4;
	unsigned char uc[] = {0x76, serialNum, cmd, 0x0};
	std::string s( reinterpret_cast<char const*>(uc), len ) ;
	boost::asio::streambuf b;
	boost::asio::read_until(serial, b, s);
	memcpy(camerabuff, boost::asio::buffer_cast<const void*>(b.data()), b.size());

	return b.size();
}

VC0706::~VC0706() {
	// TODO Auto-generated destructor stub
}

