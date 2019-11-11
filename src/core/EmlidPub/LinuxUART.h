#pragma once
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

class UART
{
	
	
private:
  const char*	 port;
  struct termios serial;
  int            file;
  char           buffer[1];
public:
	 enum BAUDRATE
	{
		BR38400 = 0000017,
		BR19200 = 0000016,
		BR9600 = 0000015,
		BR4800 = 0000014,
		BR2400 = 0000013,
		BR1800 = 0000012,
		BR1200 = 0000011
	}
	;
	UART(const char* port_, UART::BAUDRATE baudrate);
  bool upd();
  char readChar();
  void writeStr(char* str);
  void endl();
	
};

