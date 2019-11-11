#include "LinuxUART.h"

UART::UART(const char* port_, UART::BAUDRATE baudrate)
  : port(port_)
{

  file = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  if (tcgetattr(file, &serial) < 0)
    perror("Getting configuration");
  serial.c_iflag = 0;
  serial.c_oflag = 0;
  serial.c_lflag = 0;
  serial.c_cflag = 0;

  serial.c_cc[VMIN]  = 0;
  serial.c_cc[VTIME] = 0;

  serial.c_cflag = baudrate | CS8 | CREAD;
  tcsetattr(file, TCSANOW, &serial);
}

char
UART::readChar()
{
  return buffer[0];
}
bool
UART::upd()
{

  return (read(file, buffer, sizeof(char)) > 0);
}
void
UART::writeStr(char* str)
{
  write(file, str, strlen(str));
}

void
UART::endl()
{
  char* str = "\n";
  write(file, str, strlen(str));
}
