#pragma once

#include "LinuxUART.h"
#include "LinuxChrono.h"

#include <regex>
#include <functional>



class LLHparser
{
public:
  struct LLHdata
  {
    int16_t YYYY   = 0;
    int16_t MM     = 0;
    int16_t DD     = 0;
    int16_t hh     = 0;
    int16_t mm     = 0;
    int16_t ss     = 0;
    double  lat    = 0;
    double  lng    = 0;
    double  height = 0;
    int8_t  Q      = 0;
    int16_t ns     = 0;
    double  sdn    = 0;
    double  sde    = 0;
    double  sdu    = 0;
    double  sdne   = 0;
    double  sdeu   = 0;
    double  sdun   = 0;
    double  age    = 0;
    double  ratio  = 0;


    void    print()
    {
  
    }
  };

  static const uint8_t bufferSize = 180;
  enum UpdateMode
  {
    automatic,
    manual
  };

  LLHparser(const char* ttyPath, UpdateMode mode_);
  LLHdata       getData();
  unsigned long lastUpd();
  bool          manualUpd();

private:
  bool                 upd();
  void*                loop(int);
  pthread_mutex_t      mutex;
  LLHdata              data;
  UART                 serial;
  bool                 readChar(char c);
  void                 clearBuffer();
  char                 buffer[bufferSize + 1];
  common_things::Alarm alarm;
  common_things::Time  t;
  UpdateMode           mode;
};












