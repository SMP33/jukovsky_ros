#include "RegexEmlidReachParser.h"

void*
LLHparser::loop(int)
{
  while (serial.upd())
    readChar(serial.readChar());
}
bool
LLHparser::readChar(char c)
{
  for (size_t i = 0; i < bufferSize - 1; i++)
  {
    buffer[i] = buffer[i + 1];
  }
  buffer[bufferSize - 1] = c;

  bool result = false;
  if (c == '\n')
  {
    result = upd();
    clearBuffer();
  }
  return result;
}

LLHparser::LLHdata
LLHparser::getData()
{
  pthread_mutex_lock(&mutex);
  auto res = data;
  pthread_mutex_unlock(&mutex);
  return res;
}

void
LLHparser::clearBuffer()
{
  for (size_t i = 0; i < bufferSize; i++)
  {
    buffer[i] = '_';
  }
  buffer[bufferSize] = '\0';
}

bool
LLHparser::upd()
{
  auto m = std::cmatch{};

  char buffer_[bufferSize + 1];
  for (size_t i = 0; i < bufferSize; i++)
  {
    buffer_[i] = buffer[i];
  }
  buffer_[bufferSize - 1] = '\0';
  buffer_[bufferSize - 2] = '\n';
  // std::cout << buffer_ << std::endl;
  if (regex_match(buffer_, m, std::regex{ R"((_+)(..../../.*\n))" }))
  {
    std::string llh    = m[2].str();
    std::string space2 = "  ";
    size_t      pos    = llh.find(space2);
    while (pos != std::string::npos)
    {
      llh.replace(pos, space2.length(), " ");
      pos = llh.find(space2);
    }

    llh.replace(4, 1, " ");
    llh.replace(7, 1, " ");

    llh.replace(13, 1, " ");
    llh.replace(16, 1, " ");

    llh.replace(llh.length() - 1, 1, " ");

    std::string delim(" ");
    size_t      prev = 0;
    size_t      next;
    size_t      delta = delim.length();

    std::vector<std::string> buf;
    while ((next = llh.find(delim, prev)) != std::string::npos)
    {
      std::string tmp = llh.substr(prev, next - prev);
      buf.push_back(llh.substr(prev, next - prev));
      prev = next + delta;
    }

    pthread_mutex_lock(&mutex);
    data.YYYY   = stoi(buf[0]);
    data.MM     = stoi(buf[1]);
    data.DD     = stoi(buf[2]);
    data.hh     = stoi(buf[3]);
    data.mm     = stoi(buf[4]);
    data.ss     = stoi(buf[5]);
    data.lat    = stod(buf[6]);
    data.lng    = stod(buf[7]);
    data.height = stod(buf[8]);
    data.Q      = stoi(buf[9]);
    data.Q += 1;
    data.Q -= 1;
    data.ns    = stoi(buf[10]);
    data.sdn   = stod(buf[11]);
    data.sde   = stod(buf[12]);
    data.sdu   = stod(buf[13]);
    data.sdne  = stod(buf[14]);
    data.sdeu  = stod(buf[15]);
    data.sdun  = stod(buf[16]);
    data.age   = stod(buf[17]);
    data.ratio = stod(buf[18]);

    t.reset();
    pthread_mutex_unlock(&mutex);
    return true;
  }
  return false;
}

LLHparser::LLHparser(const char* ttyPath, UpdateMode mode_)
  : serial(ttyPath, UART::BR38400)
{
  t.reset();
  pthread_mutex_init(&mutex, NULL);
  clearBuffer();
  mode = mode_;
  if (mode == UpdateMode::automatic)
  {
    auto f = std::bind(&LLHparser::loop, this, std::placeholders::_1);
    alarm.start(f, 20000, 1);
  }
}

unsigned long
LLHparser::lastUpd()
{
  pthread_mutex_lock(&mutex);
  auto time = t.millis();
  pthread_mutex_unlock(&mutex);
  return time;
}

bool
LLHparser::manualUpd()
{
  while (serial.upd())
    if (readChar(serial.readChar()))
      return true;
	
	return false;
}