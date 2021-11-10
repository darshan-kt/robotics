// Get current date/time, format is YYYY-MM-DD.HH:mm:ss

#ifndef __DATE_TIME__
#define __DATE_TIME__
#include <chrono>  // chrono::system_clock
#include <ctime>   // localtime
#include <sstream> // stringstream
#include <iomanip> // put_time
#include <string>  // string
#include <iostream>

std::string CurrentDateTime();

#endif