#ifndef __COLOR_H
#define __COLOR_H

#include <iostream>
#include <string>

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

void error_message(std::string str);
void warning_message(std::string str);
void important_message(std::string str);

#endif