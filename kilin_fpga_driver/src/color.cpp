#include <color.hpp>

void error_message(std::string str) { std::cout << red << str << reset << std::endl; }

void warning_message(std::string str) { std::cout << yellow << str << reset << std::endl; }

void important_message(std::string str) { std::cout << green << str << reset << std::endl; }
