#ifndef UTILS_H_
#define UTILS_H_

#include <ctime>
#include <sstream>
#include <iomanip>
#include <fstream>

class utils{
public:
	utils(){}
	~utils(){}
	std::string get_timestamp();
	std::string get_timestamp_filename();
};

#endif
