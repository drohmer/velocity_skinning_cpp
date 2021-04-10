#pragma once

#include <string>
#include <vector>
#include <fstream>
#include "vcl/base/error/error.hpp"

namespace vcl
{

/** Ensure that a file already exists and is accessible
 * Display an error message with some reminder on path setting in case the file cannot be accessed */
void assert_file_exist(const std::string& filename);

/** Return true if the file can be accessed, false otherwise */
bool check_file_exist(const std::string filename);

/** Read a file given by its path and return its content as a string */
std::string read_file_text(const std::string& filename);

/** Read a file and store all its lines as vector of string */
std::vector<std::string> read_file_text_lines(const std::string& filename);

}






