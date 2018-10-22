#include <string>
#include <sstream>

#include "lib/common.h"

#pragma once

namespace scene3d {
bool EndsWith(std::string const &full_string, std::string const &ending);
bool StartsWith(std::string const &full_string, std::string const &prefix);
std::string toString(const Eigen::MatrixXd &mat);
}