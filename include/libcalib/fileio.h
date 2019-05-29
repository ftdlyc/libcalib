/**
* Copyright 2019, ftdlyc <yclu.cn@gmail.com>
* Licensed under the MIT license.
*/

#pragma once
#ifndef CALIB_FILEIO_H
#define CALIB_FILEIO_H

#include <string>
#include <vector>

#include "libcalib/calib_def.h"

CALIB_DLL_DECL bool get_all_file(const char* path, const char* type, std::vector<std::string>& files);

#endif //CALIB_FILEIO_H
