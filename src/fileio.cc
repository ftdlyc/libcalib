/**
* Copyright 2019, ftdlyc <yclu.cn@gmail.com>
* Licensed under the MIT license.
*/

#include "libcalib/fileio.h"

#ifdef __linux__

#include <algorithm>
#include <dirent.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#endif
#ifdef _WIN32

#include <algorithm>
#include <io.h>
#include <string>
#include <vector>

#endif

#include <regex>
#include <string>
#include <vector>

#ifdef __linux__

bool get_all_file(const char* path, const char* pattern, std::vector<std::string>& files) {
  DIR* dir;
  if(!(dir = opendir(path))) {
    return false;
  };
  files.clear();
  struct dirent* p_dirent;
  std::regex reg(pattern);
  std::string path_str(path);
  if(path_str[path_str.size()] != '/') {
    path_str += "/";
  }
  while((p_dirent = readdir(dir))) {
    if(p_dirent->d_type != 4) {
      if(std::regex_match(p_dirent->d_name, reg)) {
        files.emplace_back(path_str + std::string(p_dirent->d_name));
      }
    }
  }
  std::sort(files.begin(), files.end());
  closedir(dir);
  return true;
}

#endif

#ifdef _WIN32

bool get_all_file(const char* path, const char* type, std::vector<std::string>& files) {
  intptr_t hFile = 0;
  struct _finddata_t fileinfo;
  files.clear();
  if((hFile = _findfirst(std::string(path).append("/*.").append(type).c_str(), &fileinfo)) != -1) {
    do {
      if(!(fileinfo.attrib & _A_SUBDIR)) {
        files.push_back(std::string(path).append("/").append(fileinfo.name));
      }
    } while(_findnext(hFile, &fileinfo) == 0);
    _findclose(hFile);
    std::sort(files.begin(), files.end());
    return true;
  } else {
    return false;
  }
}

#endif
