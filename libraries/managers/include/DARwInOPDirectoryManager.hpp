// Description:   Helper class allowing to retrieve directories

#ifndef DARWINOP_DIRECTORY_MANAGER_HPP
#define DARWINOP_DIRECTORY_MANAGER_HPP

#include <string>

namespace managers {
  class DARwInOPDirectoryManager {
    public:
      static const std::string &getDataDirectory();
  };
}

#endif
