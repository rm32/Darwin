#include "DARwInOPDirectoryManager.hpp"

#include <cstdlib>

using namespace std;
using namespace managers;

extern "C" {
  char *wbu_system_getenv(const char *);
}

const string &DARwInOPDirectoryManager::getDataDirectory() {
#ifdef CROSSCOMPILATION
  static string path = "/darwin/Data/";
#else
  char *WEBOTS_MODULES_PATH = wbu_system_getenv("WEBOTS_MODULES_PATH");
  static string path = string(WEBOTS_MODULES_PATH) + "/projects/robots/darwin-op/libraries/darwin/darwin/Data/";
#endif
  return path;
}
