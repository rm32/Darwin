// Description:   Manage the entree point function

#include "VisualTracking.hpp"

#include <cstdlib>

using namespace webots;

int main(int argc, char **argv)
{
  VisualTracking *controller = new VisualTracking();
  controller->run();
  delete controller;
  return EXIT_FAILURE;
}
