#include "TransferWidget.hpp"
#include "Viewer.hpp"

using namespace webotsQtUtils;

Viewer::Viewer() :
  GenericWindow()
{
  mTabWidget->addTab(new TransferWidget(this), "Transfer");
}

Viewer::~Viewer() {
}
