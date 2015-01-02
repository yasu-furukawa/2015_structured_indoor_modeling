#include <QApplication>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>

#include <iostream>
#include <fstream>
#include <string>

#include "configuration.h"

#ifndef QT_NO_OPENGL
#include "main_widget.h"
#endif

using namespace std;
using namespace structured_indoor_modeling;

int main(int argc, char *argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " configuration_file" << endl;
    exit (1);
  }
  Configuration configuration;
  ReadConfiguration(argv[1], &configuration);
  
  QApplication app(argc, argv);
  app.setApplicationName("viewer");
#ifndef QT_NO_OPENGL
  QWidget window;
  window.resize(1024, 720);
  // window.resize(1280, 720);
  //window.resize(1000, 600);

  /*
  QWidget* side_panel = new QWidget();
  QVBoxLayout* side_layout = new QVBoxLayout();
  {
    side_layout->addWidget(new QPushButton("one"));
    side_layout->addWidget(new QPushButton("two"));
    side_layout->addWidget(new QPushButton("three"));
    side_layout->addWidget(new QPushButton("four"));
  }
  side_layout->addWidget(side_panel);
  */

  MainWidget* main_widget = new MainWidget(configuration);
  QHBoxLayout* layout = new QHBoxLayout();
  {
    const int kMainIndex = 0;
    const int kMainSize = 5;
    layout->addWidget(main_widget);
    layout->setStretch(kMainIndex, kMainSize);

    /*
    const int kSideIndex = 1;
    const int kSideSize = 1;
    layout->addLayout(side_layout);
    layout->setStretch(kSideIndex, kSideSize);
    */
  }

  window.setLayout(layout);

  window.show();

#else
  QLabel note("OpenGL Support required");
  note.show();
#endif
  return app.exec();
}
