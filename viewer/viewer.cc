#include <QApplication>
#include <QHBoxLayout>
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
  // window.resize(1000, 600);

  MainWidget* main_widget = new MainWidget(configuration);

  QHBoxLayout* layout = new QHBoxLayout();
  layout->addWidget(main_widget);
  window.setLayout(layout);
  window.show();

#else
  QLabel note("OpenGL Support required");
  note.show();
#endif
  return app.exec();
}
