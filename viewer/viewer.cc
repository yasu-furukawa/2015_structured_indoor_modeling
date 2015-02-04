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
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    exit (1);
  }
  QApplication app(argc, argv);
  app.setApplicationName("viewer");

#ifndef QT_NO_OPENGL
  QWidget window;
  window.resize(1024, 720);
  // window.resize(1280, 720);
  // window.resize(1000, 600);

  string data_directory;
  for (int i = 1; i < argc; ++i) {
    string word = argv[i];
    /*
    if (word[word.length() - 1] == '\\')
      data_directory = data_directory + word.substr(0, word.length() - 1);
    else
      data_directory = data_directory + word;
    */
    data_directory = data_directory + word;    
    if (i != argc - 1)
      data_directory = data_directory + string(" ");
  }
  cout << argc << "-" << data_directory << "-" << endl;
  
  Configuration configuration;
  {
    configuration.data_directory = data_directory;
    const double kAirAngle = 60.0 * M_PI / 180.0;
    const double kAirFieldOfViewDegrees = 10.0;
    const double kFloorplanAngle = 80.0 * M_PI / 180.0;
    const double kFloorplanFieldOfViewDegrees = 10.0;
    configuration.air_angle = kAirAngle;
    configuration.air_field_of_view_degrees = kAirFieldOfViewDegrees;
    configuration.floorplan_angle = kFloorplanAngle;
    configuration.floorplan_field_of_view_degrees = kFloorplanFieldOfViewDegrees;
  }
  
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
