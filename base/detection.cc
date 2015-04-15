#include <fstream>
#include <iostream>
#include "detection.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

istream& operator >>(istream& istr, Detection& detection) {
  string header;
  istr >> header
       >> detection.panorama
       >> detection.us[0] >> detection.us[1]
       >> detection.vs[0] >> detection.vs[1];
  int num_names;
  istr >> num_names;
  detection.names.resize(num_names);
  for (int i = 0; i < num_names; ++i)
    istr >> detection.names[i];
  istr >> detection.score;

  if (header == "DETECTION_WITH_ICON") {
    istr >> detection.room >> detection.object;
    for (int a = 0; a < 3; ++a) {
      for (int i = 0; i < 2; ++i) {
        istr >> detection.ranges[a][i];
      }
    }
  }
  
  return istr;
}

ostream& operator <<(ostream& ostr, const Detection& detection) {
  ostr << "DETECTION_WITH_ICON" << endl
       << detection.panorama << endl
       << detection.us[0] << ' ' << detection.us[1] << endl
       << detection.vs[0] << ' ' << detection.vs[1] << endl
       << (int)detection.names.size();
  for (int i = 0; i < (int)detection.names.size(); ++i)
    ostr << ' ' << detection.names[i];
  ostr << endl
       << detection.score << endl
       << detection.room << ' ' << detection.object << endl;
  for (int a = 0; a < 3; ++a)
    ostr << detection.ranges[a][0] << ' ' << detection.ranges[a][1] << endl;
  ostr << "POLYGON" <<endl;
  ostr << detection.elist.size()<<endl;
  for(const auto&v: detection.elist)
       ostr<<detection.vlist[v[0]][0]<<' '<<detection.vlist[v[0]][1]<<' '<<
	    detection.vlist[v[1]][0]<<' '<<detection.vlist[v[1]][1]<<endl;
  return ostr;
}

istream& operator >>(istream& istr, vector<Detection>& detections) {
  string header;
  int num_detections;
  istr >> header >> num_detections;
  detections.clear();
  detections.resize(num_detections);
  for (int d = 0; d < num_detections; ++d)
    istr >> detections[d];
  
  return istr;
}
  
ostream& operator <<(ostream& ostr, const vector<Detection>& detections) {
  ostr << "DETECTIONS" << endl
       << (int)detections.size() << endl;
  for (const auto& detection : detections)
    ostr << detection << endl;

  return ostr;
}

}  // namespace structured_indoor_modeling
  
