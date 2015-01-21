#include "patch2dTrack.h"

using namespace std;
using namespace ImageProcess;

Cpatch2dTrack::Cpatch2dTrack(void) {
}

Cpatch2dTrack::~Cpatch2dTrack() {
}

void Cpatch2dTrack::resize(const int fnum) {
  m_fnum = fnum;
  m_patch2ds.resize(m_fnum);
}

std::istream& ImageProcess::operator>>(std::istream& istr, Cpatch2dTrack& rhs) {
  string header;
  istr >> header;
  if (header != "PATCH2DTRACK") {
    cerr << "Header is not PATCH2DTRACK: " << header << endl;
    exit (1);
  }

  int fnum;
  istr >> fnum;
  rhs.resize(fnum);

  for (int f = 0; f < fnum; ++f)
    istr >> rhs.m_patch2ds[f];
  return istr;
}

std::ostream& ImageProcess::operator<<(std::ostream& ostr, Cpatch2dTrack& rhs) {
  ostr << "PATCH2DTRACK" << endl
       << (int)rhs.m_patch2ds.size() << endl;
  for (int f = 0; f < (int)rhs.m_patch2ds.size(); ++f)
    ostr << rhs.m_patch2ds[f] << flush;
  return ostr;
}
