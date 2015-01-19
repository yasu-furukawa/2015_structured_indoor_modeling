#include <fstream>
#include "patch2dTracker.h"

using namespace ImageProcess;
using namespace Image;
using namespace std;

Cpatch2dTracker::Cpatch2dTracker(void) {
  pthread_rwlock_init(&m_rwlock, NULL);    
}

Cpatch2dTracker::~Cpatch2dTracker() {
  pthread_rwlock_destroy(&m_rwlock);    
}

void Cpatch2dTracker::init(const std::string prefix,
			   const int firstFrame, const int lastFrame,
			   const int refFrame, const int maxLevel,
			   const int margin, const int usedetect,
			   const float inccThreshold) {
  m_prefix = prefix;
  m_firstFrame = firstFrame;
  m_lastFrame = lastFrame;
  m_refFrame = refFrame;
  m_maxLevel = maxLevel;
  m_margin = margin;
  m_inccThreshold = inccThreshold;

  if (m_firstFrame < m_refFrame) {
    cerr << "Invalid frame numbers" << endl;
    exit (1);
  }

  if (usedetect && (refFrame + 1) != firstFrame) {
    cerr << "When usedetect, refFrame must be firstFrame" << endl;
    exit (1);
  }
  
  //?????
  // imdface
  //m_halfsize = 3;
  // flag
  //m_halfsize = 5;
  // book
  m_halfsize = 13;
  
  char iprefix[1024];
  sprintf(iprefix, "%simages/", prefix.c_str());
  m_movie.init(iprefix, iprefix, maxLevel, lastFrame);
  
  // read from a key point file at firstFrame
  if (usedetect) {
    char buffer[1024];
    sprintf(buffer, "%s%04d.key", prefix.c_str(), refFrame);
    readKey(buffer);
  }
  else {
    ifstream ifstr;
    char buffer[1024];
    sprintf(buffer, "%sobject.tracks", prefix.c_str());
    ifstr.open(buffer);
    ifstr >> *this;
    ifstr.close();
  }
}

void Cpatch2dTracker::write(void) {
  char buffer[1024];
  sprintf(buffer, "%sobject.tracks", m_prefix.c_str());

  ofstream ofstr;
  ofstr.open(buffer);
  ofstr << *this;
  ofstr.close();
}

std::istream& ImageProcess::operator>>(std::istream& istr, Cpatch2dTracker& rhs) {
  string header;
  istr >> header;
  if (header != "PATCH2DTRACKS") {
    cerr << "Invalid file format: " << header << endl;
    exit (1);
  }

  // number of tracks
  int num;
  istr >> num;
  rhs.m_patch2dTracks.resize(num);

  for (int i = 0; i < num; ++i)
    istr >> rhs.m_patch2dTracks[i];
  
  return istr;
}

std::ostream& ImageProcess::operator<<(std::ostream& ostr, Cpatch2dTracker& rhs) {
  ostr << "PATCH2DTRACKS" << endl;
  ostr << (int)rhs.m_patch2dTracks.size() << endl;
  for (int i = 0; i < (int)rhs.m_patch2dTracks.size(); ++i) {
    ostr << rhs.m_patch2dTracks[i] << endl;
  }
  return ostr;
}

void Cpatch2dTracker::readKey(const std::string file) {
  ifstream ifstr;
  ifstr.open(file.c_str());

  int num, itmp;
  ifstr >> num >> itmp;
  m_patch2dTracks.resize(num);

  for (int i = 0; i < num; ++i) {
    float ftmp[5];
    for (int j = 0; j < 5; ++j)
      ifstr >> ftmp[j];
    
    m_patch2dTracks[i].resize(m_lastFrame);
    getPatch2d(i, m_refFrame).m_center[0] = ftmp[0];
    getPatch2d(i, m_refFrame).m_center[1] = ftmp[1];
    getPatch2d(i, m_refFrame).m_score = 1.0f;

    //?????
    // scale axis
    const float scale = 1.0;
    getPatch2d(i, m_refFrame).m_xaxis *= scale;
    getPatch2d(i, m_refFrame).m_yaxis *= scale;
  }
  ifstr.close();
}

void Cpatch2dTracker::optimize(const int id,
			       const Cpatch2d& pre, Cpatch2d& cur,
			       const Image::Cimage& preimage,
			       const Image::Cimage& curimage) {
  cur.m_score = 2.0;
  for (int level = m_maxLevel - 1; 0 <= level; --level) {
    //?????
    cur.m_score =
      //m_patch2dOptimizer.optimizePos(pre, cur,
      m_patch2dOptimizer.optimize(pre, cur,
				  preimage, curimage,
				  m_halfsize, level, id);

    if (cur.m_score == 2.0)
      return;
  }
}

void Cpatch2dTracker::run(void) {
  // Start tracking from m_firstFrame+1 -> m_lastFrame
  m_movie.alloc(m_refFrame);

  if (m_refFrame < m_firstFrame - 1)
    m_movie.alloc(m_firstFrame - 1);

  for (m_fnum = m_firstFrame; m_fnum < m_lastFrame; ++m_fnum) {    
    m_patch2dOptimizer.initStats();

    //?????
    // Use the previous frame in the new tracking algorithm.
    // For final tightening, do not free any frames.
    //if (m_refFrame < m_fnum - 2)
    //m_movie.free(m_fnum - 2);
    
    m_movie.alloc(m_fnum);
    cerr << m_fnum << endl;

    m_count = 0;
    pthread_t threads[m_patch2dOptimizer.m_CPU];
    for (int i = 0; i < m_patch2dOptimizer.m_CPU; ++i)
      pthread_create(&threads[i], NULL, runThreadTmp, (void*)this);
    for (int i = 0; i < m_patch2dOptimizer.m_CPU; ++i)
      pthread_join(threads[i], NULL);     

    int success = 0;    int total = 0;
    for (int i = 0; i < (int)m_patch2dTracks.size(); ++i) {
      if (getPatch2d(i, m_fnum).m_score != 2.0)
	success++;
      total++;
    }
    
    cerr << "Success: " << success << '/' << total << endl;
    m_patch2dOptimizer.showStats();
  }

  //???????
  //tighten();
}

void Cpatch2dTracker::filter(const float passratio) {
  vector<Cpatch2dTrack> vc;
  for (int t = 0; t < (int)m_patch2dTracks.size(); ++t) {
    int pass = 0;
    int total = 0;
    for (int i = 0; i < (int)m_patch2dTracks[t].m_patch2ds.size(); ++i) {
      if (m_patch2dTracks[t].m_patch2ds[i].isTracked())
	pass++;
      total++;
    }
    if (total * passratio <= pass)
      vc.push_back(m_patch2dTracks[t]);
  }
  vc.swap(m_patch2dTracks);
}

void* Cpatch2dTracker::runThreadTmp(void* arg) {
  Cpatch2dTracker* patch2dTracker = (Cpatch2dTracker*)arg;  
  patch2dTracker->runThread();
  return NULL;
}

void Cpatch2dTracker::runThread(void) {
  pthread_rwlock_wrlock(&m_rwlock);
  const int id = m_count++;
  pthread_rwlock_unlock(&m_rwlock);

  const int itmp = (int)ceil((int)m_patch2dTracks.size()
			     / (float)m_patch2dOptimizer.m_CPU);
  const int begin = id * itmp;
  const int end = min((int)m_patch2dTracks.size(), (id + 1) * itmp);

  for (int i = begin; i < end; ++i) {
    const Cpatch2d& ref = getPatch2d(i, m_refFrame);
    const Cpatch2d& pre = getPatch2d(i, m_fnum - 1);
    Cpatch2d& cur = getPatch2d(i, m_fnum);
    cur.m_score = 2.0;
    
    const float step = 0x0001 << (m_maxLevel - 1);
      
    for (int y = -m_margin; y <= m_margin; ++y) {
      for (int x = -m_margin; x <= m_margin; ++x) {
	Cpatch2d curtmp = pre;
	curtmp.m_center[0] += x * step;
	curtmp.m_center[1] += y * step;

	optimize(id, pre, curtmp,
		 m_movie.m_images[m_fnum - 1],
		 m_movie.m_images[m_fnum]);

	// We should use thresholds
	//????? if enforce threshold, put something here
	if (curtmp.m_score < m_inccThreshold && curtmp.m_score < cur.m_score)
	  cur = curtmp;
      }
    }

    if (cur.m_score == 2.0)
      continue;

    optimize(id, ref, cur, m_movie.m_images[m_refFrame], m_movie.m_images[m_fnum]);
  }
  
  /*
  pthread_rwlock_wrlock(&m_rwlock);
  const int id = m_count++;
  pthread_rwlock_unlock(&m_rwlock);

  const int itmp = (int)ceil((int)m_patch2dTracks.size()
			     / (float)m_patch2dOptimizer.m_CPU);
  const int begin = id * itmp;
  const int end = min((int)m_patch2dTracks.size(), (id + 1) * itmp);

  for (int i = begin; i < end; ++i) {
    const Cpatch2d& pre = getPatch2d(i, m_refFrame);
    Cpatch2d& cur = getPatch2d(i, m_fnum);
    cur.m_score = 2.0;
    
    const float step = 0x0001 << (m_maxLevel - 1);
      
    for (int y = -m_margin; y <= m_margin; ++y) {
      for (int x = -m_margin; x <= m_margin; ++x) {
	Cpatch2d curtmp = getPatch2d(i, m_fnum - 1);	  
	curtmp.m_center[0] += x * step;
	curtmp.m_center[1] += y * step;

	optimize(id, pre, curtmp,
		 m_movie.m_images[m_refFrame],
		 m_movie.m_images[m_fnum]);

	//???????? if enforce threshold, put something here
	if (curtmp.m_score < cur.m_score)
	  cur = curtmp;
      }
    }
  }
  */
}

void Cpatch2dTracker::tighten(void) {
  const int level = 0;
  for (int t = 0; t < (int)m_patch2dTracks.size(); ++t) {
    //m_firstFrame, m_lastFrame
    m_patch2dOptimizer.optimize(m_patch2dTracks[t],
				m_movie.m_images,
				m_firstFrame, m_lastFrame, m_refFrame,
				m_halfsize, level);
  }
}
