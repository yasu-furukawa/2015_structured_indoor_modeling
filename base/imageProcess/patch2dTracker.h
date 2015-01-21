#ifndef IMAGEPROCESS_PATCH2DTRACKER_H
#define IMAGEPROCESS_PATCH2DTRACKER_H

#include <vector>
#include <string>
#include "patch2dTrack.h"
#include "patch2dOptimizer.h"
#include "../image/movie.h"

namespace ImageProcess {

class Cpatch2dTracker {
 public:
  Cpatch2dTracker(void);
  virtual ~Cpatch2dTracker();

  virtual void init(const std::string prefix,
		    const int firstFrame, const int lastFrame,
		    const int refFrame, const int maxLevel,
		    const int margin, const int usedetect,
		    const float inccThreshold);
  
  virtual void run(void);
  // Filter out tracks which success rate is below passratio.
  virtual void filter(const float passratio = 1.0f);
  void write(void);
  
  inline Cpatch2d& getPatch2d(const int index,
			      const int frame) {
    return m_patch2dTracks[index].m_patch2ds[frame];
  }
  
  inline const Cpatch2d& getPatch2d(const int index,
				    const int frame) const {
    return m_patch2dTracks[index].m_patch2ds[frame];
  }

 protected:
  void optimize(const int id,
		const Cpatch2d& pre, Cpatch2d& cur,
		const Image::Cimage& preimage,
		const Image::Cimage& curimage);
  
  void readKey(const std::string file);

  void tighten(void);
  
  // A set of tracks
  std::vector<Cpatch2dTrack> m_patch2dTracks;
  // Affine tracker engine
  Cpatch2dOptimizer m_patch2dOptimizer;
  // Image sequence
  Image::Cmovie m_movie;

  // prefix to the data folder
  std::string m_prefix;

  // threshold
  float m_inccThreshold;
  
  // how many search area
  int m_margin;
  
  // current frame num
  int m_fnum;
  // frame number
  int m_firstFrame;
  int m_lastFrame;
  int m_refFrame;
  // max level
  int m_maxLevel;
  // half size
  int m_halfsize;

  // Threads
  pthread_rwlock_t m_rwlock;
  int m_count;
  
  void runThread(void);
  static void* runThreadTmp(void*arg);  

  friend std::istream& operator>>(std::istream& istr, Cpatch2dTracker& rhs);
  friend std::ostream& operator<<(std::ostream& ostr, Cpatch2dTracker& rhs); 
};

std::istream& operator>>(std::istream& istr, Cpatch2dTracker& rhs);
std::ostream& operator<<(std::ostream& ostr, Cpatch2dTracker& rhs); 
 
};

#endif //IMAGEPROCESS_PATCH2DTRACKER_H
