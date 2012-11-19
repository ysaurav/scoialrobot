#ifndef STATEDATA_H
#define STATEDATA_H

#include "hist.h"
#include "filter.h"
#include "lbp.h"

using namespace std;
using namespace cv;

class StateData
  {
  public:
    Mat image;
    Mat lbp;
    Mat target;
    Mat target_histogram;
    Rect selection;
    bool use_lbp;
    bool paused;
    bool draw_particles;
    ParticleFilter *filter;

    StateData ( );
    void tracking ();
    void initialise ( int num_particles, bool use_lbp_, Mat image_, Rect selection_ );

  private:
    void update_target_histogram ( void );
  };

#endif // STATEDATA_H
