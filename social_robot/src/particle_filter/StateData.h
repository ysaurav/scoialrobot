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
    bool draw_particles;
    ParticleFilter *filter;

    StateData ( void );
    void tracking ( void );
    void initialise ( int num_particles, bool use_lbp_, Mat image_, Rect selection_ );
    Rect get_target_position (void );

  private:
    void update_target_histogram ( void );
  };

#endif // STATEDATA_H
