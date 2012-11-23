#ifndef STATEDATA_H
#define STATEDATA_H

#include "hist.h"
#include "filter.h"

using namespace std;
using namespace cv;

class StateData
  {
  public:
    Mat image;
    Mat image_depth;
    Mat target;
    Mat target_histogram;
    Rect selection;
    double detection_confidence;
    int hist_type;
    bool is_associated;
    ParticleFilter *filter;

    StateData ( void );
    void tracking ( double cost = 0.01 );
    void initialise ( int num_particles, Mat image_, Rect selection_, Mat image_depth_, int hist_type_ );
    Rect get_target_position (void );
    void update_target_histogram ( Mat& newimage, Mat& newdepth, Rect new_selection );

  private:    
  };

#endif // STATEDATA_H
