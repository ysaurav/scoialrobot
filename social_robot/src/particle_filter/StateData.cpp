#include "StateData.h"
#include "../CvUtils.h"

using namespace std;
using namespace cv;

StateData::StateData ( void )
{

}

void StateData::tracking ( double cost )
{
  is_associated = false;
  detection_confidence = detection_confidence - cost;
  filter->update ( image, image_depth, selection.size(), target_histogram, hist_type );
}

void StateData::initialise ( int num_particles, Mat image_, Rect selection_, Mat image_depth_, int hist_type_ )
{
  is_associated = true;
  filter = new ParticleFilter ( num_particles );
  image_.copyTo ( image );
  image_depth_.copyTo ( image_depth );
  selection = selection_;
  hist_type = hist_type_;
  detection_confidence = 1.0;

  update_target_histogram ( image, image_depth, selection );

  // Initialise condensation filter with center of selection
  filter->init ( selection );
}

Rect StateData::get_target_position ( void )
{
  return filter->get_estimated_state();
}

void StateData::update_target_histogram ( Mat& newimage, Mat& newdepth, Rect new_selection )
{
  selection = new_selection;
  Mat roi ( newimage, selection ), depth_roi ( newdepth, selection );
  roi.copyTo ( target );

  calc_hist ( roi, depth_roi, target_histogram, hist_type );
  normalize ( target_histogram, target_histogram );  
}
