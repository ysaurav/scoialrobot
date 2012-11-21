#include "StateData.h"
#include "../cv_utils.h"

using namespace std;
using namespace cv;

StateData::StateData ( void )
{

}

void StateData::tracking ( void )
{
  filter->update ( image, lbp, selection.size(), target_histogram, use_lbp );
}

void StateData::initialise ( int num_particles, bool use_lbp_, Mat image_, Rect selection_ )
{
  use_lbp = use_lbp_;
  draw_particles = false;
  filter = new ParticleFilter ( num_particles );
  image_.copyTo ( image );
  lbp = Mat::zeros ( image.rows, image.cols, CV_8UC1 );
  selection = selection_;

  update_target_histogram();

  // Initialise condensation filter with center of selection
  filter->init ( selection );
}

Rect StateData::get_target_position ( void )
{  
  return filter->get_estimated_state();
}


void StateData::update_target_histogram ( void )
{
  Mat roi ( image, selection ), lbp_roi ( lbp, selection );
  roi.copyTo ( target );
  Mat new_hist;
  float alpha = 0.2;

  calc_hist ( roi, lbp_roi, new_hist, use_lbp );
  normalize ( new_hist, new_hist );

  if ( target_histogram.empty() )
    {
      target_histogram = new_hist;
    }
  else
    {
      // TODO - support for adaptive updates not fully implemented.
      target_histogram = ( ( 1.f - alpha ) * target_histogram ) + ( alpha * new_hist );
      normalize ( target_histogram, target_histogram );
    }
  cout << "Target updated" << endl;
}