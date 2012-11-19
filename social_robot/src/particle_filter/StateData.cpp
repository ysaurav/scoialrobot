#include "StateData.h"
#include "../cv_utils.h"

using namespace std;
using namespace cv;

StateData::StateData ( )
{

}

void StateData::tracking()
{
  this->filter->update ( this->image, this->lbp, this->selection.size(), this->target_histogram, this->use_lbp );

  Size target_size ( this->target.cols, this->target.rows );

  // Draw particles
  if ( this->draw_particles )
    {
      this->filter->draw_particles ( this->image, target_size, WHITE );
    }

  // Draw estimated state with color based on confidence
  float confidence = this->filter->confidence();

  // TODO - Make these values not arbitrary
  if ( confidence > 0.1 )
    {
      this->filter->draw_estimated_state ( this->image, target_size, GREEN );
    }
  else if ( confidence > 0.025 )
    {
      this->filter->draw_estimated_state ( this->image, target_size, YELLOW );
    }
  else
    {
      this->filter->draw_estimated_state ( this->image, target_size, RED );
    }
}

void StateData::initialise ( int num_particles, bool use_lbp_, Mat image_, Rect selection_ )
{
  this->use_lbp = use_lbp_;
  this->paused = false ;
  this->draw_particles = false;
  this->filter = new ParticleFilter ( num_particles );
  image_.copyTo ( image );
  lbp  = Mat::zeros ( image.rows, image.cols, CV_8UC1 );
  selection = selection_;


  this->update_target_histogram ( );

  // Initialize condensation filter with center of selection
  this->filter->init ( this->selection );

  // Start video running if paused
  this->paused = false;
}

void StateData::update_target_histogram ( void )
{
  Mat roi ( this->image, this->selection ), lbp_roi ( this->lbp, this->selection );
  roi.copyTo ( this->target );
  Mat new_hist;
  float alpha = 0.2;

  calc_hist ( roi, lbp_roi, new_hist, use_lbp );
  normalize ( new_hist, new_hist );

  if ( this->target_histogram.empty() )
    {
      this->target_histogram = new_hist;
    }
  else
    {
      // TODO - support for adaptive updates not fully implemented.
      this->target_histogram = ( ( 1.f - alpha ) * this->target_histogram ) + ( alpha * new_hist );
      normalize ( this->target_histogram, this->target_histogram );
    }
  cout << "Target updated" << endl;
}

