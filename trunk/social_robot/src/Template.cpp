#include "Template.h"

Template::Template( )
{

}

Template::Template( cv::Mat template2d, cv::Mat template3d )
{
  this->template2d = template2d;
  this->template3d = template3d;
}



