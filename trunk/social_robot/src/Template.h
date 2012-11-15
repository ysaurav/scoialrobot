#ifndef TEMPLATE_H
#define TEMPLATE_H

#include <opencv2/core/core.hpp>

class Template
{
public:
  cv::Mat template2d;
  cv::Mat template3d;
  
  Template( );
  Template( cv::Mat, cv::Mat ); 
};

#endif // TEMPLATE_H
