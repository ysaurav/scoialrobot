#include "PixelSimilarity.h"

PixelSimilarity::PixelSimilarity( )
{
  point = cv::Point(0,0);
  radius = 0;
  similarity = 0;
}

PixelSimilarity::PixelSimilarity ( cv::Point point, float radius, float similarity )
{
  this->point = point;
  this->radius = radius;
  this->similarity = similarity;
}





