#ifndef ROSUTILS_H
#define ROSUTILS_H

/** 
* @class RosUtils
*
* @brief This class is used for publishing the ouput rectangles in ROS. 
* 
* @author Social Robot
* 
*/

#include <sensor_msgs/RegionOfInterest.h>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;
using namespace sensor_msgs;

class RosUtils
  {
  public:
    RegionOfInterest cvrect2rosroi ( Rect rect );
    Rect rosroi2cvrect ( RegionOfInterest roi );
    vector<RegionOfInterest> cvrects2rosrois ( vector<Rect> rects );
    vector<Rect> rosrois2cvrects ( vector<RegionOfInterest> rois );
  };

#endif // ROSUTILS_H
