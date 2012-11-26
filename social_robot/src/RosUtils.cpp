#include "RosUtils.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;

RegionOfInterest RosUtils::cvrect2rosroi ( Rect rect )
{
  RegionOfInterest roi;
  roi.x_offset = rect.x;
  roi.y_offset = rect.y;
  roi.width = rect.width;
  roi.height = rect.height;
  return roi;
}
/**<
* This function returns a given rectangle in a RegionOfInterest used in ROS for published.
* @return RegionOfInterest
* @param rect A Rect with location, width and height.
 */

Rect RosUtils::rosroi2cvrect ( RegionOfInterest roi )
{
  Rect rect;
  rect.x = roi.x_offset;
  rect.y = roi.y_offset;
  rect.width = roi.width;
  rect.height = roi.height;
  return rect;
}
/**<
* This function returns a rectangle from a given RegionOfInterest.
* @return Rect
* @param roi A RegionOfInterest with location, width and height.
 */

vector<RegionOfInterest> RosUtils::cvrects2rosrois ( vector< Rect > rects )
{
  unsigned int nrects = rects.size();
  vector<RegionOfInterest> rois ( nrects );
  for ( unsigned int i  = 0; i < nrects; i++ )
    {
      rois[i] = cvrect2rosroi ( rects[i] );
    }
  return rois;
}
/**<
* This function returns a given array of rectangles in an array of RegionOfInterest used in ROS for published.
* @return vector of RegionOfInterest
* @param rects A vector<Rect> with location, width and height.
 */

vector<Rect> RosUtils::rosrois2cvrects ( vector<RegionOfInterest> rois )
{
  unsigned int nrois = rois.size();
  vector<Rect> rects ( nrois );
  for ( unsigned int i  = 0; i < nrois; i++ )
    {
      rects[i] = rosroi2cvrect ( rois[i] );
    }
  return rects;
}
/**<
* This function returns an array of rectangles given an array of  RegionOfInterest.
* @return vector<Rect>
* @param rois A vector<RegionOfInterest> with location, width and height.
 */
