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
* This function returns a given rectangle in a regionofinterest used in ROS.
* @return 
* @param template2d A Mat containing a binary image.
* @param template3d A Mat containing a grayscale image.
* Notice that template3d is a grayscale image, it is used 3d when the matching is done with the depth image.
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
* Constructor given 2D and 3D templates.
* @param template2d A Mat containing a binary image.
* @param template3d A Mat containing a grayscale image.
* Notice that template3d is a grayscale image, it is used 3d when the matching is done with the depth image.
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
* Constructor given 2D and 3D templates.
* @param template2d A Mat containing a binary image.
* @param template3d A Mat containing a grayscale image.
* Notice that template3d is a grayscale image, it is used 3d when the matching is done with the depth image.
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
* Constructor given 2D and 3D templates.
* @param template2d A Mat containing a binary image.
* @param template3d A Mat containing a grayscale image.
* Notice that template3d is a grayscale image, it is used 3d when the matching is done with the depth image.
 */