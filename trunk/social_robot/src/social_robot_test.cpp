#include <ros/ros.h>

#include "kinect_proxy.h"
#include "PixelSimilarity.h"
#include "Template.h"
#include "CvUtils.h"

#define HEAD_TEMPLATE1 "pictures/template3.png"
#define HEAD_TEMPLATE2 "pictures/template.png"
#define NUM_TEMPLATES 2
#define HEAD_TEMPLATE3D2 "pictures/template3D.png"
#define HEAD_TEMPLATE3D "pictures/template3D.png"
#define THR1          5
#define THR2          7
#define THR3          20
#define SCALES        5
#define SCALE_FACTOR  0.75//sqrt(2)

using namespace std;
using namespace cv;

string cascade_name = "rsrc/haarcascades/haarcascade_frontalface_alt.xml";
CvUtils cv_utils;

const static Scalar colors[] =
{
  CV_RGB ( 0, 0, 255 ),
  CV_RGB ( 0, 128, 255 ),
  CV_RGB ( 0, 255, 255 ),
  CV_RGB ( 0, 255, 0 ),
  CV_RGB ( 255, 128, 0 ),
  CV_RGB ( 255, 255, 0 ),
  CV_RGB ( 255, 0, 0 ),
  CV_RGB ( 255, 0, 255 )
};

Mat canny_im;

bool selectObject = false;
int trackObject = 0;
Point origin;
Rect selection;
Mat image_rgb;

int main ( int argc, char **argv )
{
  for ( int i = 1; i < 7; i++ )
    {
      string dispname = "pictures/disparity0";
      dispname.append(inttostr(i));
      dispname.append(".png");
      string depthpname = "pictures/depth0";
      depthpname.append(inttostr(i));
      depthpname.append(".png");
      string rgbpname = "pictures/rgb0";
      rgbpname.append(inttostr(i));
      rgbpname.append(".png");      
      
      Mat image_dispa = imread ( dispname, CV_LOAD_IMAGE_ANYDEPTH );
      Mat image_depth = imread ( depthpname, CV_LOAD_IMAGE_ANYDEPTH );
      image_rgb = imread ( rgbpname, CV_LOAD_IMAGE_COLOR );

      vector<Rect> faces = cv_utils.detect_face_depth ( image_depth, image_dispa );
      cv_utils.draw_depth_faces ( image_rgb, faces );
      imshow ( "Arash", image_rgb );
      waitKey ( 0 );
    }
  return 0;

  vector<vector <Point> > groundtruths;
  vector<string> filenames;
  filenames.push_back("/home/safir/Dropbox/Vibot/Vibot 03 Semester/Roboticsproject/projects/Ground_truth/Munich_data/munich_person1_centre.yaml");
  filenames.push_back("/home/safir/Dropbox/Vibot/Vibot 03 Semester/Roboticsproject/projects/Ground_truth/Munich_data/munich_person2_centre.yaml");  
//   filenames.push_back("/home/safir/Dropbox/Vibot/Vibot 03 Semester/Roboticsproject/projects/Ground_truth/Scenario6_data/simon_scenario6_centre.yaml");  
  cv_utils.create_combine_gt_vector(filenames, groundtruths);
  cout << groundtruths.size() << endl;

  vector<vector <Point> > ourresults;
  cv_utils.read_from_file("/home/safir/Dropbox/Vibot/Vibot 03 Semester/Roboticsproject/projects/Ground_truth/RGBD-Results-Sceniro6/default_results_centre_min.yaml", &ourresults); 
  vector<vector <Point> > ourresults_new;
  cout << "Oursize: " << ourresults.size() << endl;
  double proportion = (double) groundtruths.size() / (double) ourresults.size();
  cout << proportion << endl;
  int j = 0;
  for ( unsigned int i = 0; i < ourresults.size(); i++, j++ ) 
    {
      ourresults_new.push_back(ourresults[i]);
      ourresults_new.push_back(ourresults[i]);
      ourresults_new.push_back(ourresults[i]);
      ourresults_new.push_back(ourresults[i]);
      ourresults_new.push_back(ourresults[i]);
      ourresults_new.push_back(ourresults[i]);
      ourresults_new.push_back(ourresults[i]);
      ourresults_new.push_back(ourresults[i]);      
      if ( j % 3 == 0 )
        {
          ourresults_new.push_back(ourresults[i]);
        }
    }
  int i = 400;
  ourresults_new.push_back(ourresults[i]);      
  ourresults_new.push_back(ourresults[i]);      
  ourresults_new.push_back(ourresults[i]);      
  ourresults_new.push_back(ourresults[i]);      
  ourresults_new.push_back(ourresults[i]);      
  ourresults_new.push_back(ourresults[i]);      
  cout << ourresults_new.size() << endl;
  
  cv_utils.compare_gt_results ( groundtruths, ourresults_new );
  
  return 0;

}
