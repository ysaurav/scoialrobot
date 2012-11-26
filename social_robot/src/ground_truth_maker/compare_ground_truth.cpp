#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

using namespace cv;
using namespace std;

void read_from_file ( string filename, vector<Point> &rois )
{
  FileStorage fs ( filename, FileStorage::READ );
  FileNode features = fs["center"];
  FileNodeIterator it = features.begin(), it_end = features.end();
  int idx = 0;

  for ( ; it != it_end; ++it, idx++ )
    {
      // cout << "frame #" << idx << ": ";
      // cout << "x=" << ( int ) ( *it ) ["x"] << ", y=" << ( int ) ( *it ) ["y"] << "\n";
      int x = ( int ) ( *it ) ["x"];
      int y = ( int ) ( *it ) ["y"];
      rois.push_back ( Point ( x,y ) );
    }
  fs.release();
}
/**<
* This function reads from a .yaml file the coordinates of the center of ROI
* @return
* @param file_name A string file containing the input file
* @param rois A vector of Points containing the centers of ROI.
 */

void write_to_file ( string filename, vector<double> rois, double mse )
{
  string distance_file = filename;
  distance_file.append ( "_dist.yaml" );

  FileStorage fsr ( distance_file, FileStorage::WRITE );
  fsr << "MSE " << mse;
  fsr<<"distance"<<"[";

  for ( unsigned int i = 0; i < rois.size(); i++ )
    {
      fsr << "{:"<< "d" << rois[i] <<"}";
    }
  fsr.release();

}
/**<
* This function writes the distance and MSE between ground-truth and tracking results to a .yaml file
* @return
* @param file_name A string file containing the output file
* @param rois A vector of doubles containing the distances.
* @param mse The Mean Square Error.
 */

void help()
{
  cout << "\nThis program computes the distance between ground-truth and tracking algorithm and also the Mean Square Error.\n"
       "The program saves to .yaml file  the results.\n"
       "Notice: the program has an optional input, an image frame, in order to display the input information.\n"
       "Usage: \n"
       "  ./compare [ground_truth.yaml file] [tracking_result.yaml file] [optional:image]\n";
}

int main ( int argc, const char** argv )
{

  help();

  vector<Point> gt;
  read_from_file ( argv[1], gt );

  vector<Point>tracking;
  read_from_file ( argv[2],tracking );
  Mat image;

  // If an image is given as input, display tracking adn ground-truth coordinates
  if ( argc == 5 )
    {

      image = imread ( argv[4] );

      Scalar color = Scalar ( 0,0,255 );
      Scalar color2 = Scalar ( 0,255,0 );

      // display results

      for ( int i = 0;i<gt.size(); i++ )
        {
          circle ( image,gt[i],4,color,4 );
        }

      for ( int i = 0;i<tracking.size(); i++ )
        {
          circle ( image,tracking[i],6,color2,6 );
        }

      namedWindow ( "Reference",0 );
      imshow ( "Reference",image );
    }

  // Compute distance

  vector<double> distance;
  double d;
  double sum;
  for ( int i = 0;i<tracking.size(); i++ )
    {
      d = sqrt ( pow ( ( gt[i].x - tracking[i].x ),2 ) + pow ( ( gt[i].y - tracking[i].y ),2 ) );
      distance.push_back ( d );
      sum += d;
    }

  // Compute mean of distances

  double size_d = tracking.size();
  double mean_d = sum/size_d;

  // Compute MSE
  double s_mse = 0;
  for ( int i = 0; i<size_d;i++ )
    {
      s_mse += pow ( distance[i],2 );
    }

  double MSE = s_mse/size_d;
  cout<<"\n \n Mean squar error: "<<MSE<<"\n";

  write_to_file ( argv[3], distance, MSE );
  waitKey ( 0 );

  return 1;
}
