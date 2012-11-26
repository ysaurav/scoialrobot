#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/background_segm.hpp>

#include <iostream>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

Mat image;

int trackObject = 0;
int skip = 0;
bool selectObject = false;
bool is_first_frame = true;
Point origin;
Rect selection;
vector<Rect>to_be_saved;

void onMouse ( int event, int x, int y, int, void* )
{
  if ( ( selectObject ) && ( skip  == 0 ) )
    {
      selection.x = MIN ( x, origin.x );
      selection.y = MIN ( y, origin.y );
      selection.width = std::abs ( x - origin.x );
      selection.height = std::abs ( y - origin.y );
      // selection &= Rect ( 0, 0, image.cols, image.rows );

    }

  switch ( event )
    {
    case CV_EVENT_LBUTTONDOWN:
      origin = Point ( x,y );
      selection = Rect ( x,y,0,0 );
      selectObject = true;
      skip = 0;
      break;
    case CV_EVENT_LBUTTONUP:
      selectObject = false;
      if ( selection.width > 0 && selection.height > 0 )
        trackObject = -1;

      break;
    case CV_EVENT_RBUTTONDBLCLK:
      selectObject = true;
      selection = Rect ( 0,0,0,0 );
      trackObject = -1;
      skip = 1;
      break;
    }
}

void write_results_to_file ( string file_name, vector<Rect> rois )
{
  string rect_file = file_name;
  rect_file.append ( "_rects.yaml" );
  string centre_file = file_name;
  centre_file.append ( "_centre.yaml" );
  FileStorage fsr ( rect_file, FileStorage::WRITE );
  FileStorage fsc ( centre_file, FileStorage::WRITE );
  fsr<<"roi"<<"[";
  fsc<<"center"<<"[";
  for ( unsigned int i = 0; i < rois.size(); i++ )
    {

      fsr << "{:"<< "x" << rois[i].x << "y" << rois[i].y << "w" << rois[i].width << "h" << rois[i].height <<"}";
      Point center ( rois[i].x + ( rois[i].width / 2 ), rois[i].y + ( rois[i].height / 2 ) );
      fsc << "{:"<< "x" << center.x << "y" << center.y <<"}";
    }
  fsr.release();
  fsc.release();
}

/**<
* This function writes the coordinates of a rectangle to a .yaml file
* @return
* @param file_name A string file containing the output file
* @param roi A vector of rectangles containing the ROI.
 */

Rect check_boundaries ( Rect window )
{
  Rect bound ( 0,0,image.cols,image.rows );
  Rect new_window = window & bound;

  return new_window;
}
/**<
* This function checks if the ROI has excedded the dimensions of the image
* @return
* @param window A Rect containing the ROI.
 */
void help()
{
  cout << "\nThis program creates the ground-truth for a tracking algorithm.\n"
       "You select the face of one person in order to create the ground-truth.\n"
       "This reads from a video or text file containing the names of the frames.\n"
       "The program saves to .yaml file  the ground-truth.\n"
       "Usage: \n"
       "  ./createGTarrows [input_video/.txt] [output file]\n";

  cout << "\n\nHot keys: \n"
       "\tw - move up ROI\n"
       "\tx - move down ROI\n"
       "\td - move left ROI\n"
       "\ta - move right ROI\n"
       "\tu - increase scale of ROI\n"
       "\tl - decrease scale of ROI\n"
       "\tc - skip ROI\n"
       "\t0 - goes to the next frame\n"
       "To initialize ground-truth, select the object with mouse\n";
}

int main ( int argc, const char** argv )
{

  Mat frame;
  Rect trackWindow;
  bool video_mode = true;
  int go_next = 1;

  help();

  VideoCapture cap ( argv[1] );

  // Check if the input is .txt or video fi;e
  if ( cap.isOpened() )
    {
      video_mode = true;
    }
  else
    {
      video_mode = false;
    }

  FILE *f = 0;
  char _filename[1024];
  frame = imread ( argv[1] );
  f = fopen ( argv[1], "r" );

  if ( f == NULL )
    {
      printf ( "Error: can't open file.\n" );
      return 1;
    }
  else
    {
      printf ( "File opened successfully.\n" );
    }

  namedWindow ( "Input", 0 );
  setMouseCallback ( "Input", onMouse, 0 );

  for ( ;; )
    {
      if ( go_next == 1 )
        {
          // Extract frames from video
          if ( video_mode )
            {

              cap >> frame;
              if ( frame.empty() )
                {
                  break;
                }
              go_next = 0;

            }
          else
            {
              // Extract frames based on .txt file
              if ( trackObject || is_first_frame )
                {

                  is_first_frame = false;
                  char *filename = _filename;
                  if ( f )
                    {
                      if ( !fgets ( filename, ( int ) sizeof ( _filename ) -2, f ) )
                        {
                          break;
                        }
                      if ( filename[0] == '#' )
                        {
                          continue;
                        }
                      int l = strlen ( filename );
                      while ( l > 0 && isspace ( filename[l-1] ) )
                        {

                          --l;
                        }
                      filename[l] = '\0';
                      frame = imread ( filename );
                      go_next = 0;


                    }
                  if ( !frame.data )
                    {
                      continue;
                    }
                }
            }

        }

      frame.copyTo ( image );

      if ( selectObject )
        {
          trackWindow = selection;
          trackWindow = check_boundaries ( trackWindow );
          go_next = 0;
        }

      if ( selectObject && selection.width > 0 && selection.height > 0 )
        {
          Mat roi ( image, selection );
          bitwise_not ( roi, roi );
          rectangle ( image, selection.tl(), selection.br(), Scalar ( 0,0,255 ), 2, 8, 0 );

        }


      rectangle ( image, trackWindow.tl(), trackWindow.br(), Scalar ( 0,0,255 ), 2, 8, 0 );
      imshow ( "Input",image );

      char c = ( char ) waitKey ( 10 );
      if ( c == 27 )
        break;
      switch ( c )
        {
        case 'd':
          // Move ROI to right
          trackWindow.x = trackWindow.x + 1;
          trackWindow.y = trackWindow.y;
          trackWindow.height = trackWindow.height;
          trackWindow.width = trackWindow.width;
          trackWindow = check_boundaries ( trackWindow );
          rectangle ( image, trackWindow.tl(), trackWindow.br(), Scalar ( 0,0,255 ), 2, 8, 0 );
          imshow ( "Input",image );
          go_next = 0;
          break;
        case 'a':
          // Move ROI to left
          trackWindow.x = trackWindow.x - 1;
          trackWindow.y = trackWindow.y;
          trackWindow.height = trackWindow.height;
          trackWindow.width = trackWindow.width;
          trackWindow = check_boundaries ( trackWindow );
          rectangle ( image, trackWindow.tl(), trackWindow.br(), Scalar ( 0,0,255 ), 2, 8, 0 );
          imshow ( "Input",image );
          go_next = 0;
          break;
        case 'w':
          // Move ROI up
          trackWindow.x = trackWindow.x;
          trackWindow.y = trackWindow.y - 1;
          trackWindow.height = trackWindow.height;
          trackWindow.width = trackWindow.width;
          trackWindow = check_boundaries ( trackWindow );
          rectangle ( image, trackWindow.tl(), trackWindow.br(), Scalar ( 0,0,255 ), 2, 8, 0 );
          imshow ( "Input",image );
          go_next = 0;
          break;
        case 'x':
          // Move ROI down
          trackWindow.x = trackWindow.x;
          trackWindow.y = trackWindow.y + 1;
          trackWindow.height = trackWindow.height;
          trackWindow.width = trackWindow.width;
          trackWindow = check_boundaries ( trackWindow );
          rectangle ( image, trackWindow.tl(), trackWindow.br(), Scalar ( 0,0,255 ), 2, 8, 0 );
          imshow ( "Input",image );
          go_next = 0;
          break;

        case 'l':
          // Decrease dimensions of ROI
          trackWindow.x = trackWindow.x;
          trackWindow.y = trackWindow.y;

          if ( ( trackWindow.height>15 ) )
            {
              trackWindow.height = trackWindow.height*0.85;
            }
          if ( ( trackWindow.width>15 ) )
            {
              trackWindow.width = trackWindow.width*0.85;
            }
          trackWindow = check_boundaries ( trackWindow );
          rectangle ( image, trackWindow.tl(), trackWindow.br(), Scalar ( 0,0,255 ), 2, 8, 0 );
          imshow ( "Input",image );
          go_next = 0;
          break;
        case 'u':
          // Increase dimensions of ROI
          trackWindow.x = trackWindow.x;
          trackWindow.y = trackWindow.y;
          if ( ( trackWindow.height<image.rows ) )
            {
              trackWindow.height = trackWindow.height*1.15;
            }
          if ( trackWindow.width<image.cols )
            {
              trackWindow.width = trackWindow.width*1.15;
            }
          trackWindow = check_boundaries ( trackWindow );
          rectangle ( image, trackWindow.tl(), trackWindow.br(), Scalar ( 0,0,255 ), 2, 8, 0 );
          imshow ( "Input",image );
          go_next = 0;
          break;
        case 'c':
          // Cancel ROI
          selectObject = true;
          selection = Rect ( 0,0,0,0 );
          trackObject = -1;
          skip = 1;
          go_next = 0;
        case '0':
          go_next = 1;
          to_be_saved.push_back ( trackWindow );

        default:
          ;
        }

    }

  // Write ROI to .yaml file
  string filename = "default";
  if ( argc > 2 )
    {
      filename = argv[2];
    }
  write_results_to_file ( filename, to_be_saved );

}
