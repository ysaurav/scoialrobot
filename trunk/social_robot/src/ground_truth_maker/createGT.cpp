#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/background_segm.hpp"

#include <iostream>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <opencv-2.3.1/opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

Mat image;

int trackObject = 0;
bool selectObject = false;
bool is_first_frame = true;
Point origin;
Rect selection;
vector<Rect>to_be_saved;

void onMouse ( int event, int x, int y, int, void* )
{
     
    if ( selectObject )
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
	
        break;
    case CV_EVENT_LBUTTONUP:
        selectObject = false;
        if ( selection.width > 0 && selection.height > 0 )
            trackObject = -1;
	
        break;
    case CV_EVENT_RBUTTONDBLCLK:
        selectObject = false;
        selection = Rect ( 0,0,0,0 );
	trackObject = -1;
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

    for ( unsigned int i = 0; i < rois.size(); i++ )
    {
        fsr << "roi" << rois[i];
        Point center ( rois[i].x + ( rois[i].width / 2 ), rois[i].y + ( rois[i].height / 2 ) );
        fsc << "center" << center;
    }
    fsr.release();
    fsc.release();
}

int main(int argc, const char** argv) {
   
    FILE *f = 0; 
    char _filename[1024];
   
	
    Mat frame = Mat::zeros(200, 320, CV_8UC3);
    
    f = fopen ( argv[1], "r" );
    
   
       
    if (f == NULL) {
        printf("Error: can't open file.\n");
        return 1;
    } else {
        printf("File opened successfully.\n");
    }
     
    namedWindow ( "Input", 0 );
    setMouseCallback ( "Input", onMouse, 0 );
    
    
        
    for ( ;; )
    {
      
      
           if ( trackObject || is_first_frame )
            {
	        
                is_first_frame = false;
                char *filename = _filename;
                if (f)
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
		    frame = imread ( filename);
		    to_be_saved.push_back(selection);
                    
                }
                if ( !frame.data )
                {
                    continue;
                }
            }
  
        
        frame.copyTo(image);
	
	if ( selectObject && selection.width > 0 && selection.height > 0 )
        {
            Mat roi ( image, selection );
            bitwise_not ( roi, roi );
	    
	   
        }
        
        

	imshow("Input",image);
	
	
	trackObject = 0;
	char c = (char)waitKey(10);
        if( c == 27 )
            break;

      }
      
      to_be_saved.push_back(selection);
      write_results_to_file(argv[2], to_be_saved);

}