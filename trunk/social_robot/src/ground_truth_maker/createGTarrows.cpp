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
int skip = 0;
bool selectObject = false;
bool is_first_frame = true;
Point origin;
Rect selection;
vector<Rect>to_be_saved;

void onMouse ( int event, int x, int y, int, void* )
{
     
    if (( selectObject ) && (skip  == 0))
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

Rect check_boundaries(Rect window)
{
   Rect bound(0,0,image.cols,image.rows);
   Rect new_window = window & bound;
   
   return new_window;
}

int main(int argc, const char** argv) {
   
   Mat frame;
   Rect trackWindow;
   bool video_mode = true;
   int go_next = 1;
   VideoCapture cap(argv[1]);
    
   if (cap.isOpened() )
   {
     video_mode = true;
     
   }
   else
   {       
    video_mode = false;
   }
  
  
     FILE *f = 0; 
     char _filename[1024];
     frame = imread(argv[1]);
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
        if (go_next == 1)
	{
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
		    go_next = 0;
		  
                    
                }
                if ( !frame.data )
                {
                    continue;
                }
           }
	}
            
	}    
      
       
        
        frame.copyTo(image);
	
	if (selectObject)
	{
	   trackWindow = selection;
	   trackWindow = check_boundaries(trackWindow);
           go_next = 0;
	}
	
	if ( selectObject && selection.width > 0 && selection.height > 0 )
        {
            Mat roi ( image, selection );
            bitwise_not ( roi, roi );
	    rectangle(image, selection.tl(), selection.br(), Scalar(0,0,255), 2, 8, 0 );
	   
        }
        
        
        rectangle(image, trackWindow.tl(), trackWindow.br(), Scalar(0,0,255), 2, 8, 0 );
	imshow("Input",image);
	

	char c = (char)waitKey(10);
        if( c == 27 )
            break;
	switch(c)
        {
        case 'd':
            trackWindow.x = trackWindow.x + 1;
	    trackWindow.y = trackWindow.y;
	    trackWindow.height = trackWindow.height;
	    trackWindow.width = trackWindow.width;
	    trackWindow = check_boundaries(trackWindow);
	    rectangle(image, trackWindow.tl(), trackWindow.br(), Scalar(0,0,255), 2, 8, 0 );
	    imshow("Input",image);
	    go_next = 0;
            break;
        case 'a':         
            trackWindow.x = trackWindow.x - 1;
	    trackWindow.y = trackWindow.y;
	    trackWindow.height = trackWindow.height;
	    trackWindow.width = trackWindow.width;
	    trackWindow = check_boundaries(trackWindow);
	    rectangle(image, trackWindow.tl(), trackWindow.br(), Scalar(0,0,255), 2, 8, 0 );
	    imshow("Input",image);
	    go_next = 0;
            break;
        case 'w':
            trackWindow.x = trackWindow.x;
	    trackWindow.y = trackWindow.y - 1;
	    trackWindow.height = trackWindow.height;
	    trackWindow.width = trackWindow.width;
	    trackWindow = check_boundaries(trackWindow);
	    rectangle(image, trackWindow.tl(), trackWindow.br(), Scalar(0,0,255), 2, 8, 0 );
	    imshow("Input",image);
	    go_next = 0;
	    break;
        case 'x':
            trackWindow.x = trackWindow.x;
	    trackWindow.y = trackWindow.y + 1;
	    trackWindow.height = trackWindow.height;
	    trackWindow.width = trackWindow.width;
	    trackWindow = check_boundaries(trackWindow);
	    rectangle(image, trackWindow.tl(), trackWindow.br(), Scalar(0,0,255), 2, 8, 0 );
	    imshow("Input",image);
	    go_next = 0;
	    break;
	    
	case 'l':
            trackWindow.x = trackWindow.x;
	    trackWindow.y = trackWindow.y;
	    
	    if ((trackWindow.height>15))
	    {
	    trackWindow.height = trackWindow.height*0.85;
	    }
	    if ((trackWindow.width>15))
	    {
	      trackWindow.width = trackWindow.width*0.85;
	    }
	    trackWindow = check_boundaries(trackWindow);
	    rectangle(image, trackWindow.tl(), trackWindow.br(), Scalar(0,0,255), 2, 8, 0 );
	    imshow("Input",image);
	    go_next = 0;
	    break; 
	case 'u':
            trackWindow.x = trackWindow.x;
	    trackWindow.y = trackWindow.y;
	    if ((trackWindow.height<image.rows))
	    {
	    trackWindow.height = trackWindow.height*1.15;	    
	    }
	    if (trackWindow.width<image.cols)
	    {
	      trackWindow.width = trackWindow.width*1.15;
	    }
	    trackWindow = check_boundaries(trackWindow);
	    rectangle(image, trackWindow.tl(), trackWindow.br(), Scalar(0,0,255), 2, 8, 0 );
	    imshow("Input",image);
	    go_next = 0;
	    break; 
	case 'c':
	    selectObject = true;
            selection = Rect ( 0,0,0,0 );
	    trackObject = -1;
	    skip = 1;
	    go_next = 0;
	case '0':
	    go_next = 1;
	    to_be_saved.push_back(trackWindow);
	    
        default:
            ;
        }

      }
      
    
      write_results_to_file(argv[2], to_be_saved);

}
