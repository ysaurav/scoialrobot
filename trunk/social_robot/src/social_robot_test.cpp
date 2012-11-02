#include <ros/ros.h>

#include "kinect_proxy.h"
#include "cv_utils.h"

#define HEAD_TEMPLATE "pictures/template.png"
#define THR1          5
#define THR2          7
#define THR3          20
#define SCALES        5

using namespace std;
using namespace cv;

string cascade_name = "rsrc/haarcascades/haarcascade_frontalface_alt.xml";

const static Scalar colors[] =
{
  CV_RGB(0, 0, 255),
  CV_RGB(0, 128, 255),
  CV_RGB(0, 255, 255),
  CV_RGB(0, 255, 0),
  CV_RGB(255, 128, 0),
  CV_RGB(255, 255, 0),
  CV_RGB(255, 0, 0),
  CV_RGB(255, 0, 255)
};

Mat canny_im;

Mat preprocessing(Mat image)
{
  Mat dst;
  Mat temp;
  resize(image, temp, Size(), 2, 2, INTER_NEAREST);
  resize(temp, dst, Size(), 0.5, 0.5, INTER_NEAREST);
  // inpainting
  //  Mat mask = Mat::zeros(image.rows, image.cols, CV_8U);
  //  Mat temp2(image.rows, image.cols, CV_8U);
  //  Mat temp3(image.rows, image.cols, CV_8U);
  //  for (int i = 0; i < image.rows; i++)
  //  {
  //    unsigned char *rowi = image.ptr<unsigned char>(i);
  //    for (int j = 0; j < image.cols; j++)
  //    {
  //      if (rowi[j] == 0)
  //      {
  //        mask.at<unsigned char>(i, j) = 255;
  //      }
  //    }
  //  }
  ////  bilateralFilter(image, temp3, 1, 0.01, 0.1);
  //  medianBlur(image, temp3, 5);
  //  inpaint(temp3, mask, temp2, 5, INPAINT_NS);
  medianBlur(dst, dst, 5);
  imshow("Preprocessing", dst);
  waitKey(0);
  return dst;
}

vector<Point> chamfer_matching(Mat image)
{
  canny_im.create(image.rows, image.cols, image.depth());
  Mat template_im = imread(HEAD_TEMPLATE, CV_LOAD_IMAGE_ANYDEPTH);

  Mat* pyramid = new Mat[SCALES];
  Mat* chamfer = new Mat[SCALES];
  Mat* matching = new Mat[SCALES];

  // calculate edge detection
  Canny(image, canny_im, THR1, THR2, 3, true);
  imshow("Canny", canny_im);
  waitKey(0);

  // calculate the Canny pyramid
  pyramid[0] = canny_im;
  for(int i = 1; i < SCALES; i++)
  {
    resize(pyramid[i - 1], pyramid[i], Size(), 0.75, 0.75, INTER_NEAREST);
  }

  // calculate distance transform
  for(int i = 0; i < SCALES; i++)
  {
    distanceTransform((255 - pyramid[i]), chamfer[i], CV_DIST_C, 3);
    normalize(chamfer[i], chamfer[i], 0.0, 1.0, NORM_MINMAX);
  }

  // matching with the template
  template_im = rgb2bw(template_im);
  template_im.convertTo(template_im,CV_32F);

  // find the best match:
  double minVal, maxVal;
  Point minLoc, maxLoc;
  double xdiff = template_im.cols / 2;
  double ydiff = template_im.rows / 2;
  Point pdiff = Point(xdiff, ydiff);
  vector<Point> head_matched_points;

  for (int i = 0; i < SCALES; i++)
  {
    matchTemplate(chamfer[i], template_im, matching[i], CV_TM_CCOEFF);
    normalize(matching[i], matching[i], 0.0, 1.0, NORM_MINMAX);
    minMaxLoc(matching[i], &minVal, &maxVal, &minLoc, &maxLoc);
    Mat matching_thr;
    threshold(matching[i], matching_thr, 1.0 / THR3, 1.0, CV_THRESH_BINARY_INV);
    double scale = pow(1.0 / 0.75, i);
    get_non_zeros(matching_thr, &head_matched_points, pdiff, scale);
  }

  return head_matched_points;
}

vector<Point3f> compute_headparameters(Mat image, vector<Point> chamfer)
{
  vector<Point3f> parameters_head(chamfer.size());

  // parameters of cubic equation
  float p1 = -1.3835 * pow(10, -9);
  float p2 =  1.8435 * pow(10, -5);
  float p3 = -0.091403;
  float p4 =  189.38;

  for (unsigned int i = 0; i < chamfer.size(); i++)
  {
    int position_x = chamfer[i].x;
    int position_y = chamfer[i].y;

    unsigned short x = image.at<unsigned short>(position_y, position_x);

    // compute height of head
    float h = (p1 * pow(x, 3) + p2 * pow(x, 2) + p3 * x + p4);

    // compute Radius of head in milimeters
    float R = 1.33 * h / 2;

    // convert Radius in pixels
    float Rp = round((1 / 1.3) * R);

    parameters_head[i].x = position_x;
    parameters_head[i].y = position_y;
    parameters_head[i].z = Rp;
  }

  return parameters_head;
}

void detect_face(Mat &img, CascadeClassifier& cascade)
{
  int i = 0;
  double t = 0;
  vector<Rect> faces;

  Mat gray;
  Mat frame( cvRound(img.rows), cvRound(img.cols), CV_8UC1 );

  cvtColor( img, gray, CV_BGR2GRAY );
  resize( gray, frame, frame.size(), 0, 0, INTER_LINEAR );
  equalizeHist( frame, frame );

  t = (double)cvGetTickCount();
  cascade.detectMultiScale( frame, faces,
                            1.1, 2, 0
                            //|CV_HAAR_FIND_BIGGEST_OBJECT
                            //|CV_HAAR_DO_ROUGH_SEARCH
                            |CV_HAAR_SCALE_IMAGE
                            ,
                            Size(30, 30) );
  t = (double)cvGetTickCount() - t;

  for(vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
  {
    Point center;
    Scalar color = colors[i%8];
    int radius;
    center.x = cvRound(r->x + r->width*0.5);
    center.y = cvRound(r->y + r->height*0.5);
    radius = (int)(cvRound(r->width + r->height)*0.25);
    circle( img, center, radius, color, 3, 8, 0 );
  }
}

int main(int argc, char **argv)
{
  Mat image_dispa = imread(argv[1], CV_LOAD_IMAGE_ANYDEPTH);
  Mat image_depth = imread(argv[2], CV_LOAD_IMAGE_ANYDEPTH);
  Mat image_rgb = imread(argv[3], CV_LOAD_IMAGE_COLOR);
  
  Mat rgim;
  vector<Point> seeds;
  seeds.push_back(Point(175, 95));
  seeds.push_back(Point(318, 117));
  seeds.push_back(Point(460, 146));
  region_growing(image_depth, rgim, 10, seeds);
  
  imshow("regiongrowing", rgim);
  waitKey(0);
  return 0;

  CascadeClassifier cascade;
  if (!cascade.load(cascade_name))
  {
      cerr << "ERROR: Could not load cascade classifier \"" << cascade_name << "\"" << endl;
      return -1;
  }

  cout << "Channels: " << image_rgb.channels() << " Type: " << image_rgb.type() << " Depth: " << image_rgb.depth() << endl;

  detect_face(image_rgb, cascade);
  imshow("Arash", image_rgb);
  waitKey(0);
  return 0;

  image_dispa = preprocessing(image_dispa);
  image_depth = preprocessing(image_depth);
  vector<Point> head_matched_points = chamfer_matching(image_dispa);

  int thr = atoi(argv[4]); // default must be 12
  int thr2 = atoi(argv[5]); // default must be 17

  vector<Point3f> tmpparams = compute_headparameters(image_depth, head_matched_points);
  for (unsigned int i = 0; i < tmpparams.size(); i++)
  {
    //    circle(image_rgb, Point(tmpparams[i].x, tmpparams[i].y), tmpparams[i].z, cvScalar(255, 255, 0, 0), 2, 8, 0);
    Rect roi(tmpparams[i].x - tmpparams[i].z, tmpparams[i].y - tmpparams[i].z, tmpparams[i].z * 2, tmpparams[i].z * 2);
    if (!(0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= canny_im.cols && 0 <= roi.y && 0 <= roi.height && roi.y + roi.height <= canny_im.rows))
    {
      continue;
    }
    Mat tmp_mat = canny_im(roi);
    vector<vector<Point> > contour;
    findContours(tmp_mat, contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    for (unsigned int j = 0; j < contour.size(); j++)
    {
      vector<Point> approx;
      approxPolyDP(contour[j], approx, 5, false);
      if (approx.size() > thr && approx.size() < thr2)
      {
        rectangle(image_rgb, roi.tl(), roi.br(), cvScalar(255, 0, 255, 0), 2, 8, 0);
        break;
      }
    }
  }

  // imwrite("circle_img.png", image_rgb);
  imshow("Hola", image_rgb);
  waitKey(0);
  return 0;
}
