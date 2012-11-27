#ifndef DEPTHFACEDETECTOR_H
#define DEPTHFACEDETECTOR_H

#include "PixelSimilarity.h"
#include "Template.h"

using namespace std;
using namespace cv;

class DepthFaceDetector
  {
  public:
    DepthFaceDetector ( void );
    vector<Rect> detect_face_depth ( Mat depth_image, Mat disparity_image );

    // values that can be chanegd during the runtime
    double canny_thr1;
    double canny_thr2;
    double chamfer_thr;
    double arc_thr_low;
    double arc_thr_high;
    double approx_poly_thr;
    double max_suppression;
    double scale_factor;
    double match3D_thr;
    int scales;
    int scales_default;
    int framenum;
    int update_rate;

  private:
    void load_templates ( void );
    vector<Point3f> chamfer_matching ( Mat image, Mat template_im );
    vector<PixelSimilarity> compute_headparameters ( Mat image, vector<Point3f> chamfer );
    vector<PixelSimilarity> false_positives ( vector<PixelSimilarity> tmpparams, int thr, int thr2 );
    void match_template3D ( Mat image_disparity, vector<PixelSimilarity> potentials, vector<PixelSimilarity> *heads, int n );
    vector<PixelSimilarity> merge_rectangles ( vector<PixelSimilarity> tmpcont );
    bool checkDimensions ( Mat depth );

    vector<Template> templates;
    vector<Point3f> head_matched_points;
    vector<PixelSimilarity> head_features;

    vector<Point3f> head_matched_points2;
    vector<PixelSimilarity> head_features2;

    Mat *pyramid;
    Mat *chamfer;
    Mat *matching;

    Mat canny_im;

  };

#endif // DEPTHFACEDETECTOR_H
