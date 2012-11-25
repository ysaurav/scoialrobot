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
    int framenum;
    int update_rate;

  private:
    void load_templates ( void );
    vector<Point3f> chamfer_matching ( Mat image, Mat template_im );
    vector<PixelSimilarity> compute_headparameters ( Mat image, vector<Point3f> chamfer );
    vector<PixelSimilarity> false_positives ( vector<PixelSimilarity> tmpparams, int thr, int thr2 );
    vector<PixelSimilarity> match_template3D ( vector<PixelSimilarity> potentials, int n );
    vector<PixelSimilarity> merge_rectangles ( vector<PixelSimilarity> tmpcont );

  };

#endif // DEPTHFACEDETECTOR_H
