#ifndef SOCIAL_ROBOT_CONSTANTS_H
#define SOCIAL_ROBOT_CONSTANTS_H

#define ROIS "/social_robot/depth/rois"
#define DEPTH_IMAGE "/camera/depth/image_raw"
#define DISPARITY_IMAGE "/camera/depth/disparity"
#define RGB_UPDATE "/social_robot/depth/update"

#define MATCH3D_THR "/social_robot/depth/match3D_thr"
#define MAX_SUPPRESSION "/social_robot/depth/max_suppression"
#define ARC_THR_HIGH "/social_robot/depth/arc_thr_high"
#define ARC_THR_LOW "/social_robot/depth/arc_thr_low"
#define SCALES "/social_robot/depth/scales"
#define CHAMFER_THR "/social_robot/depth/chamfer_thr"
#define DEPTH_REGISTRATION "/camera/driver/depth_registration"

#define SOCIAL_ROBOT_DEPTH "social_robot_depth"
#define SOCIAL_ROBOT "social_robot"

// calculating histograms
#define HIST_BGR    0
#define HIST_D      1
#define HIST_BGRD   2
#define HIST_HS     3
#define HIST_HSD    4

#endif