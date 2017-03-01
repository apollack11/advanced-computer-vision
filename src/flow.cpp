#include <ros/ros.h>
// #include <ros/console.h>

#include <image_transport/image_transport.h> // for subscribing/publishing image topics
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/calib3d/calib3d.hpp"
// #include "opencv2/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/nonfree/features2d.hpp> // SurfFeatureDetector
#include <opencv2/legacy/legacy.hpp> // BruteForceMatcher
#include <opencv2/video/tracking.hpp> // estimateRigidTransform
// #include "opencv2/xfeatures2d.hpp"
#include <iostream>
// #include <list>
#include <vector>
#include <cmath>
#include <iomanip> // std::setw
#include <typeinfo>
#include <algorithm>
#include <functional> // std::transform

#define MAX_INDICES 200 // # of points to track w/ optical flow
#define TRAIL_LENGTH 30 // not currently being used for any real purposes
// #define CAM_ANGLE_U 31.67 // degrees subtended by camera in u direction (wider of the two)
// #define CAM_ANGLE_U 0.5527 // radians subtended by camera in u direction (wider of the two)
#define PIX_ANGLE_U 0.0008637 // radians subtended by each pixel in u direction (wider of the two)


using namespace std;
using namespace cv;



class FlowCalculator
{
private:
  // set up ROS
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  // image_transport::Publisher image_pub_;

  // OpenCV shared variables
  cv::Mat prev;
  // cv::Mat prev_color; // don't need this, since the only data we get in is curr_color
  cv::Mat curr;
  cv::Mat curr_color;
  vector<float> flow_errs;
  vector<unsigned char> flow_status;
  vector<Point2f> curr_track_indices;
  vector<Point2f> prev_track_indices;
  vector<Point2f> prev_track_centered;
  vector<Point2f> curr_track_centered;
  vector<Point2f> curr_track_undistorted;
  vector<Point2f> prev_track_undistorted;
  string ColorWinName;
  string GrayWinName;
  cv_bridge::CvImagePtr in_ptr;

  // other stuff
  // vector<vector<Point2f> > tracking_indices;
  // vector<vector<Point2f> >::iterator trail_it;
  int counter;
  cv::Matx33d F; // Fundamental Matrix
  cv::Matx33d E; // Essential Matrix
  cv::Matx33d camera_matrix; // Camera Matrix (from camera calibration file)
  cv::Mat distortion_coefficients; // distorition coefficients (from camera calibration file)
  cv::Mat rectification_matrix; // rectification matrix coefficients (from camera calibration file)
  cv::Mat projection_matrix; // projection matrix coefficients (from camera calibration file)
  Mat out_img; // output image, marked up with flow points and stuff
  Matx33d W;
  Matx33d Winv;
  Matx34d P1;
  Mat R; // rotation matrix
  Mat t; // translation matrix

  Point2f accumulated;


public:
  FlowCalculator()
  : it_(nh_)
  {
    std::cout << "instance of FlowCalculator class instantiated" << std::endl;

    // subscribe to input video stream
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &FlowCalculator::img_cb, this);

    // publish if we want
    // image_pub_ = it_.advertise("/image_converter/output_video", 1);

    // vector sizes must be declared inside a class method
    // tracking_indices.resize(TRAIL_LENGTH, vector<Point2f>(MAX_INDICES));
    // tracking_indices.resize(MAX_INDICES, vector<Point2f>(TRAIL_LENGTH));
    counter = 0;

    // create a single window instance, overwrite as needed
    ColorWinName = "Color Output Window";
    cv::namedWindow(ColorWinName, WINDOW_AUTOSIZE);

    // GrayWinName = "Gray Output Window";
    // cv::namedWindow(GrayWinName, WINDOW_AUTOSIZE);

    // cv::FileStorage fs;
    // fs.open("/home/njk/Courses/EECS432/Project/ros_ws/src/eecs432_project/calibration/webcam.xml", cv::FileStorage::READ);
    // fs["camera_matrix"] >> camera_matrix;
    // cout << "camera_matrix:\n" << camera_matrix << endl;
    // fs.release();

    // hack for now since can't initialize the way I want to
    // cv::Matx33d ktmp(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    cv::Matx33d ktmp(1149.322298, 0.0, 351.778662, 0.0, 1151.593614, 276.459807, 0.0, 0.0, 1.0);
    camera_matrix = ktmp;

    // cv::Mat_ dtmp(-0.073669, 1.170392, 0.000976, -0.00244, 0.0);
    // distortion_coefficients = dtmp;

    float distortion_coefficients_data[5] = {-0.073669, 1.170392, 0.000976, -0.00244, 0.0};
    distortion_coefficients = cv::Mat(1, 5, CV_32F, distortion_coefficients_data);

    Matx33d Wtmp(0, -1, 0, 1, 0, 0, 0, 0, 1);
    W = Wtmp;

    Matx33d Winvtmp(0, 1, 0, -1, 0, 0, 0, 0, 1);
    Winv = Winvtmp;

    float rectification_matrix_data[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    rectification_matrix = cv::Mat(3, 3, CV_32F, rectification_matrix_data);

    float projection_matrix_data[12] = {1160.519653, 0, 349.420934, 0, 0, 1164.307007, 275.445505, 0, 0, 0, 1, 0};
    projection_matrix = cv::Mat(3, 4, CV_32F, projection_matrix_data);

    accumulated = Point2f(0.0, 0.0);

  } // END OF CONSTRUCTOR ######################################################


  ~FlowCalculator()
  {
    cv::destroyAllWindows();
    std::cout << "Nate Kaiser is the best. The end." << std::endl;
  } // END OF DESTRUCTOR #######################################################


  void img_cb(const sensor_msgs::ImageConstPtr& input)
  {
    // grab the current frame from the camera stream
    try
    {
      in_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    curr_color = in_ptr->image; // will use the color version later (for output)
    cv::cvtColor(curr_color, curr, CV_BGR2GRAY); // create curr (grayscale version of the input)


    if(!prev.data) // Check for invalid input
    {
      ROS_WARN("prev data not available, assigning to curr data");
      curr.copyTo(prev); // deep copy, none of that shared pointer stuff
      return;
    }

    // create vector of good points to track from previous image
    goodFeaturesToTrack(prev, prev_track_indices, MAX_INDICES, 0.2, 0.0);

    if(prev_track_indices.empty()) // check
    {
      ROS_WARN("no tracking objects found");
      curr.copyTo(prev); // deep copy, none of that shared pointer stuff
      return;
    }

    // find optical flow between previous and current images, store in curr_track_indices
    calcOpticalFlowPyrLK(prev, curr, prev_track_indices, curr_track_indices, flow_status, flow_errs);
    // cout << "prev_track_indices:\n" << prev_track_indices << endl;
    // cout << "curr_track_indices:\n" << curr_track_indices << endl;
    // cout << "prev_track_indices POINT TYPE:\n" << typeid(prev_track_indices[0]).name() << endl;
    // cout << "prev_track_indices POINT 0 X:\n" << prev_track_indices[0].x << endl;

    // NOT SURE WE NEED THIS CHECK, BUT ONE OF MY CUSTOM FUNCTIONS STATES WE HAVE IT:
    if(curr_track_indices.size() != prev_track_indices.size())
    { ROS_ERROR("tracking index data size different between previous and current images"); }

    Point2f derp = uv_left_right(prev_track_indices, curr_track_indices);
    accumulated += derp * PIX_ANGLE_U * 57.29;
    // cout << "(u, v) = " << setw(12) << derp.x << ", " << setw(12) << derp.y << endl;
    cout << "accumulated u, v = " << setw(12) << accumulated.x << ", " << setw(12) << accumulated.y << endl;



    // undistort image (before calculating Fundamental Matrix) - turns out to be too laggy
    // try undistorting only tracked points instead (before calculating Fundamental Matrix)
    // http://stackoverflow.com/questions/25251676/opencv-findfundamentalmat-very-unstable-and-sensitive
    // undistortPoints(curr_track_indices, curr_track_undistorted, camera_matrix, distortion_coefficients);//, rectification_matrix, projection_matrix);
    // undistortPoints(prev_track_indices, prev_track_undistorted, camera_matrix, distortion_coefficients);//, rectification_matrix, projection_matrix);
    // DOES THIS WORK WITH INDICES OR ACTUAL POINT DATA???? SHOULD BE INDICES, BUT GETTING NANS
    // The function can be used for both a stereo camera head or a monocular camera (when R is empty).
    // ^ DOES THIS MEAN I SHOULDN'T BE USIN R OR P (rectification_matrix OR projection_matrix)?

    // undistorting using built-in function isn't workint, try homebrew solution instead:
    prev_track_undistorted = normalize(prev_track_indices);
    curr_track_undistorted = normalize(curr_track_indices);
    // cout << "prev_track_undistorted:\n" << prev_track_undistorted << endl;
    // cout << "curr_track_undistorted:\n" << curr_track_undistorted << endl;

    // center data per Wu's lecture 12
    // prev_track_centered = centerData(prev_track_indices);
    // curr_track_centered = centerData(curr_track_indices);
    prev_track_centered = centerData(prev_track_undistorted);
    curr_track_centered = centerData(curr_track_undistorted);
    // cout << "prev_track_centered:\n" << prev_track_centered << endl;
    // cout << "curr_track_centered:\n" << curr_track_centered << endl;


    // syntax inspiration found at:
    // http://www.morethantechnical.com/2012/02/07/structure-from-motion-and-3d-reconstruction-on-the-easy-in-opencv-2-3-w-code/

    // comparison of findFundamentalMat solver techniques:
    // http://fhtagn.net/prog/2012/09/27/opencv-fundamentalmat.html
    // may be better to use LMedS instead of RANSAC...
    // for(int k = 0; k < 5; ++k)
    // {
    //   // F = findFundamentalMat(prev_track_indices, curr_track_undistorted, FM_RANSAC, 1, 0.99, flow_status);
    // F = findFundamentalMat(prev_track_indices, curr_track_indices, FM_RANSAC, 1, 0.99, flow_status);
    // ransac sucks, how about this?:
    // F = findFundamentalMat(prev_track_indices, curr_track_indices, CV_FM_LMEDS, 1, 0.99, flow_status);
    // that sucked too, how about this?:
    // F = findFundamentalMat(prev_track_indices, curr_track_indices, CV_FM_8POINT, 1, 0.99, flow_status);
      // F = findFundamentalMat(prev_track_centered, curr_track_centered, FM_RANSAC, 1, 0.99, flow_status);
    //   F = findFundamentalMat(prev_track_centered, curr_track_centered, FM_RANSAC, 0.01, 0.99, flow_status); // 0.01 is guess
    // //   cout << "F:\n" << F << endl;
    // // }
    //
    // E = camera_matrix.t() * F * camera_matrix; // calculate essential matrix from fundamental matrix and camera matrix
    //
    // // helpful clarification:
    // // http://stackoverflow.com/questions/16639106/camera-motion-from-corresponding-images
    // SVD svd(E);
    // R = svd.u * Mat(W) * svd.vt;
    // t = svd.u.col(2);
    // cout << "R:\n" << R << '\n' << endl;
    //
    // double roll = atan2(R.at<float>(2, 1), R.at<float>(2, 2));
    // double pitch = asin(R.at<float>(2, 0));
    // double yaw = -atan2(R.at<float>(1, 0), R.at<float>(0, 0));
    // //
    // cout << "RPY: " << setw(15) << roll << setw(15) << pitch << setw(15) << yaw << endl;

    // if(!(counter%15)) // every 1/2 second, print:
    // {
    //   // cout << "curr_track_indices:\n" << curr_track_indices << endl;
    //   // cout << "prev_track_indices:\n" << prev_track_indices << endl;
    //   // cout << "sizes of each:\t" << curr_track_indices.size() << '\t' << prev_track_indices.size() << endl;
    //   // cout << "curr_track_undistorted:\n" << curr_track_undistorted << endl;
    //   // cout << "prev_track_undistorted:\n" << prev_track_undistorted << endl;
    //   // cout << "curr_track_centered:\n" << curr_track_centered << endl;
    //   // cout << "prev_track_centered:\n" << prev_track_centered << endl;
    //   // cout << "curr pixel val:\n" << curr.at<cv::Vec3b>(30,30) << endl;
    //   // cout << "prev pixel val:\n" << prev.at<cv::Vec3b>(30,30) << endl;
    //   // cout << "F:\n" << F << endl;
    //   // cout << "E:\n" << E << endl;
    //   // cout << "R:\n" << R << endl;
    //   // cout << "t:\n" << t << endl;
    // }

    // P1 = Matx34d(R(0,0),	R(0,1),	R(0,2),	t(0), R(1,0),	R(1,1),	R(1,2),	t(1), R(2,0),	R(2,1),	R(2,2), t(2));


    // cout << "curr_track_indices:\n" << curr_track_indices << endl;


    // troubleshooting, check if F is right:
    // for(int k = 0; k < 5; ++k)
    // {
    //   int x1 = prev_track_indices[k].x;
    //   int y1 = prev_track_indices[k].y;
    //   int x2 = curr_track_indices[k].x;
    //   int y2 = curr_track_indices[k].y;
    //
    //   Matx13d xp(x1, y1, 1);
    //   Matx31d x(x2, y2, 1);
    //
    //   cout << k << '\t' << xp * F * x << endl; // this should be 0 if F is right
    // }



// THIS SECTION WAS SUPPOSED TO DRAW TRAILS BEHIND THE TRACKED POINTS
// IT WORKS, BUT I IMPLEMENTED IT AS DECAYING POINTS INSTEAD
// (SINCE THE POINTS MIGHT OTHERWISE JUMP AROUND)
    // tracking_indices[counter] = curr_track_indices;
    // int radius = TRAIL_LENGTH;
    // int radius = 0;
    // for(trail_it = tracking_indices.begin(); trail_it != tracking_indices.end(); ++trail_it)
    // {
    //   // draw line endpoints for visualization purposes
    //   for(vector<Point2f>::iterator it = (*trail_it).begin(); it != (*trail_it).end(); ++it)
    //   {
    //     circle(out_img, *it, radius, Scalar(0, 255, 0), 1);
    //   }
    //   // draw lines between tracked points for visualization purposes
    //   // for(trail_it = tracking_indices[i].begin(); trail_it != tracking_indices[i].end(); ++trail_it)
    //   // {
    //   //   line(out_img, prev_track_indices[i], curr_track_indices[i], Scalar(0, 255, 0), 2, 8, 0);
    //   //
    //   // }
    //   // --radius;
    //   ++radius;
    // }


    // draw tracked points for visualization purposes
    out_img = curr_color; // copy over so we can draw tracked points over top
    for(int i = 0; i < curr_track_indices.size(); ++i)
    {
      circle(out_img, curr_track_indices[i], 3, Scalar(0, 255, 0), -1); // -1 = filled
    }
    imshow(ColorWinName, out_img);
    // imshow(GrayWinName, prev);
    cv::waitKey(30); // 30 Hz camera = 33.3 ms per callback loop

    // THIS ONLY DOES A SHALLOW COPY, WILL BE OVERWRITTEN NEXT TIME I UPDATE CURR
    // prev = curr; // set for next iteration
    curr.copyTo(prev); // deep copy, none of that shared pointer stuff
    // cout << counter  << endl;
    ++counter %= TRAIL_LENGTH;

  } // END OF FUNCTION img_cb() ################################################


  vector<Point2f> centerData(vector<Point2f> coords)
  {
    // see Lecture 12: Structure From Motion (slide on Factorization: Data Centering)
    int L = coords.size();

    float xsum = 0.0;
    float ysum = 0.0;
    for(vector<Point2f>::iterator it = coords.begin(); it != coords.end(); ++it)
    {
      xsum += (*it).x;
      ysum += (*it).y;
    }

    float xavg = xsum / L;
    float yavg = ysum / L;
    for(vector<Point2f>::iterator it = coords.begin(); it != coords.end(); ++it)
    {
      (*it).x -= xavg;
      (*it).y -= yavg;
    }

    return coords; // return input, modified in place
  } // END OF FUNCTION normalize() #############################################


  vector<Point2f> normalize(vector<Point2f> coords)
  {
    // http://stackoverflow.com/questions/25251676/opencv-findfundamentalmat-very-unstable-and-sensitive
    cv::Matx31d P;
    cv::Matx31d Pprime;
    for(vector<Point2f>::iterator it = coords.begin(); it != coords.end(); ++it)
    {
      // P[1] = (*it).x;
      P(0, 0) = (*it).x;
      P(1, 0) = (*it).y;
      P(2, 0) = 1; // homogeneous coordinates

      Pprime = camera_matrix.inv() * P;
      (*it).x = Pprime(0, 0);
      (*it).y = Pprime(1, 0);
    }
    return coords; // return input, modified in place
  } // END OF FUNCTION normalize() #############################################


  Point2f uv_left_right(vector<Point2f> &prev_coords, vector<Point2f> &curr_coords)
  {
    // function to calculate the average (u, v) coordinate change from
    // frame-to-frame, used to estimate camera motion relative to world
    int L = prev_coords.size();
    float xsum = 0.0;
    float ysum = 0.0;

    vector<Point2f>::iterator it1 = prev_coords.begin(); // sizes should already
    vector<Point2f>::iterator it2 = curr_coords.begin(); // be verified equal
    for( ; it2 != curr_coords.end(); ++it1, ++it2)
    {
      xsum += (*it2).x - (*it1).x;
      ysum += (*it2).y - (*it1).y;
    }

    // calc average left/right tracked point movement
    // also apply deadband of 1 pixel, so we don't accrue unnecessary error
    float xavg = (abs(xsum/L) > 1 ? xsum/L : 0);
    float yavg = (abs(ysum/L) > 1 ? ysum/L : 0);

    return Point2f(xavg, yavg);
  } // END OF FUNCTION uv_left_right() #########################################


  Point2f uv_fore_aft(vector<Point2f> &prev_coords, vector<Point2f> &curr_coords)
  {
    // function to calculate the average (u, v) coordinate change from
    // frame-to-frame, used to estimate camera motion relative to world
    int L = prev_coords.size();
    float xsum = 0.0;
    float ysum = 0.0;

    vector<Point2f>::iterator it1 = prev_coords.begin(); // sizes should already
    vector<Point2f>::iterator it2 = curr_coords.begin(); // be verified equal
    for( ; it2 != curr_coords.end(); ++it1, ++it2)
    {
      xsum += (*it2).x - (*it1).x;
      ysum += (*it2).y - (*it1).y;
    }

    return Point2f(xsum/L, ysum/L);
  } // END OF FUNCTION uv_fore_aft() ###########################################

}; // END OF CLASS FlowCalculator ##############################################



int main(int argc, char** argv)
{
  ros::init(argc, argv, "dontknowwhattocallthisnode");
  FlowCalculator fc;
  ros::spin();
  return 0;
}
