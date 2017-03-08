// ROS and related stuff:
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

// for subscribing to compressed image topics:
#include <image_transport/image_transport.h>
#include "compressed_image_transport/compressed_subscriber.h"
#include "compressed_image_transport/compression_common.h"

// core OpenCV stuff:
#include <cv_bridge/cv_bridge.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/video/tracking.hpp> // calcOpticalFlowPyrLK

// core c++ headers:
#include <iostream>
#include <vector>
#include <cmath> // sqrt
#include <iomanip> // std::setw
#include <algorithm> // a few things


#define PI 3.1416 // mmm, delicious pi
#define MAX_POINTS 300 // max # of points to track w/ optical flow
#define TRAIL_LENGTH 30 // not currently being used for any real purposes
#define CAM_PIX_U 640 // pixels
#define CAM_PIX_V 480 // pixels
#define CAM_HEIGHT 0.238 // meters from lens center to ground
#define CAM_RADIAL 0.2413 // meters radial distance from center of robot
#define CAM_DEG 45 // degrees from horizontal
#define CAM_RAD 0.7854 // radians from horizontal
#define CAM_M_U 0.1334 // meters subtended by camera in u direction (wider of the two)
#define CAM_M_V 0.1334 // meters subtended by camera in v direction (narrower of the two) - THIS IS MAGICALLY THE SAME
#define CAM_DEG_U 23.86 // degrees subtended by camera in u direction (wider of the two)
#define CAM_DEG_V 18.15 // degrees subtended by camera in v direction (narrower of the two)
#define CAM_RAD_U 0.416 // radians subtended by camera in u direction (wider of the two)
#define CAM_RAD_V 0.317 // radians subtended by camera in v direction (narrower of the two)
#define PIX_DEG_U 0.03728 // degrees subtended by each pixel in u direction
#define PIX_DEG_V 0.03781 // degrees subtended by each pixel in v direction
#define PIX_RAD_U 0.00065 // radians subtended by each pixel in u direction
#define PIX_RAD_V 0.00066 // radians subtended by each pixel in v direction


using namespace std;
using namespace cv;


class FlowCalculator
{
private:
  // set up ROS
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub;
  ros::Publisher pose_pub;
  ros::Publisher twist_pub;

  // image structures and indices:
  cv::Mat curr_color;
  cv::Mat curr;
  cv::Mat prev;
  vector<Point2f> curr_track_indices;
  vector<Point2f> prev_track_indices;
  vector<Point2f> prev_track_centered;
  vector<Point2f> curr_track_centered;
  vector<Point2f> curr_track_undistorted;
  vector<Point2f> prev_track_undistorted;

  // optical flow, Fundamental & Essential matrices:
  vector<float> flow_errs;
  vector<unsigned char> flow_status;
  vector<unsigned char> F_indices_mask; // outliers from RANSAC or LMedS when finding F
  cv::Matx33d F; // Fundamental Matrix
  cv::Matx33d E; // Essential Matrix

  // setup and generic stuff:
  int counter;
  string ColorWinName;
  // string GrayWinName;
  Mat out_img; // output image, marked up with flow points and stuff

  // camera calibration data:
  cv::Matx33d camera_matrix; // Camera Matrix (from calibration file)
  cv::Mat distortion_coefficients; // distortion coefficients (from calibration file)
  cv::Mat rectification_matrix; // rectification matrix (from calibration file)
  cv::Mat projection_matrix; // projection matrix (from calibration file)
  // W MUST BE A Matx33d OR THE ROTATION MATRIX WILL COME OUT WRONG (e-280 ISSUE)
  // INVESTIGATE LATER, NO TIME TO DIVE INTO THIS NOW
  Matx33d W; // multiview geometry eqn 9.13, H&Z
  Mat R; // rotation matrix
  Mat t; // translation vector

  // containers for output and motion estimate values:
  Point2f accumulated_xy;
  float accumulated_heading;
  float accumulated_travel;
  Point3f accumulated_motion;
  Point3f derpz;
  geometry_msgs::Pose2D pose_out;
  geometry_msgs::Twist twist_out;

  // testing out Farneback's instead of LK to solve OF:
  Mat curr_track_indices_mat;
  // cv::Mat H; // Perspective Transformation (Homography) Matrix
  // H MUST BE A Matx33d
  // INVESTIGATE LATER, NO TIME TO DIVE INTO THIS NOW
  cv::Matx33d H; // Perspective Transformation (Homography) Matrix

  double t_curr;
  double t_prev;

  geometry_msgs::Twist filter[5];
  int filter_count;


public:
  FlowCalculator()
  : it_(nh_)
  {
    std::cout << "instance of FlowCalculator class instantiated" << std::endl;

    // subscribe to input video stream from camera
    image_sub = it_.subscribe("/usb_cam/image_raw", 1, &FlowCalculator::img_cb, this, image_transport::TransportHints("compressed"));
    // image_sub = it_.subscribe("/camera/image_color", 1, &FlowCalculator::img_cb, this, image_transport::TransportHints("compressed"));

    // publish output pose estimate to EKF
    pose_pub = nh_.advertise<geometry_msgs::Pose2D>("/optical_flow/pose", 1);
    twist_pub = nh_.advertise<geometry_msgs::Twist>("/optical_flow/twist", 1);

    // create a single window instance, overwrite each loop
    ColorWinName = "Color Output Window";
    cv::namedWindow(ColorWinName, WINDOW_AUTOSIZE);

    // hack these for now since can't initialize the way I want to (stupid pre-c++11!)
    double camera_matrix_data[9] = {1149.322298, 0.0, 351.778662, 0.0, 1151.593614, 276.459807, 0.0, 0.0, 1.0};
    camera_matrix = cv::Mat(3, 3, CV_64F, camera_matrix_data);

    float distortion_coefficients_data[5] = {-0.073669, 1.170392, 0.000976, -0.00244, 0.0};
    distortion_coefficients = cv::Mat(1, 5, CV_32F, distortion_coefficients_data);

    double Wtmp_data[9] = {0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    W = cv::Mat(3, 3, CV_64F, Wtmp_data);

    float rectification_matrix_data[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    rectification_matrix = cv::Mat(3, 3, CV_32F, rectification_matrix_data);

    float projection_matrix_data[12] = {1160.519653, 0.0, 349.420934, 0.0, 0.0, 1164.307007, 275.445505, 0.0, 0.0, 0.0, 1.0, 0.0};
    projection_matrix = cv::Mat(3, 4, CV_32F, projection_matrix_data);

    double H_data[9] = {0.0002347417933653588, -9.613823951336309e-20, -0.07500000298023225, -7.422126200315807e-19, -0.0002818370786240783, 0.5159999728202818, 1.683477982667922e-19, 5.30242624981192e-18, 1};

    H = cv::Mat(3, 3, CV_64F, H_data);

    // accumulated_xy = Point2f(0.0, 0.0);
    accumulated_motion = Point3f(0.0, 0.0, 0.0);
    // accumulated_heading = 0.0;
    // accumulated_travel = 0.0;

    counter = 0.0;

    // THIS IS JUST FOR DEVELOPMENT PURPOSES:
    // const Point2f dataz1[] = {Point2f(0.0, 0.0), Point2f(639.0, 0.0), Point2f(0.0, 479.0), Point2f(639.0, 479.0)};
    // const Point2f dataz2[] = {Point2f(-0.075, 0.516), Point2f(0.075, 0.516), Point2f(-0.075, 0.381), Point2f(0.075, 0.381)};
    // const Point2f dataz2[] = {Point2f(0.516, -0.075), Point2f(0.516, 0.075), Point2f(0.381, -0.075), Point2f(0.381, 0.075)};

    // float turn_rad = 0.381 // meters, from center of robot to base of visible trapezoid
    // float trap_base = 0.12; // meters, base of trapezoid visible to camera
    // float trap_top = 0.15; // meters, top of trapezoid visible to camera
    // float trap_height = 0.135; // meters, h

    // cout << "HOMOGRAPHY / PERSPECTIVE PROJECTION = \n" << cv::getPerspectiveTransform(dataz1, dataz2) << endl;

    t_prev = ros::Time::now().toSec();

    geometry_msgs::Twist blah;
    blah.linear.x = 0;
    blah.linear.y = 0;
    blah.linear.z = 0;
    blah.angular.x = 0;
    blah.angular.y = 0;
    blah.angular.z = 0;

    for(int i = 0; i < 5; ++i)
    {
      filter[i] = blah;
    }
    filter_count = 0;

  } // END OF CONSTRUCTOR ######################################################


  ~FlowCalculator()
  {
    cv::destroyAllWindows();
    std::cout << "Nate Kaiser is the best. The end." << std::endl;
  } // END OF DESTRUCTOR #######################################################


  void img_cb(const sensor_msgs::ImageConstPtr& input)
  {
    // grab current frame from camera stream
    try
    {
      curr_color = (cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8))->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::cvtColor(curr_color, curr, CV_BGR2GRAY); // create curr (grayscale version of the input image)

    if(!prev.data) // Check for invalid input, also handles 1st time being called
    {
      ROS_WARN("prev image data not available, assigning to curr data");
      curr.copyTo(prev); // deep copy, none of that shared pointer stuff
      return;
    }


// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// BEGIN LK METHOD >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

    if(prev_track_indices.size() < 0.75*MAX_POINTS) // if enough tracking indices are dropped, calculate a new set
    {
      // create vector of good points to track from previous image
      goodFeaturesToTrack(prev, prev_track_indices, MAX_POINTS, 0.1, 5.0);
    }

    if(prev_track_indices.empty()) // check, even though we shouldn't have this problem
    {
      ROS_WARN("no tracking objects found");
      curr.copyTo(prev); // deep copy, none of that shared pointer stuff
      return;
    }

    // find optical flow between previous and current images
    calcOpticalFlowPyrLK(prev, curr, prev_track_indices, curr_track_indices, flow_status, flow_errs, Size(21,21), 4);

    // NOT SURE WE NEED THIS CHECK, BUT ONE OF MY CUSTOM FUNCTIONS STATES WE HAVE IT:
    if(curr_track_indices.size() != prev_track_indices.size())
    { ROS_ERROR("tracking index data size different between previous and current images"); }

    // Point2f derp = uv_left_right(prev_track_indices, curr_track_indices);
    // float derpdyderp = estimate_heading(prev_track_indices, curr_track_indices);
    // THIS PART CURRENTLY ONLY WORKS WELL WHEN THERE ARE > 50 TRACKING POINTS
    // AND ALSO AT ~0.5 TURNING SPEED. AT 0.1 IT OVERESTIMATED (90deg WAS MEASURED AS 120deg)

    // float derp2 = uv_fore_aft(prev_track_indices, curr_track_indices);
    // float derp2 = estimate_travel(prev_track_indices, curr_track_indices);

    derpz = estimate_motion(prev_track_indices, curr_track_indices);
    accumulated_motion += derpz;
    cout << "ACCUMULATED MOVEMENT = " << accumulated_motion << ", ANGLE = " << accumulated_motion.x / (2 * PI * 0.4485) * 360 * 1.57 << endl;
    // x movement multiplied by circumference (2 * PI * Radius) multiplied by 360 (to convert to degrees)
    // also multiplied by a constant of 1.57 based on test data
    // cout << "ANGLE = " << accumulated_motion.x / (2 * PI * 0.4485) * 360 * 1.57 << endl;
    // accumulated_travel += derp2;
    // accumulated_heading += derpdyderp * 57.29; // converted to degrees just for visualization for now

    // if(derpdyderp > 0.001 || derp2 > 0.1)
    // {
      // cout << "(#=" << prev_track_indices.size() << ")\tyaw, forward motion = " << setw(10) << accumulated_heading << ", " << setw(10) << accumulated_travel << endl;
    // }

    // comparison of findFundamentalMat solver techniques:
    // http://fhtagn.net/prog/2012/09/27/opencv-fundamentalmat.html
    // per this, may be better to use LMedS instead of RANSAC...
    F = findFundamentalMat(prev_track_indices, curr_track_indices, FM_RANSAC, 1, 0.99, flow_status);
    E = camera_matrix.t() * F * camera_matrix; // calculate Essential matrix from Fundamental matrix and camera matrix

    // helpful clarification:
    // http://stackoverflow.com/questions/16639106/camera-motion-from-corresponding-images
    // syntax inspiration found at:
    // http://www.morethantechnical.com/2012/02/07/structure-from-motion-and-3d-reconstruction-on-the-easy-in-opencv-2-3-w-code/
    SVD svd(E);
    R = svd.u * Mat(W) * svd.vt;
    t = svd.u.col(2);
    // cout << "R:\n" << R << '\n' << endl;

    double roll = atan2(R.at<float>(2, 1), R.at<float>(2, 2));
    double pitch = asin(R.at<float>(2, 0));
    double yaw = -atan2(R.at<float>(1, 0), R.at<float>(0, 0));

    // cout << "Trace(R) = " << traceof(R) << endl;
    // if(traceof(R) > 0)
    // {
    //   // cout << "RPY: " << setw(15) << roll << setw(15) << pitch << setw(15) << yaw << endl;// '\t' << R.type() << endl;
    //   cout << "R:\n" << R << '\n' << endl;
    // }

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
    //   cout << "F:\n" << F << endl;
    //   cout << "E:\n" << E << endl;
    //   cout << "R:\n" << R << endl;
    //   cout << "Trace(R) = " << traceof(R) << endl;
    //   cout << "t:\n" << t << endl;
    // }

    // package and send output pose to EKF
    // pose_out.x = accumulated_motion.y;
    // pose_out.y = 0;
    // pose_out.theta = accumulated_motion.x / (2 * PI * 0.4485) * 2 * PI * 1.57;
    // pose_pub.publish(pose_out);

    // trying to do twist output instead:
    t_curr = ros::Time::now().toSec();
    twist_out.linear.x = derpz.y / (t_curr - t_prev);
    twist_out.angular.z = derpz.x / (2 * PI * 0.4485) * 2 * PI * 1.57 / (t_curr - t_prev);

    filter[filter_count] = twist_out;

    twist_out.linear.x = 0;
    twist_out.linear.y = 0;
    twist_out.linear.z = 0;
    twist_out.angular.x = 0;
    twist_out.angular.y = 0;
    twist_out.angular.z = 0;
    for(int i = 0; i < 5; ++i)
    {
      twist_out.linear.x += filter[i].linear.x / 5;
      twist_out.linear.y += filter[i].linear.y / 5;
      twist_out.linear.z += filter[i].linear.z / 5;
      twist_out.angular.x += filter[i].angular.x / 5;
      twist_out.angular.y += filter[i].angular.y / 5;
      twist_out.angular.z += filter[i].angular.z / 5;
    }



    twist_pub.publish(twist_out);

    ++filter_count %= 5;


    // draw tracked points for visualization purposes
    out_img = curr_color; // copy over so we can draw tracked points over top
    // for(int i = 0; i < curr_track_indices.size(); ++i)
    // {
    //   circle(out_img, curr_track_indices[i], 3, Scalar(0, 255, 0), -1); // -1 = filled
    // }

    // draw epipolar lines for visualization purposes
    std::vector<cv::Vec<float, 3> > epilines1, epilines2;
    cv::computeCorrespondEpilines(prev_track_indices, 1, F, epilines1); //Index starts with 1
    cv::computeCorrespondEpilines(curr_track_indices, 2, F, epilines2);

    CV_Assert(prev_track_indices.size() == epilines1.size() && epilines1.size() == epilines2.size());

    cv::RNG rng(0);
    for(int i = 0; i < prev_track_indices.size(); i++)
    {
      // Epipolar lines of the 1st point set are drawn in the 2nd image and vice-versa
      cv::Scalar color(rng(256),rng(256),rng(256));

      // cv::line(out_img, cv::Point(0,-epilines1[i][2]/epilines1[i][1]), cv::Point(prev.cols,-(epilines1[i][2]+epilines1[i][0]*prev.cols)/epilines1[i][1]), color);
      // cv::circle(out_img, curr_track_indices[i], 3, color, -1, CV_AA);
      cv::circle(out_img, curr_track_indices[i], 3, Scalar(0, 255, 0), -1, CV_AA);
    }

    imshow(ColorWinName, out_img);
    cv::waitKey(30); // 30 Hz camera = 33.3 ms per callback loop, hold for 30

    curr.copyTo(prev); // deep copy, none of that shared pointer stuff

    prev_track_indices = curr_track_indices; // deep copy, MAKE SURE TO COMMENT OUT IF I REVERT TO CALLING goodFeaturesToTrack EACH LOOP

    // this function emulates std::remove_if, which is technically only available
    // in c++ version 11 or greater, and also does not work with OpenCV types
    vector<Point2f>::iterator first = prev_track_indices.begin();
    vector<Point2f>::iterator new_start = first;
    vector<Point2f>::iterator last = prev_track_indices.end();
    while(first!=last)
    {
      if ((*first).x < 0.0 || (*first).x > CAM_PIX_U || (*first).y < 0.0 || (*first).y > CAM_PIX_V)
      {
        // cout << "made it into the mystical for loop!" << endl;
        // cout << "swapping this:" << *new_start;
        // *new_start = *first;
        prev_track_indices.erase(first);
        //
        // cout << " and this:" << *new_start << endl;
        // ++new_start;
      }
      ++first;
      // return new_start;
    }
    // cout << "made it here" << endl;

    prev_track_indices.begin() = new_start;

    // THIS IS SEGFAULTING RIGHT NOW, FIGURE IT OUT LATER
    // RemoveOutOfBounds(prev_track_indices);

    ++counter %= TRAIL_LENGTH;
    t_prev = t_curr;



// // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// // BEGIN FARNEBACK METHOD >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//
//     // 0.4- image pyramid or simple image scale
//     // 1 is number of pyramid layers. 1 mean that flow is calculated only from previous image.
//     // 12 is win size.. Flow is computed over the window larger value is more robust to the noise.
//     // 2 mean number of iteration of algorithm
//     // 8 is polynomial degree expansion recommended value are 5 - 7
//     // 1.2 standard deviation used to smooth used derivatives recommended values from 1.1 - 1,5
//     calcOpticalFlowFarneback(prev, curr, curr_track_indices_mat, 0.5, 4, 21, 3, 7, 1.2, 0);
//     // cout << "curr_track_indices_mat SIZE:\n" << curr_track_indices_mat.size() << '\n' << endl;
//
//     // F = findFundamentalMat(prev_track_centered, curr_track_centered, FM_RANSAC, 0.01, 0.99, F_indices_mask); // 0.01 is guess
//     // cout << "F:\n" << F << endl;
//
//     out_img = curr_color; // copy over so we can draw tracked points over top
//     for(int y = 0; y < curr_track_indices_mat.rows; y += 10)
//     {
//       for(int x = 0; x < curr_track_indices_mat.cols; x += 10)
//       {
//         // get the flow from y, x position * 10 for better visibility
//         const Point2f flowatxy = curr_track_indices_mat.at<Point2f>(y, x) * 2;
//         // draw line at flow direction
//
//         line(out_img, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(0,255,0));
//         // draw initial point
//         circle(out_img, Point(x, y), 1, Scalar(0, 0, 0), -1);
//       }
//     }
//
//     // draw the results
//     imshow(ColorWinName, out_img);
//     cv::waitKey(30); // 30 Hz camera = 33.3 ms per callback loop
//
//     curr.copyTo(prev); // deep copy, none of that shared pointer stuff
//
//     ++counter %= TRAIL_LENGTH;

  } // END OF FUNCTION img_cb() ################################################





// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// BEGIN MEMBER FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

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
  } // END OF FUNCTION centerData() ############################################


  vector<Point2f> normalize(vector<Point2f> coords)
  {
    // http://stackoverflow.com/questions/25251676/opencv-findfundamentalmat-very-unstable-and-sensitive
    cv::Matx31d P;
    cv::Matx31d Pprime;
    for(vector<Point2f>::iterator it = coords.begin(); it != coords.end(); ++it)
    {
      P(0, 0) = (*it).x;
      P(1, 0) = (*it).y;
      P(2, 0) = 1; // homogeneous coordinates

      Pprime = camera_matrix.inv() * P;
      (*it).x = Pprime(0, 0);
      (*it).y = Pprime(1, 0);
    }
    return coords; // return input, modified in place

    // TRIED THIS BUT DON'T FEEL LIKE WASTING TIME WHEN I ALREADY HAVE A SOLUTION
    // vector<Point3f> homogeneous_coords;
    // convertPointsToHomogeneous(coords, homogeneous_coords);
    // for(vector<Point3f>::iterator it = homogeneous_coords.begin(); it != homogeneous_coords.end(); ++it)
    // {
    //   *it = camera_matrix.inv() * (*it);
    // }
    // cout << "homogeneous_coords:\n" << homogeneous_coords << endl;
    // would still have to run convertPointsFromHomogeneous() to get 2D point back

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
    float xavg = (fabs(xsum/L) > 1 ? xsum/L : 0);
    float yavg = (fabs(ysum/L) > 1 ? ysum/L : 0);

    return Point2f(xavg, yavg);
  } // END OF FUNCTION uv_left_right() #########################################


  Point3f estimate_motion(vector<Point2f> &prev_coords, vector<Point2f> &curr_coords)
  { // function to calculate the rotational & translational motion from frame-to-frame

    int N = prev_coords.size();
    vector<Point3f> curr_homog;
    vector<Point3f> prev_homog;
    convertPointsToHomogeneous(curr_coords, curr_homog);
    convertPointsToHomogeneous(prev_coords, prev_homog);

    Point3f motion_avg = Point3f(0.0, 0.0, 0.0);

    // calc total left/right tracked point movement
    vector<Point3f>::iterator it1 = prev_homog.begin(); // sizes should already
    vector<Point3f>::iterator it2 = curr_homog.begin(); // be verified equal
    for( ; it2 != curr_homog.end(); ++it1, ++it2)
    { // calculates robot coordinates from camera coordinates
      motion_avg += Point3f((H * Mat(*it2) - H * Mat(*it1)) / N); // you can tell it's the average by the way that it is!
    }

    // correct y-axis (fore-aft) sign (so forward motion = positive value)
    // factor for speed of 0.1 (from spreadsheet)
    motion_avg.y *= -1.57 * 4.0 / 3.0;

    // now apply deadband of 0.5 mm or so (so we don't accrue unnecessary noise errors)
    if(fabs(motion_avg.x) > 0.0005 || fabs(motion_avg.y) > 0.0005)
    {
      return motion_avg;
    }
    else
    {
      return Point3f(0.0, 0.0, 0.0);
    }
  } // END OF FUNCTION estimate_motion() #######################################


  float estimate_heading(vector<Point2f> &prev_coords, vector<Point2f> &curr_coords)
  { // function to calculate the rotational motion from frame-to-frame
    float turn_rad = 0.381; // meters, from center of robot to base of visible trapezoid
    float trap_base = 0.12; // meters, base of trapezoid visible to camera
    float trap_top = 0.15; // meters, top of trapezoid visible to camera
    float trap_height = 0.135; // meters, height of trapezoid visible to camera

    int N = prev_coords.size();
    float tang_dist = 0.0;

    // calc total left/right tracked point movement
    vector<Point2f>::iterator it1 = prev_coords.begin(); // sizes should already
    vector<Point2f>::iterator it2 = curr_coords.begin(); // be verified equal
    for( ; it2 != curr_coords.end(); ++it1, ++it2)
    {
      // this calculates total tangential distance traveled in meters
      tang_dist += ((*it2).x - (*it1).x) / CAM_PIX_U * trap_top - (trap_top - trap_base) * ((*it2).y + (*it1).y) / 2 / CAM_PIX_V;
    }

    // now calculate average and apply deadband of 0.001 m
    // (so we don't accrue unnecessary noise errors)
    // float uavg = (fabs(tang_dist/N) > 0.001 ? tang_dist/N : 0);
    float uavg = tang_dist/N;

    // return movement IN RADIANS
    return uavg / CAM_PIX_U * CAM_M_U / CAM_RADIAL; // again, this is in RADIANS
  } // END OF FUNCTION estimate_heading() ######################################


  float estimate_travel(vector<Point2f> &prev_coords, vector<Point2f> &curr_coords)
  { // function to calculate the fore/aft distance driven from frame-to-frame
    int N = prev_coords.size();
    float vsum = 0.0;

    // calc total left/right tracked point movement IN PIXELS
    vector<Point2f>::iterator it1 = prev_coords.begin(); // sizes should already
    vector<Point2f>::iterator it2 = curr_coords.begin(); // be verified equal
    for( ; it2 != curr_coords.end(); ++it1, ++it2)
    {
      vsum += (*it2).y - (*it1).y;
    }

    // now calculate average and apply deadband of 1 pixel
    // (so we don't accrue unnecessary noise errors)
    float vavg = (fabs(vsum/N) > 1 ? vsum/N : 0);

    // return fore/aft movement IN METERS
    return vavg / CAM_PIX_V * CAM_M_U;

  } // END OF FUNCTION estimate_travel() #######################################


  float uv_fore_aft(vector<Point2f> &prev_coords, vector<Point2f> &curr_coords)
  {
    // function to calculate the z coordinate change from frame-to-frame
    // used to estimate camera motion relative to world


    // int top = CAM_PIX_V * 2/3;
    // int bot = CAM_PIX_V * 1/3; // ignoring middle 1/3 of image
    // int top_count = 0;
    // int bot_count = 0;
    //
    // int L = prev_coords.size();
    // float ysum_top = 0.0;
    // float ysum_bot = 0.0;
    //
    // float y_prev;
    // vector<Point2f>::iterator it1 = prev_coords.begin(); // sizes should already
    // vector<Point2f>::iterator it2 = curr_coords.begin(); // be verified equal
    // for( ; it2 != curr_coords.end(); ++it1, ++it2)
    // {
    //   // loop through and find y-component of motion for all matching points
    //   y_prev = (*it1).y;
    //   if(y_prev < bot)
    //   {
    //     ysum_bot += (*it2).y - (*it1).y;
    //     ++bot_count;
    //   }
    //   else if(y_prev > top)
    //   {
    //     ysum_top += (*it2).y - (*it1).y;
    //     ++top_count;
    // if(top_count < 15 || bot_count < 15)
    // {
    //   return Point2f(0.0, 0.0); // not enough valid points to get a good average
    // }
    // else
    // {
    //   float y_avg = ysum_bot/bot_count - ysum_top/top_count;
    //   return Point2f(0.0, y_avg);
    // }


    // DO RADIAL INSTEAD OF TOP/BOT DIFFERENTIAL
    float x_prev;
    float y_prev;
    float x_curr;
    float y_curr;
    float x_mid;
    float y_mid;

    double v_len;
    float x_radial;
    float y_radial;
    float radial;
    double radial_tot = 0;

    vector<Point2f>::iterator it1 = prev_coords.begin(); // sizes should already
    vector<Point2f>::iterator it2 = curr_coords.begin(); // be verified equal
    for( ; it2 != curr_coords.end(); ++it1, ++it2)
    { // loop through and find y-component of motion for all matching points
      x_prev = (*it1).x;
      y_prev = (*it1).y;
      x_curr = (*it2).x;
      y_curr = (*it2).y;
      x_mid = (x_curr - x_prev)/2;
      y_mid = (y_curr - y_prev)/2;
      v_len = sqrt(pow(x_mid - CAM_PIX_U/2, 2) + pow(y_mid - CAM_PIX_V/2, 2));

      x_radial = (2 * x_mid) * (x_mid - CAM_PIX_U/2)/v_len;
      y_radial = (2 * y_mid) * (y_mid - CAM_PIX_V/2)/v_len;
      radial = x_radial + y_radial;
      radial_tot += radial;
    }

    double radial_avg = radial_tot/prev_coords.size(); // only care about average radial motion
    return (radial_avg > 5 ? radial_avg : 0); // deadband of 5 pixels
  } // END OF FUNCTION uv_fore_aft() ###########################################


  double traceof(cv::Mat &input)
  {
    // function to calculate the trace of a matrix
    // from http://math.stackexchange.com/questions/1737931/robustly-map-rotation-matrix-to-axis-angle:
    // A fully robust approach will use different code when t, the trace of the
    // matrix Q, is negative, as with quaternion extraction.
    int N = input.rows;
    if(N != input.cols)
    {
      ROS_ERROR("Can't find trace of non-square matrix!");
      return 0;
    }

    double sum = 0.0;
    for(int i = 0; i < N; ++i)
    {
      sum += input.at<double>(i, i);
    }
    return sum;
  } // END OF FUNCTION traceof() ###############################################

  void RemoveOutOfBounds(vector<Point2f> &v)
  {
    // this function emulates std::remove_if, which is technically
    // only available in c++ version 11 or greater, and also does
    // not work with OpenCV types anyway
    for(vector<Point2f>::iterator it = v.begin(); it != v.end(); ++it)
    {
      if((*it).x < 0.0 || (*it).x > CAM_PIX_U || (*it).y < 0.0 || (*it).y > CAM_PIX_V)
      {
        prev_track_indices.erase(it);
      }
    }

    return;
  } // END OF FUNCTION RemoveOutOfBounds() #####################################

}; // END OF CLASS FlowCalculator ##############################################



int main(int argc, char** argv)
{
  ros::init(argc, argv, "reallydontknowwhattocallthisnode");
  // ros::param::set("_image_transport", "compressed");
  FlowCalculator fc;
  ros::spin();
  return 0;
}
