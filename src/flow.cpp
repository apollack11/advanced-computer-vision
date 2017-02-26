#include <ros/ros.h>

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

#define MAX_INDICES 200
#define TRAIL_LENGTH 30


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
  vector<Point2f> prev_track_normalized;
  vector<Point2f> curr_track_normalized;
  string ColorWinName;
  string GrayWinName;
  cv_bridge::CvImagePtr in_ptr;

  // other stuff
  // vector<vector<Point2f> > tracking_indices;
  // vector<vector<Point2f> >::iterator trail_it;
  int counter;
  cv::Matx33d F; // Fundamental Matrix
  cv::Matx33d E; // Essential Matrix
  cv::Matx33d K; // Camera Matrix (from camera calibration file)
  cv::Mat D; // distorition coefficients (from camera calibration file)
  Mat out_img; // output image, marked up with flow points and stuff
  Matx33d W;
  Matx33d Winv;
  Matx34d P1;
  Mat R; // rotation matrix
  Mat t; // translation matrix


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
    // fs["camera_matrix"] >> K;
    // cout << "K (camera matrix):\n" << K << endl;
    // fs.release();

    // hack for now since can't initialize the way I want to
    cv::Matx33d ktmp(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    K = ktmp;
    // cv::Mat_ dtmp(-0.073669, 1.170392, 0.000976, -0.00244, 0.0);
    // D = dtmp;
    float D_data[5] = {-0.073669, 1.170392, 0.000976, -0.00244, 0.0};
    D = cv::Mat(1, 5, CV_32F, D_data);
    Matx33d Wtmp(0, -1, 0, 1, 0, 0, 0, 0, 1);
    W = Wtmp;
    Matx33d Winvtmp(0, 1, 0, -1, 0, 0, 0, 0, 1);
    Winv = Winvtmp;

  } // END OF CONSTRUCTOR

  ~FlowCalculator()
  {
    cv::destroyAllWindows();
    std::cout << "Nate Kaiser is the best. The end." << std::endl;
  } // END OF DESTRUCTOR

  void img_cb(const sensor_msgs::ImageConstPtr& input)
  {
    // grab the current frame from the camera stream
    try
    { in_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8); }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    curr_color = in_ptr->image;
    cv::cvtColor(curr_color, curr, CV_BGR2GRAY);


    if(!prev.data) // Check for invalid input
    {
      cout << "prev data not available, assigning to curr data" << std::endl;
      prev = curr;
    }

    // create vector of good points to track from previous image
    goodFeaturesToTrack(prev, prev_track_indices, MAX_INDICES, 0.2, 0.0);

    // find optical flow between previous and current images, store in curr_track_indices
    calcOpticalFlowPyrLK(prev, curr, prev_track_indices, curr_track_indices, flow_status, flow_errs);


    // undistort image (before calculating Fundamental Matrix)
    // THIS CAUSED IT TO BE REALLY SLOW AND LAGGY
    // undistortPoints(curr_track_indices, curr_track_undistorted, K, D);

    // try undistorting points instead (before calculating Fundamental Matrix)
    // http://stackoverflow.com/questions/25251676/opencv-findfundamentalmat-very-unstable-and-sensitive
    prev_track_normalized = normalize(prev_track_indices);
    curr_track_normalized = normalize(curr_track_indices);

    // syntax inspiration found at:
    // http://www.morethantechnical.com/2012/02/07/structure-from-motion-and-3d-reconstruction-on-the-easy-in-opencv-2-3-w-code/

    // comparison of findFundamentalMat solver techniques:
    // http://fhtagn.net/prog/2012/09/27/opencv-fundamentalmat.html
    // may be better to use LMedS instead of RANSAC...
    // for(int k = 0; k < 5; ++k)
    // {
    //   // F = findFundamentalMat(prev_track_indices, curr_track_undistorted, FM_RANSAC, 1, 0.99, flow_status);
      F = findFundamentalMat(prev_track_indices, curr_track_indices, FM_RANSAC, 1, 0.99, flow_status);
    //   cout << "F:\n" << F << endl;
    // }

    // xn = inv(K) * x
    // xn' = inv(K') * x'

    E = K.t() * F * K; // calculate essential matrix from fundamental matrix and camera matrix

    // helpful clarification:
    // http://stackoverflow.com/questions/16639106/camera-motion-from-corresponding-images
    SVD svd(E);
    R = svd.u * Mat(W) * svd.vt;
    t = svd.u.col(2);

    if(!(counter%15)) // every 1/2 second, print:
    {
      cout << "F:\n" << F << endl;
      cout << "E:\n" << E << endl;
      cout << "R:\n" << R << endl;
      cout << "t:\n" << t << endl;
    }

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
    cv::waitKey(30); // 30 Hz camera = 33.3 ms per callback loop

    prev = curr; // set for next iteration
    // cout << counter << endl;
    ++counter %= TRAIL_LENGTH;

  } // END OF FUNCTION img_cb()


  vector<Point2f> normalize(vector<Point2f> coords)
  {
    // vector<int> pointIndexes1;
    // pointNormalize1= K.inv()*pointIndexes1 where pointIndexes1(2), z is equal 1.

    // vector<Point2f> output(input.size());
    // int L = input.size();
    // vector<Point2f> output = *input;
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
  } // END OF FUNCTION normalize()

}; // END OF CLASS FlowCalculator



int main(int argc, char** argv)
{
  ros::init(argc, argv, "dontknowwhattocallthisnode");
  FlowCalculator fc;
  ros::spin();
  return 0;
}
