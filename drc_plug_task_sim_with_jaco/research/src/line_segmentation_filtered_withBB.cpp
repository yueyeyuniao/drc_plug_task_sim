#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include <image_geometry/pinhole_camera_model.h>
#include <vector>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;
double boundingbox_xmin, boundingbox_xmax, boundingbox_ymin, boundingbox_ymax;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Subscriber info_sub_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
 
  image_geometry::PinholeCameraModel pin_hole_camera;
  ros::Subscriber sub_yolo;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    info_sub_ = nh_.subscribe("/kinect2/qhd/camera_info", 10, &ImageConverter::info_callback, this);
    image_sub_ = it_.subscribe("/kinect2/qhd/image_color", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    sub_yolo = nh_.subscribe("darknet_ros/bounding_boxes", 1, &ImageConverter::yolo_callback, this);

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  void info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
  {
    pin_hole_camera.fromCameraInfo(msg);
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
// line segment
    Mat Gray;
    Mat image_canny;
    Mat filter_canny;
    cv::cvtColor(cv_ptr->image, Gray, CV_BGR2GRAY);  // CV_BGR2GRAY

    Canny(cv_ptr->image, image_canny, 50, 200, 3);
    cv::imshow("Image Canny", image_canny);

// new filter canny with bounding box
    Canny(cv_ptr->image, filter_canny, 50, 200, 3);
    cout<<image_canny.at<bool>(1,1)<<endl;



    for (int i=0; i<959; i++) { //959 y
      for (int j=0; j<539; j++){ //539 x
          filter_canny.at<bool>(j,i)=0;  
        }
    }

    cout << filter_canny.size << endl;
    
    for (int i=boundingbox_xmin; i<boundingbox_xmax; i++) { //959
      for (int j=boundingbox_ymin; j<boundingbox_ymax; j++){ //539
          filter_canny.at<bool>(j,i)=image_canny.at<bool>(j,i);  
        }
    }
    cv::imshow("Filter Canny", filter_canny);
 //

    Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
    double start = double(getTickCount());
    vector<Vec4f> lines_std;
    // Detect the lines
    ls->detect(filter_canny, lines_std);
    double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
    std::cout << "It took " << duration_ms << " ms." << std::endl;
    // Show found lines
    Mat drawnLines(filter_canny);
    ls->drawSegments(drawnLines, lines_std);
    cv::imshow("Standard refinement", drawnLines);
    cv::waitKey(300);
    image_pub_.publish(cv_ptr->toImageMsg());
  }
  
  void yolo_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
  { 
    if (msg->bounding_boxes.size() != 0)
    {
      if (msg->bounding_boxes[0].Class.compare("wire") == 0)
      {
        // cout << "find an object" << endl;
        boundingbox_xmin = msg->bounding_boxes[0].xmin;
        boundingbox_xmax = msg->bounding_boxes[0].xmax;
        boundingbox_ymin = msg->bounding_boxes[0].ymin;
        boundingbox_ymax = msg->bounding_boxes[0].ymax;
      }
    }
  }  

 };


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
