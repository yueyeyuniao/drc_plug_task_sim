#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <kinova_driver/kinova_ros_types.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <kinova_driver/kinova_ros_types.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>

static const std::string OPENCV_WINDOW = "Image window";
const double FINGER_MAX = 6400;
using namespace cv;
using namespace std;
cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImageConstPtr depth_img_cv;
geometry_msgs::Pose target_pose;
geometry_msgs::Pose target_pose2;
double boundingbox_xmin, boundingbox_xmax, boundingbox_ymin, boundingbox_ymax;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Subscriber info_sub_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher display_publisher;
  ros::Publisher point_cloud_ziptie_pub;
  ros::Subscriber sub_yolo;

  Mat imgThresholded;
  int iLowH = 0;
  int iHighH = 0;

  int iLowS = 0; 
  int iHighS = 0;

  int iLowV = 0;
  int iHighV = 0;

  vector<int> pixel_x; // x of wire in the 2d image
  vector<int> pixel_y; // y of wire in the 2d image
  vector<float> co_x; // x coordinate in 3d with respect to camera
  vector<float> co_y; // y coordinate in 3d with respect to camera
  vector<float> co_z; // z coordinate in 3d with respect to camera

  image_geometry::PinholeCameraModel pin_hole_camera;
  int grasp_point_index_1;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    info_sub_ = nh_.subscribe("/kinect2/qhd/camera_info", 10, &ImageConverter::info_callback, this);
    image_sub_ = it_.subscribe("/kinect2/qhd/image_color_rect", 1, &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("/kinect2/qhd/image_depth_rect", 1, &ImageConverter::depthCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    sub_yolo = nh_.subscribe("darknet_ros/bounding_boxes", 1, &ImageConverter::yolo_callback, this);
    display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true); //why can not use it_
    point_cloud_ziptie_pub = nh_.advertise<PointCloud> ("/wire" , 10);

    // namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    // //Create trackbars in "Control" window
    // cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    // cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    // cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    // cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    // cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    // cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

  void info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
  {
    pin_hole_camera.fromCameraInfo(msg);
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    /////////////// color detection

    Mat imgHSV;
    cv::cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    // Mat imgThresholded;

    // inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
    //HSV color space is the most suitable color space for color based image segmentation.
    inRange(imgHSV, Scalar(0,0,0,0), Scalar(180,255,33.16,0), imgThresholded);
    // inRange(imgHSV, Scalar(0,0,0), Scalar(0,0,0), imgThresholded);

      
    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );


    // cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
    // cv::imshow("Original", cv_ptr->image); //show the original image

    // if (waitKey(300) == 17) //wait for 'esc' key press for 300ms. If 'esc' key is pressed, break loop
    //    {
    //         cout << "esc key is pressed by user" << endl;
    //         exit(1);
    //    }

  }
  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {

    try
    {
    depth_img_cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  } 

  void imageProcessing()
  {
    // while (ros::ok()){
    //   ros::spinOnce();
    pixel_x.clear();
    pixel_y.clear();
    for (int i=boundingbox_xmin; i<boundingbox_xmax; i++) { //959
      for (int j=boundingbox_ymin; j<boundingbox_ymax; j++){ //539
        // cout << imgThresholded.at<bool>(i,j) << "," << endl;
        // cout << imgThresholded.size<< endl;
        if (imgThresholded.at<bool>(j,i) != 0) {
          // cout << "here" << i << " " << j << endl;
          pixel_x.push_back(i);
          pixel_y.push_back(j);
          cout << "(" << i << "," << j << ")" << endl;
        }
      }
    }


    //new manually visualize the x and y values of 2d image of wire
      // pixel_x.clear();
      // pixel_y.clear();
      // for (int i=108; i<189; i++){
      //     pixel_x.push_back(431);
      //     pixel_y.push_back(i);

      // }

      int pixel_size = pixel_x.size();
      for (int i = 0; i < pixel_size; i++){
        cv::Point2d pixel_point(pixel_x[i], pixel_y[i]); 
        float depth = depth_img_cv->image.at<short int>(pixel_point);
        depth = depth;
        //cout << "depth" << depth<< endl;
        if (depth > 0.2){
          cv::Point3d xyz =pin_hole_camera.projectPixelTo3dRay(pixel_point);
          cv::Point3d coordinate = xyz * depth;
          co_x.push_back(coordinate.x/1000);
          co_y.push_back(coordinate.y/1000);
          co_z.push_back(coordinate.z/1000-0.14);  

          ofstream myfile;
          myfile.open ("/home/pengchang/Desktop/wire.txt", std::ofstream::out | std::ofstream::app); //append mode
          for (int i=0; i<co_x.size(); i++)
          {
            cout << "co_x:" << co_x[i] << "co_y:" << co_y[i] << "co_z:" << co_z[i] << endl;
            myfile << co_x[i] << " " << co_y[i] << " " << co_z[i] << endl;
          }
          myfile.close();
        } 

      } 
      	grasp_point_index_1 = (int)(pixel_x.size()/20);
        //cout << "ixyz " <<" i:" << grasp_point_index_1 << "x:" << co_x[grasp_point_index_1] << "y:"<< co_y[grasp_point_index_1] << "z:" << co_z[grasp_point_index_1] << ";" << endl;

        tf::Transform transform = tf_transform_calculation(co_x[grasp_point_index_1], co_y[grasp_point_index_1], co_z[grasp_point_index_1]);
        tf::Quaternion quaternion = tf_quaternion_calculation();
        target_pose.position.x = transform.getOrigin().x();
        target_pose.position.y = transform.getOrigin().y();
        target_pose.position.z = transform.getOrigin().z();
        target_pose.orientation.x = quaternion.getX(); 
        target_pose.orientation.y = quaternion.getY(); 
        target_pose.orientation.z = quaternion.getZ(); 
        target_pose.orientation.w = quaternion.getW(); 
        //cout << "XYZ" << "X:" << target_pose.position.x << "Y:" << target_pose.position.y << "Z:" << target_pose.position.z << endl;
// test 
        // tf::Transform transform = tf_transform_calculation(0, 0, 0.6);
        // tf::Quaternion quaternion = tf_quaternion_calculation();
        // target_pose.position.x = transform.getOrigin().x();
        // target_pose.position.y = transform.getOrigin().y();
        // target_pose.position.z = transform.getOrigin().z();
        // target_pose.orientation.x = quaternion.getX(); 
        // target_pose.orientation.y = quaternion.getY(); 
        // target_pose.orientation.z = quaternion.getZ(); 
        // target_pose.orientation.w = quaternion.getW(); 
        target_pose2 = target_pose;
        target_pose2.position.z = target_pose.position.z + 0.3;



    // }
  }

  tf::Transform tf_transform_calculation(float x, float y, float z)
  {
    //calculate transform from /endeffector to /object
    tf::TransformListener listener;
    tf::StampedTransform stampedtransform_transform;
    listener.waitForTransform("/root", "/kinect2_rgb_optical_frame", ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform("/root", "/kinect2_rgb_optical_frame", ros::Time(0), stampedtransform_transform); // target frame to source frame
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf::Transform ntransform;
    ntransform = stampedtransform_transform * transform;
    return ntransform;
  }

  tf::Quaternion tf_quaternion_calculation()
  {   
    //calculate quaternion of the object
    tf::TransformListener listener_robot;
    tf::StampedTransform stampedtransform_quaternion;
    listener_robot.waitForTransform("/root", "/j2n6s300_end_effector", ros::Time::now(), ros::Duration(3.0)); // use "/j2n6s300_end_effector" if you want to keep the orientation of the endeffector
    listener_robot.lookupTransform("/root", "/j2n6s300_end_effector", ros::Time(0), stampedtransform_quaternion); // target frame to source frame
    return stampedtransform_quaternion.getRotation();
  }

  void PointCloudPub()
  {
    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "/kinect2_rgb_optical_frame";
    msg->height = 1;
    msg->width = co_x.size();
    for (int i=0; i<co_x.size();i++)
    {
      msg->points.push_back (pcl::PointXYZ(co_x[i],co_y[i],co_z[i]));
      // msg->points.push_back (pcl::PointXYZ(1,1,1));
    }

    ros::Rate loop_rate(4);
    while (nh_.ok())
    {
      pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
      point_cloud_ziptie_pub.publish (msg);
      ros::spin();
      loop_rate.sleep ();
    }

  }



  bool gripper_action(double finger_turn)
  {

    if (finger_turn < 0)
    {
        finger_turn = 0.0;
    }
    else
    {
        finger_turn = std::min(finger_turn, FINGER_MAX);
    }

    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = finger_turn;
    goal.fingers.finger2 = goal.fingers.finger1;
    goal.fingers.finger3 = goal.fingers.finger1;

    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> finger_client("/j2n6s300_driver/fingers_action/finger_positions" , false);
    while(!finger_client.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the finger action server to come up");
    } 


    finger_client.sendGoal(goal);

    if (finger_client.waitForResult(ros::Duration(5.0)))
    {
        finger_client.getResult();
        return true;
    }
    else
    {
        finger_client.cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
  }


  void move_robot()
  { 
    ros::AsyncSpinner spinner(4);  // important
    spinner.start();
    //sleep(10.0);
    
    moveit::planning_interface::MoveGroupInterface group("arm");
    moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
   
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

    // Create a publisher for visualizing plans in Rviz.
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Getting Basic Information
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    
    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    // Planning to a Pose goal
    group.setPoseTarget(target_pose);


    // Now, we call the planner to compute the plan and visualize it.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    // Visualizing plans
    if (1)
    {
      ROS_INFO("Visualizing plan 1 (again)");    
      display_trajectory.trajectory_start = my_plan.start_state_;
      display_trajectory.trajectory.push_back(my_plan.trajectory_);
      display_publisher.publish(display_trajectory);
      /* Sleep to give Rviz time to visualize the plan. */
      sleep(5.0);
    }
    
    // move the robot 
    ROS_INFO("Attention: moving the arm");
    gripper_action(0.0); // open the gripper
    //group.move();
    gripper_action(0.9*FINGER_MAX); // close the gripper
	  sleep(1.0);

    // move the robot with the object
    group.setPoseTarget(target_pose2);

    // Now, we call the planner to compute the plan and visualize it.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    bool success2 = (group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 2 (pose goal) %s",success2?"":"FAILED");    
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    // Visualizing plans
    if (1)
    {
      ROS_INFO("Visualizing plan 2 (again)");    
      display_trajectory.trajectory_start = my_plan2.start_state_;
      display_trajectory.trajectory.push_back(my_plan2.trajectory_);
      display_publisher.publish(display_trajectory);
      /* Sleep to give Rviz time to visualize the plan. */
      sleep(5.0);
    }


    ROS_INFO("Attention: moving the arm with the object");
    //group.move();
  }

 };


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  while (cv_ptr == 0 || depth_img_cv == 0 || boundingbox_xmin == 0)
  {
    ros::spinOnce();
  }

  ic.imageProcessing();
  cout << "boundingbox_xmin:" << boundingbox_xmin << endl; 
  cout << "boundingbox_xmax:" << boundingbox_xmax << endl;
  cout << "boundingbox_ymin:" << boundingbox_ymin << endl; 
  cout << "boundingbox_ymax:" << boundingbox_ymax << endl;
  ic.PointCloudPub();
  //ic.move_robot();
  ros::waitForShutdown();
  return 0;
}
