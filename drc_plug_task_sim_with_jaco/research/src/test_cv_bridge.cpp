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
#include <math.h>
#include <Eigen/Dense>
#include <kinova_driver/kinova_ros_types.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>

#define PI 3.14159265
static const std::string OPENCV_WINDOW = "Image window";
const double FINGER_MAX = 6400;
using namespace cv;
using namespace std;
cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImageConstPtr depth_img_cv;
geometry_msgs::Pose target_pose;
geometry_msgs::Pose target_pose2;
geometry_msgs::Pose target_pose3;
geometry_msgs::Pose target_pose4;
geometry_msgs::Pose target_pose5;
double boundingbox_xmin, boundingbox_xmax, boundingbox_ymin, boundingbox_ymax;
std::pair<tf::Vector3, tf::Vector3> ziptie;


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Subscriber info_sub_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher display_publisher;
  ros::Subscriber sub_yolo;

  Mat imgThresholded;
  int iLowH = 0;
  int iHighH = 0;

  int iLowS = 0; 
  int iHighS = 0;

  int iLowV = 0;
  int iHighV = 0;

  vector<int> pixel_x; // x of zip-tie in the 2d image
  vector<int> pixel_y; // y of zip-tie in the 2d image
  vector<float> co_x; // x coordinate in 3d with respect to camera
  vector<float> co_y; // y coordinate in 3d with respect to camera
  vector<float> co_z; // z coordinate in 3d with respect to camera

  image_geometry::PinholeCameraModel pin_hole_camera;
  int grasp_point_index_1;
  //new
  int grasp_point_index_2;
  float center_x;
  float center_y;
  float center_z;

  float normal_x;
  float normal_y;
  float normal_z;
  float roll, yaw, pitch;

  tf::TransformBroadcaster br;

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
    if (msg->bounding_boxes[0].Class.compare("zip-tie") == 0)
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
      // for (int i=boundingbox_xmin; i<boundingbox_xmax; i++) { //539
      //   for (int j=boundingbox_ymin; j<boundingbox_ymax; j++){ //959
      //     // cout << imgThresholded.at<bool>(i,j) << "," << endl;
      //     // cout << imgThresholded.size<< endl;
      //     if (imgThresholded.at<bool>(i,j) != 0) {
      //       // cout << "here" << i << " " << j << endl;
      //       pixel_x.push_back(i);
      //       pixel_y.push_back(j);
      //     }
      //   }
      // }
      // //new
      // std::vector<tf::Vector3> ziptie_plane_vector;
      // tf::Vector3 ziptie_plane;
      // //new
      // int pixel_size = pixel_x.size();
      // for (int i = 0; i < pixel_size; i++){
      //   cv::Point2d pixel_point(pixel_x[i], pixel_y[i]); 
      //   float depth = depth_img_cv->image.at<short int>(pixel_point);
      //   depth = depth;
      //   cout << "depth" << depth<< endl;
      //   if (depth > 0.1){
      //     cv::Point3d xyz =pin_hole_camera.projectPixelTo3dRay(pixel_point);
      //     // cout << xyz<<endl;
      //     cv::Point3d coordinate = xyz * depth;
      //     float co_x_temp = -coordinate.x/1000;
      //     float co_y_temp = -coordinate.y/1000;
      //     float co_z_temp = coordinate.z/1000-0.17;
      //     co_x.push_back(co_x_temp);
      //     co_y.push_back(co_y_temp);
      //     co_z.push_back(co_z_temp);

      //     // new

      //     ziptie_plane.setX(co_x_temp);
      //     ziptie_plane.setY(co_y_temp);
      //     ziptie_plane.setZ(co_z_temp);
      //     ziptie_plane_vector.push_back(ziptie_plane);
      //     // new
         
      //   } 

      // } 
      //   ziptie = best_plane_from_points(ziptie_plane_vector); //new
      //   cout << "zip-tie-centroid-x" << ziptie.first.getX() << "zip-tie-plane-normal-x" << ziptie.second.getX() << endl;
      //   // new using best plane from points function to get the centrid and normal of the ziptie plane and calculate the frame of the zip-tie
      //   center_x = ziptie.first.getX();
      //   center_y = ziptie.first.getY();
      //   center_z = ziptie.first.getZ();

      //   normal_x = ziptie.second.getX();
      //   normal_y = ziptie.second.getY();
      //   normal_z = ziptie.second.getZ();

      //   roll = 0;
      //   pitch = atan((sqrt(pow((center_x - normal_x),2)+pow((center_y - normal_y),2)))/(center_z - normal_z));
      //   yaw = atan((center_x - normal_x)/(normal_y - center_y));
      //   tf::Transform transform = tf_transform_calculation(center_x, center_y, center_z);
      //   // transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
      //   tf::Quaternion q_rot = tf::createQuaternionFromRPY(0, -1.57, 0);
      //   tf::Quaternion q_new = q_rot * transform.getRotation();
      //   transform.setRotation(q_new);

/*      using two points on the zip-tie to calculate the rpy and adjust the orientation of the ee
        grasp_point_index_1 = (int)(pixel_x.size()/2);
        //new
        grasp_point_index_2 = (int)(pixel_x.size()/pixel_x.size());
        //new
        cout << "ixyz_1 " <<" i:" << grasp_point_index_1 << "x:" << co_x[grasp_point_index_1] << "y:"<< co_y[grasp_point_index_1] << "z:" << co_z[grasp_point_index_1] << ";" << endl;
        cout << "ixyz_2 " <<" i:" << grasp_point_index_2 << "x:" << co_x[grasp_point_index_2] << "y:"<< co_y[grasp_point_index_2] << "z:" << co_z[grasp_point_index_2] << ";" << endl;

        tf::Transform transform = tf_transform_calculation(co_x[grasp_point_index_1], co_y[grasp_point_index_1], co_z[grasp_point_index_1]);
        // new
        tf::Transform transform2 = tf_transform_calculation(co_x[grasp_point_index_2], co_y[grasp_point_index_2], co_z[grasp_point_index_2]);
        roll = atan((transform2.getOrigin().x()-transform.getOrigin().x())/(transform2.getOrigin().z()-transform.getOrigin().z()));
        pitch = atan((transform2.getOrigin().z()-transform.getOrigin().z())/(transform2.getOrigin().y()-transform.getOrigin().y()));
        yaw = atan((transform2.getOrigin().y()-transform.getOrigin().y())/(transform2.getOrigin().x()-transform.getOrigin().x())); 
        // tf::Quaternion quaternion = tf_quaternion_calculation_new(transform);
        tf::Quaternion quaternion = tf_quaternion_calculation();
        cout << quaternion.getX() << quaternion.getY() << quaternion.getZ() << quaternion.getW() << endl;
        tf::Matrix3x3 m(quaternion);
        double roll_ee, pitch_ee, yaw_ee;
        m.getRPY(roll_ee,pitch_ee,yaw_ee);
        cout << "yaw_ee angle" << yaw_ee << endl;
        transform.setRotation(tf::createQuaternionFromRPY(roll_ee+roll, pitch_ee+pitch, yaw_ee+yaw));
        cout << transform.getRotation().x() << transform.getRotation().y() << transform.getRotation().z() << transform.getRotation().w() << endl;
*/  
        // publish the tf of the zip-tie
        // plan 1
        tf::Transform transform = tf_transform_calculation(-0.1, 0.25, 0.53);
        tf::Quaternion quaternion = tf_quaternion_calculation();
        transform.setRotation(quaternion);


        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "zip-tie"));
        for (int i = 0; i < 5000; i++)
          ros::spinOnce();

        target_pose.position.x = transform.getOrigin().x();
        target_pose.position.y = transform.getOrigin().y();
        target_pose.position.z = transform.getOrigin().z();
        target_pose.orientation.x = transform.getRotation().x(); 
        target_pose.orientation.y = transform.getRotation().y(); 
        target_pose.orientation.z = transform.getRotation().z(); 
        target_pose.orientation.w = transform.getRotation().w(); 
        cout << "XYZ" << "X:" << target_pose.position.x << "Y:" << target_pose.position.y << "Z:" << target_pose.position.z << endl;

        // plan 2
        tf::Transform transform2 = tf_transform_calculation(0.05, 0.27, 0.38);
        tf::Quaternion q_orig, q_rot, q_new;
        double r=0, p=0, y=-1.57;// Rotate the previous pose by 180* about Z
        q_rot = tf::createQuaternionFromRPY(r, p, y);
        quaternionMsgToTF(target_pose.orientation , q_orig);
        q_new = q_rot*q_orig;
        q_new.normalize();
        quaternionTFToMsg(q_new, target_pose2.orientation);

        target_pose2.position.x = transform2.getOrigin().x();
        target_pose2.position.y = transform2.getOrigin().y();
        target_pose2.position.z = transform2.getOrigin().z();
        // target_pose2.orientation = target_pose.orientation;


        // plan 3
        tf::Transform transform3 = tf_transform_calculation(0.18, 0.27, 0.30);
        tf::Quaternion q_orig3, q_rot3, q_new3;
        double r3=0, p3=0, y3=-1.57;// Rotate the previous pose by 180* about Z
        q_rot3 = tf::createQuaternionFromRPY(r3, p3, y3);
        quaternionMsgToTF(target_pose2.orientation , q_orig3);
        q_new3 = q_rot3*q_orig3;
        q_new3.normalize();
        quaternionTFToMsg(q_new3, target_pose3.orientation);

        target_pose3.position.x = transform3.getOrigin().x();
        target_pose3.position.y = transform3.getOrigin().y();
        target_pose3.position.z = transform3.getOrigin().z();
        // target_pose2.orientation = target_pose.orientation;

        // plan 4
        tf::Transform transform4 = tf_transform_calculation(0.18, 0.27, 0.34);
        tf::Quaternion q_orig4, q_rot4, q_new4;
        double r4=0, p4=0, y4=-1.57;// Rotate the previous pose by 180* about Z
        q_rot4 = tf::createQuaternionFromRPY(r4, p4, y4);
        quaternionMsgToTF(target_pose3.orientation , q_orig4);
        q_new4 = q_rot4*q_orig4;
        q_new4.normalize();
        quaternionTFToMsg(q_new4, target_pose4.orientation);

        target_pose4.position.x = transform4.getOrigin().x();
        target_pose4.position.y = transform4.getOrigin().y();
        target_pose4.position.z = transform4.getOrigin().z();
        // target_pose2.orientation = target_pose.orientation;

        // plan 5
        tf::Transform transform5 = tf_transform_calculation(0.18, 0.27, 0.40);
        target_pose5.position.x = transform5.getOrigin().x();
        target_pose5.position.y = transform5.getOrigin().y();
        target_pose5.position.z = transform5.getOrigin().z();


  }

  std::pair<tf::Vector3, tf::Vector3> best_plane_from_points(const std::vector<tf::Vector3> & c)
  {
    // copy coordinates to  matrix in Eigen format
    size_t num_atoms = c.size();
    Eigen::Matrix< float, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
    for (size_t i = 0; i < num_atoms; i++){       
        coord(0,i) = c[i].getX(); 
        coord(1,i) = c[i].getY();
        coord(2,i) = c[i].getZ();  
    }


    // calculate centroid
    tf::Vector3 centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

    // subtract centroid
    coord.row(0).array() -= centroid.getX(); 
    coord.row(1).array() -= centroid.getY(); 
    coord.row(2).array() -= centroid.getZ();

    // we only need the left-singular matrix here
    //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    tf::Vector3 plane_normal(svd.matrixU().rightCols(1)(0), svd.matrixU().rightCols(1)(1), svd.matrixU().rightCols(1)(2));
    return std::make_pair(centroid, plane_normal);
  }





  tf::Transform tf_transform_calculation(float x, float y, float z)
  {
    //calculate transform from /endeffector to /object
    tf::TransformListener listener;
    tf::StampedTransform stampedtransform_transform;
    listener.waitForTransform("/world", "/camera_link", ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform("/world", "/camera_link", ros::Time(0), stampedtransform_transform); // target frame to source frame
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    //new
    //transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
    //new
    tf::Transform ntransform;
    ntransform = stampedtransform_transform * transform;
    return ntransform;
  }

  tf::Quaternion tf_quaternion_calculation()
  {   
    //calculate quaternion of the object
    tf::TransformListener listener_robot;
    tf::StampedTransform stampedtransform_quaternion;
    listener_robot.waitForTransform("/world", "/endeffector_default", ros::Time::now(), ros::Duration(3.0)); // use "/j2n6s300_end_effector" if you want to keep the orientation of the endeffector
    listener_robot.lookupTransform("/world", "/endeffector_default", ros::Time(0), stampedtransform_quaternion); // target frame to source frame
    return stampedtransform_quaternion.getRotation();
  }
// new 
  tf::Quaternion tf_quaternion_calculation_current()
  {   
    //calculate quaternion of the current endeffector
    tf::TransformListener listener_robot;
    tf::StampedTransform stampedtransform_quaternion;
    listener_robot.waitForTransform("/world", "/j2n6s300_end_effector", ros::Time::now(), ros::Duration(3.0)); // use "/j2n6s300_end_effector" if you want to keep the orientation of the endeffector
    listener_robot.lookupTransform("/world", "/j2n6s300_end_effector", ros::Time(0), stampedtransform_quaternion); // target frame to source frame
    return stampedtransform_quaternion.getRotation();
  }
// new
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
    // goal.fingers.finger3 = goal.fingers.finger1;

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
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    // const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName());


    // Create a publisher for visualizing plans in Rviz.
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Getting Basic Information
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    
    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());


    // Planning to a Pose goal -- plan 1
    group.setPoseTarget(target_pose);

    // Now, we call the planner to compute the plan and visualize it.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(10.0);

    // Visualizing plans
    if (1)
    {
      ROS_INFO("Visualizing plan 1 (again)");    
      display_trajectory.trajectory_start = my_plan.start_state_;
      display_trajectory.trajectory.push_back(my_plan.trajectory_);
      display_publisher.publish(display_trajectory);
      /* Sleep to give Rviz time to visualize the plan. */
      sleep(10.0);
    }
    
    // move the robot 
    ROS_INFO("Attention: moving the arm");
    gripper_action(0.0); // open the gripper
    group.move();
    gripper_action(0.75*FINGER_MAX); // close the gripper


    // move the robot with the object ---- plan 2
    group.setPoseTarget(target_pose2);

    // Now, we call the planner to compute the plan and visualize it.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    bool success2 = (group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 2 (pose goal) %s",success2?"":"FAILED");    
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    ROS_INFO("Attention: moving the arm with the object");
    group.move();


    // move the robot with the object ---- plan 3
    group.setPoseTarget(target_pose3);

    // Now, we call the planner to compute the plan and visualize it.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
    // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    bool success3 = (group.plan(my_plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 3 (pose goal) %s",success3?"":"FAILED");    
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    ROS_INFO("Attention: moving the arm with the object");
    group.move();

    // plan 4
    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[5] = joint_group_positions[5]-2;  // radians
    group.setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan4;
    bool success4 = (group.plan(my_plan4) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 4 (pose goal) %s",success4?"":"FAILED");    
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    ROS_INFO("Attention: moving the arm with the object");
    group.move();

    // plan 5
    tf::Quaternion quaternion2 = tf_quaternion_calculation_current();

    target_pose5.orientation.x = quaternion2.getX(); 
    target_pose5.orientation.y = quaternion2.getY(); 
    target_pose5.orientation.z = quaternion2.getZ(); 
    target_pose5.orientation.w = quaternion2.getW(); 

    group.setPoseTarget(target_pose5);
    // Now, we call the planner to compute the plan and visualize it.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan5;
    // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    bool success5 = (group.plan(my_plan5) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 5 (pose goal) %s",success5?"":"FAILED");    
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    ROS_INFO("Attention: moving the arm with the object");
    group.move();

  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  // while (cv_ptr == 0 || depth_img_cv == 0 || boundingbox_xmin == 0)
  // {
  //   ros::spinOnce();
  // }

  ic.imageProcessing();
  cout << "boundingbox_xmin:" << boundingbox_xmin << endl; 
  cout << "boundingbox_xmax:" << boundingbox_xmax << endl;
  ic.move_robot();
  ros::waitForShutdown();
  return 0;
}
