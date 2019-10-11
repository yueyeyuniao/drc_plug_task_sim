#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "/root";
  msg->height = 12;
  msg->width = 1;
  msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));
msg->points.push_back (pcl::PointXYZ(1.1, 2.0, 3.0));
msg->points.push_back (pcl::PointXYZ(1.2, 2.0, 3.0));
msg->points.push_back (pcl::PointXYZ(1.3, 2.0, 3.0));
msg->points.push_back (pcl::PointXYZ(1.4, 2.0, 3.0));
msg->points.push_back (pcl::PointXYZ(1.5, 2.0, 3.0));
msg->points.push_back (pcl::PointXYZ(1.6, 2.0, 3.0));
msg->points.push_back (pcl::PointXYZ(1.7, 2.0, 3.0));
msg->points.push_back (pcl::PointXYZ(1.8, 2.0, 3.0));
msg->points.push_back (pcl::PointXYZ(1.9, 2.0, 3.0));
msg->points.push_back (pcl::PointXYZ(2.0, 2.0, 3.0));
msg->points.push_back (pcl::PointXYZ(2.1, 2.0, 3.0));

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
