/**
 * Makes all the non-planar things green, publishes the final result on the /obstacles topic.
 */
#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>


ros::Publisher pub;

void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.02);
  seg.setInputCloud (cloud_filtered);
    
  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  pcl::IndicesPtr remaining (new std::vector<int>);
  remaining->resize (nr_points);
  for (size_t i = 0; i < remaining->size (); ++i) { (*remaining)[i] = static_cast<int>(i); }

  // While 30% of the original cloud is still there
  while (remaining->size () > 0.5 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setIndices (remaining);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0) break;

    // Extract the inliers
    std::vector<int>::iterator it = remaining->begin();
    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
      int curr = inliers->indices[i];
      // Remove it from further consideration.
      while (it != remaining->end() && *it < curr) { ++it; }
      if (it == remaining->end()) break;
      if (*it == curr) it = remaining->erase(it);
    }
    i++;
  }
  std::cout << "Found " << i << " planes." << std::endl;

  // Color all the non-planar things.
  for (std::vector<int>::iterator it = remaining->begin(); it != remaining->end(); ++it)
  {
    uint8_t r = 0, g = 255, b = 0;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    cloud_filtered->at(*it).rgb = *reinterpret_cast<float*>(&rgb);
  }

  // Publish the planes we found.
  pcl::PCLPointCloud2 outcloud;
  pcl::toPCLPointCloud2 (*cloud_filtered, outcloud);
  pub.publish (outcloud);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "obstacles");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/kinect2/qhd/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("obstacles", 1);

  // Spin
  ros::spin ();
}


/*
version 2: 
#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <math.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>

#include <iomanip>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <list>

#include <visualization_msgs/Marker.h>
using namespace std;
geometry_msgs::Point center_object;

class PointCloudProc
{
  public:
    PointCloudProc() : cloud_transformed_(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered_(new pcl::PointCloud<pcl::PointXYZ>),
                       cloud_hull(new pcl::PointCloud<pcl::PointXYZ>), cloud_raw_(new pcl::PointCloud<pcl::PointXYZ>)
    {

        //filter_range_ = 0.7;

        //planar_segment_src_ = nh_.advertiseService("planer_segment", &PointCloudProc::planarSegmentationCB, this);
        pc_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &PointCloudProc::pointcloudcb, this);
        point_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("transformed_point_cloud", 1000);
        seg_point_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("segmented_plane_point_cloud", 1000);
        filtered_point_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("filtered_point_cloud", 1000);
        marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        fixed_frame_ = "/root";
    }

    void pointcloudcb(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
    {
        geometry_msgs::Point plane_normal;
        pcl::ExtractIndices<pcl::PointXYZ> extract_;
        pcl::ConvexHull<pcl::PointXYZ> chull;
        cloud_raw_ = msg;
        // cloud_transformed_ = cloud_raw_;
        // listener_.waitForTransform(fixed_frame_, "/kinect2_rgb_optical_frame", (ros::Time)(*cloud_raw_).header.stamp, ros::Duration(3.0));
        bool transform_success = pcl_ros::transformPointCloud(fixed_frame_, *cloud_raw_, *cloud_transformed_, listener_);
        //std::cout << transform_success << std::endl;
        point_cloud_pub_.publish(cloud_transformed_);

        if (transform_success == 0) 
        {
          ROS_INFO("failed transform point cloud");
          return;
        }

        // get the position of the end-effector
        // listener_.waitForTransform("/root", "j2n6s300_end_effector",ros::Time(0), ros::Duration(3.0));
        // listener_.lookupTransform("/root", "j2n6s300_end_effector",ros::Time(0), transform);

        // std::cout << "Position of endeffector: " << transform.getOrigin().x() << ", " << transform.getOrigin().y() << ", " << transform.getOrigin().z() << std::endl;


        //pub_plane.publish(cloud_transformed_);
        // filter point cloud based on detected object, gripper position, socket position
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_transformed_);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(center_object.x - 0.1, center_object.x + 0.5);
        pass.filter(*cloud_filtered_);
        pass.setInputCloud(cloud_filtered_);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(center_object.y - 0.6, center_object.y + 0.0);
        pass.filter(*cloud_filtered_);
        pass.setInputCloud(cloud_filtered_);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(center_object.z - 0.0, center_object.z + 0.7);
        pass.filter(*cloud_filtered_);

        ROS_INFO("Point cloud is filtered!");
        std::cout << cloud_filtered_->points.size() << " of points in the filtered point cloud" << std::endl;
        if (cloud_filtered_->points.size() == 0)
        {
          ROS_INFO("Point cloud is empty after filtering!");
        }


        // save the filtered point cloud to a txt
        // if (filter_flag == 0)
        // {
        //   //std::cout << cloud_filtered_->points.size() << std::endl;
        //   std::ofstream myfile;
        //   myfile.open ("/home/pengchang/Desktop/wire/filtered_wire_realsense_0107.txt", std::ofstream::out | std::ofstream::app);
        //   for (int i=0; i<cloud_filtered_->points.size(); i++)
        //   {
        //       myfile << cloud_filtered_->points[i].x << " " << cloud_filtered_->points[i].y << " " << cloud_filtered_->points[i].z << std::endl;      
        //   }
        //   myfile.close();
        //   filter_flag = 1;
        // }

        // fit the pointcloud
        CurveFit(cloud_filtered_);

        // publish filtered point cloud
        filtered_point_cloud_pub_.publish(cloud_filtered_);
        
        // planar segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0); //z axis
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory set plane to be parallel to Z axis within a 15 degrees tolerance
        seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
        seg.setMaxIterations(500); // iteration limits decides segmentation goodness
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setAxis(axis);
        seg.setEpsAngle(pcl::deg2rad(15.0f));
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud_transformed_);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
          {
              PCL_ERROR("Could not estimate a planar model for the given dataset.");
              return;
          }

        // extract inline points original point coulds
        extract_.setInputCloud(cloud_transformed_);
        extract_.setNegative(false);
        extract_.setIndices(inliers);
        extract_.filter(*cloud_plane);
        ROS_INFO_STREAM("# of points in plane: " << cloud_plane->points.size());

        // publish segmented point cloud
        seg_point_cloud_pub_.publish(cloud_plane);

        // Create a Convex Hull representation of the plane
        chull.setInputCloud(cloud_plane);
        chull.setDimension(2);
        chull.reconstruct(*cloud_hull);

        // Get plane center
        Eigen::Vector4f center;
        pcl::compute3DCentroid(*cloud_plane, center);
        std::cout << "center: " << center[0] << "," << center[1] << "," << center[2] << std::endl;

        // Get plane min and max values
        Eigen::Vector4f min_vals, max_vals;
        pcl::getMinMax3D(*cloud_plane, min_vals, max_vals); 

        // Get plane polygon
        for (int i = 0; i < cloud_hull->points.size(); i++)
        {
            geometry_msgs::Point32 p;
            p.x = cloud_hull->points[i].x;
            p.y = cloud_hull->points[i].y;
            p.z = cloud_hull->points[i].z;
        }  

        // Get plane coefficients
        // coefficients->values[0]

        // Get plane normal
        float length = sqrt(coefficients->values[0] * coefficients->values[0] +
                            coefficients->values[1] * coefficients->values[1] +
                            coefficients->values[2] * coefficients->values[2]);
        plane_normal.x = coefficients->values[0] / length;
        plane_normal.y = coefficients->values[1] / length;
        plane_normal.z = coefficients->values[2] / length;
        std::cout << "normal: " << plane_normal.x << "," << plane_normal.y << "," << plane_normal.z << std::endl;


    }
    void CurveFit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {

      cout.precision(6);                        //set precision
      cout.setf(ios::fixed);


      xx.clear();
      yy.clear();
      zz.clear();
      aa.clear();
      bb.clear();
      av.clear();
      sparse_x.clear();
      sparse_y.clear();
      sparse_z.clear();
      points.points.clear();
      line_strip.points.clear();
      line_list.points.clear();

      for (int i=0; i<cloud->points.size(); i++)
      {
          xx.push_back(cloud->points[i].x);
          yy.push_back(cloud->points[i].y); 
          zz.push_back(cloud->points[i].z);      
      }
      
      N = xx.size(); // N is the no. of data pairs
      aa = PolyfitXY(yy,xx);  //x = aa[0]*y^0 + aa[1]*y^1 + aa[2]*y^2
      av.clear();
      bb = PolyfitXY(yy,zz);  //z = bb[0]*y^0 + bb[1]*y^1 + bb[2]*y^2

      for (double i = yy[0]; i < yy[N-1]; i = i + 0.02)  //(-0.6 -0.386)
      {
        sparse_y.push_back(i);
        sparse_x.push_back(aa[0]+aa[1]*i+aa[2]*i*i);
        sparse_z.push_back(bb[0]+bb[1]*i+bb[2]*i*i);
      }

      // display the sparse points in rviz
      points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/root";
      points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
      points.ns = line_strip.ns = line_list.ns = "points_and_lines";
      points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
      points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;



      points.id = 0;
      line_strip.id = 1;
      line_list.id = 2;



      points.type = visualization_msgs::Marker::POINTS;
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;
      line_list.type = visualization_msgs::Marker::LINE_LIST;

      // POINTS markers use x and y scale for width/height respectively
      points.scale.x = 0.02;
      points.scale.y = 0.02;

      // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
      line_strip.scale.x = 0.01;
      line_list.scale.x = 0.01;



      // Points are green
      points.color.g = 1.0f;
      points.color.a = 1.0;

      // Line strip is blue
      line_strip.color.b = 1.0;
      line_strip.color.a = 1.0;

      // Line list is red
      line_list.color.r = 1.0;
      line_list.color.a = 1.0;

      for (int i = 0; i < sparse_x.size()-1; i++)
      {
        geometry_msgs::Point p;
        p.x = sparse_x[i];
        p.y = sparse_y[i];
        p.z = sparse_z[i];
        points.points.push_back(p);
        line_strip.points.push_back(p);

        // The line list needs two points for each line
        line_list.points.push_back(p);
        p.z += 0.04;
        line_list.points.push_back(p);
      }
      marker_pub.publish(points);
      marker_pub.publish(line_strip);
      marker_pub.publish(line_list);

    }

    std::vector<double> PolyfitXY(std::vector<double> x, std::vector<double> y)
    {
      int i,j,k;
      int n = 2; // n is the degree of polynomial
      double X[2*n+1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
      for (i=0;i<2*n+1;i++)
      {
          X[i]=0;
          for (j=0;j<N;j++)
              X[i]=X[i]+pow(x[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
      }
      double B[n+1][n+2];
      double a[n+1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
      for (i=0;i<=n;i++)
          for (j=0;j<=n;j++)
              B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
      double Y[n+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
      for (i=0;i<n+1;i++)
      {    
          Y[i]=0;
          for (j=0;j<N;j++)
          Y[i]=Y[i]+pow(x[j],i)*y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
      }
      for (i=0;i<=n;i++)
          B[i][n+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
      n=n+1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
      cout<<"\nThe Normal(Augmented Matrix) is as follows:\n";    
      for (i=0;i<n;i++)            //print the Normal-augmented matrix
      {
          for (j=0;j<=n;j++)
              cout<<B[i][j]<<setw(16);
          cout<<"\n";
      }    
      for (i=0;i<n;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
          for (k=i+1;k<n;k++)
              if (B[i][i]<B[k][i])
                  for (j=0;j<=n;j++)
                  {
                      double temp=B[i][j];
                      B[i][j]=B[k][j];
                      B[k][j]=temp;
                  }
      
      for (i=0;i<n-1;i++)            //loop to perform the gauss elimination
          for (k=i+1;k<n;k++)
              {
                  double t=B[k][i]/B[i][i];
                  for (j=0;j<=n;j++)
                      B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
              }
      for (i=n-1;i>=0;i--)                //back-substitution
      {                        //x is an array whose values correspond to the values of x,y,z..
          a[i]=B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
          for (j=0;j<n;j++)
              if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                  a[i]=a[i]-B[i][j]*a[j];
          a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
      }
      cout<<"\nThe values of the coefficients are as follows:\n";
      for (i=0;i<n;i++)
          cout<<"x^"<<i<<"="<<a[i]<<endl;            // Print the values of x^0,x^1,x^2,x^3,....    
      cout<<"\nHence the fitted Polynomial is given by:\ny=";
      for (i=0;i<n;i++)
          cout<<" + ("<<a[i]<<")"<<"x^"<<i;
      cout<<"\n";

      for (i=0;i<(n+1);i++)
      {
        av.push_back(a[i]);
      }
      return av;
    }

   

  private:
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub_;
    ros::Publisher plane_pub_;
    ros::Publisher point_cloud_pub_, seg_point_cloud_pub_, filtered_point_cloud_pub_;
    ros::Publisher marker_pub;
    ros::ServiceServer planar_segment_src_;

    std::string fixed_frame_;


    tf::TransformListener listener_;
    tf::StampedTransform transform;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_, cloud_transformed_, cloud_filtered_;
    pcl::ExtractIndices<pcl::PointXYZ> extract_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull;
    pcl::ConvexHull<pcl::PointXYZ> chull;
    int filter_flag;
    //float filter_range_;
    //for curve fitting
    std::vector<double> xx;
    std::vector<double> yy;
    std::vector<double> zz;
    std::vector<double> aa;
    std::vector<double> bb;
    std::vector<double> av;
    std::vector<double> sparse_x;
    std::vector<double> sparse_y;
    std::vector<double> sparse_z;
    visualization_msgs::Marker points, line_strip, line_list;
    int n, N;


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_segmentation");

    center_object.x = 0;
    center_object.y = 0;
    center_object.z = 0;
    PointCloudProc pc_tools;
    ROS_INFO("Initialized");
    ros::Rate r(30);
    ros::spin();

    return 0;
}

*/
