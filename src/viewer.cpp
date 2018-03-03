#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

using namespace std;

ros::Publisher pub;

boost::shared_ptr<pcl::visualization::PCLVisualizer> xyziVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); 
  viewer->setBackgroundColor (0, 0, 0); 
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity"); 
  viewer->addPointCloud<pcl::PointXYZI> (cloud, intensity_distribution, "sample cloud"); 
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); 
  viewer->addCoordinateSystem (1.0); 
  viewer->initCameraParameters (); 
  return viewer;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void cloud_cb_display_colored (const sensor_msgs::PointCloud2ConstPtr& input)
{
  static int t = 0;
  t++;
  cout<<"Got points-"<<t<<" :)"<<endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg (*input, *cloud_ptr);

  for(pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud_ptr->begin(); it != cloud_ptr->end(); it++){ 
    cout << it->x << ", " << it->y << ", " << it->z << "," << it->intensity << endl; 
    } 

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = xyziVis(cloud_ptr);
  while (!viewer->wasStopped ())
  {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  static int t = 0;
  t++;
  cout<<"Got points-"<<t<<" :)"<<endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg (*input, *cloud_ptr);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  for(pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud_ptr->begin(); it != cloud_ptr->end(); it++){ 
      //cout << it->x << ", " << it->y << ", " << it->z << "," << it->intensity << endl; 

      pcl::PointXYZRGB point;
      int i = (it->intensity/100.0)*255;
      uint8_t r(i), g(i), b(i);
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point.x = it->x;
      point.y = it->y;
      point.z = it->z;
      display_cloud_ptr->points.push_back (point);
  }

  display_cloud_ptr->width = (int) display_cloud_ptr->points.size ();
  display_cloud_ptr->height = 1; 

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = rgbVis(display_cloud_ptr);
  while (!viewer->wasStopped ())
  {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_viewer");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);

  // Spin
  ros::spin ();
}