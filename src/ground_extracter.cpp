#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

using namespace std;

ros::Publisher pub;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> updateVis (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    viewer->removeAllShapes();
    viewer->removeAllPointClouds();
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    //viewer->initCameraParameters ();
    return (viewer);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    static int t = 0;
    t++;
    cout<<"Got points-"<<t<<" :)"<<endl;


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*input, *cloud_ptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    float min_z = numeric_limits<float>::max(), max_z = -1;

    for(pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud_ptr->begin(); it != cloud_ptr->end(); it++){

        if(it->intensity>250)
        {
            cout<<(it->intensity)<<endl;
            cout<<"Error! Intensity greater than 250!\n";
            exit(1);
        }

        if(it->z > max_z)
            max_z = it->z;

        if(it->z < min_z)
            min_z = it->z;

        pcl::PointXYZRGB point;
        int i = (it->intensity/250.0)*255;
        uint8_t r(i), g(i), b(i);
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        point.rgb = *reinterpret_cast<float*>(&rgb);
        point.x = it->x;
        point.y = it->y;
        point.z = it->z;
        display_cloud_ptr->points.push_back (point);
    }

    cout<<"min_z="<<min_z<<" max_z="<<max_z<<endl;

    display_cloud_ptr->width = (int) display_cloud_ptr->points.size ();
    display_cloud_ptr->height = 1; 

    if(t==1)
        viewer = rgbVis(display_cloud_ptr);
    else
        viewer = updateVis(viewer, display_cloud_ptr);

    viewer->spinOnce(100);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "pcl_viewer");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);

    // Spin
    ros::spin ();
}