#include <artificial_potential_fields/cloud_viewer.h>

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& c){
    pcl::fromROSMsg(*c, *cloud);
}

void quit(int sig){
  viewer->close();
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pointcloud_viewer");
    ros::NodeHandle node_handle;

    point_cloud_subscriber = node_handle.subscribe("/point_cloud", 1, cloudCallback);

    signal(SIGINT, quit);

    viewer->setBackgroundColor(0.5, 0.5, 0.5);
    /*pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "point cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();*/

    viewer->addPointCloud<pcl::PointXYZ> (cloud, "point cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while(!viewer->wasStopped()){
        ros::spinOnce();

        // Plot the point cloud
        //rgb.setInputCloud(cloud);
        //viewer->updatePointCloud<pcl::PointXYZ>(cloud, rgb, "point cloud");
        viewer->updatePointCloud(cloud, "point cloud");
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}
