#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string> 
#include <nav_msgs/Odometry.h>

#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <math.h>

#include <sstream>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

camodocal::CameraPtr m_camera;
cv::Mat undist_map1_, undist_map2_ , K_;

ros::Publisher pub_point_line;
sensor_msgs::PointCloudConstPtr linefeature;
void callback(const sensor_msgs::PointCloudConstPtr &point_feature_msg,
              const sensor_msgs::PointCloudConstPtr &line_feature_msg,
              const sensor_msgs::ImageConstPtr& img_msg) 
{
    //在这里把点和线画出来就可以了
    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat show_img = ptr->image;
    cv::Mat img1;
    cv::cvtColor(show_img, img1, cv::COLOR_GRAY2BGR);


    for(int i=0; i<point_feature_msg->points.size(); i++)
    {
        cv::Point endPoint = cv::Point(point_feature_msg->channels[1].values[i], point_feature_msg->channels[2].values[i]);
        cv::circle(img1, endPoint, 2, cv::Scalar(0, 255, 0), 2);
    }

    cv::remap(img1, show_img, undist_map1_, undist_map2_, CV_INTER_LINEAR);
    for(int i=0; i<line_feature_msg->points.size(); i++)
    {
        cv::Point startPoint = cv::Point(line_feature_msg->channels[3].values[i], line_feature_msg->channels[4].values[i]);
        cv::Point endPoint = cv::Point(line_feature_msg->channels[5].values[i], line_feature_msg->channels[6].values[i]);
        cv::line(show_img, startPoint, endPoint, cv::Scalar(0, 0, 255),2 ,8);
    }

    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", show_img).toImageMsg();
    pub_point_line.publish(output_msg);
    // cv::namedWindow("LSD matches", CV_WINDOW_NORMAL);
    // cv::imshow( "LSD matches", show_img );
    // cv::waitKey(5);
}

int main(int argc, char **argv) {

  m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile("/../../config/euroc/euroc_config_fix_extrinsic.yaml");
  K_ = m_camera->initUndistortRectifyMap(undist_map1_,undist_map2_);  

  ros::init(argc, argv, "sync_control_node"); 
  ros::NodeHandle nh;                      

  message_filters::Subscriber<sensor_msgs::PointCloud> point_feature_sub(nh, "/feature_tracker/feature",1000); 
  message_filters::Subscriber<sensor_msgs::PointCloud> line_feature_sub(nh, "/linefeature_tracker/linefeature", 1000);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/cam0/image_raw", 1000);
  pub_point_line = nh.advertise<sensor_msgs::Image>("/PointLine_image",1000);
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud,sensor_msgs::PointCloud,sensor_msgs::Image> MySyncPolicy;
  
 
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point_feature_sub, line_feature_sub,image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));
  
  ros::Rate loop_rate(30);
  while (nh.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
