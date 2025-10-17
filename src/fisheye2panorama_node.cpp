#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <yaml-cpp/yaml.h>

YAML::Node config = YAML::LoadFile("./src/fisheye2panorama/config/config.yaml");
int fisheye_w  = config["fisheye_w"].as<int>();
int fisheye_h  = config["fisheye_h"].as<int>();
float fisheye_alpha  = config["fisheye_alpha"].as<float>();
float fisheye_beta  = config["fisheye_beta"].as<float>();
float fisheye_fx  = config["fisheye_fx"].as<float>();
float fisheye_fy  = config["fisheye_fy"].as<float>();
float fisheye_cx  = config["fisheye_cx"].as<float>();
float fisheye_cy  = config["fisheye_cy"].as<float>();

int cylinder_w  = config["cylinder_w"].as<int>();
int cylinder_h  = config["cylinder_h"].as<int>();
float cylinder_cx  = config["cylinder_cx"].as<float>();
float cylinder_cy  = config["cylinder_cy"].as<float>();

double cylinder_fx = (float)640/M_PI;
double cylinder_fy = cylinder_fx;


cv::Mat mapfisheye2cylinder1 = cv::Mat::zeros(cylinder_h,cylinder_w,CV_32F);
cv::Mat mapfisheye2cylinder2 = cv::Mat::zeros(cylinder_h,cylinder_w,CV_32F);

cv::Mat fisheye2cylindermap1 = cv::Mat::zeros(cylinder_h,cylinder_w,CV_32F);
cv::Mat fisheye2cylindermap2 = cv::Mat::zeros(cylinder_h,cylinder_w,CV_32F);


void fisheye3d2pixel(Eigen::Vector3d &fisheye_3d,Eigen::Vector2d &fisheye_2d)
{
    double norm = fisheye_3d.norm();
    double d = sqrt(fisheye_beta * (fisheye_3d(0) * fisheye_3d(0) + fisheye_3d(1) * fisheye_3d(1)) + fisheye_3d(2) * fisheye_3d(2));
    double fisheye_u = fisheye_fx * fisheye_3d(0) / (fisheye_alpha * d + (1 - fisheye_alpha) * fisheye_3d(2)) + fisheye_cx;
    double fisheye_v = fisheye_fy * fisheye_3d(1) / (fisheye_alpha * d + (1 - fisheye_alpha) * fisheye_3d(2)) + fisheye_cy;
    fisheye_2d = Eigen::Vector2d(fisheye_u,fisheye_v);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // 将ROS图像消息转换为OpenCV的Mat格式（BGR8编码）
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat fisheye_img = cv_ptr->image;
    cv::Mat cylinder_img;
    ros::Time start = ros::Time::now();  // 记录起始时间

    //cv::cvtColor(fisheye_img, fisheye_img, cv::COLOR_BGR2GRAY);
    cv::remap(fisheye_img, cylinder_img, fisheye2cylindermap1, fisheye2cylindermap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    ros::Duration elapsed = ros::Time::now() - start;
    ROS_INFO("fisheye2panorama_node consuming: %.3f ms", elapsed.toSec() * 1000.0);
  
    // 显示图像
    cv::imshow("Cylinder Image", cylinder_img);
    cv::waitKey(1);  // 必须调用waitKey以更新窗口

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fisheye2panorama_node");
    ros::NodeHandle nh;

    // 创建图像订阅者
    image_transport::ImageTransport it(nh);
    image_transport::TransportHints hints("compressed");
    image_transport::Subscriber sub = it.subscribe("/camera/color/image", 1, imageCallback, hints);
    
    for(int H = 0 ;H < cylinder_h;H++)
        for(int W = 0;W <cylinder_w;W++)
        {
            float phi = (float)(W-cylinder_cx)/cylinder_fx;
            float y_by_rho = (float)(H - cylinder_cy)/cylinder_fy;
            float normalized_z = fabs(phi) >(M_PI/2) ? -1.0f : 1.0f;
            float normalized_x = normalized_z*tan(phi);
            float rho = sqrt(normalized_x * normalized_x + normalized_z * normalized_z);
            float normalized_y = y_by_rho * rho;
            Eigen::Vector3d cylinder_normalized_point;
            cylinder_normalized_point << normalized_x , normalized_y , normalized_z;
            Eigen::Quaterniond rotation_fc = Eigen::Quaterniond(Eigen::AngleAxis<double>(-M_PI / 2, Eigen::Vector3d(1, 0, 0)));
            Eigen::Vector3d fisheye_3d = rotation_fc * cylinder_normalized_point;
            Eigen::Vector2d fisheye_2d;
            fisheye3d2pixel(fisheye_3d,fisheye_2d);
            double fisheye_u,fisheye_v;
            fisheye_u = fisheye_2d(0);
            fisheye_v = fisheye_2d(1);

            if(fisheye_u >= 0 && fisheye_u < fisheye_w && fisheye_v >= 0 && fisheye_v < fisheye_h)
            {
                fisheye2cylindermap1.at<float>(H,W) = fisheye_u;
                fisheye2cylindermap2.at<float>(H,W) = fisheye_v;
            }

        }



    ros::spin();  // 保持节点运行
    return 0;
}