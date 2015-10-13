#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2,cv_ptr->image.rows/2), 30, CV_RGB(0,0,255));
	  cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2,cv_ptr->image.rows/2), 5, CV_RGB(255,0,0));
	  //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
		//cv::circle(cv_ptr->image, cv::Point(10, 100), 10, CV_RGB(255,0,0));
		//cv::circle(cv_ptr->image, cv::Point(10, 500), 10, CV_RGB(255,0,0));
		//cv::circle(cv_ptr->image, cv::Point(10, 1000), 10, CV_RGB(255,0,0));
	//image.row = 960, image.cols = 1280 .
        //ROS_INFO("image.row = %d, image.cols = %d .",cv_ptr->image.rows,cv_ptr->image.cols);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
