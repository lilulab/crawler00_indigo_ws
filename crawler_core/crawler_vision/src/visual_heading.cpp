#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>       /* atan2 */
#define PI 3.14159265

#include <cv.h>
#include <highgui.h>

#include "crawler_msgs/VisualHeading.h"

using namespace cv;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;


// static const std::string OPENCV_WINDOW = "Image window";

cv::Mat output,overlay;
Mat temp, color_temp; //setup some temps

//cv_bridge::CvImagePtr cv_ptr;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher msg_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cam_up/image_raw", 1, &ImageConverter::imageCb, this);
    msg_pub_ = nh_.advertise<crawler_msgs::VisualHeading>("crawler/visual_heading", 1);

    // cv::namedWindow(OPENCV_WINDOW);

  }

  ~ImageConverter()
  {
   //  cv::destroyWindow(OPENCV_WINDOW);
  }

    float getHeading(const cv::Mat& src, cv::Mat& out) {
        Mat temp, color_temp; //setup some temps
        cvtColor(src, temp, CV_BGR2GRAY); //convert to grayscale for the edge detector
        //Sobel(temp, temp, CV_8U, 1, 1);

        //Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
        //Canny( temp, temp, 50, 200, 3 ); //run Canny edge detector with some default values
        Canny( temp, temp, 50, 100, 3 ); //Works Nice        //Canny( temp, temp, 30, 90, 3 ); //run Canny edge detector with some default values
        //Canny( temp, temp, lowThreshold, lowThreshold*ratio, kernel_size );
        cvtColor( temp, color_temp, CV_GRAY2BGR ); //Convert Canny edges back to 3-channel

        vector<Vec4i> lines;  // Hough lines result

        HoughLinesP( temp, lines, 1, CV_PI/180, 80, 30, 10 ); //Find lines in the Canny image
        size_t i;

        src.copyTo(temp);

        //robot heading vector
        // for each Vec4f element: [X_vec][Y_vec][heading_angle][heading_epsilon]
        vector<Vec4f> heading;

        // Resize according to how many line are detected.
        heading.resize(lines.size());

        // Average val holder
        float heading_average = 0;

        for(i = 0; i < lines.size(); i++ ) {
        //line( color_temp, Point(lines[i][0], lines[i][1]),
        //    Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 ); //Draw
        //ROS_INFO("Edge %d: P1(%d,%d) to P2(%d,%d).",i,lines[i][0], lines[i][1], lines[i][2], lines[i][3]);
        line( temp, Point(lines[i][0], lines[i][1]),
        Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 ); //Draw

        // Calulate heading vector 
        // for each Vec4f element: [X_vec][Y_vec][heading_angle][heading_epsilon]
        // x1-x2
        heading[i][0] = (float)(lines[i][0] - lines[i][2]);
        // y1-y2
        heading[i][1]= (float)(lines[i][1] - lines[i][3]);

        // Get heading angle in radian using atan2.
        heading[i][3] = atan2(heading[i][0],heading[i][1]); //Yaw angle in radian
        //heading[3] = -heading[3] - 90;  //correct orientation. 

        // get sum for calulate average out side the loop
        heading_average += heading[i][3]; // get sum

      }

        // Calulate the average = sum/size
        heading_average = heading_average/(float)lines.size(); // Calculate average
        
        // Calulate the Mode.

        // Heading val ranging from (-PI to +PI], that is (-3.14,3.14].
        // Let's add PI to make it ranging from 0 to 2PI, (0,6.28].
        // Then ceiling it after times 10, to (0, 63];
        // Now Let's create a 63 column histogram.
        // So each column covers 360/63~=5.7 degree range. should be ok? (LL)
        // 

        
        float heading_mode; //the mode number of the vector
        float heading_mode_belief; //histogram_max_count/line_found
        static const int HISTOGRAM_SIZE = 63; // column size of histogram

        vector<int> heading_histogram_count;   // histogram vector [count]
        vector<float> heading_histogram_val;   // histogram vector [val_sum/avg]

        heading_histogram_count.resize(HISTOGRAM_SIZE);
        heading_histogram_val.resize(HISTOGRAM_SIZE);

        int histogram_column; // temp for calulate which column the value should belone to.

        static const int HEADING_MIN = -PI;
        int histogram_max_count = 0;
        int histogram_max_element = 0;

        int temp_column_count =0; // histogram colum count
        float temp_culumn_mean =0; // possible mode number of the heading

        // loop through all elements in the unsortted vector
        for( int i=0; i<lines.size(); i++ ) {

            // for each Vec4f element: [X_vec][Y_vec][heading_angle][heading_epsilon]
            //heading_epsilon = (heading_angle - heading_average)
            heading[i][4] = abs(heading[i][3] - heading_average); 

            histogram_column = (int) ceil((heading[i][3]+PI) *10); // range = [1,63]
            histogram_column -= 1; // make it [0 to 62]

            // add column count
            heading_histogram_count[histogram_column] += 1;

            // add culumn sum
            heading_histogram_val[histogram_column] += heading[i][3];

            //ROS_INFO("heading = %f(%f), column =  %d, count = %d, sum = %f",heading[i][3],histogram_column,heading_histogram_count[histogram_column],heading_histogram_val[histogram_column]);
        }        

        // Get the max element
        for( int i=0; i<HISTOGRAM_SIZE; i++ ) {

            temp_column_count = heading_histogram_count[i];
            temp_culumn_mean = heading_histogram_val[i] / (float)heading_histogram_count[i];

            // If current count greater than max, than update max element and count holder
            if (temp_column_count > histogram_max_count) {
                histogram_max_element = i;
                histogram_max_count = temp_column_count;
                heading_mode = temp_culumn_mean;
            } 

            // Debug
            //ROS_INFO("column =  %d, count = %d, avg = %f, max_id = %d .", i, temp_column_count, temp_culumn_mean, histogram_max_element);
        }

        // Debug
        //ROS_INFO("Heading Average = %f(radian) , Mode = %f(radian), %f(degree). ",heading_average, heading_mode, (heading_mode*180/PI)+90);

        // Debug
        //ROS_INFO("Found %d Edges!",i);

        heading_mode_belief = (float) histogram_max_count / (float) i;
//        ROS_INFO("Found %d Edges, Heading = %f(radian) | %f(degree), Belief = (%d/%d=%f).",(int)i, heading_mode, (heading_mode*180/PI)+90, histogram_max_count, i, heading_mode_belief);
// comment on Oct.23,2015 by Miao
          out = color_temp;
        addWeighted( temp, 0.5, color_temp, 0.5, 0.0, overlay);
        //overlay = temp;

<<<<<<< HEAD
       // return heading_average;
	return heading_mode; // modified by Miao, Oct.21,2015,22:38
=======
        return heading_mode;
>>>>>>> 707a3894523df2b7b9afaad30773b10131ddb5db
    }
/*

    void CannyThreshold(int, void*){
      Mat temp, color_temp; //setup some temps
      cvtColor(cv_ptr->image, temp, CV_BGR2GRAY); //convert to grayscale for the edge detector
      //Sobel(temp, temp, CV_8U, 1, 1);

      //Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
      //Canny( temp, temp, 50, 200, 3 ); //run Canny edge detector with some default values
      Canny( temp, temp, lowThreshold, lowThreshold*ratio, kernel_size );
      cvtColor( temp, color_temp, CV_GRAY2BGR ); //Convert Canny edges back to 3-channel

      vector<Vec4i> lines;
      HoughLinesP( temp, lines, 1, CV_PI/180, 80, 30, 10 ); //Find lines in the Canny image
      for( size_t i = 0; i < lines.size(); i++ )
      {
        line( color_temp, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 ); //Draw
      }
        
    }
*/

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;

    //cv_bridge::CvImage out_msg;
    crawler_msgs::VisualHeading visual_heading;

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
      //cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2,cv_ptr->image.rows/2), 30, CV_RGB(0,0,255));
      //cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2,cv_ptr->image.rows/2), 5, CV_RGB(255,0,0));
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
        //cv::circle(cv_ptr->image, cv::Point(10, 100), 10, CV_RGB(255,0,0));
        //cv::circle(cv_ptr->image, cv::Point(10, 500), 10, CV_RGB(255,0,0));
        //cv::circle(cv_ptr->image, cv::Point(10, 1000), 10, CV_RGB(255,0,0));
    //image.row = 960, image.cols = 1280 .
        //ROS_INFO("image.row = %d, image.cols = %d .",cv_ptr->image.rows,cv_ptr->image.cols);

    
    visual_heading.RPY_radian.z = getHeading(cv_ptr->image,output);

    //cv::imshow(OPENCV_WINDOW, overlay);

    //cv::waitKey(3);
    
    // Output modified video stream
    visual_heading.header   = cv_ptr->header; // Same timestamp and tf frame as input image
    
    visual_heading.RPY_degree.z = visual_heading.RPY_radian.z * 180 / PI;

    visual_heading.RPY_degree.z = visual_heading.RPY_degree.z + 90;

     msg_pub_.publish(visual_heading);
    //saliency_img_pub.publish(out_msg.toImageMsg());
    //msg_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "edge_detection");
  ImageConverter ic;
  ros::spin();
  return 0;
}
