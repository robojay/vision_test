#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Pose2D.h>

//static const std::string OPENCV_WINDOW = "Image window";
double rhoAverage = 0.0;
double thetaAverage = 0.0;
int sampleCount = 0;
const double RMA = 5.0;
ros::Publisher linePub;

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
    //image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
    //  &ImageConverter::imageCb, this);
    image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg); // sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat scaledImage;
    cv_ptr->image.convertTo(scaledImage, CV_8UC1);

    cv::Mat blurredImage;
    cv::blur(scaledImage, blurredImage, cv::Size(9,9));

    cv::Mat threshImage;
    cv::threshold(blurredImage, threshImage, 128, 255, CV_THRESH_BINARY);    
    
    cv::Mat edgeImage;
    cv::Canny(threshImage, edgeImage, 32, 128, 3);
    //cv::Canny(blurredImage, edgeImage, 32, 128, 3);

    cv::Mat colorImage = cv::Mat::zeros(scaledImage.size(), CV_8UC3);
    cv::Mat colorImage2 = cv::Mat::zeros(scaledImage.size(), CV_8UC3);
    cv::cvtColor(edgeImage, colorImage, CV_GRAY2RGB);
    cv::cvtColor(edgeImage, colorImage2, CV_GRAY2RGB);


    #if 0
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edgeImage, lines, 1, CV_PI/180, 50, 50, 10 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
      cv::Vec4i l = lines[i];
      cv::line( colorImage, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
    }
    imshow("HoughLinesP", colorImage);
    #endif


    std::vector<cv::Vec2f> lines2;
    HoughLines(edgeImage, lines2, 1, CV_PI/180, 32, 0, 0 );
    float saveRho = 0.0;
    float saveTheta = 0.0;
    float rho;
    float theta;

    for( size_t i = 0; i < lines2.size(); i++ )
    {
      rho = lines2[i][0];
      theta = lines2[i][1];
        if ( (theta > 0.75*(CV_PI/2)) && (theta < 1.25*(CV_PI/2)) ) {
          if (rho > saveRho) {
            saveRho = rho;
            saveTheta = theta;
          }
       }
    }

    rho = saveRho;
    theta = saveTheta;
    if (sampleCount < RMA) {
      rhoAverage += rho;
      thetaAverage += theta;
      sampleCount++;
    }
    else if (sampleCount == RMA) {
      rhoAverage /= RMA;
      rhoAverage = rho/RMA + (1 - 1/RMA)*rhoAverage; 
      thetaAverage /= RMA;
      thetaAverage = theta/RMA + (1 - 1/RMA)*thetaAverage; 
      sampleCount++;
    }
    else {
      rhoAverage = rho/RMA + (1 - 1/RMA)*rhoAverage; 
      thetaAverage = theta/RMA + (1 - 1/RMA)*thetaAverage; 
    }

    if (sampleCount >= RMA) {
      cv::Point pt1, pt2;
      double a = cos(thetaAverage), b = sin(thetaAverage);
      double x0 = a*rhoAverage, y0 = b*rhoAverage;
      pt1.x = cvRound(x0 + 1000*(-b));
      pt1.y = cvRound(y0 + 1000*(a));
      pt2.x = cvRound(x0 - 1000*(-b));
      pt2.y = cvRound(y0 - 1000*(a));
      cv::line( colorImage2, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
      //printf("rho %f theta %f\n", rhoAverage, thetaAverage);
      //printf("===\n");

      geometry_msgs::Pose2D lineData;
      lineData.x = rho;
      lineData.theta = theta;
      lineData.y = 0;
      linePub.publish(lineData);
    }

    imshow("HoughLines", colorImage2);




    //printf("%d %d %d %d\n", colorImage.channels(), colorImage.rows, colorImage.cols, colorImage.dims);

    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    //if (colorImage.rows > 50 && colorImage.cols > 50)
    //  cv::circle(colorImage, cv::Point(50, 50), 50, CV_RGB(0,0,65535));


    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::imshow(OPENCV_WINDOW, colorImage);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_test");
  ros::NodeHandle nh;

  ImageConverter ic;
  linePub = nh.advertise<geometry_msgs::Pose2D>("/lineDetect", 1);
  ros::spin();
  return 0;
}