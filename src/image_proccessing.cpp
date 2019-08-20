#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <kkctbn2019/ObjectCoordinate.h>
#include <mavros_msgs/RCIn.h>

using namespace cv;
using namespace std;

ros::Publisher coordinate_publisher;

Mat receive_image;


void imageProccessing(Mat image_camera) {
  //bikin window
  namedWindow("TRACKBAR", CV_WINDOW_AUTOSIZE);

  //tentuin angka di trackbar, sementara merah
  int iLowH = 0;
  int iHighH = 179;

  int iLowS = 110;
  int iHighS = 243;

  int iLowV = 220;
  int iHighV = 255;

  //buat trackbar
  cvCreateTrackbar("LowH", "Control", &iLowH, 179);
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);
  cvCreateTrackbar("LowS", "Control", &iLowS, 243);
  cvCreateTrackbar("HighS", "Control", &iHighS, 243);
  cvCreateTrackbar("LowV", "Control", &iLowV, 255);
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  int iLastX = -1;
  int iLastY = -1;

  Mat imgTmp = image_camera;

  while(true){
    //baca frame video
    Mat imgOriginal = image_camera;

    //bikin image dari rgb ke hsv
    Mat imgHSV;
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

    //bikin window Thresholded
    Mat imgThresholded;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

    //ngecilin noise
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)) );
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)) );

    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)) );


    //cari lokasi pake moments
    Moments oMoments = moments(imgThresholded);

    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double areaObjek = oMoments.m00;

    //kalo image nya lebih dari 10000px
    if(areaObjek>10000){
      int posX = dM10 / areaObjek;
      int posY = dM01 / areaObjek;
      if(iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0){
        kkctbn2019::ObjectCoordinate coordinate;
        coordinate.x = posX;
        coordinate.y = posY;
        coordinate_publisher.publish(coordinate);
        cout << "posX dan posY: " << posX << " ----- " << posY << endl;
      }
      iLastX = posX;
      iLastY = posY;
    }

    imshow("Thresholded Image", imgThresholded);
    imshow("Original", imgOriginal);
  }
}

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg){
  try
  {
    receive_image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
    waitKey(10);
    imageProccessing(receive_image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to image!");
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "image_proccessing");
  ros::NodeHandle nh;
  
  coordinate_publisher = nh.advertise<kkctbn2019::ObjectCoordinate>("KOORDINAT", 8);
  
  ros::Subscriber terima_gambar = nh.subscribe("/camera/image", 8, imageCallback);

  ros::spin();
}


