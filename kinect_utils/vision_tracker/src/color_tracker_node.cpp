/*************************************************************************
	> File Name: color_tracker_node.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 10 Apr 2017 05:36:44 PM PDT
 ************************************************************************/

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <cassert>

#include <Object.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <vision_tracker/WriteToDisk.h>

using namespace std;
using namespace ros;

// default capture width and height
const int FRAME_HEIGHT = 480;
const int FRAME_WIDTH = 640;
// max number of objects to be detected in the frame 
const int MAX_NUM_OBJECTS = 50;
// minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;


class Trackers
{
    image_transport::ImageTransport it;
    image_transport::Subscriber sub_;
    ros::NodeHandle nh_;

    public:
        // Matrix to store each frame of the Kinet feed
        Mat cameraFeed;
        Mat cameraFeed_cropped; 
        Mat cameraFeed_backup;
        Mat threshold;
        Mat HSV;
        Mat HSV_cropped;

        void ImageCallback(const sensor_msgs::ImageConstPtr &msg);
        void RecordCallback(const sensor_msgs::ImageConstPtr &msg);
        static void MouseCallback(int event, int x, int y, int flags, void* userdata); // static member function can only access static member of the class 
        void trackFilteredObject(Object theObject, Mat threshold, Mat &cameraFeed, bool drawLine = false);
        void drawObject(vector<Object> theObjects, Mat &frame, vector<vector<Point> > contours, vector<Vec4i> hierarchy);
        void morphOps(Mat &threshold);
        void startTracking();
        string intToString(int number);

        string windowName;
        string windowName1;
        string windowName2;
        string windowName3;

        double max_y; // coordinate used to crop the image 
        static bool clicked;
        static int mouse_x, mouse_y; // coordinate pointed by the left button of the mouse 

        ros::ServiceServer write_to_disk_server_;
        bool WriteToDiskCallback(vision_tracker::WriteToDisk::Request &req, vision_tracker::WriteToDisk::Response &res);

        Trackers(NodeHandle &nh1);
        ~Trackers();

        

   //private:
   //     image_transport::ImageTransport it(ros::NodeHandle nh_);
   //     image_transport::Subscriber sub_; // subscriber to the sensor_msgs/image topic
        
};

// Initializa the static member of class Trackers 
int Trackers::mouse_x = 0;
int Trackers::mouse_y = 0;
bool Trackers::clicked = false;

Trackers::Trackers(NodeHandle& nh1) : it(nh1), nh_(nh1)
{
    // set up the window for the kinect output 
    windowName = "RGBImage"; 
    windowName1 = "ThresholdImage for Red";
    windowName2 = "RGBImage_cropped";
  
    namedWindow(windowName, CV_WINDOW_NORMAL);
    resizeWindow(windowName, FRAME_WIDTH, FRAME_HEIGHT);

    namedWindow(windowName1, CV_WINDOW_NORMAL);
    resizeWindow(windowName1, FRAME_WIDTH, FRAME_HEIGHT);
    
    namedWindow(windowName2, CV_WINDOW_NORMAL);
    resizeWindow(windowName2, FRAME_WIDTH, FRAME_HEIGHT);
    
    max_y = 0; // subject to change 
    
    setMouseCallback(windowName, MouseCallback, NULL); // register the mouse call back  

    write_to_disk_server_ = nh_.advertiseService("write_img_to_disk", &Trackers::WriteToDiskCallback, this);
     
    // use image_transport instead of ROS primitives gives you great flexibility in how images are communicated between nodes
    cout << "Trackers class constructed!" << endl;
    //startTracking(sub_, it);
    //sub_ = it.subscribe("/camera/rgb/image_color", 1, &Trackers::ImageCallback, this);

}

Trackers::~Trackers(){}

void Trackers::startTracking()
{
    cout << "Start tracking now!" << endl;
    sub_ = it.subscribe("/camera/rgb/image_color", 1, &Trackers::ImageCallback, this);
    //sub_ = it.subscribe("/camera/rgb/image_color", 1, &Trackers::RecordCallback, this);
}

void Trackers::MouseCallback(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          mouse_x = x;
          mouse_y = y;
          clicked = true;
     }

     /*
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if ( event == EVENT_MOUSEMOVE )
     {
          cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

     }
     */
}

void Trackers::ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    // Convert the Image class to OpenCV naive BGR color 
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

    cv_ptr->image.copyTo(cameraFeed);
    cv_ptr->image.copyTo(cameraFeed_backup);
    // first denoise the image 
    //Mat temp;
    //fastNlMeansDenoisingColoredMulti(cameraFeed,cameraFeed_backup, 3, 3, 3); //
    //temp.copyTo(cameraFeed);

    // cropped the image and start to track the white object
    if (max_y == 0)
    {
        cv_ptr->image.copyTo(cameraFeed_cropped);
    }
    else
    {
        cv::Rect croppedArea(0, max_y, FRAME_WIDTH, FRAME_HEIGHT - max_y);
        cameraFeed(croppedArea).copyTo(cameraFeed_cropped);
    }
    
    if (!cameraFeed.data)
    {
        ROS_ERROR("empty image!");
        return;
    }

    // convert the frame from BGR to HSV colorspace
    cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
    cvtColor(cameraFeed_cropped, HSV_cropped, COLOR_BGR2HSV);

    // print the hsv value of the picxel clicked by the mouse 
    
    //Vec3b hsv_clicked = HSV.at<Vec3b>(mouse_x, mouse_y);
    //cout <<"Position: [" << mouse_x << ", " << mouse_y <<  "]. The HSV value at this pixel is " << hsv_clicked << endl;
    
	//create some temp fruit objects so that
	//we can use their member functions/information
	Object red("red"),green("green"), white("white");
    
    /*
	//first find green objects
    cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
	inRange(HSV,green.getHSVmin(),green.getHSVmax(),threshold);
	morphOps(threshold);
	trackFilteredObject(green, threshold, cameraFeed);
    cv::imshow(windowName2, threshold);
    
    */

	inRange(HSV_cropped,green.getHSVmin(),green.getHSVmax(),threshold);
	morphOps(threshold);
	trackFilteredObject(green, threshold, cameraFeed_cropped);
    cv::imshow(windowName2,cameraFeed_cropped);
    
    

	// then find red objects
	inRange(HSV,red.getHSVmin(),red.getHSVmax(),threshold);
	morphOps(threshold);
	trackFilteredObject(red, threshold, cameraFeed, true);
    cv::imshow(windowName1, threshold);
    //show the image on the window 
    cv::imshow(windowName,cameraFeed);
    //cv::imshow(windowName1, threshold);
    waitKey(30);


}

void Trackers::trackFilteredObject(Object theObject, Mat threhold, Mat &cameraFeed, bool drawLine)
{
	vector <Object> objects;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

		//if the area is less than 20 px by 20px then it is probably just noise
		//if the area is the same as the 3/2 of the image size, probably just a bad filter
		//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA){

					Object object;

					object.setXPos(moment.m10/area);
					object.setYPos(moment.m01/area);
					object.setType(theObject.getType());
					object.setColor(theObject.getColor());

					objects.push_back(object);

					objectFound = true;

				}else objectFound = false;
			}
			//let user know you found an object
			if(objectFound ==true)
            {
				//draw object location on screen
				drawObject(objects,cameraFeed,contours,hierarchy);

                if (objects.size() >= 2 && drawLine)
                {
                    // only draw line between the first two contours, between only two red markers are on the peeler
                    Scalar mean1 = mean(contours[0]);
                    Scalar mean2 = mean(contours[1]);
            
                    max_y = (mean1(1) <= mean2(1) ? mean2(1) : mean1(1));
                    cv::Point pt2 = cv::Point(mean2(0), max_y);
                    cv::Point pt1 = cv::Point(mean1(0), max_y);
            
                    // always draw a horizontal line at the lowerest y level.
                    line(cameraFeed, pt1, pt2, (0, 255, 255), 5); 

                    // cropped the image and start to track the white object
                    cv::Rect croppedArea(0, max_y, FRAME_WIDTH, FRAME_HEIGHT - max_y);
                    cameraFeed_cropped = cameraFeed(croppedArea);
                }
            }
            else
            {
                cv::putText(cameraFeed, "No Object Found!", cv::Point(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 1, 2, cv::Scalar(0, 0, 255), 2);
                //cout << "No Object Found!" << endl;
            }

		}
        else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}    
}

void Trackers::drawObject(vector<Object> theObjects, Mat &cameraFeed, vector<vector<Point> >contours, vector<Vec4i> hierarchy)
{

	for(int i =0; i<theObjects.size(); i++)
    {
        cv::drawContours(cameraFeed,contours,i,theObjects.at(i).getColor(),-1,8,hierarchy);
	    cv::circle(cameraFeed,cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()),5,theObjects.at(i).getColor());
	    cv::putText(cameraFeed,intToString(theObjects.at(i).getXPos())+ " , " + intToString(theObjects.at(i).getYPos()),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()+20),1,1,theObjects.at(i).getColor());
	    cv::putText(cameraFeed,theObjects.at(i).getType(),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()-20),1,2,theObjects.at(i).getColor());
	}
}

void Trackers::morphOps(Mat &threshold)
{
	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

    erode(threshold,threshold,erodeElement, cv::Point(-1, -1), 3);
    dilate(threshold,threshold,dilateElement, cv::Point(-1, -1), 3);
}

string Trackers::intToString(int number)
{
	std::stringstream ss;
	ss << number;
	return ss.str();    
}

bool Trackers::WriteToDiskCallback(vision_tracker::WriteToDisk::Request &req, vision_tracker::WriteToDisk::Response &res)
{
    std::string image_path = ros::package::getPath("vision_tracker") + "/img/" + req.file_name;
    ROS_INFO("Start writing the image to disk......");

    Mat temp;
    fastNlMeansDenoisingColored(cameraFeed_backup, temp); //
    
    cv::imwrite(image_path, temp);

    ROS_INFO("Image written to disk");

    return (res.status = true);
}

void Trackers::RecordCallback(const sensor_msgs::ImageConstPtr &msg)
{
    // Convert the Image class to OpenCV naive BGR color 
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

    double secs =ros::Time::now().toSec();
    
    cv::Mat recorded_img;
    cv_ptr->image.copyTo(recorded_img);
    

    std::string image_path = ros::package::getPath("vision_tracker") + "/img/" + std::to_string(secs) + ".png";
    cv::imwrite(image_path, recorded_img);
    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "color_based_tracker");

    ros::NodeHandle nh;
    Trackers tracker = Trackers(nh);

    //setMouseCallback(tracker.windowName, CallBackFunc, NULL); // register the mouse call back
    
    tracker.startTracking();
    
    while(ros::ok())
    {
        usleep(200000);
        ros::spinOnce();
    }

    return 0;
}



