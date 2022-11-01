#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <wiringPi.h>
#include "function.cpp"


using namespace std;
using namespace cv;
using namespace raspicam;

        // Image Processing variables
Mat frame, Matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal, frameFinalDuplicate, frameFinalDuplicate1;
Mat ROILane, ROILaneEnd;
int LeftLanePos, RightLanePos, frameCenter, laneCenter, Result, laneEnd;

RaspiCam_Cv Camera; // creating camera object

stringstream ss; // to print the Result value on video screen


vector<int> histrogramLane; // to store the intensity values of pixels in stripes which is in ROI (For lane)

vector<int> histrogramLaneEnd; // to store the intensity values of pixels in stripes which is in ROI (For Lane End)

Point2f Source[] = {Point2f(30,145),Point2f(355,145),Point2f(2,195), Point2f(385,195)};    // Four points of Region Of Interest
Point2f Destination[] = {Point2f(100,0),Point2f(280,0),Point2f(100,240), Point2f(280,240)};  // Four points for perspective 

//Point2f Source[] = {Point2f(40,135),Point2f(360,135),Point2f(0,185), Point2f(400,185)};
//Point2f Destination[] = {Point2f(100,0),Point2f(280,0),Point2f(100,240), Point2f(280,240)};


            // Machine Learning Variables
// STOP SIGN
CascadeClassifier Stop_Cascade; // creating stop cascade object
Mat frame_stop, ROI_stop, gray_Stop;
vector<Rect> Stop; // vector for stop sign
int dist_Stop; // to store distance value

// CAR 
CascadeClassifier Car_Cascade; // creating car cascade object
Mat frame_car, ROI_car, gray_Car;
vector<Rect> Car; // vector for car
int dist_Car; // to store distance value

// Speed 20 and 50
CascadeClassifier Speed20_Cascade, Speed50_Cascade; // creating speed cascade object
Mat frame_Speed, ROI_Speed, gray_Speed;
vector<Rect> Speed20, Speed50; // vector for speed
int dist_Speed20, dist_Speed50; // to store distance value

// Traffic Lights 
CascadeClassifier Traffic_Cascade; // creating car cascade object
Mat frame_traffic, ROI_traffic, gray_traffic;
vector<Rect> Traffic; // vector for car
int dist_Traffic; // to store distance value


void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
{
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,400 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240 ) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,0));

}

void Capture()  // Capture the video
{
	Camera.grab();   // grabbing images from my camera
    Camera.retrieve( frame);  // Decodes and returns the grabbed video frame
    cvtColor(frame, frame_stop, COLOR_BGR2RGB); // Covert BGR image into RGB - STOP FRAME
    cvtColor(frame, frame_Speed, COLOR_BGR2RGB); // Covert BGR image into RGB - SPEED FRAME
    cvtColor(frame, frame_car, COLOR_BGR2RGB); // Covert BGR image into RGB
    cvtColor(frame, frame, COLOR_BGR2RGB); // Covert BGR image into RGB
}

void Perspective()  // Drawing the lines to 4 points and doing Perspective Transformation
{
        // Drawing the lines to all 4 points (with red color line)
	line(frame,Source[0], Source[1], Scalar(0,0,255), 2);
	line(frame,Source[1], Source[3], Scalar(0,0,255), 2);  
	line(frame,Source[3], Source[2], Scalar(0,0,255), 2);
	line(frame,Source[2], Source[0], Scalar(0,0,255), 2);
	
	//Calculates a perspective transform from four pairs of the corresponding points.
	Matrix = getPerspectiveTransform(Source, Destination);  
    // Applies a perspective transformation to an image
	warpPerspective(frame, framePers, Matrix, Size(400,240));
}


void Threshold()  // Threshold Operation and Canny Edge Detection
{
	cvtColor(framePers, frameGray, COLOR_RGB2GRAY); // converting RGB image to grayscale image
	inRange(frameGray, 230, 255, frameThresh);  // Defining the minimum & maximum thresholds range for white
	Canny(frameGray,frameEdge, 800, 800, 3, false); // Finds edges in an image using the Canny algorithm 
                                                    //  with custom image gradient.
                                                    
	add(frameThresh, frameEdge, frameFinal);  // merging frameThresh & frameEdge
	cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB); // Covert grayscale image into RGB
    //used in histrogram function only
	cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR); // Covert RGB image into BGR
    cvtColor(frameFinal, frameFinalDuplicate1, COLOR_RGB2BGR); // Covert RGB image into BGR
	
}

void Histrogram() // To find the lane line position (stored in dynamic array)
{
            // FOR LANE
    histrogramLane.resize(400); // mentioning the size of vector
    histrogramLane.clear();   // clearing the values in vector
    
    for(int i=0; i<400; i++)  //frame.size().width = 410
    {
        // creating rectangle (strips) function for Region of Interest
        ROILane = frameFinalDuplicate(Rect(i,140,1,100));
        divide(255, ROILane, ROILane); // maximum posible intensity value to each pixel
        // pushing all intensity value to vector
        histrogramLane.push_back((int)(sum(ROILane)[0]));  
    }
    
            // FOR LANE END
    histrogramLaneEnd.resize(400); // mentioning the size of vector
    histrogramLaneEnd.clear();   // clearing the values in vector
    
    for(int i=0; i<400; i++)  //frame.size().width = 410
    {
        // creating rectangle (strips) function for Region of Interest
        ROILaneEnd = frameFinalDuplicate1(Rect(i,0,1,240));
        divide(255, ROILaneEnd, ROILaneEnd); // maximum posible intensity value to each pixel
        // pushing all intensity value to vector
        histrogramLaneEnd.push_back((int)(sum(ROILaneEnd)[0]));  
    }
    
    laneEnd = sum(histrogramLaneEnd)[0]; // Adding all the elemet from lane end array
    cout<<"Lane End = "<<laneEnd<<endl;  // to view the total value
    
}

// To find the lane line position by using dynamic array
// And Drawing the line on left lane & right lane 
void LaneFinder()  
{
    // this iterator will point to the maximum intensity location in the array
    vector<int>:: iterator LeftPtr; 
    // "max_element" function to find the maximum element in the array
    LeftPtr = max_element(histrogramLane.begin(), histrogramLane.begin() + 150);
    // to get the position of left lane (first lane) 
    LeftLanePos = distance(histrogramLane.begin(), LeftPtr); 
    
    // this iterator will point to the maximum intensity location in the array
    vector<int>:: iterator RightPtr;
    // "max_element" function to find the maximum element in the array
    RightPtr = max_element(histrogramLane.begin() +250, histrogramLane.end());
    // to get the position of right lane (second lane) 
    RightLanePos = distance(histrogramLane.begin(), RightPtr);
    
    // Displaying "LeftLanePos" and "RightLanePos" by "line" function (By drawing a line)
    // Color of the line is Green   
    line(frameFinal, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, 240), Scalar(0, 255,0), 2);
    line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0,255,0), 2); 
}

void LaneCenter() // Finding the lane center
{
    // getting the lane center  
    laneCenter = (RightLanePos-LeftLanePos)/2 +LeftLanePos;
    // frame center position
    frameCenter = 188;
    
    // Drawing vertical line for lane center (Green color line)
    line(frameFinal, Point2f(laneCenter,0), Point2f(laneCenter,240), Scalar(0,255,0), 3);
    // Drawing vertical line for frame center (Blue color line)
    line(frameFinal, Point2f(frameCenter,0), Point2f(frameCenter,240), Scalar(255,0,0), 3);

    // Result is different between lane center and frame center 
    Result = laneCenter-frameCenter;
}



void Stop_detection() // function to load the stop cascade xml file and detect it
{
    // if file didnt load it print error message
    if(!Stop_Cascade.load("//home/pi/Desktop/MACHINE LEARNING//Stopsign_cascade.xml"))
    {
        printf("Unable to open stop cascade file");
    }
    
    ROI_stop = frame_stop(Rect(0,0,400,240));// Region of Interest for "Stop Sign"
    cvtColor(ROI_stop, gray_Stop, COLOR_RGB2GRAY);// Covert RGB image into gray
    equalizeHist(gray_Stop, gray_Stop);// equalize all the intensity of grayscale image
    Stop_Cascade.detectMultiScale(gray_Stop, Stop); // to detect stop sign
    
    // to iterate all the points in vector (Stop). To find the stop sign
    for(int i=0; i<Stop.size(); i++)
    {
        // creating 2 points to draw a rectangle around the detected stop sign 
        
        Point P1(Stop[i].x, Stop[i].y);
        Point P2(Stop[i].x + Stop[i].width, Stop[i].y + Stop[i].height);
        
        // Drawing a red color rectangle by using Point P1 and P2
        rectangle(ROI_stop, P1, P2, Scalar(0, 0, 255), 2);
        // Print the "Stop Sign" text near Point P1.
        putText(ROI_stop, "Stop Sign", P1, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255, 255), 2);
        
        //dist_Stop = (-1.5)*(P2.x-P1.x)+109.5; //linear equation to calculate the distance
        dist_Stop = distanceStop(P2.x, P1.x);
        
        // Displaying the distance on the frame
        ss.str("");
        ss.clear();
        ss<<"D = "<<dist_Stop<<"cm";
        putText(ROI_stop,ss.str(),Point2f(1,130),0,1,Scalar(0,0,255),2);
        
    } 
}


void Speed20_detection() // function to load the car cascade xml file and detect it
{
    // if file didnt load it print error message
    if(!Speed20_Cascade.load("//home/pi/Desktop/MACHINE LEARNING//cascade_20.xml"))
    {
        printf("Unable to open car cascade file");
    }
    
    // left,top,width,height
     ROI_Speed = frame_Speed(Rect(0,0,400,240));// Region of Interest for "Speed"
     
    cvtColor(ROI_Speed, gray_Speed, COLOR_RGB2GRAY);// Covert RGB image into gray
    equalizeHist(gray_Speed, gray_Speed);// equalize all the intensity of grayscale image
    Speed20_Cascade.detectMultiScale(gray_Speed, Speed20); // to detect car
    
    // to iterate all the points in vector (Car). To find the car
    for(int i=0; i<Speed20.size(); i++)
    {
        // creating 2 points to draw a rectangle around the detected car 
        
        Point P1(Speed20[i].x, Speed20[i].y);
        Point P2(Speed20[i].x + Speed20[i].width, Speed20[i].y + Speed20[i].height);
        
        // Drawing a red color rectangle by using Point P1 and P2
        rectangle(ROI_Speed, P1, P2, Scalar(0, 0, 255), 2);
        // Print the "car" text near Point P1.
        putText(ROI_Speed, "Speed 20", P1, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255, 255), 2);
        
        // dist_Speed20 = (-0.652)*(P2.x-P1.x)+79.565; //linear equation to calculate the distance
        dist_Speed20 = distanceSpeedSign20(P2.x, P1.x);
        
        
        // Displaying the distance on the frame
        ss.str("");
        ss.clear();
        //ss<<"D = "<<P2.x-P1.x<<"cm";
        ss<<"D = "<<dist_Speed20<<"cm";
        putText(ROI_Speed,ss.str(),Point2f(1,130),0,1,Scalar(0,0,255),2);
        
    } 
}


void Speed50_detection() // function to load the car cascade xml file and detect it
{
    // if file didnt load it print error message
    if(!Speed50_Cascade.load("//home/pi/Desktop/MACHINE LEARNING//cascade_50.xml"))
    {
        printf("Unable to open car cascade file");
    }
    
    // left,top,width,height
    ROI_Speed = frame_Speed(Rect(0,0,400,240));// Region of Interest for "Speed"
     
    cvtColor(ROI_Speed, gray_Speed, COLOR_RGB2GRAY);// Covert RGB image into gray
    equalizeHist(gray_Speed, gray_Speed);// equalize all the intensity of grayscale image
    Speed50_Cascade.detectMultiScale(gray_Speed, Speed50); // to detect car
    
    // to iterate all the points in vector (Car). To find the car
    for(int i=0; i<Speed50.size(); i++)
    {
        // creating 2 points to draw a rectangle around the detected car 
        
        Point P1(Speed50[i].x, Speed50[i].y);
        Point P2(Speed50[i].x + Speed50[i].width, Speed50[i].y + Speed50[i].height);
        
        // Drawing a red color rectangle by using Point P1 and P2
        rectangle(ROI_Speed, P1, P2, Scalar(0, 0, 255), 2);
        // Print the "car" text near Point P1.
        putText(ROI_Speed, "Speed 50", P1, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255, 255), 2);
        
        //dist_Speed50 = (-0.652)*(P2.x-P1.x)+79.565; //linear equation to calculate the distance
         dist_Speed50 = distanceSpeedSign50(P2.x, P1.x);
        
        // Displaying the distance on the frame
        ss.str("");
        ss.clear();
        //ss<<"D = "<<P2.x-P1.x<<"cm";
        ss<<"D = "<<dist_Speed50<<"cm";
        putText(ROI_Speed,ss.str(),Point2f(1,160),0,1,Scalar(0,0,255),2);
        
    } 
}

void Car_detection() // function to load the car cascade xml file and detect it
{
    // if file didnt load it print error message
    if(!Car_Cascade.load("//home/pi/Desktop/MACHINE LEARNING//Car_cascade.xml"))
    {
        printf("Unable to open car cascade file");
    }
    
    // left,top,width,height
    ROI_car = frame_car(Rect(100,40,200,180));// Region of Interest for "Car"
    cvtColor(ROI_car, gray_Car, COLOR_RGB2GRAY);// Covert RGB image into gray
    equalizeHist(gray_Car, gray_Car);// equalize all the intensity of grayscale image
    Car_Cascade.detectMultiScale(gray_Car, Car); // to detect car
    
    // to iterate all the points in vector (Car). To find the car
    for(int i=0; i<Car.size(); i++)
    {
        // creating 2 points to draw a rectangle around the detected car 
        
        Point P1(Car[i].x, Car[i].y);
        Point P2(Car[i].x + Car[i].width, Car[i].y + Car[i].height);
        
        // Drawing a red color rectangle by using Point P1 and P2
        rectangle(ROI_car, P1, P2, Scalar(0, 0, 255), 2);
        // Print the "car" text near Point P1.
        putText(ROI_car, "Car", P1, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255, 255), 2);
        
        //dist_Car = (-0.652)*(P2.x-P1.x)+79.565; //linear equation to calculate the distance
        dist_Car = distanceCar(P2.x, P1.x);
        
        // Displaying the distance on the frame
        ss.str("");
        ss.clear();
        ss<<"D = "<<dist_Car<<"cm";
        putText(ROI_car,ss.str(),Point2f(1,130),0,1,Scalar(0,0,255),2);
        
    } 
}


void Traffic_detection() // function to load the traffic sign cascade xml file and detect it
{
    // if file didnt load it print error message
    if(!Traffic_Cascade.load("//home/pi/Desktop/MACHINE LEARNING//Traffic_cascade.xml"))
    {
        printf("Unable to open traffic sign cascade file");
    }
    
    // left,top,width,height
    ROI_Speed = frame_Speed(Rect(0,0,400,240));// Region of Interest for traffic sign
    cvtColor(ROI_Speed, gray_traffic, COLOR_RGB2GRAY);// Covert RGB image into gray
    equalizeHist(gray_traffic, gray_traffic);// equalize all the intensity of grayscale image
    Traffic_Cascade.detectMultiScale(gray_traffic, Traffic); // to detect traffic sign
    
    // to iterate all the points in vector (Traffic). To find the traffic sign
    for(int i=0; i<Traffic.size(); i++)
    {
        // creating 2 points to draw a rectangle around the detected traffic sign 
        
        Point P1(Traffic[i].x, Traffic[i].y);
        Point P2(Traffic[i].x + Traffic[i].width, Traffic[i].y + Traffic[i].height);
        
        // Drawing a red color rectangle by using Point P1 and P2
        rectangle(ROI_Speed, P1, P2, Scalar(0, 0, 255), 2);
        // Print the "traffic red" text near Point P1.
        putText(ROI_Speed, "Traffic RED", P1, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255, 255), 2);
        
         //linear equation to calculate the distance
        dist_Traffic = distanceTrafficLight(P2.x, P1.x);
        
        // Displaying the distance on the frame
        ss.str("");
        ss.clear();
        ss<<"D = "<<dist_Traffic<<"cm";
        //ss<<"D = "<<P2.x-P1.x<<"cm";
        putText(ROI_Speed,ss.str(),Point2f(1,130),0,1,Scalar(0,0,255),2);
        
    } 
}


int main(int argc,char **argv)
{
    
    wiringPiSetup();
    pinMode(21, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT);
	
	Setup(argc, argv, Camera);
	cout<<"Connecting to camera"<<endl;
	
    if (!Camera.open())
	{
        cout<<"Failed to Connect"<<endl;
    }
     
    cout<<"Camera Id = "<<Camera.getId()<<endl;
     
    
    while(1)
    {
        auto start = std::chrono::system_clock::now();

        Capture();
        Perspective();
        Threshold();
        Histrogram();
        LaneFinder();
        LaneCenter();
        
        Stop_detection();
        Car_detection();
        Speed20_detection();
        Speed50_detection();
        Traffic_detection();
        
    
        if (dist_Stop > 5 && dist_Stop < 20)
        {
            digitalWrite(21, 0);
            digitalWrite(22, 0);    //decimal = 8
            digitalWrite(23, 0);
            digitalWrite(24, 1);
            cout<<"Stop Sign"<<endl;
            dist_Stop = 0;
            
            goto Stop_Sign;
        }
        if (dist_Car > 5 && dist_Car < 20)
        {
            digitalWrite(21, 1);
            digitalWrite(22, 0);    //decimal = 9
            digitalWrite(23, 0);
            digitalWrite(24, 1);
            cout<<"Car"<<endl;
            dist_Car = 0;
            
            goto Car_Sign;
        }
       /* if(laneEnd > 4000)
        {
            digitalWrite(21, 1);
            digitalWrite(22, 1);    //decimal = 7
            digitalWrite(23, 1);
            digitalWrite(24, 0);
            cout<<"Lane End (U-Turn)"<<endl;
        }*/
        
        if (Result == 0)
        {
            digitalWrite(21, 0);
            digitalWrite(22, 0);    //decimal = 0
            digitalWrite(23, 0);
            digitalWrite(24, 0);
            cout<<"Forward"<<endl;
        }
        
            
        else if (Result >0 && Result <10)
        {
            digitalWrite(21, 1);
            digitalWrite(22, 0);    //decimal = 1
            digitalWrite(23, 0);
            digitalWrite(24, 0);
            cout<<"Right1"<<endl;
        }
        
        else if (Result >=10 && Result <20)
        {
            digitalWrite(21, 0);
            digitalWrite(22, 1);    //decimal = 2
            digitalWrite(23, 0);
            digitalWrite(24, 0);
            cout<<"Right2"<<endl;
        }
        
        else if (Result >20)
        {
            digitalWrite(21, 1);
            digitalWrite(22, 1);    //decimal = 3
            digitalWrite(23, 0);
            digitalWrite(24, 0);
            cout<<"Right3"<<endl;
        }
        
        else if (Result <0 && Result >-10)
        {
            digitalWrite(21, 0);
            digitalWrite(22, 0);    //decimal = 4
            digitalWrite(23, 1);
            digitalWrite(24, 0);
            cout<<"Left1"<<endl;
        }
        
        else if (Result <=-10 && Result >-20)
        {
            digitalWrite(21, 1);
            digitalWrite(22, 0);    //decimal = 5
            digitalWrite(23, 1);
            digitalWrite(24, 0);
            cout<<"Left2"<<endl;
        }
        
        else if (Result <-20)
        {
            digitalWrite(21, 0);
            digitalWrite(22, 1);    //decimal = 6
            digitalWrite(23, 1);
            digitalWrite(24, 0);
            cout<<"Left3"<<endl;
        }
    
        Stop_Sign:
        Car_Sign:
        
    
        if(laneEnd > 4000)
        {
            ss.str(" ");
            ss.clear();
            ss<<"Lane End";
            putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(255,0,0), 2);
        }
        else if(Result == 0)
        {
            ss.str(" ");
            ss.clear();
            ss<<"Result = "<<Result<<"(Move Forward)";
            putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
        }
         else if(Result > 0)
        {
            ss.str(" ");
            ss.clear();
            ss<<"Result = "<<Result<<"(Move Right)";
            putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
        }
         else if(Result < 0)
        {
            ss.str(" ");
            ss.clear();
            ss<<"Result = "<<Result<<"(Move Left)";
            putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
        }
        
        namedWindow("orignal", WINDOW_KEEPRATIO);
        moveWindow("orignal", 0, 100);
        resizeWindow("orignal", 640, 480);
        imshow("orignal", frame);
        
        namedWindow("Perspective", WINDOW_KEEPRATIO);
        moveWindow("Perspective", 640, 100);
        resizeWindow("Perspective", 640, 480);
        imshow("Perspective", framePers);
        
        namedWindow("Final", WINDOW_KEEPRATIO);
        moveWindow("Final", 1280, 100);
        resizeWindow("Final", 640, 480);
        imshow("Final", frameFinal);
        
        namedWindow("Stop Sign", WINDOW_KEEPRATIO);
        moveWindow("Stop Sign", 1280, 580);
        resizeWindow("Stop Sign", 640, 480);
        imshow("Stop Sign", ROI_stop);
        
        namedWindow("Car", WINDOW_KEEPRATIO);
        moveWindow("Car", 640, 580);
        resizeWindow("Car", 640, 480);
        imshow("Car", ROI_car);
        
        namedWindow("Speed", WINDOW_KEEPRATIO);
        moveWindow("Speed", 0, 580);
        resizeWindow("Speed", 640, 480);
        imshow("Speed", ROI_Speed);
        
        
        waitKey(1);
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        
        float t = elapsed_seconds.count();
        int FPS = 1/t;
        cout<<"FPS = "<<FPS<<endl;
    
    }

    
    return 0;
     
}
