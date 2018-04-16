#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

Point GetWrappedPoint(Mat M, const Point& p);
void calcHugh(Mat gIPM, Mat mFram, Mat Ipm, Mat IpmInv);
vector<Mat>  applIPM (Mat raw, Mat frame);
VideoWriter video;




#define VIDEO_INPUT "/home/qwe/aprojs/kandidato/recorded10.h264"
//#define VIDEO_INPUT "/home/qwe/aprojs/kandidato/recorded7.h264"
//#define VIDEO_INPUT "/home/qwe/recordingvarv0411/output.avi"
//#define VIDEO_INPUT  "/home/qwe/lastrecording_separe/output.avi"
//#define STEERING_CMDS "/home/qwe/lastrecording_separe/ackermann_values.txt"
#define M_PI 3.14159265358979323846 
#define X1 0
#define X2 2
#define Y1 1
#define Y2 3
#define THICKNESS 10
#define BLUE Scalar(255,0,0)
#define BLACK Scalar(255,255,255)
   


int main(){
  VideoCapture cap;
  //VideoWriter video("outcpp.avi",CV_FOURCC('M','J','P','G'),40, Size(720,480));
  video.open("outcpp2oooo.avi",CV_FOURCC('M','J','P','G'),20, Size(720,480));
  Mat mFrame, mGray, mCanny, imageROI, mGray1, mGray2, carTrack , mask
    , IPM_ROI, IPM, IPM_Gray, IPM1, IPM2, IPM_Gray2, mFrame2, IPM_Matrix_inverse;

  string ackerline;
  ifstream myfile;
  //  myfile.open(STEERING_CMDS);

  // if(!myfile.is_open()) {
  //   perror("Error open");
  //   exit(EXIT_FAILURE);
  //  }

  cap.open(VIDEO_INPUT);
  while (cap.read(mFrame)){
    //    if (getline(myfile, ackerline))
    //      cout << ackerline << endl;
    //Determine region of interest
    mFrame2  = mFrame.clone();
    imageROI = mFrame(Rect(0,mFrame.rows/3, mFrame.cols, mFrame.rows - mFrame.rows/3));
    IPM_ROI  = imageROI(Rect(0,0,imageROI.cols,(imageROI.rows-0)));
    IPM_ROI  = IPM_ROI.clone();

    //Gray-scale image
    cvtColor(imageROI, mGray, COLOR_BGR2GRAY);
    cvtColor(mFrame, mGray2, COLOR_BGR2GRAY);

    //Apply histogram equalization
    equalizeHist(mGray, mGray);

    //Apply inverse perspective mapping
    IPM = applIPM(IPM_ROI, mFrame)[0];
    IPM_Matrix_inverse = applIPM(IPM_ROI, mFrame)[1];
    //imshow("IPM", IPM);
    IPM.copyTo(IPM1);
    IPM.copyTo(IPM2);


    cvtColor(IPM, IPM_Gray, COLOR_BGR2GRAY);
    //Blur image and apply canny edge detection 
    GaussianBlur(IPM_Gray, IPM_Gray, Size(7,7), 1.5, 1.5);
    Canny(IPM_Gray, IPM_Gray, 5, 40, 3);

    //Pause button p
    if(cv::waitKey(1) == 'p')
      while(cv::waitKey(0) != 'p');
    GaussianBlur( IPM_Gray,IPM_Gray, Size( 5, 5 ), 1.5, 1.5 );
    //imshow("IPM BINRARY AFTER FILTERING", IPM_Gray); 
    calcHugh(IPM_Gray, mFrame, IPM2, IPM_Matrix_inverse);
    if(cv::waitKey(1) == 'p')
      while(cv::waitKey(0) != 'p');
    
    //waitKey(0);
  }
	return 0;
}

//Apply inverse perspective mapping
//from region of interest to mframe.
vector<Mat> applIPM(Mat IPM_ROI, Mat mFrame){

  Point2f inputQuad[4];
  Point2f outputQuad[4];
  
  Mat IPM;
  Mat IPM_Matrix(2, 4, CV_32FC1);
  Mat IPM_Matrix_inverse;
  // Set the IPM matrix the same type and size as input
  IPM_Matrix = Mat::zeros(mFrame.rows, mFrame.cols, mFrame.type());

  //Value that determines how much the image will be bird-viewified
  //around 280-300 works for the current angle of the camera.  
  int widthVal = 295;
  //int widthVal = 275;
  inputQuad[0] = Point2f(0,0);
  inputQuad[1] = Point2f(IPM_ROI.cols,0);
  inputQuad[2] = Point2f(IPM_ROI.cols,IPM_ROI.rows);
  inputQuad[3] = Point2f(0,IPM_ROI.rows);
  
  outputQuad[0] = Point2f(0,0);
  outputQuad[1] = Point2f(mFrame.cols,0);
  outputQuad[2] = Point2f(mFrame.cols-widthVal,mFrame.rows);
  outputQuad[3] = Point2f(widthVal,mFrame.rows);
  
  Mat result;
  Mat transformedImage;

  //calculate the perspective transform given the input and output points
  IPM_Matrix = getPerspectiveTransform(inputQuad, outputQuad);

  //calculate inverse matrix, later used to map points from
  //the bird-view-plane to the standard-view-fplane
  invert(IPM_Matrix, IPM_Matrix_inverse);
  //apply the inverse mapping transform 
  warpPerspective(IPM_ROI, IPM, IPM_Matrix, mFrame.size());

  return {IPM, IPM_Matrix_inverse};
}

void calcHugh(Mat IPM_Gray, Mat mFrame, Mat IPMclean, Mat IPM_Matrix_inverse){
  
  vector<Vec4i> lines;
  //HoughLinesP(InputArray image, OutputArray lines, double rho, double theta,
  //            int threshold, double minLineLength=0, double maxLineGap=0)

  HoughLinesP(IPM_Gray,lines,1, CV_PI/180, 150 ,0,100 );
  vector<Vec4i> obstlines;
  vector<Vec4i> detectedObjs;
  
  Mat IPM1, IPM2, IPM3, roadFrame_Gray, roadFrame_Clean;
  IPMclean.copyTo(IPM1);
  IPMclean.copyTo(IPM2);
  IPMclean.copyTo(IPM3);
  Point e,f,g,h,q,w,v,b,A,B,C,D;
  Point E,F,G,H;
  float obst_a, obst_angle;
  bool isNewObj = true;
  
  float angle;float a;
  float currMLL = 0.f;
  float currMRL = 0.f;
  float currOLL, currORL;
  float pLength;
  Point midp1, midp2;
  midp1 = Point(mFrame.cols/2, mFrame.rows);
  midp2 = Point(mFrame.cols/2, 0);
  cout << "-------------------" << endl;
  for( size_t i = 0; i < lines.size(); i++ ){
    float p=0,t=0;
    Vec4i l = lines[i];
    if((l[0]-l[2])==0){
      a=-CV_PI/2;
      angle=-90;
    }else{
      t=(l[1]-l[3])/(l[0]-l[2]);
      a=atan(t);
      angle=a*180/CV_PI;
    }
    
    //right and left lane identification. 
    if(angle<(-60) || angle>(60)){
      
      p= l[0]; //or p = l[2] or p=(l[0]+l[2])/2; //average offset in x-direction
      pLength = (l[1] -l[3]); //Length of the current line
      //Draw unfiltered detected lines on IPM1 
      line(IPM1,Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 3,CV_AA);
      
      //x-position of detected line is less than the vehicle's => left lane 
      if ( p < midp1.x){ 
      	if(abs(pLength)>abs(currMLL)){
	  currMLL=pLength;      	  
	  //e=Point(l[0] ,l[1]);
	  if (pLength < 0){
	    e=Point(l[2] ,mFrame.rows);
	    f=Point(l[0],l[1]); 
	  }
	  else{
	    e=Point(l[0] + (int) ( (cos (CV_PI + a))) ,mFrame.rows);
      	    f=Point(l[2],l[3]);
	  }
	  E=GetWrappedPoint(IPM_Matrix_inverse, e);
	  E.y += 160;  //correction for selected ROI (160 = 480/3)	  
	  F=GetWrappedPoint(IPM_Matrix_inverse, f);
	  F.y += 160;
	}
      }
      //else => right lane
      else { 
	if(abs(pLength) >abs(currMRL)){
	  currMRL=pLength;
	  if (pLength < 0){
	    g=Point(l[2], mFrame.rows);
	    h=Point(l[0], l[1]);
	  }
	  else{
	    g=Point(l[0] + + (int) ( (cos (CV_PI + a))), mFrame.rows);
	    h=Point(l[2],l[3]);
	  }
	  G=GetWrappedPoint(IPM_Matrix_inverse, g);
	  G.y += 160;
	  H=GetWrappedPoint(IPM_Matrix_inverse, h);
	  H.y +=160;
	}
      }

    }  
  }
  //Road object detection through detecting lines between the two identified lanes. 
  //Create rectangular frame of the identified road segments through
  //using x-axis-values of the identified lanes. 
  if (g.x - f.x > 0){
    roadFrame_Gray  = IPM_Gray(Rect(f.x, 0 , g.x-f.x,480));
    roadFrame_Clean =     IPM3(Rect(f.x, 0 , g.x-f.x,480));
    }
  //Make sure that the rectangle never has any negative lengths
  else{
    roadFrame_Gray  = IPM_Gray(Rect(f.x, 0 , 0,0));
    roadFrame_Clean =     IPM3(Rect(f.x, 0 , 0,0));
  }
    //imshow("object detection frame", roadFrame);
    //    waitKey(10); 
  
  HoughLinesP(roadFrame_Gray,obstlines,1,CV_PI/180,50,5,0);
  for(size_t i = 0; i < obstlines.size(); i++ ){
    float p=0,t=0;
    
    Vec4i obstl = obstlines[i];
    if((obstl[0]-obstl[2])==0){
      obst_a=-CV_PI/2;
      obst_angle=-90;
    }else{
      t=(obstl[1]-obstl[3])/(obstl[0]-obstl[2]);
      obst_a=atan(t);
      obst_angle=obst_a*180/CV_PI;
    }

    //Only detect objects horizontal enough
    if (obst_angle<20 && obst_angle > (-20)){
      if (detectedObjs.size() == 0){
  	detectedObjs.push_back(obstl);
  	//line(roadFrame_Clean,Point(obstl[0], obstl[1]), Point(obstl[2], obstl[3])
  	//     , Scalar(0,40,255), 3,CV_AA);
      }
      else{
	//Loop through the list of detected objects so far and compare it with the
	//current detected line in order to make sure it's a new object and not an
	//object already identified. 
  	for(size_t j= 0; j < detectedObjs.size(); j++){
  	  Vec4i obj = detectedObjs[j];
  	  if (abs (obj[1] - obstl[1]) < 15){
  	    isNewObj = false;
  	  }
  	}
  	if (isNewObj){
  	  detectedObjs.push_back(obstl);
  	}	
      }
      p = obstl[2];
      pLength = (obstl[1] - obstl[3]);
      //cout << "Object detected" << endl;
    }
  }
  
  //Mark lanes on bird-view frame
  line(IPM2, e, f, Scalar(0,255,255), 3,CV_AA);
  line(IPM2, g, h, Scalar(0,255,255), 3,CV_AA);
  line(IPM2,midp1, Point(midp1.x, 480-120) , Scalar(0,0,0), 3,CV_AA);
  line(IPM2,Point((e.x + g.x)/2, 480) , Point( ((3*e.x+f.x)/4 + (3*g.x+h.x)/4) /2, 480 - 120)
       , BLUE , 3,CV_AA);
   //cout << midp1.x - e.x << " " <<  g.x - midp1.x << endl;
  //Display the difference between the vehicle's centerpoint and the centre of the lanes
  putText(IPM2,to_string((e.x+g.x)/2 - midp1.x) , Point(midp1.x, midp1.y-10), 
	  FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);


  //Mark lanes on standard-view frame.
  line(mFrame, E, F, Scalar(0,255,255), 7,CV_AA);
  line(mFrame, G, H, Scalar(0,255,255), 7,CV_AA);
  line(mFrame,Point((E.x + G.x)/2, 480) , Point( ((3*E.x+F.x)/4 + (3*G.x+H.x)/4) /2, 480 - 120)
       , BLUE , 3,CV_AA);
  line(mFrame,midp1, Point(midp1.x, 480-120) , Scalar(0,0,0), 3,CV_AA);
  putText(mFrame,to_string((E.x+G.x)/2 - midp1.x) , Point(midp1.x, midp1.y-10), 
	  FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);

  //Mark detected objects
  for(size_t i = 0; i < detectedObjs.size(); i++ ){
    Vec4i obstl = detectedObjs[i];
    Point obstlp1 = Point(obstl[0] + f.x, obstl[1]);
    Point obstlp2 = Point(obstl[2] + f.x, obstl[3]);

    Point stdObstlp1 = GetWrappedPoint(IPM_Matrix_inverse, obstlp1);
    stdObstlp1.y += 160;
    Point stdObstlp2 = GetWrappedPoint(IPM_Matrix_inverse, obstlp2);
    stdObstlp2.y += 160;
    
    line(IPM2, obstlp1, obstlp2, Scalar(0,40,255), 3,CV_AA);
    line(mFrame, stdObstlp1, stdObstlp2, Scalar(0,40,255), 3,CV_AA);
  }  
  
//  imshow("object marked", roadFrame_Clean);
  imshow("IPM before filtering", IPM1);
  imshow("IPM after  filtering", IPM2);
  
  video.write(mFrame);
  imshow("inputWithLanes", mFrame);
  waitKey();
}



//Apply inverse IPM and collect the corresponding
//points in standard-view-cartesian-plane
Point GetWrappedPoint(Mat M, const Point& p){
    cv::Mat_<double> src(3,1);
    
    src(0,0)=p.x;
    src(1,0)=p.y;
    src(2,0)=1.0;
    
    cv::Mat_<double> dst = M*src;
    dst(0,0) /= dst(2,0);
    dst(1,0) /= dst(2,0);
    return Point(dst(0,0),dst(1,0));
}
   
