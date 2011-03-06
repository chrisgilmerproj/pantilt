//--- STL Includes
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

//--- OpenCV Includes
#include <OpenCV/cv.h>
#include <OpenCV/cvaux.h>
#include <OpenCV/highgui.h>
#include "BlobResult.h"

//--- Phidget Includes
#include <Phidget21/phidget21.h>
#include "PhidgetServoClass.h"

//--- Camera Includes
#include "PanTiltCameraClass.h"

//--- Define the servos used in the program
#define PanServoNumber 0
#define TiltServoNumber 1

using namespace std;

//==============================================================================
//================== Pan and Tilt Camera Class =================================
//==============================================================================

//==============================================================================
//==============================================================================
PanTiltCameraClass::PanTiltCameraClass(void)
{
   //--- Tracking
   this->NUMBER_OF_CIRCLES = 3;
   this->minBlobSize = 20;
   
   //--- Image size
   this->imageSizeX = 320;
   this->imageSizeY = 240;
   this->imageCenterX = this->imageSizeX/2;
   this->imageCenterY = this->imageSizeY/2;
   
   //--- Servo Center
   this->panCenter = 115;
   this->tiltCenter = 70;
   
   //--- Camera
   this->TrackBlobs = true;
   this->ScanObject = false;
   this->TrackObject = true;
   this->TrackingStatus = false;
   this->OneFrameProcess = false;
   
   //--- Set the colors
   this->resetColors();
}

//==============================================================================
void PanTiltCameraClass::resetColors(void)
{
   this->HueLow = 160;
   this->HueHigh = 7;
   this->SatLow = 50;
   this->SatHigh = 255;
   this->ValLow = 100;
   this->ValHigh = 255;
}

//==============================================================================
int PanTiltCameraClass::CameraLoop(CvCapture* pCap, char* avi_name = NULL )
{
   int key = 0;            // The value of keys pressed on the keyboard
   int FrameNum = 0;       // The number of frames that have been looped

   // PID Variables ------------------------------------------------------------
   CvPoint camerr = cvPoint(0,0);      // The last computed position error
   CvPoint camerrInt = cvPoint(0,0);   // The integral error
   
   //--- Scanning variables ----------------------------------------------------
   double scan = 90;        // Initialize scan position
   int scanStep = 2;        // The scan step size
   int scandir = scanStep;  // Scan direction step
   
   //--- Pan and Tilt Servo Settings -------------------------------------------
   this->phidgetServoSetup();

   // Image and Slider Window --------------------------------------------------
   this->tuningWindowSetup();
   
   const char* imageWindowName = "Four";  // Set the name of the image window
   cvNamedWindow(imageWindowName,1);      // Create the image window

   //--- Main processing loop.  Grabs a frame and processes it.
   //    cvWaitKey handles user commands by waiting a defined number of seconds
   //    or indefinitely when set to 0. Setting this->OneFrameProcess to 1 or '
   //    true' will cause the program to pause indefinitely.
   //    Pressing the escape key will cause this program to end, returning the
   //    value '27' to the cvWaitKey function
   for( FrameNum=0; pCap && (key = cvWaitKey(this->OneFrameProcess?0:1)) != 27; FrameNum++)
   {
      //--- Process the user command key if not the default value
      if(key!=-1){          
         ProcessUserCommand(key);			
      }
      
      //--- Receive the image from the USB Camera ------------------------------
      IplImage* pSource  = NULL;
      pSource = cvQueryFrame(pCap);
      if(pSource == NULL){
         break;
      }
      
      //--- Create a new image and zero it out
      IplImage* pImg = cvCreateImage(cvSize(this->imageCenterX*2,this->imageCenterX*2),8,3);
      cvZero(pImg);
      
      //--- Down sample the original large image to a smaller image
      cvResize(pSource,pImg,CV_INTER_LINEAR);
      
      //--- Get the size of the image
      CvSize sz = cvGetSize(pImg);
      
      //--- Create a 4x4 image -------------------------------------------------
      //    The window is created to hold:
      //       - Original image,
      //       - noisy red image
      //       - clean red image
      //       - blobs
      
      //--- Creates image (twice width, twice height plus extra space for text)
      IplImage* pFour = cvCreateImage(cvSize(pImg->width*2,pImg->height*2+80),8,3);
      
      //--- Sets Region of Interest
      cvSetImageROI(pFour,cvRect(0,40,pImg->width,pImg->height));
      
      //--- Copies original into upper lefthand spot
      cvCopy(pImg,pFour);
      
      //--- Find all the reds --------------------------------------------------
      //--- Creates an equal-sized binary image
      IplImage* hsv_mask = cvCreateImage( sz, 8, 1);
      
      //--- Creates a color mask according to the trackbar settings
      createColorMask(pImg,hsv_mask);
      
      //--- Sets Region of Interest to upper right box
      cvSetImageROI(pFour,cvRect(pImg->width,40,pImg->width,pImg->height));
      
      //--- Merges the binary image into the 4x4 image.
      //    You can't use copy because the source image is binary and the
      //    destination is RGB.  So you have to tell it to copy the binary
      //    image to all 3 channels. That's why it appears 3 times.
      cvMerge(hsv_mask,hsv_mask,hsv_mask,NULL,pFour);
      
      //--- Image Check --------------------------------------------------------
      //    At this point you should have:
      //       - Original image (pImg) -- 3 channels
      //       - Color mask (hsv_mask) -- 1-channel
      //       - 4x4 image (pFour) -- 3 channels, 2x width and height with
      //          extra 80px for text.  Top 2 sections filled in.
      
      //--- Edit the image for tracking ----------------------------------------
      //--- Smooth, threshold, and dilate the picture
      cvSmooth(hsv_mask, hsv_mask, CV_GAUSSIAN, 5, 5 );
      cvThreshold(hsv_mask,hsv_mask,230,255,CV_THRESH_TOZERO);
      cvDilate(hsv_mask,hsv_mask,0,4);		
      
      //--- Update the display -------------------------------------------------
      //--- Sets ROI to lower left
      cvSetImageROI(pFour,cvRect(0,pImg->height+80,pImg->width,pImg->height));
      
      //--- Copies the (now filtered) image to the 4x4.
      cvMerge(hsv_mask,hsv_mask,hsv_mask,NULL,pFour);		                               
      
      //--- Extract the center of gravity and area moment ----------------------
      CvPoint cg;
      cvCenterOfGravity(hsv_mask,&cg);
      
      double blobArea = 0;
      cvGetAreaMomemt(hsv_mask,&blobArea);
      
      //--- Blob Tracking ------------------------------------------------------
      if(this->TrackBlobs)
         this->blobTracking(hsv_mask, pFour, pImg);
      
      //--- Control the camera -------------------------------------------------
      //--- if area and Trackobjects
      if(blobArea > 200 && this->TrackObject == true){
         pidServoControl(&cg,&camerr,&camerrInt);
         
         this->TrackingStatus = true; 
      }
      //--- else make the camera scan around for new objects
      else if(this->ScanObject) 
      {						
         // Reset Integral error and scan for objects
         this->TrackingStatus = false;
         (camerrInt=cvPoint(0,0));
         scan = this->phidget.getServoPosition(PanServoNumber);
         
         //--- Increment the scan direction
         scan += scandir;
         
         //--- If limits reached, scan in the opposite direction
         if(scan >= this->phidget.getServoPositionUpperBound(PanServoNumber)){
            scandir = -scanStep;
         } else if (scan <= this->phidget.getServoPositionLowerBound(PanServoNumber)) {
            scandir = scanStep;
         }
            
         //--- Pan the camera while keeping tilt fixed
         this->phidget.servo_command(PanServoNumber,scan);
         this->phidget.servo_command(TiltServoNumber,this->tiltCenter);
      }
      //--- else fix the pan and tilt camera position to center
      else {
         //--- Fix the pan and tilt position
         this->phidget.servo_command(PanServoNumber,this->panCenter);
         this->phidget.servo_command(TiltServoNumber,this->tiltCenter);
      }
      
      //--- Place Tracking status on picture -----------------------------------
      char txt[40];
      if(this->TrackingStatus) {
         //--- Place the text
         sprintf(txt,"Tracking!");
         cvFormatText(pFour,txt,cvPoint(0,20),CV_RGB(255,0,0));
         
         //--- Place the CG
         TargetReticle(pFour,&cg,"CG",6,CV_RGB(255,0,0));
      } else {
         //--- Place the text
         sprintf(txt,"Scanning...");
         cvFormatText(pFour,txt,cvPoint(0,20),CV_RGB(255,255,255));
      }
      
      //--- Place the text on the image
      cvResetImageROI(pFour);
      cvShowImage(imageWindowName,pFour);
      
      //--- Release the images -------------------------------------------------
      cvReleaseImage(&hsv_mask);
      cvReleaseImage(&pFour);
      
   }//for

   return 0;
}

//==============================================================================
void PanTiltCameraClass::phidgetServoSetup(void)
{
   //--- Pan and Tilt Bounds and Center
   this->phidget.setServoPositionBounds(PanServoNumber,40,200);
   this->phidget.setServoPositionBounds(TiltServoNumber,50,160);
   
   //--- Set up some initial acceleration and velocity values
   double maxVel = 1;
   double minVel = 1;
   double maxAccel = 1;
   double minAccel = 1;

   //--- Set the velocity in deg/s
   CPhidgetAdvancedServo_getVelocityMax(this->phidget.phid, PanServoNumber, &maxVel);
   CPhidgetAdvancedServo_getVelocityMin(this->phidget.phid, PanServoNumber, &minVel);
   CPhidgetAdvancedServo_setVelocityLimit(this->phidget.phid, PanServoNumber, (maxVel+minVel)/2);
   CPhidgetAdvancedServo_setVelocityLimit(this->phidget.phid, TiltServoNumber, (maxVel+minVel)/2);
   
   //--- Set the acceleration in deg/s/s
   CPhidgetAdvancedServo_getAccelerationMax(this->phidget.phid, PanServoNumber, &maxAccel);
   CPhidgetAdvancedServo_getAccelerationMin(this->phidget.phid, PanServoNumber, &minAccel);
   CPhidgetAdvancedServo_setAcceleration(this->phidget.phid, PanServoNumber, (maxAccel+minAccel)/2);
   CPhidgetAdvancedServo_setAcceleration(this->phidget.phid, TiltServoNumber, (maxAccel+minAccel)/2);
}

//==============================================================================
void PanTiltCameraClass::tuningWindowSetup(void)
{
   const char* sliderWindowName = "Tuning";
   cvNamedWindow(sliderWindowName,1);

   // Trackbars for tuning the color -------------------------------------------
   cvCreateTrackbar("HueHigh",sliderWindowName,&this->HueHigh,180,NULL);
   cvCreateTrackbar("HueLow",sliderWindowName,&this->HueLow,180,NULL);
   cvCreateTrackbar("SatHigh",sliderWindowName,&this->SatHigh,255,NULL);
   cvCreateTrackbar("SatLow",sliderWindowName,&this->SatLow,255,NULL);
   cvCreateTrackbar("ValHigh",sliderWindowName,&this->ValHigh,255,NULL);
   cvCreateTrackbar("ValLow",sliderWindowName,&this->ValLow,255,NULL);
   cvCreateTrackbar("Min Blob Size",sliderWindowName,&this->minBlobSize,300,NULL);
}

//==============================================================================
int PanTiltCameraClass::ProcessUserCommand(int key)
{
   switch(key)
   {
      case 'b':
         this->FlipBool(&this->TrackBlobs,"Blob Tracking");
         break;
      case 's':
         this->FlipBool(&this->ScanObject,"Scanning");
         break;
      case 'p':
         this->FlipBool(&this->OneFrameProcess,"Pause");
         break;
      case 't':
         this->FlipBool(&this->TrackObject,"Object Tracking");
         break;
      case 'r':
         this->resetColors();
         break;
      case 'h':
         this->DisplayHelp();
         break;
      default:
         printf("Unrecognized command.\n");
         break;
   }

   return 0;
}

//==============================================================================
inline void PanTiltCameraClass::FlipBool(bool* a,
                                         char* varname)
{
   *a=!(*a);
   
   if(*a)
      printf("%s ON\n",varname);
   else
      printf("%s OFF\n",varname);
}

//==============================================================================
void PanTiltCameraClass::DisplayHelp()
{
   printf("\n\n");
   printf("====================== List of Commands ============================");
   printf("\n\n");
   printf(" b -- Blob Tracking ON/OFF\n");
   printf(" s -- Scanning ON/OFF\n");
   printf(" p -- Pause / Resume\n");
   printf(" t -- Track Objects ON/OFF\n");
   printf(" r -- Reset the tracking colors");
   printf("\n");
   printf(" h -- Display this help screen\n");
   printf(" \n");
}

//==============================================================================
int PanTiltCameraClass::createColorMask(IplImage* src,
                                      IplImage* dst)
{
   IplImage* hsv_image = cvCreateImage( cvGetSize(src), 8, 3);
   IplImage* hsv_mask2 = cvCreateImage( cvGetSize(src), 8, 1);	    

   CvScalar  hsv_min = cvScalar(this->HueLow, this->SatLow, this->ValLow, 0);
   CvScalar  hsv_max = cvScalar(this->HueHigh, this->SatHigh, this->ValHigh, 0);
   
   CvScalar  min = cvScalar(0, this->SatLow, this->ValLow, 0);
   CvScalar  max = cvScalar(180, this->SatHigh, this->ValHigh, 0);

   cvCvtColor(src, hsv_image, CV_BGR2HSV);	    
   
   if(this->HueLow > this->HueHigh)
   {
      //In order to support inclusive ranges (like 160 to 20, including the 0), you have to
      //split into two parts.
      cvInRangeS (hsv_image, hsv_min, max, dst);       // Does the 160 to 180 part
      cvInRangeS (hsv_image, min, hsv_max, hsv_mask2); // does the 0 to 20 part
      
      // Recombine
      cvAdd(dst,hsv_mask2,dst);
   } else {
      cvInRangeS (hsv_image, hsv_min, hsv_max, dst);	
   }

   // Clean up
   cvReleaseImage(&hsv_image);
   cvReleaseImage(&hsv_mask2);

   return 0;
}

//==============================================================================
void PanTiltCameraClass::cvCenterOfGravity(CvArr* img,
                                           CvPoint* cg)
{
   double	area;
   CvMoments MyMoment;	
   CvMoments* pMom = &MyMoment;

   //--- Fill the moments structure
   cvMoments(img,pMom,1);
   
   //--- Get the area
   area = cvGetSpatialMoment(pMom,0,0);
   
   //--- Get the center of gravity
   cg->x = cvRound(cvGetSpatialMoment(pMom,1,0)/area);
   cg->y = cvRound(cvGetSpatialMoment(pMom,0,1)/area);
}

//==============================================================================
void PanTiltCameraClass::cvGetAreaMomemt(CvArr* img,
                                         double* area)
{
   CvMoments MyMoment;	
   CvMoments* pMom = &MyMoment;

   cvMoments(img,pMom,1);	          // Fill the moments structure
   *area = cvGetSpatialMoment(pMom,0,0);  // get the area
}

//==============================================================================
void PanTiltCameraClass::TargetReticle(CvArr* img,
                                       CvPoint* pt,
                                       const char* txt,
                                       int radius,
                                       CvScalar color)
{
   //--- Create the reticle dimensions
   int length = cvRound(radius * 1.5);
   CvPoint beginPoint = *pt;
   CvPoint endPoint = *pt;
   CvPoint text = *pt;
   
   //--- Create the circle
   cvCircle(img,*pt,radius,color);
   
   //--- Create the horizontal line
   beginPoint.x -= length;
   endPoint.x += length;
   cvLine(img,beginPoint,endPoint,color);
   
   //--- Recenter the x coordinate points
   beginPoint = *pt;
   endPoint = *pt;
   
   //--- Create the vertical line
   beginPoint.y -= length;
   endPoint.y += length;
   cvLine(img,beginPoint,endPoint,color);

   //--- Place Text to the side
   CvFont font;
   text.x += radius*2;
   text.y += radius*2;
   cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,1,1);
   cvPutText(img,txt,text,&font,color);
}

//==============================================================================
void PanTiltCameraClass::cvFormatText(IplImage* img,
                                    char* txt,
                                    CvPoint pt,
                                    CvScalar color)
{
   CvFont Font;
   CvFont* pFont = &Font;
   cvInitFont(pFont,CV_FONT_HERSHEY_PLAIN,1,1);
   cvPutText(img,txt,pt,pFont,color);	
}

//==============================================================================
void PanTiltCameraClass::pidServoControl(CvPoint* cg, CvPoint* preverr, CvPoint* erri)
{
   //--- Set up objects to calculate the error and derivative error
   CvPoint err = cvPoint(0,0);   // The position error
   CvPoint errd = cvPoint(0,0);  // The derivative error
   
   //--- Compute position error
   //    This is the distance from the cg to the center of the image
   err.x = (cg->x - this->imageCenterX);
   err.y = (cg->y - this->imageCenterY);

   //--- Compute derivative error
   //    This is the distance from the previous error to the current error
   errd.x = preverr->x - err.x;
   errd.y = preverr->y - err.y;

   //--- Compute integral error
   //    This is the integral error plus the new error (simply a sum)
   erri->x += err.x;
   erri->y += err.y;

   //--- Update Previous Error with the current error
   *preverr = err;

   //--- Get the current position
   double pan = this->phidget.getServoPosition(PanServoNumber);
   double tilt = this->phidget.getServoPosition(TiltServoNumber);
   
   //--- Reset the pan and tilt commands
   double pancmd = 0;
   double tiltcmd = 0;
   
   //--- Set the PID coefficients
   //    These should change if you edit the servo setup or the image size
   double kp = 0.08;    // Position coefficient
   double kd = -0.05;   // Derivative coefficient
   double ki = 0.00;    // Integral coefficient
   
   //--- Use PID control to correct the position
   //    The tilt servo is actually in the opposite direction of the camera coords
   pancmd = pan + (((double)err.x)*kp + ((double)errd.x)*kd + ((double)erri->x)*ki);
   tiltcmd = tilt - (((double)err.y)*kp + ((double)errd.y)*kd + ((double)erri->y)*ki);
   
   //--- Command the servo to the new position
   this->phidget.servo_command(PanServoNumber,pancmd);
   this->phidget.servo_command(TiltServoNumber,tiltcmd);	
}

//==============================================================================
void PanTiltCameraClass::blobTracking(IplImage* hsv_mask,
                                      IplImage* pFour,
                                      IplImage* pImg)
{
   //--- Get blobs and filter them using the blob area
   CBlobResult blobs;
   CBlob *currentBlob;
   
   //--- Create a thresholded image and display image --------------------
   //--- Creates binary image
   IplImage* originalThr = cvCreateImage(cvGetSize(hsv_mask), IPL_DEPTH_8U,1);
   
   //--- Create 3-channel image
   IplImage* display = cvCreateImage(cvGetSize(hsv_mask),IPL_DEPTH_8U,3);
   
   //--- Copies the original
   cvMerge( hsv_mask, hsv_mask, hsv_mask, NULL, display );
   
   //--- Makes a copy for processing
   cvCopy(hsv_mask,originalThr);
   
   //--- Find blobs in image ---------------------------------------------
   int blobThreshold = 0;
   bool blobFindMoments = true;
   blobs = CBlobResult( originalThr, originalThr, blobThreshold, blobFindMoments);
   
   //--- filters blobs according to size and radius constraints
   blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, this->minBlobSize );
   
   //--- display filtered blobs ------------------------------------------
   
   //--- copies the original in (for background)
   cvMerge( originalThr, originalThr, originalThr, NULL, display );
   
   CvPoint pts[this->NUMBER_OF_CIRCLES];
   
   //--- This sequence marks all the blobs
   for (int i = 0; i < blobs.GetNumBlobs(); i++ )
   {
      currentBlob = blobs.GetBlob(i);
      currentBlob->FillBlob( display, CV_RGB(0,0,255));				
      
      //--- Get blobs centerpoint
      CvPoint bcg;
      bcg.x = (int)(currentBlob->MinX()+((currentBlob->MaxX()-currentBlob->MinX())/2));
      bcg.y = (int)(currentBlob->MinY()+((currentBlob->MaxY()-currentBlob->MinY())/2));
      
      //--- Print the CG on the picture
      char blobtext[40];
      for(int k=0;k<this->NUMBER_OF_CIRCLES;k++)
      {
         sprintf(blobtext,"%d",k+1);
         TargetReticle(display,&pts[k],blobtext,6,CV_RGB(255,0,0));
      }//for
   }//for each blob
   
   //--- Set the ROI in the pFour image
   cvSetImageROI(pFour,cvRect(pImg->width,pImg->height+80,pImg->width,pImg->height));
   cvCopy(display,pFour);
   
   //Reset region of interest
   cvResetImageROI(display);						
   
   //Clean up
   cvReleaseImage( &originalThr );
   cvReleaseImage( &display);
}
