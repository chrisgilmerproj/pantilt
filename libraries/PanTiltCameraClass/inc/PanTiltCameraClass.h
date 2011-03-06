#ifndef PANTILTCAMERACLASS
#define PANTILTCAMERACLASS

//--- STL Includes
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//--- OpenCV Includes
#include "cvaux.h"
#include "highgui.h"
#include "cv.h"
#include "BlobResult.h"

//--- Phidget Includes
#include <Phidget21/phidget21.h>
#include "PhidgetServoClass.h"

//! A utility for running the Pan and Tilt Camera
/*!\class PanTiltCameraClass
 *
 * This class is designed to use a Phidget servo controller to control two
 * servos in a pan and tilt camera system.  It uses OpenCV libraries to take
 * images from a webcam, to turn those images into commands, and to command
 * the motors.
 *
 */
class PanTiltCameraClass
{
   private:
   
   public:
      
      //---- Class Variables ---------------------------------------------------
      
      //---- Instance Variables ------------------------------------------------
      //--- Pan and tile servo control
      PhidgetServoClass phidget;
      
      //--- Tracking
      int NUMBER_OF_CIRCLES;  /*!< \brief The number of circles for the program to track */
      int minBlobSize;        /*!< \brief The minimum blob size used for tracking */
      
      //--- Image size
      long imageSizeX;        /*!< \brief The size of the image in the X direction */
      long imageSizeY;        /*!< \brief The size of the image in the Y direction */
      long imageCenterX;      /*!< \brief The center of the image in the X direction */
      long imageCenterY;      /*!< \brief The center of the image in the Y direction */
      
      //--- Servo Center
      long panCenter;         /*!< \brief The center pan servo position */
      long tiltCenter;        /*!< \brief The center tilt servo position */
      
      //--- Camera
      bool TrackBlobs;      /*!< \brief Boolean telling the program to track blobs */
      bool ScanObject;        /*!< \brief Boolean telling the program to scan for an object */
      bool TrackObject;       /*!< \brief Boolean telling the program to track an object */
      bool TrackingStatus;    /*!< \brief Boolean telling the program the current tracking status */
      bool OneFrameProcess;   /*!< \brief Boolean telling program to pause and wait for user input */
      
      //--- Colors (Hue, Saturation, and Value)
      int HueLow;             /*!< \brief The hue low value */
      int HueHigh;            /*!< \brief The hue high value */
      int SatLow;             /*!< \brief The saturation low value */
      int SatHigh;            /*!< \brief The saturation high value */
      int ValLow;             /*!< \brief The value low value */
      int ValHigh;            /*!< \brief The value high value */
      
      //--- Constructors -------------------------------------------------------
      /*!\brief The Constructor
       */
      PanTiltCameraClass(void);
      
      //--- Methods ------------------------------------------------------------
      /*!\brief Reset the colors to the original settings
       */
      void resetColors(void);
      
      /*!\brief The main camera loop method
       */
      int CameraLoop( CvCapture* pCap, char* avi_name);
      
      /*!\brief Set up the phidget motor controls
       */
      void phidgetServoSetup(void);
      
      /*!\brief Setup for a window specifically meant for tuning color parameters
       */
      void tuningWindowSetup(void);
      
      /*!\brief Handles All Typed Commands
       */
      int ProcessUserCommand(int key);
      
      /*!\brief Flip the value of a boolean
       */
      inline void FlipBool(bool* a,char* varname);
      
      /*!\brief Display help information for key commands
       */
      void DisplayHelp();
      
      /*!\brief Finds a color according to global settings on HueLow, HueHigh, etc...
       */
      int createColorMask(IplImage* src,IplImage* dst);
      
      /*!\brief Function to return the Center of Gravity of an image
       */
      void cvCenterOfGravity(CvArr* img, CvPoint* cg);
      
      /*!\brief Function to return the Area Moment of an image
       */
      void cvGetAreaMomemt(CvArr* img, double* area);
      
      /*!\brief Places a targetting reticle with text on an image
       */
      void TargetReticle(CvArr* img,CvPoint* pt,const char* txt,int radius,CvScalar color);
      
      /*!\brief Formats the text to be placed on an image
       */
      void cvFormatText(IplImage* img,char* txt,CvPoint pt,CvScalar color);
      
      /*!\brief Control the camera using a PID control system
       */
      void pidServoControl(CvPoint* cg, CvPoint* preverr,CvPoint* erri);
      
      /*!\brief Enable blob tracking of an object
       */
      void blobTracking(IplImage* hsv_mask, IplImage* pFour, IplImage* pImg);
      
      //--- Friends ------------------------------------------------------------
};

#endif
