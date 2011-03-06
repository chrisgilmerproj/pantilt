//--- STL Includes
#include <stdio.h>
#include <stdlib.h>
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

using namespace std;

/*!\addtogroup pantilt
 *
 * This program is used to follow images with a Pan and Tilt Camera.
 *
 * \section description Description
 * This program uses the OpenCV library and a USB Phidget to control servos
 * and follow images described by the program.
 *
 * \section usage Usage
 *
 *
 * \section original Original Documentation
 * 
 * Notes on Visual C++ Project Properties changes:
 *    C++->Additional Include Directories = "c:\BlobsLib\lib"
 *    Linker->Additional Include Directories = "c:\BlobsLib\lib"
 *    Linker->Input->Additional Dependencies = "cxcore.lib cv.lib highgui.lib cvaux.lib cvblobslib.lib"
 *
 * Cvblobslib has to be installed.
 * Phidget header files need to be accessed (see includes)
 */

/* \{*/
//==============================================================================
/*!\brief Follow images with a Pan and Tilt Camera
 * \param argc the number of arguments given on the command line
 * \param argv a list of arguments given on the command line
 * 
 */
int main(int argc, char* argv[])
{
   /*
   PhidgetServoClass phidget;
   phidget.setServoPositionBounds(0,40,200);
   phidget.setServoPositionBounds(1,50,160);
   phidget.test_servos();
   */
   
   //--- Declare a capture and avi file
   CvCapture* pCap = NULL;
   char* avi_name = NULL;
   
   //--- Fill the capture with either a live feed or a recorded video
   if((argc == 1) || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0]))){
      pCap = cvCaptureFromCAM( argc == 2 ? argv[1][0] - '0' : 0 );
   } else if( argc == 2 ){
      pCap = cvCaptureFromAVI( argv[1] );
      avi_name = argv[1];
   }
   
   //--- If no image then quit
   if(!pCap){
      return -1;
   }
   
   //--- Run the pipeline
   PanTiltCameraClass camera;
   camera.CameraLoop( pCap, avi_name );

   //--- Release the image
   if(pCap){
      cvReleaseCapture(&pCap);
   }
   

   return 0;
} 
/* \}*/
