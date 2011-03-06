//--- STL Includes
#include <stdio.h>
#include <iostream>
#include <map>

//--- Phidget Includes
#include <Phidget21/phidget21.h>
#include "PhidgetServoClass.h"

using namespace std;


//==============================================================================
//================== Event Handler Functions ===================================
//==============================================================================

int AttachHandler(CPhidgetHandle ADVSERVO, void *userptr);

int DetachHandler(CPhidgetHandle ADVSERVO, void *userptr);

int ErrorHandler(CPhidgetHandle ADVSERVO, void *userptr,
		 int ErrorCode, const char *Description);

int PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO,
			  void *usrptr, int Index = 0, double Value = 0);



//==============================================================================
int AttachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
   int serialNo;
   const char *name;

   CPhidget_getDeviceName(ADVSERVO, &name);
   CPhidget_getSerialNumber(ADVSERVO, &serialNo);
   printf("%s %d attached!\n", name, serialNo);

   return 0;
}

//==============================================================================
int DetachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
   int serialNo;
   const char *name;

   CPhidget_getDeviceName(ADVSERVO, &name);
   CPhidget_getSerialNumber(ADVSERVO, &serialNo);
   printf("%s %d detached!\n", name, serialNo);

   return 0;
}

//==============================================================================
int ErrorHandler(CPhidgetHandle ADVSERVO, void *userptr,
				    int ErrorCode, const char *Description)
{
   printf("Error handled. %d - %s\n", ErrorCode, Description);
   return 0;
}

//==============================================================================
int PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO,
					     void *usrptr, int Index, double Value)
{
   //printf("Motor: %d > Current Position: %f\n", Index, Value);
   return 0;
}

//==============================================================================
//================== Phidget Servo Class =======================================
//==============================================================================

//==============================================================================
//==============================================================================
PhidgetServoClass::PhidgetServoClass(void)
{
   //--- Set the boundary limits on the servo
   this->servoPositionLowerBound = 40;
   this->servoPositionUpperBound = 200;
   
   //--- Initialize the servo
   this->InitializeServo();
   
   for(long num = 0; num < motors; ++num)
   {
      servoPositionBounds[num] = make_pair(this->servoPositionLowerBound,this->servoPositionUpperBound);
   }
}

//==============================================================================
PhidgetServoClass::~PhidgetServoClass(void)
{
   this->servo_disengage();
}

//==============================================================================
void PhidgetServoClass::InitializeServo(void)
{
   //--- Initialize an advanced phidget servo handler
   this->phid = 0;
   
   //create the advanced servo object
   CPhidgetAdvancedServo_create(&this->phid);

   //--- Set the handlers to be run when the device is plugged in or opened from software
   CPhidget_set_OnAttach_Handler((CPhidgetHandle)this->phid, AttachHandler, NULL);
   
   //--- Set the handlers to be run when the device is unplugged or closed from software
   CPhidget_set_OnDetach_Handler((CPhidgetHandle)this->phid, DetachHandler, NULL);
   
   //--- Set the handlers to be run when the device generates an error.
   CPhidget_set_OnError_Handler((CPhidgetHandle)this->phid, ErrorHandler, NULL);

   //--- Registers a callback that will run when the motor position is changed.
   //    Requires the handle for the Phidget, the function that will be called,
   //    and an arbitrary pointer that will be supplied to the callback function
   //    (may be NULL).
   CPhidgetAdvancedServo_set_OnPositionChange_Handler(this->phid, PositionChangeHandler, NULL);

   //--- open the device for connections
   CPhidget_open((CPhidgetHandle)this->phid, -1);

   //--- get the program to wait for an advanced servo device to be attached
   cout << "\nWaiting for Phidget to be attached...." << endl;
   int result;
   const char *err;
   if((result = CPhidget_waitForAttachment((CPhidgetHandle)this->phid, 10000)))
   {
      CPhidget_getErrorDescription(result, &err);
      cout << "\nError in PhidgetServoClass::InitializeServo" << endl;
      cout << "\tProblem waiting for attachment: " << err << endl;
      exit(1);
   } else {
      cout << "Phidget attached!" << endl;
   }
   
   //--- Get the number of motors
   int numMotors = 2;
   CPhidgetAdvancedServo_getMotorCount(this->phid, &numMotors);
   this->motors = numMotors;
   
   //--- SET LIMITS ON ACCELERATION AND VELOCITY
}

//==============================================================================
void PhidgetServoClass::test_servos(void)
{
   //--- Get the number of motors
   int numMotors = 2;
   CPhidgetAdvancedServo_getMotorCount(this->phid, &numMotors);
   numMotors = 2; //CAN DELETE THIS LINE LATER

   //--- Display the properties of the attached device
   this->display_properties();
   
   //--- read event data
   printf("\nReading.....\n");

   //--- This example assumes servo motor is attached to index 0

   //--- Set up some initial acceleration and velocity values
   double maxVel = 1;
   double minVel = 1;
   double curVel = 1;
   double maxAccel = 1;
   double minAccel = 1;
   double curAccel = 1;

   //--- Disengage all servos
   for(long mNum = 0; mNum < numMotors; ++mNum)
   {
      //--- Set the velocity in deg/s
      CPhidgetAdvancedServo_getVelocityMax(this->phid, mNum, &maxVel);
      CPhidgetAdvancedServo_getVelocityMin(this->phid, mNum, &minVel);
      CPhidgetAdvancedServo_setVelocityLimit(this->phid, mNum, (maxVel+minVel)/2);
      CPhidgetAdvancedServo_getVelocity(this->phid, mNum, &curVel);
      
      //--- Set the acceleration in deg/s/s
      CPhidgetAdvancedServo_getAccelerationMax(this->phid, mNum, &maxAccel);
      CPhidgetAdvancedServo_getAccelerationMin(this->phid, mNum, &minAccel);
      CPhidgetAdvancedServo_setAcceleration(this->phid, mNum, (maxAccel+minAccel)/2);
      CPhidgetAdvancedServo_getAcceleration(this->phid, mNum, &curAccel);
      
      //--- Print the rate information for the motor
      cout << "\nRate Information for motor: " << mNum << endl;
      cout << "\tVel:\t" << minVel << "\t" << curVel <<  "\t" << maxVel << endl;
      cout << "\tAccel:\t" << minAccel << "\t" << curAccel << "\t" << maxAccel << endl;
      
      //--- Set up some boundaries for the motors
      long servoLower = this->servoPositionBounds[mNum].first;
      long servoUpper = this->servoPositionBounds[mNum].second;
      
      //display current motor position
      double curr_pos = 0;
      if(CPhidgetAdvancedServo_getPosition(this->phid, 0, &curr_pos) == EPHIDGET_OK)
	 printf("\nMotor: %d > Current Position: %f\n", mNum, curr_pos);
      
      //keep displaying servo event data until user input is read
      printf("\nPress any key to continue\n");
      getchar();
      
      //change the motor position
      //valid range is -23 to 232, but for most motors ~40-200 to be safe
      //we'll set it to a few random positions to move it around
      
      //--- Engage the motor
      CPhidgetAdvancedServo_setEngaged(this->phid, mNum, true);
      
      for(long position = servoLower; position < servoUpper; position+=10)
      {
	 printf("Move to position %d and engage. Press any key to Continue\n", position);
	 getchar();
	 
	 CPhidgetAdvancedServo_setPosition(this->phid, mNum, position);
      }
      
      CPhidgetAdvancedServo_setPosition(this->phid, mNum, (servoLower+servoUpper)/2);
   }
   
   printf("\nPress any key to end\n");
   getchar();
}

//==============================================================================
void PhidgetServoClass::display_properties(void)
{
   int serialNo = 0;
   int version = 0;
   int numMotors = 0;
   const char* ptr;

   //--- Get the phidget device information
   CPhidget_getDeviceType((CPhidgetHandle)this->phid, &ptr);
   CPhidget_getSerialNumber((CPhidgetHandle)this->phid, &serialNo);
   CPhidget_getDeviceVersion((CPhidgetHandle)this->phid, &version);
   CPhidgetAdvancedServo_getMotorCount(this->phid, &numMotors);
 
   cout << "\nDevice Type: \t" << ptr << endl;
   cout << "Serial Number:\t" << serialNo << endl;
   cout << "Version:\t" << version << endl;
   cout << "Num Motors:\t" << numMotors << endl;
}

//==============================================================================
double PhidgetServoClass::getServoPosition(int servonum)
{
   double scanPosition = 0;
   
   CPhidgetAdvancedServo_getPosition(this->phid, servonum, &scanPosition);
   
   return scanPosition;
}

//==============================================================================
void PhidgetServoClass::setServoPositionBounds(int servonum, double lowerBound, double upperBound)
{
   this->servoPositionBounds[servonum] = make_pair(lowerBound, upperBound);
}

//==============================================================================
double PhidgetServoClass::getServoPositionLowerBound(int servonum)
{
   return this->servoPositionBounds[servonum].first;
}

//==============================================================================
double PhidgetServoClass::getServoPositionUpperBound(int servonum)
{
   return this->servoPositionBounds[servonum].second;
}

//==============================================================================
void PhidgetServoClass::servo_command(int servonum, double position)
{
   //--- Disengage the motor
   int engaged = false;
   
   //--- Set limits on the servo position
   if(position < this->servoPositionBounds[servonum].first)
      position = this->servoPositionBounds[servonum].first;
   if(position > this->servoPositionBounds[servonum].second)
      position = this->servoPositionBounds[servonum].second;
   
   //--- Set the position of the servo
   CPhidgetAdvancedServo_setPosition (this->phid, servonum, position);
   
   //--- Get the engaged state of the motor
   CPhidgetAdvancedServo_getEngaged (this->phid,servonum,&engaged);
   
   //--- If not engaged then engage the motor so it will move
   if(!engaged)
      CPhidgetAdvancedServo_setEngaged(this->phid, servonum, true);
}

//==============================================================================
void PhidgetServoClass::servo_disengage(void)
{
   //--- Get the number of motors
   int numMotors = 0;
   CPhidgetAdvancedServo_getMotorCount(this->phid, &numMotors);
   
   //--- Use to get the engaged state
   int engagedState = 0;
   
   //--- Servo disengage settings
   bool disengageServo = false;
   
   //--- Print stdout message
   cout << "\nDisengaging motors..." << endl;
   
   //--- Disengage all servos
   for(long mNum = 0; mNum < numMotors; ++mNum)
   {
      //--- Get the power state of the motor
      CPhidgetAdvancedServo_getEngaged (this->phid,mNum,&engagedState);
      
      //--- Disengage the motor if it's powered on
      if(engagedState){
	 CPhidgetAdvancedServo_setEngaged(this->phid,mNum,disengageServo);
      }
   }

   // Close and Delete the Phidget Handle
   CPhidget_close((CPhidgetHandle)this->phid);
   CPhidget_delete((CPhidgetHandle)this->phid);
}
