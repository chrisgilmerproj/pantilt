#ifndef PHIDGETSERVOCLASS
#define PHIDGETSERVOCLASS

//--- STL Includes
#include <stdio.h>
#include <map>

//--- Phidget Includes
#include <Phidget21/phidget21.h>

//! A utility for phidget servos
/*!\class PhidgetServoClass
 *
 * - Servo simple -
 * This simple example sets up a Servo objectm hooks the event handlers
 * and opens it for device connections.  Once a Servo is attached
 * with a motor in motor 0 it will simulate moving the motor from
 * position 15 to 231, displaying the event details to the console.
 *
 * Copyright 2008 Phidgets Inc.  All rights reserved.
 * This work is licensed under the Creative Commons Attribution 2.5 Canada
 * License. To view a copy of this license, visit
 * http://creativecommons.org/licenses/by/2.5/ca/
 *
 */
class PhidgetServoClass
{
   private:
   
   public:
      
      //---- Class Variables ---------------------------------------------------
      
      //---- Instance Variables ------------------------------------------------
      CPhidgetAdvancedServoHandle phid;   /*!< \brief The Advanced Phidget Servo Handle */
      
      double servoPositionUpperBound;     /*!< \brief The Servo Position Upper Bound */
      double servoPositionLowerBound;     /*!< \brief The Servo Position Lower Bound */
      
      /*!\brief A map containing the upper and lower position bounds for all motors */
      std::map<long, std::pair<double, double> > servoPositionBounds;
      
      /*!\brief The number of motors */
      int motors;
      
      //--- Constructors -------------------------------------------------------
      /*!\brief The Phidget Constructor
       */
      PhidgetServoClass(void);
      
      /*!\brief The Phidget Deconstructor
       */
      ~PhidgetServoClass(void);
      
      //--- Methods ------------------------------------------------------------
      
      /*!\brief Initialize the phidget servo
       *
       * This method is used to initialize the phidget servo
       */
      void InitializeServo(void);
      
      /*!\brief Test all the servos
       *
       * This method is used to test all the servos through their range
       * of motion.
       */
      void test_servos(void);
      
      /*!\brief Display the properties of the attached phidget to the screen.
       *
       * This method is used to display the properties of the phidget servo
       */
      void display_properties(void);
      
      /*!\brief Get the position of a phidget servo
       * \param servonum The phidget servo number
       *
       * This method is used to get the position of a servo
       */
      double getServoPosition(int servonum);
      
      /*!\brief Set the position boundaries of a phidget servo
       * \param servonum The phidget servo number
       * \param lowerBound The lower position bound
       * \param upperBound The upper position bound
       *
       * This method is used to set the position boundaries of a servo
       */
      void setServoPositionBounds(int servonum, double lowerBound, double upperBound);
      
      /*!\brief Get the lower bound of a phidget servo
       * \param servonum The phidget servo number
       *
       * This method is used to get the lower bound of a servo
       */
      double getServoPositionLowerBound(int servonum);
      
      /*!\brief Get the upper bound of a phidget servo
       * \param servonum The phidget servo number
       *
       * This method is used to get the upper bound of a servo
       */
      double getServoPositionUpperBound(int servonum);
      
      /*!\brief Send a command to the phidget servo
       * \param servonum The phidget servo number
       * \param position The desired phidget servo position
       *
       * This method is used to set and change the position of the phidget
       * servos.
       */
      void servo_command(int servonum,double position);
      
      /*!\brief Disengage all phidget servo motors
       *
       * This method is used to disengage all the phidget servo motors
       */
      void servo_disengage(void);
      
      //--- Friends ------------------------------------------------------------
};

#endif
