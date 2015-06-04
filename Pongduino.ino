/**
* @file Pongduino.ino
*
* @section desc File description
*
* Track a ball with an Arduino board and a CMUcam4.
* Then, shoot the ball when it's close enough.
*
* @section copyright Copyright
*
* (c) IRCCyN 2005-2015
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; version 2
* of the License.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*
* @section infos File informations
*
* @date 2015/05/26
* @author Benjamin Sientzoff
* @version 0.1.3
*
*/

// header file for distance sensor DIST-Nx
#include "distnx.h"
// motor PID and trajectory computation
#include "motor_control.h"
//  Arduino I2C Master library
#include <I2C.h>
//  Matrics robotics controller library
#include <Matbotics.h>
// CMUcam4 library
#include <CMUcam4.h>


// macros for CMUcam4
// setting
// switch off auto gain control
#define CAM_GAIN_CON false
// switch off white balance
#define CAM_WHITE_BAL false
//  blink led at 2Hz when set it up
#define CAM_LED_SETUP_BLINK 5
// blink led at 1Hz when errors occur
#define CAM_LED_ERROR_BLINK 1
// blink led at 5Hz when red is detected
#define CAM_LED_DETECTED_BLINK 2
// among of time to init properly camera
#define CAM_SETTING_TIME 5000 // 5 seconds
// color tolerance
#define CAM_NOISE_FILTER 2
// color filter setting (RGB)
// RED
#define RED_MIN 120
#define RED_MAX 255
// GREEN
#define GREEN_MIN 0
#define GREEN_MAX 90
// BLUE
#define BLUE_MIN 0
#define BLUE_MAX 90

// field boundaries
#define DISTANCE_MIN 350
#define DISTANCE_MAX 1090

// presence sensor ports
#define PSENS_0 A0
#define PSENS_1 A1
#define PSENS_2 A2
#define PSENS_3 A3
#define PSENS_4 A4

// max angle should be 160 but shooting is less powerfull
// servo angle min and max
#define SERVO_ANGLE_MAX 200
#define SERVO_ANGLE_MIN 30
// servo speed
#define SERVO_SPEED 0
// servo tme need to shoot
#define SERVO_SHOOT_TIME 200

// pins for presence sensors
uint8_t sensor_pins[5] =
{
  PSENS_0,
  PSENS_1,
  PSENS_2,
  PSENS_3,
  PSENS_4
};

// robot states
enum RBT_STATE
{
  RBT_STATE_START = 0,
  RBT_STATE_SHOOTING,
  RBT_STATE_HALT,
  RBT_STATE_TRACK,
  RBT_STATE_MOVING
};

// target to reach when ball is far away
#define MOVE_OFFSET_LONGD 100

// store previous ball position
int old_mx = 0;
int old_my = 0;

// CMUcam use serial port 3
CMUcam4 cam( CMUCOM4_SERIAL3 );
// data send by camera in stream mode
CMUcam4_tracking_data_t CMUcam_data;

// Matrix Robotics controller
MTController ctlr;

// motor stuff
int motor_speed = 0;

// last known distance to the field border
int distance;
// target to reach
int goal;
// error i.e. different between goal and real distance
int dist_error;
RBT_STATE robot_state = RBT_STATE_START;

// =============================
//            SETUP
// =============================
void setup()
{
  // set pins for presence sensors
  for ( int i = 0; i < 5; i++) pinMode( sensor_pins[i], INPUT );
  // get robot position
  get_distance_LSB( &distance );
  // go to enter
  goal = 920;
  // activate CMUcam4 interface
  cam.begin();
  // Wait for auto gain and auto white balance to run
  cam.LEDOn( CAM_LED_SETUP_BLINK );
  delay( CAM_SETTING_TIME );
  // switch off analog output
  cam.monitorOff();
  cam.autoGainControl( CAM_GAIN_CON );
  cam.autoWhiteBalance( CAM_WHITE_BAL );
  cam.noiseFilter( CAM_NOISE_FILTER );
  cam.cameraContrast( 15 );
  // turn off auxiliary LED off
  cam.LEDOff();
  // don't automatically switch off motors.
  ctlr.timeout( 0 );
  // enable servos
  ctlr.enableServos();
  // change servo angle instantly
  ctlr.servoTwoSpeed( SERVO_SPEED );
  // arm
  ctlr.servoTwoAngle( SERVO_ANGLE_MIN );
  robot_state = RBT_STATE_TRACK;
  // starting tracking using RGB parameters
  cam.trackColor( RED_MIN, RED_MAX, GREEN_MIN, GREEN_MAX, BLUE_MIN, BLUE_MAX );
}

// =============================
//             LOOP
// =============================
void loop()
{
  // shoot the ball if it's here
  for (int i = 0; i < 5; i++)
  {
    if ( 0 == digitalRead( sensor_pins[i] ) )
    {
      // set the angle to the first servo
      ctlr.servoTwoAngle( SERVO_ANGLE_MAX );
      delay( SERVO_SHOOT_TIME ); // quiet bad!
      ctlr.servoTwoAngle( SERVO_ANGLE_MIN );
      delay( 100 );
    }
  }

  // update robot current distance
  get_distance_LSB( &distance );

  // =============================
  // get ball position
  //if( 0 == cam.trackColor(RED_MIN, RED_MAX, GREEN_MIN, GREEN_MAX, BLUE_MIN, BLUE_MAX)){
  if ( 0 == cam.trackColor() )
  {
    // if cam in stream mode get T type packet
    if ( 0 == cam.getTypeTDataPacket( &CMUcam_data ) )
    {
      //cam.idleCamera();
      if (old_mx != CMUcam_data.mx || old_my != CMUcam_data.mx)
      {
        cam.LEDOn( CAM_LED_DETECTED_BLINK );
        old_mx = CMUcam_data.mx;
        old_my = CMUcam_data.my;
        //cam.dumpBitmap();
        if ( RBT_STATE_TRACK == robot_state )
        {
          // if the ball is at the right
          if ( 80 > old_mx )
          {
            // go to the right
            goal = distance + MOVE_OFFSET_LONGD;
          }
          else 
          {
            // the the ball is at the left
            if ( 100 < old_mx )
            {
              // go to the left
              goal = distance - MOVE_OFFSET_LONGD;
            }
          }
          // Serial.println( goal );
        }
      }
      else
      {
        cam.LEDOff();
      }
    }
  }
  else
  {
    cam.LEDOn( CAM_LED_ERROR_BLINK );
    robot_state = RBT_STATE_HALT;
  }

  // if robot is moving
  if ( RBT_STATE_HALT < robot_state )
  {
    // do not overtake field boundaries
    if ( ( (DISTANCE_MAX < distance) && (motor_speed < 0) )
         || ( (DISTANCE_MIN > distance) && (motor_speed > 0) )
       )
    {
      motor_speed = 0;
      ctlr.motorTwoSpeed( motor_speed );
      //robot_state = RBT_STATE_HALT;
    }

    // compute distance current error
    dist_error = distance - goal;

    // stop when robot is close enought to his targeted position
    if ( ((dist_error) < 10) && ((dist_error) > -10) )
    {
      //robot_state = RBT_STATE_HALT;
      motor_speed = 0;
      ctlr.motorTwoSpeed( motor_speed );
    }
    // =============================
    //   speed computation
    //if( millis()%500 && ( ((dist_error) < -10) || ((dist_error > 10)) ) )
    motor_speed = compute_speed( dist_error );
    ctlr.motorTwoSpeed( motor_speed );
  }
}
