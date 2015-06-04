/**
 * @file pid.h
 *
 * @section desc File description
 *
 * Calculation motor speed and correting it to avoid overshoot
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
 * @section PID About PID
 *
 * PID : proportional-integral-derivative controller
 * "A PID controller calculates an error value as the difference between a
 * measured process variable and a desired setpoint. The controller
 * attempts to minimize the error by adjusting the process through use of
 * a manipulated variable." - Wikipedia
 *
 * @section infos File informations
 *
 * @date 2015/05/07
 * @author Benjamin Sientzoff
 *
 */
#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

// boundaries
#define SPEED_MAX 100
#define SPEED_MIN -100
// proportionnal coefficient
#define PI_COEF_PROP 45

int compute_speed( int consigne )
{
  int speed = consigne * PI_COEF_PROP >> 8;
  if ( speed > 60 ) return 60;
  if ( speed < -60 ) return -60;
  // filter speed
  if ( speed < 0 )
  {
    if (speed > -20) speed = -20;
    else
    {
      if (speed < -60) speed = -60;
    }
  }
  else
  {
    if ( speed < 25) speed = 25;
    else
    {
      if ( speed > 70 ) speed = 70;
    }
  }
  return speed;
}

#endif

