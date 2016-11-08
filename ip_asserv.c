/**
 *
 * @file    ip_asserv.c
 *
 * @brief   Asservissement of inverted pendulum Robot.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    07 Septembre 2015
 *
 * @update  24 October 2016
 *
 * @version 1.2
 *
 */

/*=========================================================================*/
/* Includes files.                                                         */
/*=========================================================================*/

/*=========================================================================*/
/* Functions.                                                              */
/*=========================================================================*/

/**
 * @fn     asserv
 * @brief  Asservissement routine of the robot.
 */
void asserv(void) {
  // TODO: Adampt the implementation of this function
    while (1) {
        /*
    // Read Angle, Acceleration and temperature is need.
    //readImu(&timer);

    //
    // Drive motors:
    // If the robot is laying down, it has to be put in a vertical position
    // before it starts balancing.
    // If it's already balancing it has to be ±45 degrees before it stops
    // trying to balance.
    //
    if ((layingDown && (pitch < 170 || pitch > 190)) ||
    (!layingDown && (pitch < 135 || pitch > 225))){
      //
      //  The robot is in a unsolvable position, so turn off both motors and
      //  wait until it's vertical again.
      //
      layingDown = true;
      motorsStopAndReset();
    } else {
      //
      // It's no longer laying down,
      // so we can try to stabilized the robot now.
      //
      layingDown = false;
      pid(targetAngle, targetOffset, turningOffset);
    }

    // Update wheel velocity every 100ms.
    motorGetWheelVelocity();

    // Use a time fixed loop.
    //lastLoopUsefulTime = micros() - loopStartTime;

    if (lastLoopUsefulTime < STD_LOOP_TIME) {
      while((micros() - loopStartTime) < STD_LOOP_TIME);
    }
    //loopStartTime = micros();
        */
  }
}
