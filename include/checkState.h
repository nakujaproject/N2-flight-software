#ifndef CHECKSTATE_H
#define CHECKSTATE_H

#include "functions.h"
#include "defs.h"

// variable for detected apogee height
float MAX_ALTITUDE = 0;

// This checks that we have starting ascent
// If we have a positive 20 metres displacement upwards
int checkInflight(float altitude)
{
  float displacement = altitude - BASE_ALTITUDE;
  if (displacement > GROUND_STATE_HEIGHT)
  {
    return COASTING_STATE;
  }
  else
  {
    return PRE_FLIGHT_GROUND_STATE;
  }
}

// This checks that we have reached maximum altitude
// At apogee velocity is zero
int checkApogee(float velocity, float altitude)
{
  if (velocity < 0)
  {
    // Fire ejection charge
    ejection();
    MAX_ALTITUDE = altitude;
    return APOGEE_STATE;
  }
  else
  {
    return COASTING_STATE;
  }
}

// This checks that we are descending from maximum altitude
// If we have moved down past 20 metres
int checkDescent(float altitude)
{
  float displacement = altitude - MAX_ALTITUDE;
  if (displacement < AFTER_APOGEE_BEFORE_DESCENT_DISPLACEMENT)
  {
    return DESCENT_STATE;
  }
  else
  {
    return APOGEE_STATE;
  }
}

// This checks that we have reached the ground
// detects landing of the rocket
// TODO: BASE_ALTITUDE might be different from the original base altitude
int checkGround(float altitude)
{
  float displacement = altitude - BASE_ALTITUDE;
  if (displacement < GROUND_STATE_HEIGHT)
  {
    return POST_FLIGHT_GROUND_STATE;
  }
  else
  {
    return DESCENT_STATE;
  }
}

// checks the current state of the rocket
// The rocket is in state 0 and we are looking out for state 1
// We check if we have started flying
// We check if we have reached apogee
// The rocket is in state 1 and we are looking out for state 2
// We check if we are descending
// The rocket is in state 2 and we are looking out for state 3
// We check if we have reached the ground
// The rocket is in state 3 and we are looking out for state 4
int checkState(float altitude, float velocity, int state)
{
  switch (state)
  {
  case PRE_FLIGHT_GROUND_STATE:
    return checkInflight(altitude);
  case COASTING_STATE:
    return checkApogee(velocity, altitude);
  case APOGEE_STATE:
    return checkDescent(altitude);
  case DESCENT_STATE:
    return checkGround(altitude);
  case POST_FLIGHT_GROUND_STATE:
    return POST_FLIGHT_GROUND_STATE;
  default:
    return checkInflight(altitude);
  }
}

#endif