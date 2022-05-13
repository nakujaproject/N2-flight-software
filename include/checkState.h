#ifndef CHECKSTATE_H
#define CHECKSTATE_H

#include "functions.h"
#include "defs.h"

// This checks that we have starting ascent
// If we have a positive 20 metres displacement upwards
int checkInflight(float altitude)
{
  float displacement = altitude - BASE_ALTITUDE;
  if (displacement > 20)
  {
    return 1;
  }
  else
  {
    return 0;
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
    return 2;
  }
  else
  {
    return 1;
  }
}

// This checks that we are descending from maximum altitude
// If we have moved down past 20 metres
int checkDescent(float altitude)
{
  float displacement = altitude - MAX_ALTITUDE;
  if (displacement < 20)
  {
    return 3;
  }
  else
  {
    return 2;
  }
}

// This checks that we have reached the ground
// detects landing of the rocket
// TODO: BASE_ALTITUDE might be different from the original base altitude
int checkGround(float altitude)
{
  float displacement = altitude - BASE_ALTITUDE;
  if ((displacement > 20) || (displacement < 20))
  {
    return 4;
  }
  else
  {
    return 3;
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
  int rval;
  switch (state)
  {
  case 0:
    rval = checkInflight(altitude);
    break;
  case 1:
    rval = checkApogee(velocity, altitude);
    break;
  case 2:
    rval = checkDescent(altitude);
    break;
  case 3:
    rval = checkGround(altitude);
    break;
  default:
    rval = checkInflight(altitude);
    break;
  }
  return rval;
}

#endif