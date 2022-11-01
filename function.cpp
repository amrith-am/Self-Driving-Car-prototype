#include <math.h>
 
// Stop Sign Distance
int distanceStop(int P2, int P1)
{
    int dist_Stop = (-1.5)*(P2-P1)+109.5;
    if (dist_Stop != dist_Stop)  // Check NaN
    {
        return 0;
    }    
    else
    {
        return dist_Stop; 
    }
}


// Car Distance
int distanceCar(int P2, int P1)
{
    int dist_Car = (-0.652)*(P2-P1)+79.565;
    if (dist_Car != dist_Car)    // Check NaN
    {
        return 0;
    }    
    else
    {
        return dist_Car; 
    }
}

// Speed Sign 20 Distance
int distanceSpeedSign20(int P2, int P1)
{
    int dist_Speed20 = (-2.142)*(P2-P1)+98.571; 
    if (dist_Speed20 != dist_Speed20)    // Check NaN
    {
        return 0;
    }    
    else
    {
        return dist_Speed20; 
    }
}

// Speed Sign 50 Distance
int distanceSpeedSign50(int P2, int P1)
{
    int dist_Speed50 = (-2.142)*(P2-P1)+98.571;
    if (dist_Speed50 != dist_Speed50)    // Check NaN
    {
        return 0;
    }    
    else
    {
        return dist_Speed50; 
    }
}

// Traffic Light Distance
int distanceTrafficLight(int P2, int P1)
{
    int distanceTraffic = (-3)*(P2-P1)+90;  // need to Change...
    if (distanceTraffic != distanceTraffic)    // Check NaN
    {
        return 0;
    }    
    else
    {
        return distanceTraffic; 
    }
}



