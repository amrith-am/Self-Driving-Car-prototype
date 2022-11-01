#include "function.cpp"
#include <gtest/gtest.h>  
    
 // Distance Stop Sign   
TEST(DistanceStopSignTest, PositiveNos) 
{ 
    ASSERT_EQ(94, distanceStop(55, 45));
    ASSERT_EQ(79, distanceStop(60, 40));
    ASSERT_EQ(27, distanceStop(75, 20));
    ASSERT_EQ(19, distanceStop(85, 25));
}
 
TEST(DistanceStopSignTest, NegativeNos) 
{
    ASSERT_EQ(34, distanceStop(-30, -80));
    ASSERT_EQ(79, distanceStop(-100, -120));
    ASSERT_EQ(12, distanceStop(-40, -105));
}
  
 
 // Distance Car Obstacle   
TEST(DistanceCarTest, PositiveNos) 
{ 
    ASSERT_EQ(11, distanceCar(170, 65));
    ASSERT_EQ(24, distanceCar(150, 65));
    ASSERT_EQ(43, distanceCar(130, 75));
    ASSERT_EQ(30, distanceCar(100, 25));
}
 
TEST(DistanceCarTest, NegativeNos) 
{
    ASSERT_EQ(30, distanceCar(-30, -105));
    ASSERT_EQ(40, distanceCar(-25, -85));
    ASSERT_EQ(14, distanceCar(-55, -155));
}
   
 
 // Distance Speed Sign 20   
TEST(DistanceSpeedSign20Test, PositiveNos) 
{ 
    ASSERT_EQ(12, distanceSpeedSign20(50, 10));
    ASSERT_EQ(12, distanceSpeedSign20(95, 55));
    ASSERT_EQ(23, distanceSpeedSign20(95, 60));
    ASSERT_EQ(34, distanceSpeedSign20(70, 40));
}
 
TEST(DistanceSpeedSign20Test, NegativeNos) 
{
    ASSERT_EQ(34, distanceSpeedSign20(-120, -150));
    ASSERT_EQ(12, distanceSpeedSign20(-130, -170));
    ASSERT_EQ(34, distanceSpeedSign20(-150, -180));
}
   
 
 // Distance Speed Sign 50   
TEST(DistanceSpeedSign50Test, PositiveNos) 
{ 
    ASSERT_EQ(12, distanceSpeedSign50(50, 10));
    ASSERT_EQ(12, distanceSpeedSign50(95, 55));
    ASSERT_EQ(23, distanceSpeedSign50(95, 60));
    ASSERT_EQ(34, distanceSpeedSign50(70, 40));
}
 
TEST(DistanceSpeedSign50Test, NegativeNos) 
{
    ASSERT_EQ(34, distanceSpeedSign50(-120, -150));
    ASSERT_EQ(12, distanceSpeedSign50(-130, -170));
    ASSERT_EQ(34, distanceSpeedSign50(-150, -180));
}  

 // Distance Traffic Light   
TEST(DistanceTrafficLightTest, PositiveNos) 
{ 
    ASSERT_EQ(30, distanceTrafficLight(150, 130));
    ASSERT_EQ(15, distanceTrafficLight(110, 85));
    ASSERT_EQ(108, distanceTrafficLight(60, 66));
    ASSERT_EQ(174, distanceTrafficLight(60, 88));
}
 
TEST(DistanceTrafficLightTest, NegativeNos) 
{
    ASSERT_EQ(6, distanceTrafficLight(-60, -88));
    ASSERT_EQ(30, distanceTrafficLight(-40, -60));
    ASSERT_EQ(45, distanceTrafficLight(-115, -130));
}   
 
int main(int argc, char **argv) 
    {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    }

