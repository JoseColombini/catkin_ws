#include "UAV.cpp"


  int main (int argc, char **argv)
{
  ros::init(argc, argv, "UAV");
  UAV mav;
  mav->takeoff();
  ros::Rate loop_rate(20.0);
  while not mav->is_aligned == "aligned!" {
    loop_rate.sleep();
  }
  mav->RTL();

  ros::spin();
}
