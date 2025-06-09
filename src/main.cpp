#include "LIVMapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_lio");
  ros::NodeHandle nh;
  
  // 禁用相机功能
  nh.setParam("laserMapping/use_camera", false);
  
  LIVMapper liv_mapper(nh);
  liv_mapper.initializeSubscribersAndPublishers(nh);  // 必须加上这句
  liv_mapper.run();                                   // 必须加上这句

  
  return 0;
}