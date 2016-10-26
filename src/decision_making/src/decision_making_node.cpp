#include "decision_making/openHouse2016ActionClient.h"
#include "decision_making/antiBullyStudy2016ActionClient.h"
#include "decision_making/receiveMsg.h"

enum Context
{
  ROBOTICS_OPENHOUSE_2016_DEMO = 1,
  ANTI_BULLY_2016_STUDY = 2
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "decision_making");


  Openhouse2016ActionClient *demo1 = new Openhouse2016ActionClient();
    AntiBullyStudy2016ActionClient *demo2 = new AntiBullyStudy2016ActionClient();



  const Context c = Context::ANTI_BULLY_2016_STUDY; // THIS IS WHERE YOU SET THE CONTEXT

  switch( c )
  {
    case Context::ROBOTICS_OPENHOUSE_2016_DEMO:
      demo1->runActionClientDev();//(argc, argv);
      break;

    case Context::ANTI_BULLY_2016_STUDY:
      demo2->runActionClientDev();//(argc, argv);
      break;
    
    default:
      std::cerr << "ERROR in decision_making node, no context found" << std::endl;
      break;
  }

  return 0;
}
