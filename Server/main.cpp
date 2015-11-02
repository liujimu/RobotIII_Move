#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <stdlib.h>

#include "Move_Gait.h"

int main()
{
    auto rs = Robots::ROBOT_SERVER::GetInstance();
    rs->CreateRobot<Robots::ROBOT_TYPE_I>();
    rs->LoadXml("/home/hex/Desktop/RobotIII_Move/resource/Robot_VIII.xml");
    rs->AddGait("wk",Robots::walk,Robots::parseWalk);
    rs->AddGait("ad",Robots::adjust,Robots::parseAdjust);
    rs->AddGait("fw", Robots::fastWalk, Robots::parseFastWalk);
    rs->AddGait("ro", Robots::resetOrigin, Robots::parseResetOrigin);
    rs->AddGait("move2",move2,parseMove2);
    rs->AddGait("sw",swing,parseSwing);
    rs->AddGait("pre2shv",pre2shv,parsePre2Shv);
    rs->AddGait("shovel",shovel,parseShovel);
    rs->AddGait("cw",continuousWalk,parseCW);
    rs->AddGait("stop",continuousWalk,parseStop);

    rs->Start();
	/**/
	std::cout<<"finished"<<std::endl;


	Aris::Core::RunMsgLoop();

	return 0;
}
