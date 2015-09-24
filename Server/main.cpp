#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <map>
#include <string>

using namespace std;

#include <stdlib.h>

#ifndef PI
#define PI 3.141592653589793
#endif

#include "Move_Gait.h"

int main()
{
	auto rs = Robots::ROBOT_SERVER::GetInstance();
	rs->CreateRobot<Robots::ROBOT_III>();
    rs->LoadXml("/home/hex/Desktop/RobotIII_Move/resource/HexapodIII_Move.xml");
    rs->AddGait("wk",Robots::walk,Robots::parseWalk);
    rs->AddGait("ad",Robots::adjust,Robots::parseAdjust);
    rs->AddGait("move2",move2,parseMove2);
    rs->AddGait("sw",swing,parseSwing);
    rs->AddGait("pre2shv",pre2shv,parsePre2Shv);
    rs->AddGait("shovel",shovel,parseShovel);
    rs->Start();
	/**/
	std::cout<<"finished"<<std::endl;


	Aris::Core::RunMsgLoop();

	return 0;
}
