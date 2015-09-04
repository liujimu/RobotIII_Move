﻿#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <stdlib.h>

#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Aris_Control.h>
#include <Robot_Server.h>

using namespace Aris::Core;

//#include "trajectory_generator.h"


Aris::Core::MSG parseWalk(const std::string &cmd, const map<std::string, std::string> &params)
{
	Robots::WALK_PARAM  param;

	for(auto &i:params)
	{
		if(i.first=="totalCount")
		{
			param.totalCount=std::stoi(i.second);
		}
		else if(i.first=="n")
		{
			param.n=stoi(i.second);
		}
		else if(i.first=="walkDirection")
		{
			param.walkDirection=stoi(i.second);
		}
		else if(i.first=="upDirection")
		{
			param.upDirection=stoi(i.second);
		}
		else if(i.first=="distance")
		{
			param.d=stod(i.second);
		}
		else if(i.first=="height")
		{
			param.h=stod(i.second);
		}
		else if(i.first=="alpha")
		{
			param.alpha=stod(i.second);
		}
		else if(i.first=="beta")
		{
			param.beta=stod(i.second);
		}
	}

	Aris::Core::MSG msg;

	msg.CopyStruct(param);

	std::cout<<"finished parse"<<std::endl;

	return msg;
}

Aris::Core::MSG parseAdjust(const std::string &cmd, const map<std::string, std::string> &params)
{
	double firstEE[18] =
	{
		-0.3,-0.75,-0.65,
		-0.45,-0.75,0,
		-0.3,-0.75,0.65,
		0.3,-0.75,-0.65,
		0.45,-0.75,0,
		0.3,-0.75,0.65,
	};

	double beginEE[18]
	{
		-0.3,-0.85,-0.65,
		-0.45,-0.85,0,
		-0.3,-0.85,0.65,
		0.3,-0.85,-0.65,
		0.45,-0.85,0,
		0.3,-0.85,0.65,
	};

    Robots::ADJUST_PARAM  param;

	std::copy_n(firstEE, 18, param.targetPee[0]);
	std::fill_n(param.targetBodyPE[0], 6, 0);
	std::copy_n(beginEE, 18, param.targetPee[1]);
	std::fill_n(param.targetBodyPE[1], 6, 0);

	param.periodNum = 2;
	param.periodCount[0]=1000;
	param.periodCount[1]=1500;

	std::strcpy(param.relativeCoordinate,"B");
	std::strcpy(param.relativeBodyCoordinate,"B");

	for(auto &i:params)
	{
		if(i.first=="all")
		{

		}
		else if(i.first=="first")
		{
			param.legNum=3;
			param.motorNum=9;

			param.legID[0]=0;
			param.legID[1]=2;
			param.legID[2]=4;

			int motors[9] = { 0,1,2,6,7,8,12,13,14 };
			std::copy_n(motors, 9, param.motorID);
		}
		else if(i.first=="second")
		{
			param.legNum=3;
			param.motorNum=9;

			param.legID[0]=1;
			param.legID[1]=3;
			param.legID[2]=5;

			int motors[9] = { 3,4,5,9,10,11,15,16,17 };
			std::copy_n(motors, 9, param.motorID);
		}
		else
		{
			std::cout<<"parse failed"<<std::endl;
			return MSG{};
		}
	}

	Aris::Core::MSG msg;

	msg.CopyStruct(param);

	std::cout<<"finished parse"<<std::endl;

	return msg;
}

Aris::Core::MSG parseMove(const std::string &cmd, const map<std::string, std::string> &params)
{
    double firstEE[18] =
    {
        -0.3,-0.75,-0.65,
        -0.45,-0.75,0,
        -0.3,-0.75,0.65,
        0.3,-0.75,-0.65,
        0.45,-0.75,0,
        0.3,-0.75,0.65,
    };

    double beginEE[18]
    {
        -0.3,-0.85,-0.65,
        -0.45,-0.85,0,
        -0.3,-0.85,0.65,
        0.3,-0.85,-0.65,
        0.45,-0.85,0,
        0.3,-0.85,0.65,
    };

    int legNo=0;//腿序号
    double legPee[3];

    for(auto &i:params)
    {
        if(i.first=="component")
        {
            if(i.second=="lf")
            {
                legNo=0;
            }
            else if(i.second=="lm")
            {
                legNo=1;
            }
            else if(i.second=="lr")
            {
                legNo=2;
            }
            else if(i.second=="rf")
            {
                legNo=3;
            }
            else if(i.second=="rm")
            {
                legNo=4;
            }
            else if(i.second=="rr")
            {
                legNo=5;
            }
        }
        else if(i.first=="x")
        {
            legPee[0]=stod(i.second);
        }
        else if(i.first=="y")
        {
            legPee[1]=stod(i.second);
        }
        else if(i.first=="z")
        {
            legPee[2]=stod(i.second);
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
            return MSG{};
        }
    }

    std::copy_n(legPee, 3, &beginEE[3*legNo]);

    Robots::ADJUST_PARAM  param;

    std::copy_n(firstEE, 18, param.targetPee[0]);
    std::fill_n(param.targetBodyPE[0], 6, 0);
    std::copy_n(beginEE, 18, param.targetPee[1]);
    std::fill_n(param.targetBodyPE[1], 6, 0);

    param.periodNum = 2;
    param.periodCount[0]=1000;
    param.periodCount[1]=1500;

    std::strcpy(param.relativeCoordinate,"B");
    std::strcpy(param.relativeBodyCoordinate,"B");

    param.legNum=1;
    param.motorNum=3;
    param.legID[0]=legNo;

    int motors[3] = { 3*legNo,3*legNo+1,3*legNo+2 };
    std::copy_n(motors, 9, param.motorID);

    Aris::Core::MSG msg;

    msg.CopyStruct(param);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}

int main()
{
	auto rs = Robots::ROBOT_SERVER::GetInstance();
	rs->CreateRobot<Robots::ROBOT_III>();
    rs->LoadXml("/usr/Robots/CMakeDemo/Robot_III/resource/HexapodIII_Move.xml");
	rs->AddGait("wk",Robots::walk,parseWalk);
	rs->AddGait("ad",Robots::adjust,parseAdjust);
    rs->AddGait("move",Robots::adjust,parseMove);
	rs->Start();
	/**/
	std::cout<<"finished"<<std::endl;


	Aris::Core::RunMsgLoop();

	return 0;
}
