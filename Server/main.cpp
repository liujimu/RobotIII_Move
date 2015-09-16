#include <iostream>
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
#include <Robot_Gait.h>
#include <Robot_Base.h>

#ifndef PI
#define PI 3.141592653589793
#endif

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

struct MOVES_PARAM :public Robots::GAIT_PARAM_BASE
{
    double targetPee[18]{0};
    double targetBodyPE[6]{0};
    std::int32_t periodCount;
    int comID; //移动的部件（component）序号
    bool isAbsolute{false}; //用于判断移动命令是绝对坐标还是相对坐标
};

struct SWING_PARAM :public Robots::GAIT_PARAM_BASE
{
    double centreP[3]{0};
    double swingRad;
    std::int32_t periodCount;
};

Aris::Core::MSG parseMove(const std::string &cmd, const map<std::string, std::string> &params)
{
    MOVES_PARAM  param;

    double targetPos[3]; //移动目标位置

    for(auto &i:params)
    {
        if(i.first=="component")
        {
            if(i.second=="lf")
            {
                param.comID=0;
            }
            else if(i.second=="lm")
            {
                param.comID=1;
            }
            else if(i.second=="lr")
            {
                param.comID=2;
            }
            else if(i.second=="rf")
            {
                param.comID=3;
            }
            else if(i.second=="rm")
            {
                param.comID=4;
            }
            else if(i.second=="rr")
            {
                param.comID=5;
            }
            else if(i.second=="bd")
            {
                param.comID=6;
            }
            else
            {
                std::cout<<"parse failed"<<std::endl;
                return MSG{};
            }
        }
        //绝对坐标移动
        else if(i.first=="x")
        {
            targetPos[0]=stod(i.second);
            param.isAbsolute=true;
        }
        else if(i.first=="y")
        {
            targetPos[1]=stod(i.second);
            param.isAbsolute=true;
        }
        else if(i.first=="z")
        {
            targetPos[2]=stod(i.second);
            param.isAbsolute=true;
        }
        //相对坐标移动
        else if(i.first=="u")
        {
            targetPos[0]=stod(i.second);
        }
        else if(i.first=="v")
        {
            targetPos[1]=stod(i.second);
        }
        else if(i.first=="w")
        {
            targetPos[2]=stod(i.second);
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
            return MSG{};
        }
    }

    if(param.comID==6)
    {
        std::copy_n(targetPos, 3, param.targetBodyPE);
    }
    else
    {
        std::copy_n(targetPos, 3, &param.targetPee[3*param.comID]);
        param.legNum=1;
        param.motorNum=3;
        param.legID[0]=param.comID;
        int motors[3] = { 3*param.comID, 3*param.comID+1, 3*param.comID+2 };
        std::copy_n(motors, 9, param.motorID);
    }

    param.periodCount=3000;

    Aris::Core::MSG msg;
    msg.CopyStruct(param);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}

int move2(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    const MOVES_PARAM *pMP = static_cast<const MOVES_PARAM *>(pParam);

    double realTargetPee[18];
    double realTargetPbody[6];
    std::copy_n(pMP->beginPee, 18, realTargetPee);
    std::copy_n(pMP->beginBodyPE, 6, realTargetPbody);

    //绝对坐标
    if (pMP->isAbsolute)
    {
        if(pMP->comID==6)
        {
            std::copy_n(pMP->targetBodyPE, 3, realTargetPbody);
        }
        else
        {
            std::copy_n(&(pMP->targetPee[pMP->comID*3]), 3, &realTargetPee[pMP->comID*3]);
        }
    }
    //相对坐标
    else
    {
        if(pMP->comID==6)
        {
            for(int i=0;i<3;i++)
            {
                realTargetPbody[i]+=pMP->targetBodyPE[i];
            }
        }
        else
        {
            for(int i=0;i<18;i++)
            {
                realTargetPee[i]+=pMP->targetPee[i];
            }
        }
    }

    double s = -(PI / 2)*cos(PI * (pMP->count  + 1) / pMP->periodCount ) + PI / 2;

    /*插值当前的末端和身体位置*/
    double pEE[18], pBody[6];

    for (int i = 0; i < 18; ++i)
    {
        pEE[i] = pMP->beginPee[i] * (cos(s) + 1) / 2 + realTargetPee[i] * (1 - cos(s)) / 2;
    }
    for (int i = 0; i < 6; ++i)
    {
        pBody[i] = pMP->beginBodyPE[i] * (cos(s) + 1) / 2 + realTargetPbody[i] * (1 - cos(s)) / 2;
    }

    pRobot->SetPee(pEE, pBody);

    /*test*/
//    if(pMP->count%500==0)
//    {
//        rt_printf("rr: %f %f %f\n"
//                        , pEE[15], pEE[16], pEE[17]);
//    }

    /*返回剩余的count数*/

    return pMP->periodCount - pMP->count - 1;

}

Aris::Core::MSG parseSwing(const std::string &cmd, const map<std::string, std::string> &params)
{
    SWING_PARAM  param;

    for(auto &i:params)
    {
        //绝对坐标移动
        if(i.first=="x")
        {
            param.centreP[0]=stod(i.second);
        }
        else if(i.first=="y")
        {
            param.centreP[1]=stod(i.second);
        }
        else if(i.first=="z")
        {
            param.centreP[2]=stod(i.second);
        }
        else if(i.first=="deg")
        {
            param.swingRad=stod(i.second)/180*PI;//计算身体摆动的弧度
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
            return MSG{};
        }
    }

    param.periodCount=6000;

    Aris::Core::MSG msg;
    msg.CopyStruct(param);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}

int swing(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    const SWING_PARAM *pMP = static_cast<const SWING_PARAM *>(pParam);

    //计算圆弧半径和起始俯仰角
    double radius;
    double beginRad;
    double beginBodyPE123[6];
    s_pe2pe("313", pMP->beginBodyPE, "123", beginBodyPE123); //将起始身体位姿转换到123欧拉角
    radius = sqrt(pow((pMP->centreP[1] - pMP->beginBodyPE[1]),2) + pow((pMP->centreP[2] - pMP->beginBodyPE[2]),2));
    beginRad = atan2((pMP->beginBodyPE[1] - pMP->centreP[1]) , -(pMP->beginBodyPE[2] - pMP->centreP[2]));

    double s = -(pMP->swingRad / 2)*cos(PI * (pMP->count  + 1) / pMP->periodCount ) + pMP->swingRad / 2;

    /*插值当前的身体位置*/
    double pBody[6];
    double pBody123[6];
    double currentRad;
    currentRad = beginRad + s;
    std::copy_n(beginBodyPE123, 6, pBody123);

    pBody123[1] = pMP->centreP[1] + radius*sin(currentRad);
    pBody123[2] = pMP->centreP[2] - radius*cos(currentRad);
    pBody123[3] = beginBodyPE123[3] + s;//当前俯仰角
    s_pe2pe("123", pBody123, "313", pBody);

    pRobot->SetPee(pMP->beginPee, pBody);

    /*test*/
    if(pMP->count%500==0)
    {
        rt_printf("beginRad: %f\n"
                  , beginRad);
        rt_printf("pBody: %f %f %f\n"
                        , pBody[3], pBody[4], pBody[5]);
    }

    /*返回剩余的count数*/

    return pMP->periodCount - pMP->count - 1;

}

int main()
{
	auto rs = Robots::ROBOT_SERVER::GetInstance();
	rs->CreateRobot<Robots::ROBOT_III>();
	rs->LoadXml("/usr/Robots/CMakeDemo/Robot_III/resource/HexapodIII_Move.xml");
	rs->AddGait("wk",Robots::walk,parseWalk);
	rs->AddGait("ad",Robots::adjust,parseAdjust);
    rs->AddGait("move",move2,parseMove);
    rs->AddGait("sw",swing,parseSwing);
	rs->Start();
	/**/
	std::cout<<"finished"<<std::endl;


	Aris::Core::RunMsgLoop();

	return 0;
}
