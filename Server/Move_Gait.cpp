#include "Move_Gait.h"

Aris::Core::MSG parseMove2(const std::string &cmd, const map<std::string, std::string> &params)
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
        if(i.first=="y")
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

    param.periodCount=3000;

    Aris::Core::MSG msg;
    msg.CopyStruct(param);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}

int swing(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    const SWING_PARAM *pSWP = static_cast<const SWING_PARAM *>(pParam);

    //计算圆弧半径和起始俯仰角
    double radius;
    double beginRad;
    double beginBodyPE123[6];
    Aris::DynKer::s_pe2pe("313", pSWP->beginBodyPE, "123", beginBodyPE123); //将起始身体位姿转换到123欧拉角
    radius = sqrt(pow((pSWP->centreP[1] - pSWP->beginBodyPE[1]),2) + pow((pSWP->centreP[2] - pSWP->beginBodyPE[2]),2));
    beginRad = atan2((pSWP->beginBodyPE[1] - pSWP->centreP[1]) , -(pSWP->beginBodyPE[2] - pSWP->centreP[2]));

    double s = -(pSWP->swingRad / 2)*cos(PI * (pSWP->count  + 1) / pSWP->periodCount ) + pSWP->swingRad / 2;

    /*插值当前的身体位置*/
    double pBody[6];
    double pBody123[6];
    double currentRad;
    currentRad = beginRad + s;
    std::copy_n(beginBodyPE123, 6, pBody123);

    pBody123[1] = pSWP->centreP[1] + radius*sin(currentRad);
    pBody123[2] = pSWP->centreP[2] - radius*cos(currentRad);
    pBody123[3] = beginBodyPE123[3] + s;//当前俯仰角
    Aris::DynKer::s_pe2pe("123", pBody123, "313", pBody);

    pRobot->SetPee(pSWP->beginPee, pBody);

    /*test*/
    if(pSWP->count%500==0)
    {
        rt_printf("beginRad: %f\n"
                  , beginRad);
        rt_printf("pBody: %f %f %f\n"
                        , pBody[3], pBody[4], pBody[5]);
    }

    /*返回剩余的count数*/

    return pSWP->periodCount - pSWP->count - 1;

}


Aris::Core::MSG parsePre2Shv(const std::string &cmd, const map<std::string, std::string> &params)
{
    PRE2SHV_PARAM  param;

    for(auto &i:params)
    {
        if(i.first=="distance")
        {
            param.distance=stod(i.second);
        }
        else if(i.first=="height")
        {
            param.height=stod(i.second);
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
            return MSG{};
        }
    }

    param.periodCount[0]=3000;
    param.periodCount[1]=3000;

    Aris::Core::MSG msg;
    msg.CopyStruct(param);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}

/*调节脚的站位，准备铲土*/
int pre2shv(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    const PRE2SHV_PARAM *pPSP = static_cast<const PRE2SHV_PARAM *>(pParam);
    int pos{0}, periodBeginCount{0}, periodEndCount{0};
    double b = pPSP->height;
    double a = pPSP->distance/2;


    /*判断当前所处的周期*/
    for (int i = 0; i < 2; ++i)
    {
        periodEndCount += pPSP->periodCount[i];

        if ((pPSP->count < periodEndCount) && (pPSP->count >= periodBeginCount))
        {
            pos = i;
            break;
        }

        periodBeginCount = periodEndCount;
    }

    double s = -(PI / 2)*cos(PI * (pPSP->count - periodBeginCount + 1) / (periodEndCount- periodBeginCount)) + PI / 2;

    /*插值当前的末端和身体位置*/
    double pEE[18], pBody[6];
    std::copy_n(pPSP->beginPee, 18, pEE);
    std::copy_n(pPSP->beginBodyPE, 6, pBody);

    if (pos == 0)
    {
        /*设置移动腿0,2,4*/
        for (int i = 0; i < 18; i += 6)
        {
            int xSign = pow(-1,i/9);//0、1、2腿左移；3、4、5腿右移
            pEE[i] = xSign*(a*cos(s) - a) + pPSP->beginPee[i];
            pEE[i + 1] = b*sin(s) + pPSP->beginPee[i + 1];
        }
        if(pPSP->count%1000==999)
        {
            rt_printf("1st pos: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n"
                            , pEE[0], pEE[1], pEE[2], pEE[3], pEE[4], pEE[5], pEE[6], pEE[7], pEE[8]
                            , pEE[9], pEE[10], pEE[11], pEE[12], pEE[13], pEE[14], pEE[15], pEE[16], pEE[17]);
        }
    }
    else
    {
        /*设置支撑腿0,2,4*/
        for (int i = 0; i < 18; i += 6)
        {
            int xSign = pow(-1,i/9);
            pEE[i] = -xSign*2*a + pPSP->beginPee[i];
        }
        /*设置移动腿1,3,5*/
        for (int i = 3; i < 18; i += 6)
        {
            int xSign = pow(-1,i/9);
            pEE[i] = xSign*(a*cos(s) - a) + pPSP->beginPee[i];
            pEE[i + 1] = b*sin(s) + pPSP->beginPee[i + 1];
        }
        if(pPSP->count%1000==1)
        {
            rt_printf("2nd pos: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n"
                            , pEE[0], pEE[1], pEE[2], pEE[3], pEE[4], pEE[5], pEE[6], pEE[7], pEE[8]
                            , pEE[9], pEE[10], pEE[11], pEE[12], pEE[13], pEE[14], pEE[15], pEE[16], pEE[17]);
        }
    }

    pRobot->SetPee(pEE, pBody);

    /*计算总共所需要花的时间，以便返回剩余的count数*/
    int totalCount{ 0 };
    for (int i = 0; i < 2; ++i)
    {
        totalCount += pPSP->periodCount[i];
    }

    return totalCount - pPSP->count - 1;
}

Aris::Core::MSG parseShovel(const std::string &cmd, const map<std::string, std::string> &params)
{
    SHOVEL_PARAM  param;

    param.radius=2;
    param.periodNum = 7;
    for(int i = 0; i < param.periodNum; i++)
    {
        param.periodCount[i]=3000;
    }

    for(auto &i:params)
    {
        if(i.first=="height")
        {
            param.height=stod(i.second);
        }
        else if(i.first=="length")
        {
            param.length=stod(i.second);
        }
        else if(i.first=="degree")
        {
            param.rad=stod(i.second)/180*PI;
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

/*完整的铲斗规划*/
int shovel(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    const SHOVEL_PARAM *pSVP = static_cast<const SHOVEL_PARAM *>(pParam);
    int pos{0}, periodBeginCount{0}, periodEndCount{0};
    double realTargetPbody[SHOVEL_PARAM::MAX_PERIOD_NUM][6];

    //第0段轨迹目标位姿
    Aris::DynKer::s_pe2pe("313", pSVP->beginBodyPE, "123", realTargetPbody[0]); //将起始身体位姿转换到123欧拉角
    realTargetPbody[0][3] += pSVP->rad;

    //第1段轨迹目标位姿
    std::copy_n(realTargetPbody[0], 6, realTargetPbody[1]);
    realTargetPbody[1][2] -= pSVP->length;

    //第2段轨迹目标位姿
    std::copy_n(realTargetPbody[1], 6, realTargetPbody[2]);
    realTargetPbody[2][1] -= pSVP->height;

    //第3段轨迹目标位姿
    std::copy_n(realTargetPbody[2], 6, realTargetPbody[3]);
    realTargetPbody[3][2] += pSVP->length;

    //第4段轨迹目标位姿
    std::copy_n(realTargetPbody[3], 6, realTargetPbody[4]);
    realTargetPbody[4][1] += pSVP->radius*(1-cos(2*pSVP->rad));
    realTargetPbody[4][2] += pSVP->radius*sin(2*pSVP->rad);
    realTargetPbody[4][3] -= 2*pSVP->rad;

    //第5段轨迹目标位姿
    std::copy_n(realTargetPbody[4], 6, realTargetPbody[5]);
    std::copy_n(pSVP->beginBodyPE, 3, realTargetPbody[5]);

    //第6段轨迹目标位姿
    std::copy_n(realTargetPbody[5], 6, realTargetPbody[6]);
    realTargetPbody[6][3] += pSVP->rad;


    /*判断当前所处的周期*/
    for (int i = 0; i < pSVP->periodNum; ++i)
    {
        periodEndCount += pSVP->periodCount[i];

        if ((pSVP->count < periodEndCount) && (pSVP->count >= periodBeginCount))
        {
            pos = i;
            break;
        }

        periodBeginCount = periodEndCount;
    }

    double s = -(PI / 2)*cos(PI * (pSVP->count - periodBeginCount + 1) / (periodEndCount- periodBeginCount)) + PI / 2;

    /*插值当前的身体位姿*/
    double pEE[18], pBody[6];

    if (pos == 0)
    {
        for (int i = 0; i < 6; ++i)
        {
            pBody[i] = pSVP->beginBodyPE[i] * (cos(s) + 1) / 2 + realTargetPbody[pos][i] * (1 - cos(s)) / 2;
        }
    }
    else if (pos == 4)
    {
        std::copy_n(realTargetPbody[3], 6, pBody);
        pBody[1] += pSVP->radius*(1-cos(s/PI*2*pSVP->rad));
        pBody[2] += pSVP->radius*sin(s/PI*2*pSVP->rad);
        pBody[3] -= s/PI*2*pSVP->rad;
    }
    else if (pos == 6)
    {
        for (int i = 0; i < 6; ++i)
        {
            pBody[i] = realTargetPbody[pos - 1][i] * (cos(s) + 1) / 2 + realTargetPbody[pos][i] * (1 - cos(s)) / 2;
        }
    }
    else
    {
        for (int i = 0; i < 6; ++i)
        {
            pBody[i] = realTargetPbody[pos - 1][i] * (cos(s) + 1) / 2 + realTargetPbody[pos][i] * (1 - cos(s)) / 2;
        }
    }

    if(pSVP->count%3000==0)
    {
        rt_printf("pBody: %f %f %f %f %f %f\n"
                        , pBody[0], pBody[1], pBody[2], pBody[3], pBody[4], pBody[5]);
    }


    /*足尖位置保持不动*/
    std::copy_n(pSVP->beginPee, 18, pEE);

    pRobot->SetPee(pEE, pBody, "G", "123");

    /*计算总共所需要花的时间，以便返回剩余的count数*/
    int totalCount{ 0 };
    for (int i = 0; i < pSVP->periodNum; ++i)
    {
        totalCount += pSVP->periodCount[i];
    }

    return totalCount - pSVP->count - 1;
}

