#ifndef MOVE_GAIT_H
#define MOVE_GAIT_H

#endif // MOVE_GAIT_H

#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>
#include <atomic>

#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Aris_Control.h>
#include <Aris_DynKer.h>
#include <Robot_Server.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>
#include <Robot_Type_I.h>

#ifndef PI
#define PI 3.141592653589793
#endif

using namespace Aris::Core;
//using namespace std;

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

struct PRE2SHV_PARAM :public Robots::GAIT_PARAM_BASE
{
    double distance;
    double height;
    std::int32_t periodCount[2];
};

struct SHOVEL_PARAM :public Robots::GAIT_PARAM_BASE
{
    enum { MAX_PERIOD_NUM = 10};

    double height;//铲斗由起始位置下落的高度
    double length;//铲斗前后单侧平移的距离
    double rad;//铲斗单侧摆动的弧度
    double radius;//铲斗摆动的半径
    std::int32_t periodCount[MAX_PERIOD_NUM]{1000};
    std::int32_t periodNum{1};
};

enum WALK_DIRECTION
{
    STOP,
    FORWARD,
    BACKWARD,
    RIGHT,
    LEFT
};

/*parse function*/
Aris::Core::MSG parseMove2(const std::string &cmd, const std::map<std::string, std::string> &params);
Aris::Core::MSG parseSwing(const std::string &cmd, const std::map<std::string, std::string> &params);
Aris::Core::MSG parsePre2Shv(const std::string &cmd, const std::map<std::string, std::string> &params);
Aris::Core::MSG parseShovel(const std::string &cmd, const std::map<std::string, std::string> &params);
Aris::Core::MSG parseCW(const std::string &cmd, const std::map<std::string, std::string> &params);
Aris::Core::MSG parseCWStop(const std::string &cmd, const std::map<std::string, std::string> &params);
Aris::Core::MSG parseCWF(const std::string &cmd, const std::map<std::string, std::string> &params);
Aris::Core::MSG parseCWFStop(const std::string &cmd, const std::map<std::string, std::string> &params);

/*operation function*/
int move2(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
int swing(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
int pre2shv(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
int shovel(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
int continuousWalk(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
int continuousWalkWithForce(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
