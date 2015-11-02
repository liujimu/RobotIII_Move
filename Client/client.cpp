#include <Robot_Client.h>

using namespace std;

int main(int argc, char *argv[])
{
    Robots::SendRequest(argc, argv, "/home/hex/Desktop/RobotIII_Move/resource/Robot_VIII.xml");

	return 0;
}
