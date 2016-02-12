#include"SteerControl.h"
#include"i2c_server.h"
#include "LaneDetect/laneDetect.h"

using namespace std;

void test_console(string);
void test_onBoard(string);

int main()
{
	//requires PID_settings.cfg file at designated dir
	printf("in main\n");
	test_onBoard("config/PID_settings.cfg");
	return 0;
}

void test_onBoard(string configDirectory = "")
{
	//init steer controller
	SteerControl steerController;
	if(configDirectory.compare("") != 0)
	{
		if(steerController.readConfig(configDirectory)){ printf("config done\n"); }
	}

	i2c_init();
	printf("i2c init done\n");

	int ret = 0;
	int angle = 90;
	
	initLD();
	while(1)
	{
		ret = laneDetect();

		if(steerController.calculate((double) ret))
		{
			printf("angle\t: %d\n", angle);

			angle = steerController.getAbsAngle() * 2 + 90;
			printf("offset\t: %f\n", steerController.getFilteredOffset());
			printf("PID_out\t: %f\n", steerController.getPidOutput());
			printf("target_angle\t: %d\n\n", angle);

			i2c_send(1, angle);
		}
	}
	i2c_exit();
}