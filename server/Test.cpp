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
	test_onBoard("config\\PID_settings.cfg");
	return 0;
}

void test_onBoard(string configDirectory = "")
{
	//init steer controller
	SteerControl steerController;
	if(configDirectory.compare("") != 0){ steerController.readConfig(configDirectory); }
	printf("config done\n");

	//i2c_init();
	printf("i2c init done\n");
	double offset = 0.0;

	int ret = 0;
	
	initLD();
	while(1)
	{
		ret = laneDetect();
		printf("%d\n", ret);
	}

	while(1)
	{
		//printf("in while\n");
		/*
		TODO : get a offset data from vision processor
		ex) offset = get_offset_from_vision();
		*/
		if(steerController.calculate(offset))
		{
			int angleChange = steerController.getControlAngle() + 90;
			
			//i2c_send(1, angleChange);
		}
	}
}