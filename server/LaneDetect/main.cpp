#include "laneDetect.h"

int main(void)
{
	int ret = 0;
	
	initLD();
	while(1)
	{
		ret = laneDetect();
		printf("%d\n", ret);
	}

	return 0;
}
