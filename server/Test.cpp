#define _USE_MATH_DEFINES
#include<iostream>
#include"SteerControl.h"
#include<Windows.h>
#include<ctime>
#include<cmath>

int main()
{
	SteerControl sc;
	double offset = 5.0;
	double angle;
	double sampleTime = 1.0 / 30.0;
	double timer = 0.0;
	double maxRange = 90;

	sc.Tune(0.7, 0.5, 0.0005);
	sc.SetSampleTime(sampleTime);
	sc.SetOutputLimit(- maxRange, maxRange);

	std::cout << "At\tOffset\t\tAngle\n";

	for(int i = 0; i < 100 / sampleTime; i ++){
		sc.Calculate(offset);

		angle = sc.GetControlAngle();
		timer += sampleTime;

		std::cout << timer << "\t" << offset << "\t\t" << angle << "\n";

		offset = offset + (1.0 * sampleTime) * sin(angle * M_PI / 180);

		Sleep(sampleTime * 1000);
	}
	return 0;
}