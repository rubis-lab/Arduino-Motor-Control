#define _USE_MATH_DEFINES
#include"SteerControl.h"
#include<ctime>
#include<cmath>

SteerControl::SteerControl()
{
	sampleTime = 1.0;
	distanceFactor = 1.0 * sampleTime;
	lastTime = - sampleTime;
	kp = - 0.4;
	ki = - 0.0 * sampleTime;
	kd = - 0.0 / sampleTime;
	lastOffset = 0.0;
	integralTerm = 0.0;
	minAngle = -90;
	maxAngle = 90;
	lastAngle = 0.0;
	controlAngle = 0.0;
	calculatorCount = 0;
}

bool SteerControl::Calculate(double newCurrentOffset)
{
	double currentTime = clock() / (double) CLOCKS_PER_SEC;
	double elapsedTime = currentTime - lastTime;

	//care for clock() overflows range of double (about 30 to 50 hours later)
	if(currentTime >= 0 && elapsedTime >= sampleTime)
	{
		//PID control values
		double currentOffset = newCurrentOffset;
		integralTerm += ki * currentOffset * sampleTime;
		double derivateError;

		//removes initial oscillation when calculation times less than 5
		if(calculatorCount < 5)
		{
			derivateError = 0;
			calculatorCount++;
		}
		else
		{
			derivateError = (currentOffset - lastOffset) / sampleTime;
		}

		//update formal values
		lastTime = currentTime;
		lastOffset = currentOffset;

		//sums PID control output
		double pidOutput = (kp * currentOffset) + (integralTerm) + (kd * derivateError);
		
		//limits output value
		double minOutput = distanceFactor * tan(minAngle * M_PI / 180);
		double maxOutput = distanceFactor * tan(maxAngle * M_PI / 180);

		if(pidOutput < minOutput)
		{
			if(integralTerm < minOutput){ integralTerm = minOutput; }
			pidOutput = minOutput;
		}
		else if(pidOutput > maxOutput)
		{
			if(integralTerm > maxOutput){ integralTerm = maxOutput; }
			pidOutput = maxOutput;
		}

		//get controlled angle
		controlAngle = (atan(pidOutput / distanceFactor) * 180 / M_PI) - lastAngle;
	}
	else
	{
		return false;
	}
	return true;
}

void SteerControl::Tune(double newKp, double newKi, double newKd)
{
	kp = - newKp;
	ki = - newKi * sampleTime;
	kd = - newKd / sampleTime;
}

void SteerControl::SetSampleTime(double newSampleTime)
{
	double ratio = newSampleTime / sampleTime;
	ki *= ratio;
	kd /= ratio;
	distanceFactor *= ratio;
	sampleTime = newSampleTime;
}

void SteerControl::SetOutputLimit(double newMinAngle, double newMaxAngle)
{
	if(newMinAngle <= newMaxAngle)
	{
		minAngle = newMinAngle;
		maxAngle = newMaxAngle;
		double minOutput = distanceFactor * tan(minAngle * M_PI / 180);
		double maxOutput = distanceFactor * tan(maxAngle * M_PI / 180);
		
		if(controlAngle < minAngle){ controlAngle = minAngle; }
		else if(controlAngle > maxAngle){ controlAngle = maxAngle; }

		if(integralTerm < minOutput){ controlAngle = minAngle; }
		else if(controlAngle > maxOutput){ controlAngle = maxAngle; }
	}
}

double SteerControl::GetControlAngle()
{
	return controlAngle;
}