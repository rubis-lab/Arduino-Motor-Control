#define _USE_MATH_DEFINES
#include<ctime>
#include<cmath>
#include<fstream>
#include"SteerControl.h"

SteerControl::SteerControl()
{
	sampleTime = 1.0;
	velocityFactor = 10.0;
	lastTime = - sampleTime;
	kp = 0.5;
	ki = 0.0;
	kd = 0.0;
	lastOffset = 0.0;
	integralTerm = 0.0;
	absAngle = 0;
	absAngleLimit = 80;
	controlAngle = 0;
	controlAngleLimit = 20;
	lastAngle = 0;
	calculatorCount = 0;
	isFiltering = false;
}

SteerControl::~SteerControl()
{
	if(isFiltering){ delete anomalyDetector; }
}

int SteerControl::floor(double value)
{
	return (((int) (value * 10.0)) % 10 >= 5)? ((int) value) + 1 : (int) value;
}

bool SteerControl::calculate(double newCurrentOffset)
{
	double currentTime = clock() / (double) CLOCKS_PER_SEC;
	double elapsedTime = currentTime - lastTime;

	if(currentTime >= 0 && elapsedTime >= sampleTime)
	{
		//PID control factors
		double currentOffset = newCurrentOffset - (velocityFactor * sampleTime * sin(absAngle * M_PI / 180));
		integralTerm += ki * currentOffset * sampleTime;
		double derivateError;

		/*
		//filter off anomaly
		if(isFiltering && anomalyDetector->updateStatistic(newCurrentOffset))
		{
			currentOffset = lastOffset;
		}
		*/

		//removes initial oscillation when calculation times less than 5
		if(calculatorCount < 10)
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
		
		//limits wheel angle
		double minOutput = velocityFactor * sampleTime * sin(- (signed int) controlAngleLimit * M_PI / 180);
		double maxOutput = velocityFactor * sampleTime * sin((signed int) controlAngleLimit * M_PI / 180);

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

		//get relative controlled angle and update last angle
		int angleChange = floor(asin(pidOutput / (velocityFactor * sampleTime)) * 180 / M_PI) - lastAngle;

		//limits absolute(top-view) angle
		if(absAngle + lastAngle + angleChange > (signed int) absAngleLimit)
		{
			angleChange = (signed int) absAngleLimit - absAngle - lastAngle;
		}
		else if(absAngle + lastAngle + angleChange < - (signed int) absAngleLimit)
		{
			angleChange = - (signed int) absAngleLimit - absAngle - lastAngle;
		}

		controlAngle = angleChange;
		lastAngle += controlAngle;
		absAngle = lastAngle;
	}
	else
	{
		return false;
	}
	return true;
}

void SteerControl::tune(double newKp, double newKi, double newKd)
{
	kp = newKp;
	ki = newKi * sampleTime;
	kd = newKd / sampleTime;
}

void SteerControl::setSampleTime(double newSampleTime)
{
	double ratio = newSampleTime / sampleTime;
	ki *= ratio;
	kd /= ratio;
	sampleTime = newSampleTime;
}

void SteerControl::setControlAngleLimit(unsigned int newControlAngleLimit)
{
	controlAngleLimit = newControlAngleLimit;
	double minOutput = velocityFactor * sampleTime * sin(- (signed int) controlAngleLimit * M_PI / 180);
	double maxOutput = velocityFactor * sampleTime * sin((signed int) controlAngleLimit * M_PI / 180);
		
	if(controlAngle < - (signed int) controlAngleLimit){ controlAngle = - (signed int) controlAngleLimit; }
	else if(controlAngle > (signed int) controlAngleLimit){ controlAngle = (signed int) controlAngleLimit; }

	if(integralTerm < minOutput){ integralTerm = minOutput; }
	else if(integralTerm > maxOutput){ integralTerm = maxOutput; }
}

void SteerControl::setAbsAngleLimit(unsigned int newAbsAngleLimit)
{
	absAngleLimit = newAbsAngleLimit;
}

bool SteerControl::readConfig(std::string configDirectory)
{
	/*
	 * config file consists of several "Control_Set = value" lines.
	 * PID control section must starts with "[PIDControlSettings]", ends with "End" statement
	 * ex)
	 * [PIDControlSettings]
	 * Kp=0.5
	 * Ki=0.2
	 * Sample_Time=0.2
	 * End
	 */

	//factor will remain the same when config file does not contain any changes for that
	double newKp = kp;
	double newKi = ki;
	double newKd = kd;
	double newSampleTime = sampleTime;
	double newVelocityFactor = velocityFactor;
	double newControlAngleLimit = controlAngleLimit;
	double newAbsAngleLimit = absAngleLimit;
	
	double newFilterThreshold = -1;
	double newFilterAverage = -1;
	double newFilterVariant = -1;
	double newFilterRatio = -1;

	//defines name of each control set
	std::string pidSet = "[PIDControlSettings]";
	std::string kpSet = "Kp";
	std::string kiSet = "Ki";
	std::string kdSet = "Kd";
	std::string sampleTimeSet = "Sample_Time";
	std::string velocityFactorSet = "Velocity_Factor";
	std::string controlAngleLimitSet = "Control_Angle_Limit";
	std::string absAngleLimitSet = "Absolute_Angle_Limit";

	std::string filterThresholdSet = "Anomaly_Threshold";
	std::string filterAverageSet = "Anomaly_Average";
	std::string filterVariantSet = "Anomaly_Variant";
	std::string filterRatioSet = "Anomaly_Ratio";

	std::ifstream fin;
	std::string controlSet = "";

	//search for "[PIDControlSettings]" statement, return false when it fails to find one
	fin.open(configDirectory);
	if(fin.fail()){ return false; }

	while(controlSet.compare(pidSet) != 0)
	{
		if(fin.eof()){ return false; }
		std::getline(fin, controlSet);
	}

	//get line by line for each "Control_Set = value" statement
	std::getline(fin, controlSet);
	while (controlSet.compare("End") != 0)
	{
		std::string value;
		std::size_t valuePosition = controlSet.find("=");
		value = controlSet.substr(valuePosition + 1);

		if (controlSet.find(kpSet) != std::string::npos)
		{
			newKp = std::stod(value);
		}
		else if (controlSet.find(kiSet) != std::string::npos)
		{
			newKi = std::stod(value);
		}
		else if (controlSet.find(kdSet) != std::string::npos)
		{
			newKd = std::stod(value);
		}
		else if (controlSet.find(sampleTimeSet) != std::string::npos)
		{
			newSampleTime = std::stod(value);
		}
		else if (controlSet.find(velocityFactorSet) != std::string::npos)
		{
			newVelocityFactor = std::stod(value);
		}
		else if (controlSet.find(controlAngleLimitSet) != std::string::npos)
		{
			newControlAngleLimit = std::stoi(value);
		}
		else if (controlSet.find(absAngleLimitSet) != std::string::npos)
		{
			newAbsAngleLimit = std::stoi(value);
		}
		else if (controlSet.find(filterThresholdSet) != std::string::npos)
		{
			newFilterThreshold = std::stod(value);
		}
		else if (controlSet.find(filterAverageSet) != std::string::npos)
		{
			newFilterAverage = std::stod(value);
		}
		else if (controlSet.find(filterVariantSet) != std::string::npos)
		{
			newFilterVariant = std::stod(value);
		}
		else if (controlSet.find(filterRatioSet) != std::string::npos)
		{
			newFilterRatio = std::stod(value);
		}
		std::getline(fin, controlSet);
	}

	fin.close();

	//set PID control factors with values read
	tune(newKp, newKi, newKd);
	setSampleTime(newSampleTime);
	setControlAngleLimit(newControlAngleLimit);
	setAbsAngleLimit(newAbsAngleLimit);
	setVelocityFactor(newVelocityFactor);

	if(!isFiltering && newFilterThreshold != -1 && newFilterAverage != -1 && newFilterVariant != -1 && newFilterRatio != -1)
	{
		isFiltering = true;
		anomalyDetector = new AnomalyDetector(newFilterThreshold, newFilterAverage, newFilterVariant, newFilterRatio);
	}

	return true;
}

void SteerControl::setVelocityFactor(double newVelocityFactor)
{
	velocityFactor = newVelocityFactor;
}

int SteerControl::getControlAngle()
{
	return controlAngle;
}

int SteerControl::getAbsAngle()
{
	return absAngle;
}

int SteerControl::getLastAngle()
{
	return lastAngle;
}