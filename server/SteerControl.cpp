/**
 * @file SteerControl.cpp
 * @brief SteerControl implementation
 * @author Hwang, Seongjae
 * @date 2016. 02. 23.
 */

#define _USE_MATH_DEFINES
#include<ctime>
#include<cmath>
#include<fstream>
#include"SteerControl.h"

/**
 * @brief 
 * Must be followed by readConfig() for true initialization since constructor only provides naive initialization
 * @see SteerControl.readConfig()
 */
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
	controlAngle = 0;
	absAngleLimit = 80;
	calculatorCount = 0;
	lastPidOutput = 0.0;
	filter = NULL;
}

/**
 * @brief
 * Does nothing unless filter is available\n
 * To make filter available, initialize object with readConfig()
 * @see SteerControl.readConfig()
 */
SteerControl::~SteerControl()
{
	if(filter != NULL){ delete filter; }
}

/**
 * @parameter value
 * Input double value
 * @return
 * Integer value rounded off from double
 */
int SteerControl::floor(double value)
{
	return (((int) (value * 10.0)) % 10 >= 5)? ((int) value) + 1 : (int) value;
}

/**
 * @parameter newCurrentOffset
 * New input value which is lateral offset from road centerline to vehicle
 * @return
 * Return calculated PID output value
 */
double SteerControl::calculatePid(double newCurrentOffset)
{
	//PID control factors
	double currentOffset = newCurrentOffset - (velocityFactor * sampleTime * sin(absAngle * M_PI / 180));
	integralTerm += ki * currentOffset * sampleTime;
	double derivateError;

	//removes initial oscillation when calculation times less than 10
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
	lastOffset = currentOffset;
	
	//sums PID control output
	return (kp * currentOffset) + (integralTerm) + (kd * derivateError);
}

/**
 * @parameter newPidOutput
 * New recently calculated PID output
 * @return
 * MAX PID output if (newPidOuptut > MAX), MIN if (newPidOuptut < MIN)
 */
double SteerControl::applyLimits(double newPidOutput)
{
	double minOutput = velocityFactor * sampleTime * sin(- (signed int) absAngleLimit * M_PI / 180);
	double maxOutput = velocityFactor * sampleTime * sin((signed int) absAngleLimit * M_PI / 180);
	double pidOutput = newPidOutput;
	
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

	lastPidOutput = pidOutput;
	return pidOutput;
}

/**
 * @brief Call getAbsAngle(), getControlAngle(), or getPidOutput() to get updated output value after calculate() called\n
 * Filter off noise with runFilter(), if filter is available
 * @param newCurrentOffset
 * New input value which is lateral offset from road centerline to vehicle
 * @return Return true if successfully updated, false if not
 * @see SteerControl.getAbsAngle()
 * @see SteerControl.getControlAngle()
 * @see SteerControl.getPidOutput()
 * @see KFilter.runFilter()
 */
bool SteerControl::calculate(double newCurrentOffset)
{
	double currentTime = clock() / (double) CLOCKS_PER_SEC;
	double elapsedTime = currentTime - lastTime;

	if(currentTime >= 0 && elapsedTime >= sampleTime)
	{
		//filter off offset
		if(filter != NULL)
		{
			//newCurrentOffset = filter->runFilter(newCurrentOffset, lastOffset - (velocityFactor * sampleTime * sin(absAngle * M_PI / 180)));
			newCurrentOffset = filter->runFilter(newCurrentOffset * 2.0, lastOffset);
		}

		//cacluate PID control output
		double pidOutput = calculatePid(newCurrentOffset);

		//update formal time
		lastTime = currentTime;

		//limits wheel angle change
		pidOutput = applyLimits(pidOutput);

		//get relative controlled angle and update last angle
		controlAngle = floor(asin(pidOutput / (velocityFactor * sampleTime)) * 180 / M_PI) - absAngle;
		absAngle += controlAngle;
	}
	else
	{
		return false;
	}
	return true;
}

/**
 * @brief It is also possible to tune factors in runtime
 * @param newKp New Kp factor for PID controller
 * @param newKi New Ki factor for PID controller
 * @param newKd New Kd factor for PID controller
 */
void SteerControl::tune(double newKp, double newKi, double newKd)
{
	kp = newKp;
	ki = newKi * sampleTime;
	kd = newKd / sampleTime;
}

/**
 * @brief It is also possible to tune factors in runtime
 * @param newSampleTime New sampling interval in second
 */
void SteerControl::setSampleTime(double newSampleTime)
{
	double ratio = newSampleTime / sampleTime;
	ki *= ratio;
	kd /= ratio;
	sampleTime = newSampleTime;
}

/**
 * @brief
 * Maximum absolute angle would be newAbsAngleLimit\n
 * Minimum absolute angle would be (- 1) * newAbsAngleLimit
 * @param newAbsAngleLimit New absolute angle limits in degree (not radian)
 */
void SteerControl::setAbsAngleLimit(unsigned int newAbsAngleLimit)
{
	absAngleLimit = newAbsAngleLimit;
		
	if(absAngle < - (signed int) absAngleLimit){ absAngle = - (signed int) absAngleLimit; }
	else if(absAngle > (signed int) absAngleLimit){ absAngle = (signed int) absAngleLimit; }

	double minOutput = velocityFactor * sampleTime * sin(- (signed int) absAngleLimit * M_PI / 180);
	double maxOutput = velocityFactor * sampleTime * sin((signed int) absAngleLimit * M_PI / 180);

	if(integralTerm < minOutput){ integralTerm = minOutput; }
	else if(integralTerm > maxOutput){ integralTerm = maxOutput; }
}

/**
 * @brief
 * Set and initialize PID controller factors and Kalman Filter\n
 * See Config_File_Example on documentation main page\n
 * Omit both "Filter_XXX_..." lines not to use Kalman Filter
 * @param configDirectory Directory of config file in string
 * @bug
 * Fail to read config file directory sometimes\n
 * Probably because of string.compare() malfunctioning
 * @return True if successfully read config file, false if not
 */
bool SteerControl::readConfig(std::string configDirectory)
{
	//factor will remain the same when config file does not contain any changes for that
	double newKp = kp;
	double newKi = ki;
	double newKd = kd;
	double newSampleTime = sampleTime;
	double newVelocityFactor = velocityFactor;
	double newAbsAngleLimit = absAngleLimit;
	double newAverageNoise = 0.1;
	double newInitialObserved = 0.0;

	//defines name of each control set
	std::string pidSet = "[PIDControlSettings]";
	std::string kpSet = "Kp";
	std::string kiSet = "Ki";
	std::string kdSet = "Kd";
	std::string sampleTimeSet = "Sample_Time";
	std::string velocityFactorSet = "Velocity_Factor";
	std::string absAngleLimitSet = "Absolute_Angle_Limit";
	std::string filterAverageNoiseSet = "Filter_Average_Noise";
	std::string filterInitialObservedSet = "Filter_Initial_Observed";

	bool filterFlag = false;

	std::ifstream fin;
	std::string controlSet = "";

	fin.open(configDirectory);
	if(fin.fail()){ return false; }

	//search for "[PIDControlSettings]" statement, return false when it fails to find one
	while(controlSet.compare(pidSet) != 1)
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
		else if (controlSet.find(absAngleLimitSet) != std::string::npos)
		{
			newAbsAngleLimit = std::stoi(value);
		}
		else if (controlSet.find(filterAverageNoiseSet) != std::string::npos)
		{
			newAverageNoise = std::stod(value);
			filterFlag = true;
		}
		else if (controlSet.find(filterInitialObservedSet) != std::string::npos)
		{
			newInitialObserved = std::stod(value);
			filterFlag = true;
		}
		std::getline(fin, controlSet);
	}

	fin.close();

	//set PID control factors with values read
	tune(newKp, newKi, newKd);
	setSampleTime(newSampleTime);
	setAbsAngleLimit(newAbsAngleLimit);
	setVelocityFactor(newVelocityFactor);

	if(filter == NULL && filterFlag)
	{
		filter = new KFilter(newAverageNoise, newInitialObserved);
	}

	return true;
}

/**
 * @brief It is also possible to tune factors in runtime
 * @param newVelocityFactor New approximate velocity of vehicle in (camera pixel / sec)
 */
void SteerControl::setVelocityFactor(double newVelocityFactor)
{
	velocityFactor = newVelocityFactor;
}

/**
 * @return Last updated absolute(top-view) angle from road centerline to vehicle
 */
int SteerControl::getAbsAngle()
{
	return absAngle;
}

/**
 * @return Last updated angle change amount
 */
int SteerControl::getControlAngle()
{
	return controlAngle;
}

/**
 * @return Last input value(filtered)
 */
double SteerControl::getFilteredOffset()
{
	return lastOffset;
}

/**
 * @return Last updated PID output value
 */
double SteerControl::getPidOutput()
{
	return lastPidOutput;
}
