/**
 * @mainpage
 * 
 * @section Intro
 * 
 * RC car steering controller module using PID Controller and Kalman Filter.\n
 * 
 * @section How_To_Use
 *
 * 1. Declare a SteerControl object and call SteerControl.readConfig() to initiate PID factors.\n
 * (see Config_File_Example below)\n
 * 2. Call SteerControl.calculate() to update PID output value.\n
 * Output successfully updated when SteerControl.calculate() returns true.
 * 3. Get PID ouput with SteerControl.getAbsAngle() or SteerControl.getControlAngle().
 * 4. Apply control angle.
 *
 * @section Config_File_Example
 * 
 * config file consists of several "Control_Set = value" lines.\n
 * PID control section always starts with "[PIDControlSettings]", ends with "End" statement.\n\n
 * It is also possible not to use filter by removing both "Filter_XXX..." lines.
 *
 * ex)\n
 * [PIDControlSettings]\n
 * Kp=0.8\n
 * Ki=0.4\n
 * Kd=0.0\n
 * Sample_Time=0.1\n
 * Velocity_Factor=1000.0\n
 * Absolute_Angle_Limit=30\n
 * Filter_Average_Noise=0.1\n
 * Filter_Initial_Observed=0.0\n
 * End
 */

/**
 * @file SteerControl.h
 * @brief SteerControl class header
 * @author Hwang, Seongjae
 * @date 2016. 02. 23.
 */

#include<string>
#include"KFilter.h"

class SteerControl{

private:
	double kp,	///< Proportional factor for PID calculation
			ki,		///< Integral factor for PID calculation
			kd,		///< Derivative factor for PID calculation
			velocityFactor;	///< Approximate RC car velocity for PID calculation
	
	double lastTime;	///< Last time when PID output value updated
	
	double sampleTime;	///< Sampling frequency in second

	double lastOffset;	///< Last PID input value which is lateral offset

	double integralTerm;	///< Interal error sum * Ki so far. Easifyies runtime PID tuning

	int absAngle,	///< Absolute (top-view) car angle from road centerline
		controlAngle;	///< Absolute angle change amount based on PID output
	
	unsigned int absAngleLimit;	///< Absolute angle limitation

	int calculatorCount;	///< Times that PID controller updated, Initially 0

	double lastPidOutput;	///< Last updated PID output value

	KFilter* filter;	///< Kalman Filter to filter off input noise

	int floor(double value);	///< Rounding off input value from double to integer

	double calculatePid(double newCurrentOffset);	///< Calculate PID output from lateral offset input

	double applyLimits(double newPidOutput);	///< Limits PID output for not being too small or too big

public:
	SteerControl();	///< Contructor. Does not initialize filter

	~SteerControl();	///< Destructor. Free filter since it's dynamically allocated object

	bool calculate(double newCurrentOffset);	///< Calculate and update PID output
	
	void tune(double newKp, double newKi, double newKd);	///< Adjust PID factors - Kp, Ki, Kd
	
	void setSampleTime(double newSampleTime);	///< Set sampling interval

	void setAbsAngleLimit(unsigned int newAbsAngleLimit);	///< Set absolute angle limitation

	void setVelocityFactor(double newVelocityFactor);	///< Set approximate RC car velocity

	bool readConfig(std::string configDirectory);	///< Read a given config file and initialize factors including filter

	int getAbsAngle();	///< Get last absolute angle value

	int getControlAngle();	///< Get last angle change amount 

	double getPidOutput();	///< Get last PID output value

	double getFilteredOffset();	///< Get last filtered offset(input) value
};