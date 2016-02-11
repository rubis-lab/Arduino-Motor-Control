#include<string>
#include"KFilter.h"

class SteerControl{

private:
	double kp, ki, kd, velocityFactor;
	
	double lastTime;
	
	double sampleTime;

	double lastOffset;

	double integralTerm;

	int absAngle, controlAngle;
	
	unsigned int absAngleLimit;

	int calculatorCount;

	KFilter* filter;

	int floor(double value);

	double calculatePid(double newCurrentOffset);

	double applyLimits(double newPidOutput);

public:
	SteerControl();

	~SteerControl();

	bool calculate(double newCurrentOffset);
	
	void tune(double newKp, double newKi, double newKd);
	
	void setSampleTime(double newSampleTime);
	
	void setControlAngleLimit(unsigned int newControlAngleLimit);

	void setAbsAngleLimit(unsigned int newAbsAngleLimit);

	void setVelocityFactor(double newVelocityFactor);

	bool readConfig(std::string configDirectory);

	int getAbsAngle();

	int getControlAngle();
};