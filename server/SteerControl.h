#include<string>
#include"AnomalyDetector.h"

class SteerControl{

private:
	double kp, ki, kd, velocityFactor;
	
	double lastTime;
	
	double sampleTime;

	double lastOffset;

	double integralTerm;

	int lastAngle, controlAngle, absAngle;
	
	unsigned int controlAngleLimit, absAngleLimit;

	int calculatorCount;

	AnomalyDetector* anomalyDetector;

	bool isFiltering;

	int floor(double value);

	double filterThreshold, filterAverage, filterVariant, filterRatio;

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

	int getControlAngle();
};