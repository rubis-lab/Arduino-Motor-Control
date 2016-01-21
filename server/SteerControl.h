class SteerControl{

private:
	double kp, ki, kd, distanceFactor;
	
	double lastTime;
	
	double sampleTime;

	double lastOffset;

	double integralTerm;
	
	double minAngle, maxAngle;

	double minSteer, maxSteer;

	double lastAngle, controlAngle;

	int calculatorCount;

public:
	SteerControl();

	bool Calculate(double newCurrentOffset);
	
	void Tune(double newKp, double newKi, double newKd);
	
	void SetSampleTime(double newSampleTime);
	
	void SetOutputLimit(double newMinAngle, double newMaxAngle);

	double GetControlAngle();
};