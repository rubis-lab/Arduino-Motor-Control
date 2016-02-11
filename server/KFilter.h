class KFilter
{
private:
	double averageNoise;

	double observed;

	double estimated;

	double predictionError;

	double kalmanGain;

public:
	KFilter(double newAverageNoise, double initialObserved);

	double runFilter(double newObserved, double newEstimated);
};