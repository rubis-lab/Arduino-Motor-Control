class AnomalyDetector
{
private:
	double average, variant;

	int count;

	double ratio;

	double threshold;

	double lastScore;

public:
	AnomalyDetector(double newThreshold, double newAverage, double newVariant, double newRatio);

	bool updateStatistic(double value);

	double getAverage();

	double getVariant();

	double getLastScore();
};