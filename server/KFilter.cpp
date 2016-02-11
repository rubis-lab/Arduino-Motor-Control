#include"KFilter.h"

KFilter::KFilter(double newAverageNoise, double initialObserved)
{
	averageNoise = newAverageNoise;
	observed = initialObserved;
	estimated = observed;
	predictionError = 1.0;
	kalmanGain = 0.0;
}

double KFilter::runFilter(double newObserved, double newEstimated)
{
	observed = newObserved;
	estimated = newEstimated;
	
	kalmanGain = predictionError / (predictionError + averageNoise);
	estimated = estimated + kalmanGain * (observed - estimated);
	predictionError = (1 - kalmanGain) * predictionError;

	return estimated;
}
