/**
 * @file KFilter.cpp
 * @brief KFilter class implementation
 * @author Hwang, Seongjae
 * @date 2016. 02. 23.
 */

#include"KFilter.h"

/**
 * @param newAverageNoise Average noise value for Kalman Filter calculation\n
 * The smaller it is, the more reliable sensor is
 * @param initialObserved The very first observed sensor input 
 */
KFilter::KFilter(double newAverageNoise, double initialObserved)
{
	averageNoise = newAverageNoise;
	observed = initialObserved;
	estimated = observed;
	predictionError = 1.0;
	kalmanGain = 0.0;
}

/**
 * @brief Kalman Filter estimates next input, compare it with actual sensor input\n
 * and update estimated value as to be final filtered output
 * @param newObserved New observed sensor input
 * @param newEstimated Estimated input which is predicted before
 * @return Filtered value computed from the observed input and estimated input
 */
double KFilter::runFilter(double newObserved, double newEstimated)
{
	observed = newObserved;
	estimated = newEstimated;
	
	kalmanGain = predictionError / (predictionError + averageNoise);
	estimated = estimated + kalmanGain * (observed - estimated);
	predictionError = (1 - kalmanGain) * predictionError;

	return estimated;
}
