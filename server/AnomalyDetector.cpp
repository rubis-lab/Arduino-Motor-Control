#define _USE_MATH_DEFINES
#include<cmath>
#include"AnomalyDetector.h"

AnomalyDetector::AnomalyDetector(double newThreshold, double newAverage, double newVariant, double newRatio)
{
	count = 0;
	average = newAverage;
	variant = newVariant;
	ratio = newRatio;
	threshold = newThreshold;
}

bool AnomalyDetector::updateStatistic(double value)
{
	double score = 0;

	if(count >= 10){
		double normalDistribution = exp(- (pow(value - average, 2) / (2 * variant))) / sqrt(2 * M_PI * variant);
		score = - log(normalDistribution);
	}
	lastScore = score;
	count++;

	if(score < threshold)
	{
		double newAverage = ((1 - ratio) * average) + (ratio * value);
		double newVariant = std::abs((1 - ratio) * variant + ratio * pow(value - average, 2));

		average = newAverage;
		variant = newVariant;

		return false;
	}
	else
	{
		return true;
	}
}

double AnomalyDetector::getAverage()
{
	return average;
}

double AnomalyDetector::getVariant()
{
	return variant;
}

double AnomalyDetector::getLastScore()
{
	return lastScore;
}