/**
 * @file KFilter.h
 * @brief KFilter class header
 * @author Hwang, Seongjae
 * @date 2016. 02. 23.
 */

class KFilter
{
private:
	double averageNoise;	///< Estimated average noise based on normal distribution

	double observed;	///< Last observed input from sensor

	double estimated;	///< Last estimated input calculated from formal input

	double predictionError;	///< Prediction error value for Kalman Filter

	double kalmanGain;	///< Kalman Gain factor for Kalman Filter

public:
	KFilter(double newAverageNoise, double initialObserved);	///< Constructor. Initialize filter

	double runFilter(double newObserved, double newEstimated);	///< Update filter values with new sensor input
};