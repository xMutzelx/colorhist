#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

bool methodCRG (float frameHist[][3], float modelHist[][3]);

double methodChiQuadratic (cv::MatND frameHist, cv::MatND modelHist);

/* [1|-1] 1 = perfect match, -1 = worst match */
double methodCorrelation (cv::MatND frameHist, cv::MatND modelHist);

/* [0|+infinity] 0 = perfect match, infinity = worst match */
double methodChiSquare (cv::MatND frameHist, cv::MatND modelHist);

/* [0|1] 0 = worst match, 1 = perfect match (CAUTION: Histograms needs to be normalized) */
double methodIntersection (cv::MatND frameHist, cv::MatND modelHist);

/* [0|1] 0 = perfect match, 1 = worst match */
double methodBhattacharyya (cv::MatND frameHist, cv::MatND modelHist);

/* Majority decision between multiple detection algorithms */
double meanDecision(cv::MatND frameHist, cv::MatND modelHist, bool correlation, bool intersection, bool bhattacharyya);


