#include "detection.h"
#include "filtering.h"

using namespace std;
using namespace cv;

double methodCorrelation (cv::MatND frameHist, cv::MatND modelHist)
{
	return cv::compareHist( frameHist, modelHist, 0);
}

double methodChiSquare (cv::MatND frameHist, cv::MatND modelHist)
{
    return cv::compareHist( frameHist, modelHist, 1);
}

double methodIntersection (cv::MatND frameHist, cv::MatND modelHist)
{
    return (cv::compareHist( frameHist, modelHist, 2)/300);
}

double methodBhattacharyya (cv::MatND frameHist, cv::MatND modelHist)
{
    return cv::compareHist( frameHist, modelHist, 3);
}

double meanDecision(cv::MatND frameHist, cv::MatND modelHist, bool correlation, bool intersection, bool bhattacharyya)
{
	double result = 0;
	int counter = 0;
	
	if(!correlation && !intersection && !bhattacharyya)
		return 0;
 	
	if(correlation)
	{
		result += calcConf(-1,1,methodCorrelation(frameHist, modelHist));
		counter++;
	}
	if(intersection)
	{
		result += calcConf(0,1, methodIntersection(frameHist, modelHist));
		counter++;
	}
	if(bhattacharyya)
	{
		result += calcConf(1,0,methodBhattacharyya(frameHist, modelHist));
		counter++;
	}

	return (result/counter);
}
