#include "filtering.h"

double calcConf(double min, double max, double result)
{
	return (((result-min)/(max-min))*100);
}

bool filterconf(double result, double minConf)
{
	if(result>=minConf)
		return true;
		
	else
		return false;
}

