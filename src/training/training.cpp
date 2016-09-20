#include "training.h"

int scale_color(int color_value)
{	
	int color;
	int scaling_factor = 10;
	
	if((color_value%scaling_factor) > (scaling_factor/2))
		color=color_value+(scaling_factor-color_value%scaling_factor);
	else
		color=color_value-(color_value%scaling_factor);
		
	return color;
}

int get_avg_color(int newColor, int avgColor, bool start, int counter)
{
	avgColor+=newColor;
	
	if(!start)
		return avgColor; 
	else
		return round((avgColor/counter));
}
