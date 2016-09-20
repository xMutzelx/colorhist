#include <math.h>

#ifndef TRAINING_H_
#define TRAINING_H_

/* Scale the color_value into steps (10 at the moment) 
 * value greater 5 = round up
 * value equal and smaller 5 = round down
 * => values from 0 - 250 (26 steps)
*/
int scale_color(int color_value);

/* Calculate the average color of all values 
 * newColor = new color value
 * avgColor = actual average color
 * start - true  = avgColor = avgColor/counter
 * 	     - false = avgColor += newColor
 * counter = How many color values were used
*/
int get_avg_color(int newColor, int avgColor, bool start, int counter);

#endif /* TRAINING_H_ */
