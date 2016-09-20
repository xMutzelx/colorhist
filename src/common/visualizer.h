#ifndef VISUALIZER_H_
#define VISUALIZER_H_

/* Get a factor to scale the histogram.
 * The greatest value (example 20%) will be the highest value in the drawn histogram 
*/
float get_factor(int value);

/* Returns the greater value */
float getMax(float newValue, float oldMax);

/* 
 * This function draws a color histogram in respect to the input values.
 * It can only work with 26 steps of values (0 - 250 steps=10)
 * The histogram will be safed as an PNG into the database
 * array [][3 ] contains all scaled color values
 * avgColor[3] contains the 3 average color values  RGB 
 */
cv::Mat visualizeHistogram(float array[][3], int avgColor[3]);

#endif /* VISUALIZER_H_ */
