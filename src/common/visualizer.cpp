#include <vector>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "visualizer.h"

using namespace cv;
using namespace std;

float getFactor(int value)
{
	return (100/value);
}

float getMax(float newValue, float oldMax)
{
	if(oldMax>=newValue)
		return oldMax;
	else
		return newValue;
}

cv::Mat visualizeHistogram(float array[][3], int avgColor[3]) //Falls zu langsam als Referenz übergeben &array[][3]
{
	/* Pixel 0 and 1 are not drawn so we need an offset of minimum 2 pixels */
	const int xDrawingOffset=2;
	const int yDrawingOffset=10;
	float maxValue=0.0f;
	float factor=1.0f;
	int xSize = 480;
	int ySize = 200;
	
	Point pt1, pt2;
	
	// Create mat with alpha channel
    Mat mat(200+yDrawingOffset, xSize+xDrawingOffset, CV_8UC3);
	mat = Scalar(255,255,255);

	/* Sucht den größten Wert(Prozentsatz) in den Farben */
	for(int i=0; i<26; i++)
	{
		for(int j=0; j<3; j++)
		{
			maxValue=getMax(array[i][j],maxValue);
		}
	}
	
	/* Umrechnungsfaktor für 100 Pixel Höhe */
	factor=getFactor(maxValue);

	for(int j=0; j<3; j++)
	{
		for(int i=1; i<26; i++)
		{
			pt1.x=((i-1)*(ySize/10))+xDrawingOffset;
			pt1.y=(factor*array[i-1][j])+yDrawingOffset;
			
			pt2.x=(i*(ySize/10))+xDrawingOffset;
			pt2.y=(factor*array[i][j])+yDrawingOffset;
			
			/* Draws each individual line
			 * j=0 red
			 * j=1 green
			 * j=2 blue
			 * Scalar (blue,green,red)
			*/
			if(j==0)
				line(mat,pt1,pt2, Scalar(0,0,255),1,CV_AA,0);
			if(j==1)
				line(mat,pt1,pt2, Scalar(0,255,0),1,CV_AA,0);
			if(j==2)
				line(mat,pt1,pt2, Scalar(255,0,0),1,CV_AA,0);
		}
	}
	
	pt1.x = xDrawingOffset;
	pt1.y = 2;
	
	pt2.x = xSize;
	pt2.y = 2;
	
	/* Draw the average color as a 4 pixel thick line over the complete width */
	line(mat, pt1, pt2, Scalar(avgColor[2],avgColor[1],avgColor[0]), 4, CV_AA, 0);
	
	/* Flip the y-Coordinate otherwise it would be up-side-down */
	flip(mat,mat,0);
	
	return mat;
}
