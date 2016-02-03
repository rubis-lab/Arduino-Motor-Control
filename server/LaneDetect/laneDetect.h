#include <stdio.h>
#include <vector>
#include <math.h>
#include <cv.h>
#include <highgui.h>

struct sPoint {
	int x, y;

	sPoint() : x(0), y(0) { }
	sPoint(int x_, int y_) : x(x_), y(y_) { }
};

struct sLine {
	int sx, sy; 
	int ex, ey; 
	int dx, dy; 
	double len;

	sLine() : sx(0), sy(0), ex(0), ey(0), dx(0), dy(0), len(0.) { } 
	sLine(int sx_, int sy_, int ex_, int ey_) 
		: sx(sx_), sy(sy_), ex(ex_), ey(ey_) 
	{   
		dx = ex - sx; 
		dy = ey - sy; 
		len = sqrt((double)(dx*dx + dy*dy));
	}   
};

struct sLaneCand {
	int x, y;
	float mag;
	float ori;

	sLaneCand() : x(0), y(0), mag(0.), ori(0.) { } 
	sLaneCand(int x_, int y_, float m_, float o_) 
		: x(x_), y(y_), mag(m_), ori(o_) { } 
};

int initLD(void);
int laneDetect(void);
