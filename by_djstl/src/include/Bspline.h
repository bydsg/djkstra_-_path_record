#ifndef _BSPLINE_H_
#define _BSPLINE_H_


#include <memory>
#include <iostream>
#include <math.h>

#pragma once
//#include "position.h"
extern vector<Point>B_spline_optimization(std::vector<Point> all_point);
typedef struct tagPosition
{
    double  x;
    double  y;
	tagPosition(double _x,double _y) { x=_x; y=_y;}
	tagPosition() {};
	bool operator==(const tagPosition & pt) { return (x==pt.x && y==pt.y);} 
} CPosition;
 
class CBSpline
{
public:
	CBSpline(void);
	~CBSpline(void);
 
	void TwoOrderBSplineSmooth(CPosition *pt,int Num);
	void TwoOrderBSplineInterpolatePt(CPosition *&pt,int &Num,int *InsertNum);
	double F02(double t);
	double F12(double t);
	double F22(double t);
 
	void ThreeOrderBSplineSmooth(CPosition *pt,int Num);
	void ThreeOrderBSplineInterpolatePt(CPosition *&pt,int &Num,int *InsertNum);
	double F03(double t);
	double F13(double t);
	double F23(double t);
	double F33(double t);
};

#endif