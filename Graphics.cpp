#include "StdAfx.h"
#include "GF.h"
#include "tools.h"
#include <vector>
#include <algorithm>
#include <list>

#ifndef M_PI
const double M_PI = 3.1415926535897932384626433832795;
#endif

#ifndef NO_VALUE
const int NO_VALUE = -777;
#endif

enum ClPointType { LEFT, RIGHT, BEYOND, BEHIND, BETWEEN, ORIGIN, DESTINATION };
ClPointType Classify(int x1, int y1, int x2, int y2, int x, int y)
{
	int ax = x2 - x1,
		ay = y2 - y1,
		bx = x - x1,
		by = y - y1;
	int s = ax*by - bx*ay;
	if (s < 0) return LEFT;
	if (s > 0) return RIGHT;
	if ((ax*bx < 0) || (ay*by < 0)) return BEHIND;
	if ((ax*ax + ay*ay) < (bx*bx + by*by)) return BEYOND;
	if ((x1 == x) && (y1 == y)) return ORIGIN;
	if ((x2 == x) && (y2 == y)) return DESTINATION;
	return BETWEEN;
}

enum IntersectType { COLLINEAR, PARALLEL, SKEW, SKEW_CROSS, SKEW_NO_CROSS };
IntersectType Intersect(int ax, int ay, int bx, int by, int cx, int cy, int dx, int dy, double *t)
{
	int nx = dy - cy,
		ny = cx - dx;
	int denom = nx*(bx - ax) + ny*(by - ay);
	if (denom == 0)
	{
		ClPointType type = Classify(cx, cy, dx, dy, ax, ay);
		if ((type == LEFT) || (type == RIGHT)) return PARALLEL;
		else return COLLINEAR;
	}
	int num = nx*(ax - cx) + ny*(ay - cy);
	*t = -((double)num) / denom;
	return SKEW;
}

IntersectType Cross(int ax, int ay, int bx, int by, int cx, int cy, int dx, int dy, double *tab, double *tcd)
{
	IntersectType type = Intersect(ax, ay, bx, by, cx, cy, dx, dy, tab);
	if ((type == PARALLEL) || (type == COLLINEAR))
		return type;
	if ((*tab < 0.0) || (*tab > 1.0))
		return SKEW_NO_CROSS;
	Intersect(cx, cy, dx, dy, ax, ay, bx, by, tcd);
	if ((*tcd < 0.0) || (*tcd > 1.0))
		return SKEW_NO_CROSS;
	return SKEW_CROSS;
}

enum Etype {TOUCHING, CROSS_LEFT, CROSS_RIGHT, INESSENTIAL};
Etype EdgeType(int xo, int yo, int xd, int yd, int xa, int ya)
{
	switch (Classify(xo, yo, xd, yd, xa, ya))
	{
	case LEFT: 
		if (ya > yo && ya <= yd)
			return CROSS_LEFT;
		else
			return INESSENTIAL;
	case RIGHT:
		if (ya > yd && ya <= yo)
			return CROSS_RIGHT;
		else
			return INESSENTIAL;
	case BETWEEN:
	case ORIGIN:
	case DESTINATION:
		return TOUCHING;
	default:
		return INESSENTIAL;
	}
}

bool PInPolygonEOMode(int x, int y, int* px, int*py, int n)
{
	int param = 0;
	for (int i = 0;i < n;++i)
	{
		switch (EdgeType(px[i], py[i], px[(i + 1) % n], py[(i + 1) % n],x,y))
		{
		case TOUCHING :
			return true;
		case CROSS_LEFT:
		case CROSS_RIGHT:
			param = 1 - param;
		}
	}
	return(param != 0);
}

bool PInPolygonNZWMode(int x, int y, int* px, int*py, int n)
{
	int param = 0;
	for (int i = 0;i < n;++i)
	{
		switch (EdgeType(px[i], py[i], px[(i + 1) % n], py[(i + 1) % n], x, y))
		{
		case TOUCHING:
			return true;
		case CROSS_LEFT:
			++param;
			break;
		case CROSS_RIGHT:
			--param;
			break;
		}
	}
	return(param != 0);
}

void DifficultyPolygon(int* x, int* y, int n)
{
	double tab;
	double tcd;
	int indexF = 0;
	int indexS = 1;
	int firstx, firsty, secondx, secondy,firstxt, firstyt, secondxt, secondyt;
	while (indexF != (n - 1))
	{
		firstx = x[indexF];
		firsty = y[indexF];
		secondx = x[indexS];
		secondy = y[indexS];
		for (int i = indexS + 1;i < n;++i)
		{
			if (indexF == 0 && i == (n - 1)) continue;
			firstxt = x[i];
			firstyt = y[i];
			if (i == (n - 1))
			{
				secondxt = x[0];
				secondyt = y[0];
			}
			else
			{
				secondxt = x[i+1];
				secondyt = y[i+1];
			}
			IntersectType result;
			result = Cross(firstx,firsty,secondx,secondy,firstxt,firstyt,secondxt,secondyt,&tab,&tcd);
			if (result == SKEW_CROSS)
			{
				gfDrawText(10, 10, "difficult", RGBPIXEL::DkMagenta());
				return;
			}
		}
		indexF++;
		indexS++;
	}
	gfDrawText(10, 10, "simple", RGBPIXEL::DkMagenta());
}

void ConvexPolygon(int* x, int* y, int n)
{
	int sumLeft, sumRight;
	for (int i = 0; i < n; ++i)
	{
		sumLeft = 0;
		sumRight = 0;
		for (int j = 0; j < n; ++j)
		{
			ClPointType result;
			if(i==(n-1))
				result = Classify(x[i], y[i], x[0], y[0], x[j], y[j]);
			else
				result = Classify(x[i], y[i], x[i + 1], y[i + 1], x[j], y[j]);
			if (result == LEFT) sumLeft++;
			if (result == RIGHT) sumRight++;
		}
		if (sumLeft != 0 && sumRight != 0)
		{
			gfDrawText(10, 30, "Not Convex", RGBPIXEL::DkMagenta());
			return;
		}
	}
	gfDrawText(10, 30, "Convex", RGBPIXEL::DkMagenta());
}

void DrawLine( int x0, int y0, int x1, int y1, RGBPIXEL color )
{
	bool check(Abs(x1 - x0)<Abs(y1 - y0));
	if (check) 
	{
		Swap(x0, y0);
		Swap(x1, y1);
	}
	if (x0 > x1)
	{
		Swap(x1, x0);
		Swap(y1, y0);
	}
	int dx = Abs(x1 - x0);
	int dy = Abs(y1 - y0);
	int d = 2*dy;
	int ystep = (y0 < y1) ? 1 : -1;
	int y = y0;
	for (int x = x0;x <= x1;++x)
	{
		gfSetPixel(check ? ((dx == 0) ? y0 : y) : ((dx == 0) ? x0 : x), check ? x : y, color);
		d += 2 * dy;
		if (d > dx)
		{
			d -= 2 * dx;
			y += ystep;
		}
	}
}

void DrawPolygon(int* x, int* y, int n, RGBPIXEL color)
{
	for (int i = 1; i < n; ++i)
	{
		DrawLine(x[i - 1], y[i - 1], x[i], y[i], color);
	}
	DrawLine(x[n-1], y[n-1], x[0], y[0], color);
}

int ArrMax(int* t, int n)
{
	int tMax = 0;
	for (int i = 0;i < n;++i)
	{
		if (t[i] > tMax) tMax = t[i];
	}
	return tMax;
}

int ArrMin(int* t, int n)
{
	int tMin = INT_MAX;
	for (int i = 0;i < n;++i)
	{
		if (t[i] < tMin) tMin = t[i];
	}
	return tMin;
}

void FillPolygon(int* x, int* y, int n, RGBPIXEL color)
{
	int xMax = ArrMax(x,n);
	int xMin = ArrMin(x,n);
	int yMax = ArrMax(y,n);
	int yMin = ArrMin(y,n);
	for (int i = xMin; i <= xMax;++i)
	{
		for (int j = yMin;j <= yMax;++j)
		{
			if(PInPolygonEOMode(i,j,x,y,n)) //EO
				gfSetPixel(i, j, color);
		}
	}
}

void DrawCircle(int x0, int y0, int radius, RGBPIXEL color)
{
	int x = radius;
	int y = 0;
	int radiusError = 1 - x;
	while (x >= y)
	{
		DrawLine(-x + x0, y + y0, x + x0, y + y0,color);
		DrawLine(-y + x0, x + y0, y + x0, x + y0, color);
		DrawLine(-x + x0, -y + y0, x + x0, -y + y0, color);
		DrawLine(-y + x0, -x + y0, y + x0, -x + y0, color);
		y++;
		if (radiusError < 0)
		{
			radiusError += 2 * y + 1;
		}
		else
		{
			x--;
			radiusError += 2 * (y - x + 1);
		}
	}
}

void RoundCap(int x0, int y0, int x1, int y1, int radius, RGBPIXEL color)
{
	int dx = x1 - x0;
	int dy = y1 - y0;
	double nx = -(double(dy)) / (sqrt(double(dx*dx + dy*dy)));
	double ny = (double(dx)) / (sqrt(double(dx*dx + dy*dy)));
	int mn;
	int x[1000];
	int y[1000];
	double angle;
	if (nx == 0) angle = (dx<0)?M_PI/2:-M_PI/2;
	else angle = atan(ny / nx);
	int num = 0;
	for (double i = angle; i <= (angle + M_PI); i += 0.01)
	{
		if (y0 > y1)
		{
			x[num] = x0 + round(radius*cos(i));
			y[num] = y0 + round(radius*sin(i));
		}
		else
		{
			x[num] = x0 - round(radius*cos(i));
			y[num] = y0 - round(radius*sin(i));
		}
		num++;
	}
	FillPolygon(x, y, num, color);
}

void DrawRoundCap(int x0, int y0, int x1, int y1, double width, RGBPIXEL color)
{
	RoundCap(x1, y1, x0, y0, (int(width/2)), color);
	RoundCap(x0, y0,x1,y1, (int(width / 2)), color);
}

void SquareCap(int x0, int y0, int x1, int y1, double width, RGBPIXEL color)
{
	int dx = x1 - x0;
	int dy = y1 - y0;
	double ny = (double(dx)) / (sqrt((double(dx*dx + dy*dy))));
	double nx = -(double(dy)) / (sqrt((double(dx*dx + dy*dy))));
	nx *= width / 2;
	ny *= width / 2;
	int rx[4];
	int ry[4];
	int n = 4;
	rx[0] = roundf(x0 + nx);
	ry[0] = roundf(y0 + ny);
	rx[1] = roundf(x1 + nx);
	ry[1] = roundf(y1 + ny);
	rx[2] = roundf(x1 - nx);
	ry[2] = roundf(y1 - ny);
	rx[3] = roundf(x0 - nx);
	ry[3] = roundf(y0 - ny);
	DrawPolygon(rx, ry, n, color);
	FillPolygon(rx, ry, n, color);
}

void DrawSquareCap(int x0, int y0, int x1, int y1, double width, RGBPIXEL color)
{	
	int dx = x1 - x0;
	int dy = y1 - y0;
	double ex = (double(dx)) / (sqrt((double(dx*dx + dy*dy))));
	double ey = (double(dy)) / (sqrt((double(dx*dx + dy*dy))));
	SquareCap(x0, y0, x0 - round(ex*(width / 2)), y0 - round(ey*(width / 2)), width, color);
	SquareCap(x1, y1, x1 + round(ex*(width / 2)), y1 + round(ey*(width / 2)), width, color);
}

void DrawLineWidth(int x0, int y0, int x1, int y1, double width, RGBPIXEL color)
{
	int dx = x1 - x0;
	int dy = y1 - y0;
	double ny = (double(dx)) / (sqrt((double(dx*dx + dy*dy))));
	double nx = -(double(dy)) / (sqrt((double(dx*dx + dy*dy))));
	nx *= width / 2;
	ny *= width / 2;
	int rx[4];
	int ry[4];
	int n = 4;
	rx[0] = roundf(x0 + nx);
	ry[0] = roundf(y0 + ny);
	rx[1] = roundf(x1 + nx);
	ry[1] = roundf(y1 + ny);
	rx[2] = roundf(x1 - nx);
	ry[2] = roundf(y1 - ny);
	rx[3] = roundf(x0 - nx);
	ry[3] = roundf(y0 - ny);
	DrawPolygon(rx, ry, n, color);
	FillPolygon(rx, ry, n, color);
	DrawLine(x0, y0, x1, y1, RGBPIXEL::White());
	//---------------------------------------------
	//TEST 2dop
	DrawRoundCap(x0, y0, x1, y1,width, color);
	//DrawSquareCap(x0, y0, x1, y1, width, color);
	//---------------------------------------------
}

void DrawDashLine(int x0, int y0, int x1, int y1, double width, std::vector<int> dash, RGBPIXEL color)
{
	int dx = x1 - x0;
	int dy = y1 - y0;
	double ex = (double(dx)) / (sqrt((double(dx*dx + dy*dy))));
	double ey = (double(dy)) / (sqrt((double(dx*dx + dy*dy))));
	int xSt = x0;
	int ySt = y0;
	int xFn = x0 + roundf(ex*dash[0]);
	int yFn = y0 + roundf(ey*dash[1]);
	int i = 0;
	int size = dash.size();
	int dlOrig = (dx*dx + dy*dy);
	int dlNow=0;
	int dxNow;
	int dyNow;
	while (dlNow<=dlOrig)
	{
		DrawLineWidth(xSt, ySt, xFn, yFn, width, color);
		xSt += round(ex*(dash[(i%size)] + dash[((i+1)%size)]));
		ySt += round(ey*(dash[(i%size)] + dash[((i + 1) % size)]));
		xFn += round(ex*(dash[((i+1)%size)] + dash[((i + 2) % size)]));
		yFn += round(ey*(dash[((i+1)%size)] + dash[((i + 2) % size)]));
		i += 2;
		dxNow = (xFn-x0);
		dyNow = (yFn-y0);
		dlNow = (dxNow*dxNow + dyNow*dyNow);
	}
}

void DrawBLine(std::vector<int> x, std::vector<int> y, RGBPIXEL color)
{
	int length = x.size();
	for (int i = 1;i < length;++i)
	{
		DrawLine(x[i-1], y[i-1], x[i], y[i], RGBPIXEL::DkBlue());
	}
	for (int i = 0; i < 2; ++i)
	{
		x.insert(x.begin(), x[0]);
		x.insert(x.end(), x[length]);
		y.insert(y.begin(), y[0]);
		y.insert(y.end(), y[length]);
		length = x.size();
	}
	int rX=0, rY=0, rXLast=0, rYLast=0;
	for (int i = 0; i < length -3;++i)
	{
		for (double t = 0; t <= 1;t += 0.01)
		{
			rXLast = rX;
			rYLast = rY;
			double firstX = (1 - t)*(1 - t)*(1 - t)*x[i] / 6;
			double secondX = (3 * t*t*t - 6 * t*t + 4)*x[i+1] / 6;
			double thirdX = (-3 * t*t*t + 3 * t*t + 3 * t + 1)*x[i+2] / 6;
			double fourthX = t*t*t*x[i+3] / 6;
			double firstY = (1 - t)*(1 - t)*(1 - t)*y[i] / 6;
			double secondY = (3 * t*t*t - 6 * t*t + 4)*y[i+1] / 6;
			double thirdY = (-3 * t*t*t + 3 * t*t + 3 * t + 1)*y[i+2] / 6;
			double fourthY = t*t*t*y[i+3] / 6;
			rX = round(firstX + secondX + thirdX + fourthX);
			rY = round(firstY + secondY + thirdY + fourthY);
			if (t == 0) continue;
			DrawLine(rXLast, rYLast, rX, rY, color);
		}
	}
}

void UnionTwoCurvesPoligon(int* x1, int* y1, int n1, int* x2, int* y2, int n2, RGBPIXEL color)
{
	FillPolygon(x1, y1, n1, RGBPIXEL::DkGreen());
	FillPolygon(x2, y2, n2, RGBPIXEL::DkBlue());
	std::vector<int> UnionX;
	std::vector<int> UnionY;
	ClPointType tempCla;
	ClPointType tempClb;
	std::vector<int> px(x2, x2 + n2);
	std::vector<int> py(y2, y2 + n2);
	for (int i = 0;i < n1;++i)
	{
		for (int j = 0; j < px.size(); ++j)
		{
			double tab, tcd;
			tempCla = Classify(x1[i], y1[i], x1[(i + 1) % n1], y1[(i + 1) % n1], px[j], py[j]);
			tempClb = Classify(x1[i], y1[i], x1[(i + 1) % n1], y1[(i + 1) % n1], px[(j + 1) % px.size()], py[(j + 1) % px.size()]);
			IntersectType tempCross = Cross(x1[i], y1[i], x1[(i + 1) % n1], y1[(i + 1) % n1], px[j], py[j], px[(j + 1) % px.size()], py[(j + 1) % px.size()], &tab, &tcd);
			if (tempCla == RIGHT && tempClb == RIGHT)
			{
				UnionX.push_back(px[(j + 1) % px.size()]);
				UnionY.push_back(py[(j + 1) % px.size()]);
				continue;
			}
			if (tempClb==RIGHT)
			{
					int pxt = x1[i] + tab*(x1[(i + 1) % n1] - x1[i]);
					int pyt = y1[i] + tab*(y1[(i + 1) % n1] - y1[i]);
					UnionX.push_back(pxt);
					UnionY.push_back(pyt);
					UnionX.push_back(px[(j + 1) % px.size()]);
					UnionY.push_back(py[(j + 1) % px.size()]);
					continue;
			}
			if (tempCla == RIGHT)
			{
					int pxt = x1[i] + tab*(x1[(i + 1) % n1] - x1[i]);
					int pyt = y1[i] + tab*(y1[(i + 1) % n1] - y1[i]);
					UnionX.push_back(pxt);
					UnionY.push_back(pyt);
					continue;
			}
		}
		px.clear();
		py.clear();
		px = UnionX;
		py = UnionY;
		if (px.size() == 0) return;
		UnionX.clear();
		UnionY.clear();
	}

	int* x = &px[0];
	int* y = &py[0];
	FillPolygon (x, y, px.size(), RGBPIXEL::DkMagenta());
}

bool ClipLine(int &xa, int &ya, int &xb, int &yb, int* px, int* py, int n)
{
	double t0 = 0.0;
	double t1 = 1.0;
	double sx = xb - xa;
	double sy = yb - ya;
	for (int i = 0; i < n; ++i)
	{
		double nx = -py[(i + 1) % n] + py[i];
		double ny = px[(i + 1) % n] - px[i];
		double denom = nx*sx + ny*sy;
		double num = nx*(xa - px[i]) + ny*(ya - py[i]);
		if (denom != 0.0)
		{
			double t = -num / denom;
			if (denom > 0.0)
			{
				if (t > t0) t0 = t;
			}
			else
			{
				if (t < t1) t1 = t;
			}
		}
		else
		{
			if (num < 0.0) return false;
		}
	}
	if (t0 <= t1)
	{
		double x1 = xa + t0*(xb - xa);
		double y1 = ya + t0*(yb - ya);
		double x2 = xa + t1*(xb - xa);
		double y2 = ya + t1*(yb - ya);
		xa = x1;
		ya = y1;
		xb = x2;
		yb = y2;
		return true;
	}
	return false;
}

int Dist(int x, int y)
{
	return(abs(x) + abs(y));
}

void Draw3Beize(int* px, int* py, int n, RGBPIXEL color)
{
	/*for (int i = 1; i <= n; ++i)
	{
		DrawLine(px[i - 1], py[i - 1], px[i], py[i], RGBPIXEL::Green());
	}*/
	int D = max(Dist(px[0] - 2 * px[1] + px[2], py[0] - 2 * py[1] + py[2]), Dist(px[0] - 2 * px[1] + px[3], py[0] - 2 * py[1] + py[3]));
	double N = 1 + sqrt(3 * (double(D)));
	int rx = 0, ry = 0, rxLast = 0, ryLast = 0;
	for (double t = 0.0; t <= 1.0; t += pow(2.0,round(log2f((1.0 / N)))))
	{
		rxLast = rx;
		ryLast = ry;
		double firstx = (1.0 - t)*(1.0 - t)*(1.0 - t)*px[0];
		double secondx = 3 * t*(1.0 - t)*(1.0 - t)*px[1];
		double thirdx = 3 * t*t*(1.0 - t)*px[2];
		double fourthx = t*t*t*px[3];
		double firsty = (1.0 - t)*(1.0 - t)*(1.0 - t)*py[0];
		double secondy = 3 * t*(1.0 - t)*(1.0 - t)*py[1];
		double thirdy = 3 * t*t*(1.0 - t)*py[2];
		double fourthy = t*t*t*py[3];
		rx = round(firstx + secondx + thirdx + fourthx);
		ry = round(firsty + secondy + thirdy + fourthy);
		if (t == 0.0) continue;
		DrawLine(rxLast, ryLast, rx, ry, color);
	}
}

double fact(int n)
{
	double res = 1;
	for (int i = 1; i <= n;++i)
	{
		res *= i;
	}
	return res;
}

double cnk(int n, int k)
{
	return fact(n) / (fact(k)*fact(n - k));
}

void DrawNBeize(int* px, int* py, int n, RGBPIXEL color)
{
	for (int i = 1; i <= n; ++i)
	{
		DrawLine(px[i - 1], py[i - 1], px[i], py[i], RGBPIXEL::Green());
	}
	int rx = 0, ry = 0, rxLast = 0, ryLast = 0;
	for (double t = 0.0; t <= 1.0; t += 1.0/32)
	{
		rxLast = rx;
		ryLast = ry;
		rx = 0;
		ry = 0;
		for (int j = 0;j <= n; ++j)
		{
			rx += cnk(n, j)*pow(t, j)*pow(1 - t, n-j)*px[j];
			ry += cnk(n, j)*pow(t, j)*pow(1 - t, n-j)*py[j];
		}
		if (t == 0.0) continue;
		DrawLine(rxLast, ryLast, rx, ry, color);
	}
}

struct Coor3D
{
	double x;
	double y;
	double z;
	Coor3D() {}
	Coor3D(double x, double y, double z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}
};

Coor3D Multiple3DCoorByParallelMat(Coor3D point, int n)
{
	double T[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,0,0 },{ 0,0,n,1 }

	};
	double oldCoor[4] = { point.x,point.y,point.z,1 };
	double newCoor[4] = { 0,0,0,0 };
	for (unsigned int i = 0; i < 4;++i)
	{
		for (unsigned int j = 0; j < 4;++j)
		{
			newCoor[i] += oldCoor[j] * T[j][i];
		}
	}
	Coor3D temp;
	temp.x = newCoor[0];temp.y = newCoor[1];temp.z = newCoor[2];
	return temp;
}

void Rotate(std::vector<Coor3D> &cube, double angle, Coor3D	axis)
{
	double Tr[4][4];
	double zn = sqrt(axis.x*axis.x + axis.y*axis.y +axis.z*axis.z);
	axis.x /= zn;
	axis.y /= zn;
	axis.z /= zn;
	Tr[0][0] = axis.x*axis.x + (1 - axis.x*axis.x)*cos(angle);
	Tr[0][1] = axis.x*axis.y*(1 - cos(angle)) + axis.z*sin(angle);
	Tr[0][2] = axis.x*axis.z*(1 - cos(angle)) - axis.y*sin(angle);
	Tr[0][3] = 0;
	Tr[1][0] = axis.x*axis.y*(1 - cos(angle)) - axis.z*sin(angle);
	Tr[1][1] = axis.y*axis.y + (1 - axis.y*axis.y)*cos(angle);
	Tr[1][2] = axis.y*axis.z*(1 - cos(angle)) + axis.x*sin(angle);
	Tr[1][3] = 0;
	Tr[2][0] = axis.x*axis.z*(1 - cos(angle)) + axis.y*sin(angle);
	Tr[2][1] = axis.y*axis.z*(1 - cos(angle)) - axis.x*sin(angle);
	Tr[2][2] = axis.z*axis.z + (1 - axis.z*axis.z)*cos(angle);
	Tr[2][3] = 0;
	Tr[3][0] = 0;
	Tr[3][1] = 0;
	Tr[3][2] = 0;
	Tr[3][3] = 1;
	for (unsigned int i = 0;i < cube.size();++i)
	{
		double oldCoor[4] = { cube[i].x,cube[i].y,cube[i].z,1

		};
		double newCoor[4] = { 0,0,0,0 };
		for (unsigned int j = 0; j < 4;++j)
		{
			for (unsigned int k = 0; k < 4;++k)
			{
				newCoor[j] += oldCoor[k] * Tr[k][j];
			}
		}
		Coor3D temp(newCoor[0], newCoor[1], newCoor[2]);
		cube[i] = temp;
	}
}

void DrawCube(std::vector<std::vector<Coor3D>> &facets, RGBPIXEL color)
{
	/*DrawLine(cube[0].x, cube[0].y, cube[1].x, cube[1].y,color);
	DrawLine(cube[0].x, cube[0].y, cube[3].x, cube[3].y, color);
	DrawLine(cube[0].x, cube[0].y, cube[4].x, cube[4].y, color);
	DrawLine(cube[1].x, cube[1].y, cube[2].x, cube[2].y, color);
	DrawLine(cube[1].x, cube[1].y, cube[5].x, cube[5].y, color);
	DrawLine(cube[2].x, cube[2].y, cube[3].x, cube[3].y, color);
	DrawLine(cube[2].x, cube[2].y, cube[6].x, cube[6].y, color);
	DrawLine(cube[3].x, cube[3].y, cube[7].x, cube[7].y, color);
	DrawLine(cube[4].x, cube[4].y, cube[5].x, cube[5].y, color);
	DrawLine(cube[5].x, cube[5].y, cube[6].x, cube[6].y, color);
	DrawLine(cube[6].x, cube[6].y, cube[7].x, cube[7].y, color);
	DrawLine(cube[7].x, cube[7].y, cube[4].x, cube[4].y,color);*/
	for (unsigned int i = 0; i < facets.size();++i)
	{
		for (unsigned int j = 0; j < facets[i].size();++j)
		{
			DrawLine(facets[i][j].x, facets[i][j].y,facets[i][(j + 1) % facets[i].size()].x, facets[i][(j + 1) % facets[i].size()].y, color);
		}
	}
}

void ParallelProection(std::vector<std::vector<Coor3D>> &facets, int n, RGBPIXEL color)
{
	for (unsigned int i = 0; i < facets.size();++i)
	{
		for (unsigned int j = 0; j < facets[i].size();++j)
		{
			facets[i][j] = Multiple3DCoorByParallelMat(facets[i][j], n);
		}
	}
	DrawCube(facets, RGBPIXEL::Green());;
}

Coor3D MultipleCoor(Coor3D Coor, double T[4][4])
{
	double oldCoor[4] = { Coor.x,Coor.y ,Coor.z,1 };
	double newCoor[4] = { 0,0,0,0 };
	for (unsigned int l = 0; l < 4;++l)
	{
		for (unsigned int k = 0; k < 4;++k)
		{
			newCoor[l] += oldCoor[k] * T[k][l];
		}
	}
	for (unsigned int l = 0; l < 3;++l)
	{
		newCoor[l] /= newCoor[3];
	}
	Coor3D temp(newCoor[0], newCoor[1], newCoor[2]);
	return temp;
}

void PerspectiveProection(std::vector<std::vector<Coor3D>> &facets,Coor3D pos, RGBPIXEL color, Coor3D Cmass)
{
	double Tt[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ -Cmass.x,-Cmass.y,-Cmass.z,1 } };
	double Twh[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{Cmass.x,Cmass.y,Cmass.z,1 } };
	double T[4][4] = { { 1,0,0,pos.x != 0 ? 1 / pos.x : 0 },{ 0,1,0,pos.y!= 0 ? 1 / pos.y : 0 },{ 0,0,1,pos.z != 0 ? 1 / pos.z : 0 },{ 0,0,0,1 } };
	for (unsigned int i = 0; i < facets.size();++i)
	{
		for (unsigned int j = 0; j < facets[i].size();++j)
		{
			//facets[i][j] = MultipleCoor(facets[i][j],Tt);
			facets[i][j] = MultipleCoor(facets[i][j], T);
			//facets[i][j] = MultipleCoor(facets[i][j], Twh);
		}
	}
	//DrawCube(facets, color);
}

std::vector<Coor3D> getnormal(std::vector<std::vector<Coor3D>>&facets)
{
	std::vector<Coor3D> normals(facets.size());
	for (unsigned int i = 0; i < facets.size();++i)
	{
		Coor3D a, b;
		a.x = facets[i][1].x - facets[i][0].x;
		a.y = facets[i][1].y - facets[i][0].y;
		a.z = facets[i][1].z - facets[i][0].z;
		b.x = facets[i][3].x - facets[i][0].x;
		b.y = facets[i][3].y - facets[i][0].y;
		b.z = facets[i][3].z - facets[i][0].z;
		normals[i].x = a.y*b.z - a.z*b.y;
		normals[i].y = a.z*b.x - a.x*b.z;
		normals[i].z = a.x*b.y - a.y*b.x;
	}
	return normals;
}

void dellfacetParalel(std::vector<std::vector<Coor3D>> &facets,Coor3D V)
{
	std::vector < Coor3D >normals = getnormal(facets);
	int j = 0;
	for (int i = 0; i<normals.size();++i)
	{
		if (normals[i].x*V.x + normals[i].y*V.y + normals[i].z*V.z >= 0)
		{
			facets.erase(facets.begin() + j);
		}
		else
		{
			j++;
		}
	}
}

void dellfacetPerspective(std::vector<std::vector<Coor3D>> &facets,Coor3D S)
{
	std::vector < Coor3D >normals = getnormal(facets);
	int j = 0;
	for (int i = 0; i<normals.size();++i)
	{
		Coor3D C((facets[j][0].x + facets[j][2].x) / 2,(facets[j][0].y + facets[j][2].y) / 2, (facets[j][0].z + facets[j][2].z) / 2);
		if (normals[i].x*(C.x + S.x) + normals[i].y*(C.y + S.y) +normals[i].z*(C.z + S.z)>0)
		{
			facets.erase(facets.begin() + j);
		}
		else
		{
			j++;
		}
	}
}

Coor3D ParallelTransform(Coor3D start, Coor3D point)
{
	double Tt[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ point.x,point.y,point.z,1 } };
	double oldCoor[4] = { start.x,start.y,start.z,1 };
	double newCoor[4] = { 0,0,0,0 };
	for (unsigned int i = 0; i < 4;++i)
	{
		for (unsigned int j = 0; j < 4;++j)
		{
			newCoor[i] += oldCoor[j] * Tt[j][i];
		}
	}
	return Coor3D(newCoor[0], newCoor[1], newCoor[2]);
}

void transform(const int* px, const int* py, std::vector<Coor3D>&cube, const double V, const Coor3D one, const Coor3D two,
	const Coor3D three, const Coor3D four, const Coor3D five,const Coor3D six, const Coor3D seven, const Coor3D eight)
{
	Coor3D Cmass((cube[0].x + cube[6].x) / 2, (cube[0].y + cube[6].y) / 2, (cube[0].z + cube[6].z) / 2);
	static int rx = 0, ry = 0, rxLast = 0, ryLast = 0;
	static double t = 0.0;
	rxLast = rx;
	ryLast = ry;
	double firstx = (1.0 - t)*(1.0 - t)*(1.0 - t)*px[0];
	double secondx = 3 * t*(1.0 - t)*(1.0 - t)*px[1];
	double thirdx = 3 * t*t*(1.0 - t)*px[2];
	double fourthx = t*t*t*px[3];
	double firsty = (1.0 - t)*(1.0 - t)*(1.0 - t)*py[0];
	double secondy = 3 * t*(1.0 - t)*(1.0 - t)*py[1];
	double thirdy = 3 * t*t*(1.0 - t)*py[2];
	double fourthy = t*t*t*py[3];
	rx = round(firstx + secondx + thirdx + fourthx);
	ry = round(firsty + secondy + thirdy + fourthy);
	if (t == 0.0)
	{
		t += 1 / V;
		return;
	}
	if (t > 1)
	{
		rx = 0, ry = 0, rxLast = 0, ryLast = 0;
		t = 0;
		cube.clear();
		cube = { one,two,three,four,five,six,seven,eight };
		return;
	}
	double dx = rx - rxLast;
	double dy = ry - ryLast;
	double dz = 0.0;
	for (unsigned int i = 0;i < cube.size();++i)
	{
		Coor3D dr(dx, dy, dz);
		cube[i] = ParallelTransform(cube[i], dr);
	}
	t += 1 / V;
}

struct Light
{
	RGBPIXEL color;
	double I;
	Coor3D point;
	Light(RGBPIXEL color, Coor3D point, double I)
	{
		this->color = color;
		this->point = point;
		this->I = I;
	}
};

bool comparer(const Coor3D &a, const Coor3D &b) {
	return a.y < b.y;
}

struct Triangle
{
	Coor3D first;
	Coor3D second;
	Coor3D third;
	Triangle(Coor3D first, Coor3D second, Coor3D third)
	{
		std::vector<Coor3D> Points = { first,second,third };
		std::sort(Points.begin(), Points.end(), comparer);
		this->first = Points[0];
		this->second = Points[1];
		this->third = Points[2];
	}
};

void MethodGuro(Triangle trSide, double kd, Coor3D norm,Light source)
{
	double Ia = 0, Ib = 0, Ic = 0;
	double I1, I2, I;
	double y, x;
	double x1, x2, t1, t2, t;
	int R, G, B;
	double dl = sqrt(norm.x*norm.x + norm.y*norm.y + norm.z*norm.z);
	norm.x /= dl;norm.y /= dl;norm.z /= dl;
	Coor3D V1(norm.x, norm.y,norm.z);
	Coor3D V2(source.point.x-(trSide.first.x+ trSide.second.x+ trSide.third.x)/3, source.point.y - (trSide.first.y + trSide.second.y + trSide.third.y) / 3, source.point.z - (trSide.first.z + trSide.second.z + trSide.third.z) / 3);
	double cosTeta = (V1.x*V2.x + V1.y*V2.y + V1.z*V2.z) / (sqrt(V1.x*V1.x+ V1.y*V1.y+ V1.z*V1.z)*sqrt(V2.x*V2.x+ V2.y*V2.y+ V2.z*V2.z));
	double da = sqrt((trSide.first.x - source.point.x)*(trSide.first.x - source.point.x) + (trSide.first.y - source.point.y)*(trSide.first.y - source.point.y) + (trSide.first.z - source.point.z)*(trSide.first.z - source.point.z));
	double db = sqrt((trSide.second.x - source.point.x)*(trSide.second.x - source.point.x) + (trSide.second.y - source.point.y)*(trSide.second.y - source.point.y) + (trSide.second.z - source.point.z)*(trSide.second.z - source.point.z));
	double dc = sqrt((trSide.third.x - source.point.x)*(trSide.third.x - source.point.x) + (trSide.third.y - source.point.y)*(trSide.third.y - source.point.y) + (trSide.third.z - source.point.z)*(trSide.third.z - source.point.z));
	Ia = (source.I*kd*cosTeta) /(da + 1);
	Ib = (source.I*kd*cosTeta) /(db + 1);
	Ic = (source.I*kd*cosTeta)/ (dc + 1);
	for (y = trSide.first.y;y <= trSide.third.y;y++)
	{
		if (trSide.third.y == trSide.second.y)
		{
			x1 = ((y - trSide.first.y) / (trSide.second.y - trSide.first.y))*(trSide.second.x - trSide.first.x) + trSide.first.x;
			x2 = ((y - trSide.first.y) / (trSide.third.y - trSide.first.y))*(trSide.third.x - trSide.first.x) + trSide.first.x;
			t1 = fabs((trSide.first.y - y) / (trSide.second.y - trSide.first.y));
			t2 = sqrt((x2 - trSide.first.x)*(x2 - trSide.first.x) + (y - trSide.first.y)*(y - trSide.first.y)) / sqrt((trSide.first.x - trSide.third.x)*(trSide.first.x - trSide.third.x) + (trSide.first.y - trSide.third.y)*(trSide.first.y - trSide.third.y));
			I1 = t1*Ia + (1 - t1)*Ib;
			I2 = t2*Ia + (1 - t2)*Ic;
		}
		else
			if (trSide.first.y == trSide.second.y)
			{
				x1 = ((y - trSide.first.y) / (trSide.third.y - trSide.first.y))*(trSide.third.x - trSide.first.x) + trSide.first.x;
				x2 = ((y - trSide.second.y) / (trSide.third.y - trSide.second.y))*(trSide.third.x - trSide.second.x) + trSide.second.x;
				t1 = sqrt((x1 - trSide.first.x)*(x1 - trSide.first.x) + (y - trSide.first.y)*(y - trSide.first.y)) / sqrt((trSide.first.x - trSide.third.x)*(trSide.first.x - trSide.third.x) + (trSide.first.y - trSide.third.y)*(trSide.first.y - trSide.third.y));
				t2 = fabs((trSide.second.y - y) / (trSide.third.y - trSide.second.y));
				I1 = t1*Ia + (1 - t1)*Ic;
				I2 = t2*Ib + (1 - t2)*Ic;
			}
			else
			{
				if (y < trSide.second.y)
				{
					x1 = ((y - trSide.first.y) / (trSide.second.y - trSide.first.y))*(trSide.second.x - trSide.first.x) + trSide.first.x;
					x2 = ((y - trSide.first.y) / (trSide.third.y - trSide.first.y))*(trSide.third.x - trSide.first.x) + trSide.first.x;
					t1 = sqrt((x1 - trSide.first.x)*(x1 - trSide.first.x) + (y - trSide.first.y)*(y - trSide.first.y)) / sqrt((trSide.first.x - trSide.second.x)*(trSide.first.x - trSide.second.x) + (trSide.first.y - trSide.second.y)*(trSide.first.y - trSide.second.y));
					t2 = sqrt((x2 - trSide.first.x)*(x2 - trSide.first.x) + (y - trSide.first.y)*(y - trSide.first.y)) / sqrt((trSide.first.x - trSide.third.x)*(trSide.first.x - trSide.third.x) + (trSide.first.y - trSide.third.y)*(trSide.first.y - trSide.third.y));
					I1 = t1*Ia + (1 - t1)*Ib;
					I2 = t2*Ia + (1 - t2)*Ic;
				}
				else
				{
					x1 = ((y - trSide.second.y) / (trSide.third.y - trSide.second.y))*(trSide.third.x - trSide.second.x) + trSide.second.x;
					x2 = ((y - trSide.first.y) / (trSide.third.y - trSide.first.y))*(trSide.third.x - trSide.first.x) + trSide.first.x;
					t1 = sqrt((x1 - trSide.second.x)*(x1 - trSide.second.x) + (y - trSide.second.y)*(y - trSide.second.y)) / sqrt((trSide.second.x - trSide.third.x)*(trSide.second.x - trSide.third.x) + (trSide.second.y - trSide.third.y)*(trSide.second.y - trSide.third.y));
					t2 = sqrt((x2 - trSide.first.x)*(x2 - trSide.first.x) + (y - trSide.first.y)*(y - trSide.first.y)) / sqrt((trSide.first.x - trSide.third.x)*(trSide.first.x - trSide.third.x) + (trSide.first.y - trSide.third.y)*(trSide.first.y - trSide.third.y));
					I1 = t1*Ib + (1 - t1)*Ic;
					I2 = t2*Ia + (1 - t2)*Ic;
				}
			}
		if (x1 > x2) { Swap(x1, x2); Swap(I1, I2); }
		for (x = x1;x <=x2;x++)
		{	
			t = fabs((x - x1) / (x2 - x1));
			I = t*I1 + (1 - t)*I2;
			if(I>1)I=1;
			if(I<0)I=0;
			R = int(source.color.red*I);
			G = int(source.color.green*I);
			B = int(source.color.blue*I);
			gfSetPixel(x, y, RGBPIXEL(R,G,B));
		}
	}
}

void paintGuro(std::vector<std::vector<Coor3D>> &facets, Light source, double kd)
{
	std::vector<Coor3D> normals = getnormal(facets);
	for (unsigned int i = 0; i < facets.size();++i)
	{
		for (unsigned int j = 1;j < facets[i].size() - 1;++j)
		{
			Triangle temp(facets[i][0], facets[i][j], facets[i][j + 1]);
			MethodGuro(temp, kd,normals[i],source);
		}
	}
}

void RotateFigure(Coor3D (&R)[4][4], double angle, Coor3D	axis)
{
	double Tr[4][4];
	double zn = sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
	axis.x /= zn;
	axis.y /= zn;
	axis.z /= zn;
	Tr[0][0] = axis.x*axis.x + (1 - axis.x*axis.x)*cos(angle);
	Tr[0][1] = axis.x*axis.y*(1 - cos(angle)) + axis.z*sin(angle);
	Tr[0][2] = axis.x*axis.z*(1 - cos(angle)) - axis.y*sin(angle);
	Tr[0][3] = 0;
	Tr[1][0] = axis.x*axis.y*(1 - cos(angle)) - axis.z*sin(angle);
	Tr[1][1] = axis.y*axis.y + (1 - axis.y*axis.y)*cos(angle);
	Tr[1][2] = axis.y*axis.z*(1 - cos(angle)) + axis.x*sin(angle);
	Tr[1][3] = 0;
	Tr[2][0] = axis.x*axis.z*(1 - cos(angle)) + axis.y*sin(angle);
	Tr[2][1] = axis.y*axis.z*(1 - cos(angle)) - axis.x*sin(angle);
	Tr[2][2] = axis.z*axis.z + (1 - axis.z*axis.z)*cos(angle);
	Tr[2][3] = 0;
	Tr[3][0] = 0;
	Tr[3][1] = 0;
	Tr[3][2] = 0;
	Tr[3][3] = 1;
	for (unsigned int i = 0;i < 4;++i)
	{
		for (unsigned int j = 0;j < 4;++j)
		{
			double oldCoor[4] = { R[i][j].x,R[i][j].y,R[i][j].z,1};
			double newCoor[4] = { 0,0,0,0 };
			for (unsigned int l = 0; l < 4;++l)
			{
				for (unsigned int k = 0; k < 4;++k)
				{
					newCoor[l] += oldCoor[k] * Tr[k][l];
				}
			}
			Coor3D temp(newCoor[0], newCoor[1], newCoor[2]);
			R[i][j] = temp;
		}
	}
}

double calcN(double k, int num)
{
	switch (num)
	{
	case 0:
	{
		return (1 - k)*(1 - k)*(1 - k) / 6;
	}
	case 1:
	{
		return (3 * k*k*k - 6 * k*k + 4) / 6;
	}
	case 2:
	{
		return (-3*k*k*k + 3*k*k + 3 *k + 1) / 6;
	}
	case 3:
	{
		return (k*k*k) / 3;
	}
	}
}

Coor3D BSurface(Coor3D P[4][4], double u, double v)
{
	Coor3D R(0,0,0);
	for (unsigned int i = 0; i < 4;++i)
	{
		for (unsigned int j = 0; j < 4;++j)
		{
			R.x += calcN(u, i) * calcN(v, j) * P[i][j].x;
			R.y += calcN(u, i) * calcN(v, j) * P[i][j].y;
			R.z += calcN(u, i) * calcN(v, j) * P[i][j].z;
		}
	}
	R.x = round(R.x);
	R.y = round(R.y);
	R.z = round(R.z);
	return R;
}


//void DrawLines(Coor3D p0,Coor3D p1, RGBPIXEL color)
//{
//	double temp;
//	bool check(Abs(p1.x - p0.x)<Abs(p1.y - p0.y));
//	if (check)
//	{
//		temp = p0.x;p0.x = p0.y;p0.y = p0.x;
//		temp = p1.x;p1.x = p1.y;p1.y = p1.x;
//	}
//	if (p0.x > p1.x)
//	{
//		Swap(p0, p1);
//	}
//	int dx = Abs(p1.x - p0.x);
//	int dy = Abs(p1.y - p0.y);
//	int d = 2 * dy;
//	int ystep = (p0.y < p1.y) ? 1 : -1;
//	int y = p0.y;
//	for (int x = p0.x;x <= p1.x;++x)
//	{
//		if (check)
//		{
//			if (dx == 0)
//			{
//				int temp = round(p0.y);
//				if (x < YMin[temp]) 
//				{
//					gfSetPixel(temp, x, RGBPIXEL::Gray());
//					YMin[temp] = x;
//				}
//				if (x > YMax[temp]) {
//					gfSetPixel(temp, x, RGBPIXEL::Gray());
//					YMax[temp] = x;
//				}
//				//gfSetPixel(p0.y, x, color);
//			}
//			else
//			{
//				if (x < YMin[y])
//				{
//					gfSetPixel(y, x, RGBPIXEL::Gray());
//					YMin[y] = x;
//				}
//				if (x > YMax[y]) {
//					gfSetPixel(y, x, RGBPIXEL::Gray());
//					YMax[y] = x;
//				}
//				//gfSetPixel(y, x, color);
//			}
//		}
//		else
//		{
//			if (dx == 0)
//			{
//				int temp = round(p0.x);
//				if (y < YMin[temp])
//				{
//					gfSetPixel(temp, y, RGBPIXEL::Gray());
//					YMin[temp] = y;
//				}
//				if (y > YMax[temp]) {
//					gfSetPixel(temp, y, RGBPIXEL::Gray());
//					YMax[temp] = y;
//				}
//				//gfSetPixel(p0.x, y, color);
//			}
//			else
//			{
//				if (y < YMin[x])
//				{
//					gfSetPixel(x, y, RGBPIXEL::Gray());
//					YMin[x] = y;
//				}
//				if (y > YMax[x]) {
//					gfSetPixel(x, y, RGBPIXEL::Gray());
//					YMax[x] = y;
//				}
//				//gfSetPixel(x, y, color);
//			}
//		}
//		d += 2 * dy;
//		if (d > dx)
//		{
//			d -= 2 * dx;
//			y += ystep;
//		}
//	}
//}


void DrawLines(Coor3D &p1, Coor3D &p2, int* YMin, int* YMax)
{
	int dx = abs(p2.x - p1.x);
	int dy = abs(p2.y - p1.y);
	int sx = p2.x >= p1.x ? 1 : -1;
	int sy = p2.y >= p1.y ? 1 : -1;
	if (dy <= dx) {
		int d = -dx;
		int d1 = dy << 1;
		int d2 = (dy - dx) << 1;
		for (int x = p1.x, y = p1.y, i = 0; i <= dx; i++, x += sx)
		{
			if (YMin[x] == NO_VALUE) {
				gfSetPixel(x, y, RGBPIXEL::Gray());
				YMin[x] = YMax[x] = y;
			}
			else if (y < YMin[x]) {
				gfSetPixel(x, y, RGBPIXEL::Gray());
				YMin[x] = y;
			}
			else if (y > YMax[x]) {
				gfSetPixel(x, y, RGBPIXEL::Gray());
				YMax[x] = y;
			}
			if (d > 0) {
				d += d2;
				y += sy;
			}
			else d += d1;
		}
	}
	else
	{
		int d = -dy;
		int d1 = dx << 1;
		int d2 = (dx - dy) << 1;
		int temp = round(p1.x);
		int m1 = YMin[temp];
		int m2 = YMax[temp];
		for (int x = temp, y = p1.y, i = 0; i <= dy; i++, y += sy)
		{
			if (YMin[x] == NO_VALUE) {
				gfSetPixel(x, y, RGBPIXEL::Gray());
				YMin[x] = YMax[x] = y;
			}
			else if (y < m1) {
				gfSetPixel(x, y, RGBPIXEL::Gray());
				if (y < YMin[x])
					YMin[x] = y;
			}
			else if (y > m2) {
				gfSetPixel(x, y, RGBPIXEL::Gray());
				if (y > YMax[x])
					YMax[x] = y;
			}
			if (d > 0) {
				d += d2;
				x += sx;
				m1 = YMin[x]; m2 = YMax[x];
			}
			else d += d1;
		}
	}
}


void floatingHorizon(Coor3D P[4][4])
{
	int YMax[640];
	int YMin[640];
	for (unsigned int i = 0; i < 640; i++)
	{
		YMin[i] = NO_VALUE;
		YMax[i] = NO_VALUE;
	}
	Coor3D prevR,R,nextR;
	
	for (double u = 0; u <= 1; u += 1.0/25)
	{
		for (double v = 0; v <= 1; v += 1.0/25)
		{
			R = BSurface(P, u, v);
			if (0 != u)
			{
				nextR = BSurface(P, u - 1.0 / 25, v);
				DrawLines(R, nextR, YMin, YMax);
			}
			if (v == 0)
			{
				prevR = R;
				continue;
			}
			DrawLines(prevR, R,YMin,YMax);
			prevR = R;
		}
	}
}

// Вызывается один раз в самом начале при инициализации приложения
bool gfInitScene()
{
	gfSetWindowSize(640, 480);

	//gfDrawRectangle( 100, 120, 170, 150, RGBPIXEL(255, 255, 0) );

	//gfDrawText( 200, 200, "Hello World", RGBPIXEL(0, 128, 255));

	//---------------------------------------------
	//TEST 1
	/*
	DrawLine(320, 240, 340, 230, RGBPIXEL::RandomColor());
	DrawLine(320, 240, 330, 220, RGBPIXEL::RandomColor());
	DrawLine(320, 240, 310, 220, RGBPIXEL::RandomColor());
	DrawLine(320, 240, 300, 230, RGBPIXEL::RandomColor());
	DrawLine(320, 240, 300, 250, RGBPIXEL::RandomColor());
	DrawLine(320, 240, 310, 260, RGBPIXEL::RandomColor());
	DrawLine(320, 240, 330, 260, RGBPIXEL::RandomColor());
	DrawLine(320, 240, 340, 250, RGBPIXEL::RandomColor());
	DrawLine(320, 240, 320, 260, RGBPIXEL::White());
	DrawLine(320, 240, 300, 240, RGBPIXEL::White());
	DrawLine(320, 240, 320, 220, RGBPIXEL::White());
	DrawLine(320, 240, 340, 240, RGBPIXEL::White());
	*/
	//----------------------------------------------

	//---------------------------------------------
	//TEST 2
	/*
	int n = 4;
	int xp[10] = { 10,300,300,10 };
	int yp[10] = { 10,10,300,300 };
	DrawPolygon(xp, yp, n, RGBPIXEL::Red());
	*/
	/*
	int n = 6;
	int xp[10] = { 100,300,300,200, 200, 150 };
	int yp[10] = { 100,300,100,400, 50, 250  };
	DrawPolygon(xp, yp, n, RGBPIXEL::Red());
	*/
	//---------------------------------------------

	//---------------------------------------------
	//TEST 3
	/*
	int n = 4;
	int xp[10] = { 10,300,300,10 };
	int yp[10] = { 10,10,300,300 };
	DrawPolygon(xp, yp, n, RGBPIXEL::Red());
	DifficultyPolygon(xp, yp, n);
	ConvexPolygon(xp, yp, n);
	*/

	/*
	int n = 6;
	int xp[10] = { 100,300,300,200, 200, 150 };
	int yp[10] = { 100,300,100,400, 50, 250  };
	DrawPolygon(xp, yp, n, RGBPIXEL::Red());
	DifficultyPolygon(xp, yp, n);
	ConvexPolygon(xp, yp, n);
	*/

	/*int n = 5;
	int xp[10] = { 10,300,300, 150,10 };
	int yp[10] = { 10,10,300,120, 300 };
	DrawPolygon(xp, yp, n, RGBPIXEL::Red());
	DifficultyPolygon(xp, yp, n);
	ConvexPolygon(xp, yp, n);
	*/
	//---------------------------------------------

	//---------------------------------------------
	//TEST 4
	/*
	int n = 4;
	int xp[10] = { 10,300,300,10 };
	int yp[10] = { 10,10,300,300 };
	DrawPolygon(xp, yp, n, RGBPIXEL::Red());
	FillPolygon(xp, yp, n, RGBPIXEL::Red());
	*/
	/*
	int n = 6;
	int xp[10] = { 100,300,300,200, 200, 150 };
	int yp[10] = { 100,300,100,400, 50, 250  };
	DrawPolygon(xp, yp, n, RGBPIXEL::Red());
	FillPolygon(xp, yp, n, RGBPIXEL::Red());
	*/
	/*
	int n = 8;
	int xp[10] = { 100,340,550,280, 150, 350,250,200 };
	int yp[10] = { 150,50,300,400, 150, 450,150,200  };
	DrawPolygon(xp, yp, n, RGBPIXEL::Red());
	FillPolygon(xp, yp, n, RGBPIXEL::Red());
	*/

	//---------------------------------------------

	//----------------------------------------------
	//TEST 1dop 2 dop
	/*
	DrawLineWidth(100, 50, 500, 50, 20, RGBPIXEL::Blue());
	DrawLineWidth(500, 75, 500, 450, 20, RGBPIXEL::Green());
	DrawLineWidth(50, 100, 300, 300, 20, RGBPIXEL::Magenta());
	DrawLineWidth(50, 450, 350, 350, 20, RGBPIXEL::Yellow());
	*/
	//----------------------------------------------

	//----------------------------------------------
	//TEST 3dop
	/*
	std::vector<int> dash{ 10,10,20,10};
	DrawDashLine(320, 50, 320, 430, 12, dash, RGBPIXEL::DkRed());
	DrawDashLine(50, 240, 590, 240, 12, dash, RGBPIXEL::DkGreen());
	DrawDashLine(50, 50, 590, 430, 12, dash, RGBPIXEL::DkBlue());
	DrawDashLine(590, 50, 50, 430, 12, dash, RGBPIXEL::DkYellow());
	*/
	//----------------------------------------------

	//----------------------------------------------
	//TEST 1bdz
	/*int x1[] = {100, 150, 250, 150, 70};
	int y1[] = {100, 70, 150, 270, 250};
	int x2[] = {50, 200, 200, 100};
	int y2[] = {170, 50, 280, 230};
	int n1 = 5;
	int n2 = 4;*/

	//int x1[] = { 100, 300, 300, 100 };
	//int y1[] = { 100, 100, 300, 300 };
	//int n1 = 4;
	//int x2[] = { 200, 350, 200, 50 };
	//int y2[] = { 50, 200, 350, 200 };
	//int n2 = 4;

	//int x1[] = { 100, 300, 300, 100 };
	//int y1[] = { 100, 100, 300, 300 };
	//int n1 = 4;
	//int x2[] = { 110, 290, 290, 110 };
	//int y2[] = { 110, 110, 290, 290 };
	//int n2 = 4;

	//int x1[] = { 100, 300, 300, 100 };
	//int y1[] = { 100, 100, 300, 300 };
	//int n1 = 4;
	//int x2[] = { 350, 450, 450, 350 };
	//int y2[] = { 100, 100, 300, 300 };
	//int n2 = 4;

	//int x1[] = {205,435,550,435,200,90};
	//int y1[] = {10,10,180,470,470,180};
	//int x2[] = {80, 300, 330, 50, 70 };
	//int y2[] = {100,110, 240, 400, 120 };
	//int n1=6;
	//int n2=5;

	/*int x1[] = {100,300,300,100};
	int y1[] = {100,100,300,300};
	int x2[] = {50,400,200,400};
	int y2[] = {200,40,200,360};
	int n1=4;
	int n2=4;

	UnionTwoCurvesPoligon(x1, y1, n1, x2, y2, n2,

	RGBPIXEL::White());*/

	//----------------------------------------------

	//----------------------------------------------
	//TEST 2bdz
	/*std::vector<int> x;
	x.push_back(100);
	x.push_back(150);
	x.push_back(250);
	x.push_back(200);
	x.push_back(180);
	x.push_back(100);
	std::vector<int> y;
	y.push_back(200);
	y.push_back(40);
	y.push_back(100);
	y.push_back(200);
	y.push_back(280);
	y.push_back(200);
	DrawBLine(x, y, RGBPIXEL::DkGreen());*/
	//----------------------------------------------

	//----------------------------------------------
	//test 2lab 1num
	/*int xa = 50;
	int ya = 50;
	int xb = 400;
	int yb = 400;
	int x[] = { 100, 200,300,320,200,100 };
	int y[] = { 100,100,200,300,340,150 };
	int n = 6;
	FillPolygon(x, y, n, RGBPIXEL::Blue());
	DrawLine(xa, ya, xb, yb, RGBPIXEL::Green());
	if (ClipLine(xa, ya, xb, yb, x, y, n))
	DrawLine(xa, ya, xb, yb, RGBPIXEL::Red());
	xa = 180;ya = 150; xb = 500; yb = 400;
	DrawLine(xa, ya, xb, yb, RGBPIXEL::Green());
	if (ClipLine(xa, ya, xb, yb, x, y, n))
	DrawLine(xa, ya, xb, yb, RGBPIXEL::Red());
	xa = 380;ya = 150; xb = 500; yb = 200;
	DrawLine(xa, ya, xb, yb, RGBPIXEL::Green());
	if (ClipLine(xa, ya, xb, yb, x, y, n))
	DrawLine(xa, ya, xb, yb, RGBPIXEL::Red());
	xa = 150;ya = 170; xb = 170; yb = 250;
	DrawLine(xa, ya, xb, yb, RGBPIXEL::Green());
	if (ClipLine(xa, ya, xb, yb, x, y, n))
	DrawLine(xa, ya, xb, yb, RGBPIXEL::Red());*/
	//----------------------------------------------	

	//----------------------------------------------
	//test 2lab 2num
	/*int px[] = { 50,100,300,350 };
	int py[] = { 300,100,100,300 };
	int n = 3;
	Draw3Beize(px, py, n, RGBPIXEL::Red());*/
	//----------------------------------------------

	//----------------------------------------------
	//test 2lab 1dop
	/*int px[] = { 50,100,300,350,200,150 };
	int py[] = { 300,100,100,300,200,150 };
	int n = 5;
	DrawNBeize(px, py, n, RGBPIXEL::Red());*/
	//----------------------------------------------

	//----------------------------------------------
	//test 2lab 2dop
	/*int x1[] = {100,300,300,100};
	int y1[] = {100,100,300,300};
	int x2[] = {50,400,200,400};
	int y2[] = {200,40,200,360};
	int n1=4;
	int n2=4;
	UnionTwoCurvesPoligon(x1, y1, n1, x2, y2, n2,

	RGBPIXEL::White());*/
	//----------------------------------------------

	//----------------------------------------------
	//test 3lab 
	/*Coor3D one(100, 100, 0);
	Coor3D two(300, 100, 0);
	Coor3D three(300, 300, 0);
	Coor3D four(100, 300, 0);
	Coor3D five(100, 100, 200);
	Coor3D six(300, 100, 200);
	Coor3D seven(300, 300, 200);
	Coor3D eight(100, 300, 200);
	Coor3D axis(100, 100, 0);
	static Coor3D pos(0, 0, 100);
	std::vector<Coor3D> cube = {

	one,two,three,four,five,six,seven,eight };
	Rotate(cube, 4 * M_PI / 180, axis);
	std::vector<std::vector<Coor3D>> facets = { { cube[0],cube

	[3],cube[2],cube[1] },{ cube[0],cube[4],cube[7],cube[3] },{ cube

	[0],cube[1],cube[5],cube[4] },
	{ cube[1],cube[2],cube[6],cube[5] },{ cube[2],cube[3],cube

	[7],cube[6] },{ cube[4],cube[5],cube[6],cube[7] } };
	drawFigure(facets, pos, RGBPIXEL::DkRed());*/
	//ParallelProection(cube, 500, RGBPIXEL::Green());

	//PerspectiveProection(cube, pos, RGBPIXEL::Green());
	//----------------------------------------------


 //   Coor3D one(100, 100, 0);
	//Coor3D two(300, 100, 0);
	//Coor3D three(300, 300, 0);
	//Coor3D four(100, 300, 0);
	//Coor3D five(100, 100, 200);
	//Coor3D six(300, 100, 200);
	//Coor3D seven(300, 300, 200);
	//Coor3D eight(100, 300, 200);
	//Coor3D pos(0, 0, 1000);
	////int px[4] = { 200,600,600,200 };
	////int py[4] = { 200,350,400,450 };
	////Draw3Beize(px, py, 3, RGBPIXEL::Yellow());
	//std::vector<Coor3D> cube = {one,two,three,four,five,six,seven,eight };
	////double v = 1000;
	////transform(px, py, cube, v, one, two, three, four, five, six,seven, eight);
	////gfSetPixel((cube[0].x + cube[6].x) / 2, (cube[0].y + cube[6].y) / 2, RGBPIXEL::DkGray());
	//Coor3D Cmass((cube[0].x + cube[6].x) / 2, (cube[0].y + cube[6].y) / 2, (cube[0].z + cube[6].z) / 2);
	//Coor3D axis(Cmass.x, Cmass.y, Cmass.z);
	//Rotate(cube, 15 * M_PI / 180, axis);
	//std::vector<std::vector<Coor3D>> facets = { { cube[0],cube[3],cube[2],cube[1] },{ cube[0],cube[4],cube[7],cube[3] },{ cube[0],cube[1],cube[5],cube[4] },
	//	{ cube[1],cube[2],cube[6],cube[5] },{ cube[2],cube[3],cube[7],cube[6] },{ cube[4],cube[5],cube[6],cube[7] } };
	//int I = 500;
	//Light source(RGBPIXEL::Red(), Coor3D(50, 150, -450), I);
	//double kd = 0.95;
	////dellfacetPerspective(facets, pos);
	//dellfacetParalel(facets, pos);
	//paintGuro(facets, source, kd);
	////PerspectiveProection(facets, pos, RGBPIXEL::Green());
	//ParallelProection(facets, 1, RGBPIXEL::Green());


	//Coor3D P[4][4] = { {Coor3D(100,100,0),Coor3D(300,100,0),Coor3D(300,300,0),Coor3D(100,300,0)},
	//			  {Coor3D(125,125,50),Coor3D(275,125,50),Coor3D(275,275,50),Coor3D(125,275,50)},
	//			  {Coor3D(150,150,100),Coor3D(250,150,100),Coor3D(250,250,100),Coor3D(150,250,100)},
	//			  {Coor3D(175,175,150),Coor3D(225,175,150),Coor3D(225,225,150),Coor3D(175,225,150)} };
	//Coor3D R[20][20];
	
	//BSurface(P,R);
	//RotateFigure(R, 40*M_PI/180, Coor3D(0, 1, 0));
//	floatingHorizon(P);
	
	//gfDisplayMessage("Message!");

	return true;
}

// Вызывается в цикле до момента выхода из приложения.
// Следует использовать для создания анимационных эффектов
void gfDrawScene()
{
	gfClearScreen(RGBPIXEL::Black());

	//   //static int x = 0;
	//   //gfDrawRectangle(x, 100, x + 50, 130, RGBPIXEL::Blue());
	//   //x = (x + 1) % gfGetWindowWidth() ;

	//   /* int x = gfGetMouseX(),
	//       y = gfGetMouseY();
	//   gfDrawRectangle(x - 10, y - 10, x + 10, y + 10, RGBPIXEL::Green());*/

	static Coor3D one(100, 100, 0);
	static Coor3D two(300, 100, 0);
	static Coor3D three(300, 300, 0);
	static Coor3D four(100, 300, 0);
	static Coor3D five(100, 100, 200);
	static Coor3D six(300, 100, 200);
	static Coor3D seven(300, 300, 200);
	static Coor3D eight(100, 300, 200);
	static Coor3D pos(0, 0, 1000);
	static int px[4] = { 200,600,600,200 };
	static int py[4] = { 200,350,400,450 };
	Draw3Beize(px, py, 3, RGBPIXEL::Yellow());
	static std::vector<Coor3D> cube = {one,two,three,four,five,six,seven,eight };
	static double v = 1000;
	transform(px, py, cube, v, one, two, three, four, five, six,seven, eight);
	gfSetPixel((cube[0].x + cube[6].x) / 2, (cube[0].y + cube[6].y) / 2, RGBPIXEL::DkGray());
	Coor3D Cmass((cube[0].x + cube[6].x) / 2, (cube[0].y + cube[6].y) / 2, (cube[0].z + cube[6].z) / 2);
	Coor3D axis(Cmass.x, Cmass.y, Cmass.z);
	Rotate(cube, 2 * M_PI / 180, axis);
	std::vector<std::vector<Coor3D>> facets = { { cube[0],cube[3],cube[2],cube[1] },{ cube[0],cube[4],cube[7],cube[3] },{ cube[0],cube[1],cube[5],cube[4] },
		{ cube[1],cube[2],cube[6],cube[5] },{ cube[2],cube[3],cube[7],cube[6] },{ cube[4],cube[5],cube[6],cube[7] } };
	static int I = 400;
	static Light source(RGBPIXEL::Red(), Coor3D(300,200, -1000), I);
	DrawCircle(300, 200, 15, RGBPIXEL::Green());
	static double kd = 0.5;
	dellfacetPerspective(facets, pos);
	//dellfacetParalel(facets, pos);
	PerspectiveProection(facets, pos, RGBPIXEL::Green(),Cmass);
	paintGuro(facets, source, kd);
	
	//ParallelProection(facets, 1, RGBPIXEL::Green());



}
// Вызывается один раз перед выходом из приложения.
// Следует использовать для освобождения выделенных
// ресурсов (памяти, файлов и т.п.)
void gfCleanupScene()
{
}

// Вызывается когда пользователь нажимает левую кнопку мыши
void gfOnLMouseClick( int x, int y )
{
    x; y;
    gfDrawRectangle(0, 0, 640, 480, RGBPIXEL::RandomColor());
}

// Вызывается когда пользователь нажимает правую кнопку мыши
void gfOnRMouseClick( int x, int y )
{
    x; y;
}

// Вызывается когда пользователь нажимает клавишу на клавиатуре
void gfOnKeyDown( UINT key )
{
  //  key;
		//gfClearScreen(RGBPIXEL::Black());
		//static Coor3D P[4][4] = { { Coor3D(100,600,0),Coor3D(200,150,0),Coor3D(300,150,0),Coor3D(400,600,0) },
		//{ Coor3D(100,600,100),Coor3D(200,150,100),Coor3D(300,150,100),Coor3D(400,600,100) },
		//{ Coor3D(100,600,200),Coor3D(200,150,200),Coor3D(300,150,200),Coor3D(400,600,200) },
		//{ Coor3D(100,600,300),Coor3D(200,150,300),Coor3D(300,150,300),Coor3D(400,600,300) } };
		//if (key == 'A')
		//{
		//	RotateFigure(P, 5 * M_PI / 180, Coor3D(0, 1, 0));
		//	for (size_t i = 0; i < 25; i++)
		//	{
		//		for (size_t j = 0; j < 25; j++)
		//		{
		//			Coor3D t = BSurface(P, double(i) / 25, double(j) / 25);
		//		//	gfSetPixel(t.x, t.y, RGBPIXEL::Magenta());
		//		}

		//	}
		//	floatingHorizon(P);
		//}
		//if (key == 'S')
		//{
		//	RotateFigure(P, -5 * M_PI / 180, Coor3D(0, 1, 0));
		//	for (size_t i = 0; i < 25; i++)
		//	{
		//		for (size_t j = 1; j < 25; j++)
		//		{
		//			Coor3D t = BSurface(P, double(i) / 25, double(j) /25);
		//			//gfSetPixel(t.x, t.y, RGBPIXEL::Magenta());
		//		}

		//	}
		//		floatingHorizon(P);
		//}

}

// Вызывается когда пользователь отжимает клавишу на клавиатуре
void gfOnKeyUp( UINT key )
{
    key;

    //if( key == 'B' )
    //    gfDisplayMessage( "'B' key has been un-pressed" );
}
