#pragma once

#include <vector>
#include "point.h"

class Vec2
{
private:
	int m_x;
	int m_y;

public:
	Vec2();
	Vec2(int x, int y);
	~Vec2();

public:
	int x();
	int y();
};

class EMSTree
{
private:
	int m_partionNum;

	std::vector<CPoint> m_Points;
	std::vector<std::vector<CPoint>> m_partionPoints;
	float m_Theta;

	std::vector<std::vector<float>> m_Distances;
	std::vector<int> m_wt;

	std::vector<Vec2> m_resultIndices;
	std::vector<std::vector<int>> m_resultLinesIndices;

	std::vector<std::vector<CPoint>> m_resultLines;

private:
	const float INF = (float)0xFFFFF;

public:
	EMSTree();
	~EMSTree();

public:
	void compute(std::vector<CPoint> points);
	void setTheta(float theta);

	void setPartionNum(int num);

	void getResult(std::vector<std::vector<CPoint>>& lines);

private:
	void splitPoints();
	void partionX(CPoint dmax, CPoint dmin);
	void partionY(CPoint dmax, CPoint dmin);
	void partionZ(CPoint dmax, CPoint dmin);

	void computeEMSTree(int iteration);
	void removeBifurcations();
	void transToLines();
};
