#pragma once

class CPoint
{
private:
	float m_x;
	float m_y;
	float m_z;

public:
	CPoint();
	CPoint(float x, float y, float z);
	~CPoint();

public:
	CPoint operator+(const CPoint& point) const;
	CPoint operator-(const CPoint& point) const;

public:
	float x();
	float y();
	float z();

	float length();

	void setX(float x);
	void setY(float y);
	void setZ(float z);
};
