#include "stdafx.h"
#include "point.h"

#include <math.h>

CPoint::CPoint()
{
	m_x = 0.0f;
	m_y = 0.0f;
	m_z = 0.0f;
}

CPoint::CPoint(float x, float y, float z)
{
	m_x = x;
	m_y = y;
	m_z = z;
}

CPoint::~CPoint()
{

}

CPoint CPoint::operator+(const CPoint& point) const
{
	CPoint result(m_x + point.m_x, m_y + point.m_y, m_z + point.m_z);
	return result;
}

CPoint CPoint::operator-(const CPoint& point) const
{
	CPoint result(m_x - point.m_x, m_y - point.m_y, m_z - point.m_z);
	return result;
}

float CPoint::x()
{
	return m_x;
}

float CPoint::y()
{
	return m_y;
}

float CPoint::z()
{
	return m_z;
}

float CPoint::length()
{
	return sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
}

void CPoint::setX(float x)
{
	m_x = x;
}

void CPoint::setY(float y)
{
	m_y = y;
}

void CPoint::setZ(float z)
{
	m_z = z;
}
