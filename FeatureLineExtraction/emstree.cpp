#include "stdafx.h"
#include "emstree.h"

Vec2::Vec2()
{
	m_x = 0;
	m_y = 0;
}

Vec2::Vec2(int x, int y)
{
	m_x = x;
	m_y = y;
}

Vec2::~Vec2()
{

}

int Vec2::x()
{
	return m_x;
}

int Vec2::y()
{
	return m_y;
}

EMSTree::EMSTree()
{
	m_Theta = 0.1f;
	m_partionNum = 2;
}

EMSTree::~EMSTree()
{

}

void EMSTree::compute(std::vector<CPoint> points)
{
	m_Points.clear();
	m_Points = points;
	splitPoints();

	m_resultLinesIndices.clear();
	m_resultLines.clear();
	for (int i = 0; i < m_partionNum; i++)
		computeEMSTree(i);
}

void EMSTree::setTheta(float theta)
{
	m_Theta = theta;
}

void EMSTree::setPartionNum(int num)
{
	m_partionNum = num;
}

void EMSTree::getResult(std::vector<std::vector<CPoint>>& lines)
{
	lines.clear();
	lines = m_resultLines;
}

void EMSTree::splitPoints()
{
	CPoint min(INF, INF, INF);
	CPoint max(-INF, -INF, -INF);

	for (int i = 0; i < m_Points.size(); i++)
	{
		CPoint p = m_Points[i];
		float x, y, z;
		x = p.x();
		y = p.y();
		z = p.z();

		if (x < min.x())
			min.setX(x);
		if (x > max.x())
			max.setX(x);

		if (y < min.y())
			min.setY(y);
		if (y > max.y())
			max.setY(y);

		if (z < min.z())
			min.setZ(z);
		if (z > max.z())
			max.setZ(z);
	}

	CPoint ds = max - min;
	int mt = -1;
	float mv = -1.0f;
	if (ds.x() > mv)
	{
		mv = ds.x();
		mt = 0;
	}
	if (ds.y() > mv)
	{
		mv = ds.y();
		mt = 1;
	}
	if (ds.z() > mv)
	{
		mv = ds.z();
		mt = 2;
	}

	switch (mt)
	{
	case 0:
		partionX(max, min);
		break;
	case 1:
		partionY(max, min);
		break;
	case 2:
		partionZ(max, min);
		break;
	default:
		break;
	}
}

void EMSTree::partionX(CPoint dmax, CPoint dmin)
{
	float diff = (dmax - dmin).x();
	float div = diff / m_partionNum;
	for (int i = 1; i < m_partionNum + 1; i++)
	{
		std::vector<CPoint> cpoints;
		float high = div * i + dmin.x();
		float low = div * (i - 1) + dmin.x();
		for (int j = 0; j < m_Points.size(); j++)
		{
			if (m_Points[j].x() > low &&  m_Points[j].x() < high)
				cpoints.push_back(m_Points[j]);
		}
		m_partionPoints.push_back(cpoints);
	}
}

void EMSTree::partionY(CPoint dmax, CPoint dmin)
{
	float diff = (dmax - dmin).y();
	float div = diff / m_partionNum;
	for (int i = 1; i < m_partionNum + 1; i++)
	{
		std::vector<CPoint> cpoints;
		float high = div * i + dmin.y();
		float low = div * (i - 1) + dmin.y();
		for (int j = 0; j < m_Points.size(); j++)
		{
			if (m_Points[j].y() > low &&  m_Points[j].y() < high)
				cpoints.push_back(m_Points[j]);
		}
		m_partionPoints.push_back(cpoints);
	}
}

void EMSTree::partionZ(CPoint dmax, CPoint dmin)
{
	float diff = (dmax - dmin).z();
	float div = diff / m_partionNum;
	for (int i = 1; i < m_partionNum + 1; i++)
	{
		std::vector<CPoint> cpoints;
		float high = div * i + dmin.z();
		float low = div * (i - 1) + dmin.z();
		for (int j = 0; j < m_Points.size(); j++)
		{
			if (m_Points[j].z() > low &&  m_Points[j].z() < high)
				cpoints.push_back(m_Points[j]);
		}
		m_partionPoints.push_back(cpoints);
	}
}

void EMSTree::computeEMSTree(int iteration)
{
	m_Distances.clear();
	m_wt.clear();
	m_wt.resize(m_partionPoints[iteration].size());

	std::vector<float> distances(m_partionPoints[iteration].size());
	for (int i = 0; i < m_partionPoints[iteration].size(); i++)
		m_Distances.push_back(distances);

	for (int i = 0; i < m_partionPoints[iteration].size(); i++)
	{
		for (int j = 0; j < m_partionPoints[iteration].size(); j++)
		{
			if (i == j)
				m_Distances[i][j] = INF;
			else
			{
				CPoint length = m_partionPoints[iteration][i] - m_partionPoints[iteration][j];
				float distance = length.length();
				m_Distances[i][j] = distance;
			}
		}
	}

	std::vector<Vec2> tree;

	float minDist = 0.0f;

	int pointSize = m_partionPoints[iteration].size();
	std::vector<int> adjVec(pointSize);
	std::vector<float> lowCost(pointSize);

	for (int i = 0; i < pointSize; i++)
	{
		lowCost[i] = m_Distances[0][i];
		adjVec[i] = 0;
		m_wt[i] = 0;
	}

	for (int i = 1; i < pointSize; i++)
	{
		minDist = INF;
		int j = 1;
		int k = 0;
		while (j < pointSize)
		{
			if (lowCost[j] < INF && lowCost[j] < minDist)
			{
				minDist = lowCost[j];
				k = j;
			}
			j++;
		}

		int los = adjVec[k];

		if (minDist < m_Theta)
		{
			m_wt[los] += 1;
			m_wt[k] += 1;
			tree.push_back(Vec2(los, k));
		}
		lowCost[k] = INF;
		for (j = 1; j < pointSize; j++)
		{
			if (lowCost[j] < INF && m_Distances[k][j] < lowCost[j])
			{
				lowCost[j] = m_Distances[k][j];
				adjVec[j] = k;
			}
		}
	}

	m_resultIndices.clear();
	m_resultIndices = tree;

	removeBifurcations();
	m_resultLinesIndices.clear();
	transToLines();

	std::vector<CPoint> line;
	int index = -1;
	for (int i = 0; i < m_resultLinesIndices.size(); i++)
	{
		line.clear();
		for (int j = 0; j < m_resultLinesIndices[i].size(); j++)
		{
			index = m_resultLinesIndices[i][j];
			CPoint point = m_partionPoints[iteration][index];
			line.push_back(point);
		}
		m_resultLines.push_back(line);
	}
}

void EMSTree::removeBifurcations()
{
	std::vector<Vec2> lineIndices(m_resultIndices);
	std::vector<int> wt(m_wt);

	std::vector<Vec2> result;

	while (true)
	{
		int count = 0;
		for (int i = 0; i < lineIndices.size(); i++)
		{
			int a = lineIndices.at(i).x();
			int b = lineIndices.at(i).y();
			int wta = wt[a];
			int wtb = wt[b];

			if (wta == 1 && wtb == 2)
			{
				wt[a]--;
				wt[b]--;
				count++;
				result.push_back(lineIndices[i]);
			}
			else if (wta == 2 && wtb == 1)
			{
				wt[a]--;
				wt[b]--;
				count++;
				result.push_back(lineIndices[i]);
			}
			else if (wta >= 2 && wtb >= 2)
				result.push_back(lineIndices[i]);
			else if (wta > 2 && wtb == 2)
				result.push_back(lineIndices[i]);
			else if (wta == 2 && wtb > 2)
				result.push_back(lineIndices[i]);

			else if (wta == 1 && wtb > 2)
			{
				wt[a]--;
				wt[b]--;
			}

			else if (wta > 2 && wtb == 1)
			{
				wt[a]--;
				wt[b]--;
			}
			else
				result.push_back(lineIndices[i]);
		}
		lineIndices.clear();
		lineIndices = result;
		result.clear();

		if (count <= 5)
			break;
	}

	m_resultIndices = lineIndices;
}

void EMSTree::transToLines()
{
	std::vector<int> line;
	std::vector<Vec2> lineIndices(m_resultIndices);
	int idxHead = -1;
	int idxEnd = -1;

	while (lineIndices.size() > 0)
	{
		Vec2 p = lineIndices[0];
		if (line.size() == 0)
		{
			idxHead = p.x();
			idxEnd = p.y();
			line.push_back(idxHead);
			line.push_back(idxEnd);
			lineIndices.erase(lineIndices.begin());
		}
		else if (line.size() > 0)
		{
			int currentSize = lineIndices.size();

			for (int i = 0; i < lineIndices.size(); i++)
			{
				p = lineIndices[i];
				if (idxHead == p.x())
				{
					line.insert(line.begin(), p.y());
					idxHead = p.y();
					lineIndices.erase(lineIndices.begin() + i);
					i = 0;
				}
				else if (idxHead == p.y())
				{
					line.insert(line.begin(), p.x());
					idxHead = p.x();
					lineIndices.erase(lineIndices.begin() + i);
					i = 0;
				}
				else if (idxEnd == p.x())
				{
					line.push_back(p.y());
					idxEnd = p.y();
					lineIndices.erase(lineIndices.begin() + i);
					i = 0;
				}
				else if (idxEnd == p.y())
				{
					line.push_back(p.x());
					idxEnd = p.x();
					lineIndices.erase(lineIndices.begin() + i);
					i = 0;
				}
			}
			if (currentSize == lineIndices.size())
			{
				m_resultLinesIndices.push_back(line);
				line.clear();
			}
		}

		if (lineIndices.size() == 0)
		{
			m_resultLinesIndices.push_back(line);
			line.clear();
		}
	}
}
