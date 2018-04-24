#include "stdafx.h"
#include "extractor.h"

#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>

#include "emstree.h"

Extractor::Extractor()
{
	m_pCloud = nullptr;
	m_pCloudWithNormals = nullptr;
	m_pFeaturePointCloud = nullptr;
	m_FeatureLines.clear();
}

Extractor::~Extractor()
{

}

void Extractor::setInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	m_pCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(cloud, *m_pCloud);
}

void Extractor::getCloudWithNormals(pcl::PointCloud<pcl::PointNormal>::Ptr pCloudOut)
{
	if (m_pCloudWithNormals != nullptr)
		pcl::copyPointCloud<pcl::PointNormal>(*m_pCloudWithNormals, *pCloudOut);
	else
		pCloudOut = nullptr;
}

void Extractor::getFeaturePointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr pCloudOut)
{
	if (m_pFeaturePointCloud != nullptr)
		pcl::copyPointCloud<pcl::PointNormal>(*m_pFeaturePointCloud, *pCloudOut);
	else
		pCloudOut = nullptr;
}

void Extractor::getFeatureLines(std::vector<std::vector<CPoint>>& linesOut)
{
	linesOut.clear();
	if (m_FeatureLines.size() != 0)
		linesOut = m_FeatureLines;
}

void Extractor::mlsCalculation(float radius)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr mlsTree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);
	mls.setInputCloud(m_pCloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(mlsTree);
	mls.setSearchRadius(radius);

	m_pCloudWithNormals =
		pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
	mls.process(*m_pCloudWithNormals);
}

void Extractor::filterFeaturePoints(float sigma)
{
	m_pFeaturePointCloud =
		pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
	std::vector<int> keypointIndices;
	for (int i = 0; i < m_pCloudWithNormals->points.size(); i++)
	{
		if (m_pCloudWithNormals->points[i].curvature > sigma)
			m_pFeaturePointCloud->push_back(m_pCloudWithNormals->points[i]);
	}
}

void Extractor::laplacianSmoothing(int k, int iterations, int attenuation)
{
	pcl::search::KdTree<pcl::PointNormal>::Ptr lsTree(new pcl::search::KdTree<pcl::PointNormal>);
	lsTree->setInputCloud(m_pFeaturePointCloud);

	int lsK = k;
	for (int iteration = 0; iteration < iterations; iteration++)
	{
		lsTree->setInputCloud(m_pFeaturePointCloud);
		for (int i = 0; i < m_pFeaturePointCloud->size(); i++)
		{
			std::vector<int> pointIdxNKNSearch(lsK);
			std::vector<float> pointNKNSquaredDistance(lsK);
			lsTree->nearestKSearch(m_pFeaturePointCloud->points[i], lsK,
				pointIdxNKNSearch, pointNKNSquaredDistance);
			if (pointIdxNKNSearch.size() <= 0)
				continue;

			float normal_x = m_pFeaturePointCloud->points[i].normal_x;
			float normal_y = m_pFeaturePointCloud->points[i].normal_y;
			float normal_z = m_pFeaturePointCloud->points[i].normal_z;
			float normal_length = normal_x * normal_x;
			normal_length += normal_y * normal_y;
			normal_length += normal_z * normal_z;
			normal_length = sqrt(normal_length);
			float normal[3];
			normal[0] = normal_x / normal_length;
			normal[1] = normal_y / normal_length;
			normal[2] = normal_z / normal_length;

			std::vector<pcl::PointNormal> pointPrjNKNSearch(pointIdxNKNSearch.size());
			for (int j = 0; j < pointIdxNKNSearch.size(); j++)
			{
				pointPrjNKNSearch[j] = m_pFeaturePointCloud->points[pointIdxNKNSearch[j]];

				float vect[3];
				vect[0] = pointPrjNKNSearch[j].x - m_pFeaturePointCloud->points[i].x;
				vect[1] = pointPrjNKNSearch[j].y - m_pFeaturePointCloud->points[i].y;
				vect[2] = pointPrjNKNSearch[j].z - m_pFeaturePointCloud->points[i].z;

				float dist_proj = vect[0] * normal[0];
				dist_proj += vect[1] * normal[1];
				dist_proj += vect[2] * normal[2];

				pointPrjNKNSearch[j].x = pointPrjNKNSearch[j].x - dist_proj * normal[0];
				pointPrjNKNSearch[j].y = pointPrjNKNSearch[j].y - dist_proj * normal[1];
				pointPrjNKNSearch[j].z = pointPrjNKNSearch[j].z - dist_proj * normal[2];
			}

			float upX = 0.0f;
			float upY = 0.0f;
			float upZ = 0.0f;
			float down = 0.0f;
			for (int j = 0; j < pointIdxNKNSearch.size(); j++)
			{
				upX += pointPrjNKNSearch[j].curvature *
					(pointPrjNKNSearch[j].x - m_pFeaturePointCloud->points[i].x);
				upY += pointPrjNKNSearch[j].curvature *
					(pointPrjNKNSearch[j].y - m_pFeaturePointCloud->points[i].y);
				upZ += pointPrjNKNSearch[j].curvature *
					(pointPrjNKNSearch[j].z - m_pFeaturePointCloud->points[i].z);
				down += pointPrjNKNSearch[j].curvature;
			}
			m_pFeaturePointCloud->points[i].x += upX / down;
			m_pFeaturePointCloud->points[i].y += upY / down;
			m_pFeaturePointCloud->points[i].z += upZ / down;
		}
		lsK -= attenuation;
	}
}

void Extractor::emstCalculation(int partionNum, float theta)
{
	std::vector<CPoint> cpoints;
	for (int i = 0; i < m_pFeaturePointCloud->points.size(); i++)
	{
		pcl::PointNormal point = m_pFeaturePointCloud->points[i];
		CPoint cpoint(point.x, point.y, point.z);
		cpoints.push_back(cpoint);
	}

	EMSTree emstree;
	emstree.setPartionNum(partionNum);
	emstree.setTheta(theta);

	emstree.compute(cpoints);
	emstree.getResult(m_FeatureLines);
}
