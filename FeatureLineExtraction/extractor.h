#pragma once

#include <vector>

#include <pcl/io/io.h>

#include "point.h"

class Extractor
{
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pCloud;
	pcl::PointCloud<pcl::PointNormal>::Ptr m_pCloudWithNormals;
	pcl::PointCloud<pcl::PointNormal>::Ptr m_pFeaturePointCloud;
	std::vector<std::vector<CPoint>> m_FeatureLines;

public:
	Extractor();
	~Extractor();

public:
	void setInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloud);
	void getCloudWithNormals(pcl::PointCloud<pcl::PointNormal>::Ptr pCloudOut);
	void getFeaturePointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr pCloudOut);
	void getFeatureLines(std::vector<std::vector<CPoint>>& linesOut);

	void mlsCalculation(float radius);
	void filterFeaturePoints(float sigma);
	void laplacianSmoothing(int k, int iterations, int attenuation);
	void emstCalculation(int partionNum, float theta);
};
