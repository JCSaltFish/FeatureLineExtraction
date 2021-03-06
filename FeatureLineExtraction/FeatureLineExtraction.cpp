// FeatureLineExtraction.cpp : 定义应用程序的入口点。
//

#include "stdafx.h"

#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>

#include <GL/glut.h>

#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>

#include "point.h"
#include "extractor.h"

#pragma comment(lib, "glut.lib")
#pragma comment(lib, "glut32.lib")

#define STATUS_ROTATE	1
#define STATUS_MOVE		2
#define STATUS_ZOOM		3

static int g_Status = 0;

static float g_RotateInc = 0.6;
static float g_MoveInc = 0.005;
static float g_ZoomInc = 0.01;

static float g_HorizonAng = 90.0;
static float g_VerticalAng = 0.0;

static int g_MousePreX = -1;
static int g_MousePreY = -1;

static float g_ViewRadius = 1.5;

static float g_CenterX = 0.0;
static float g_CenterY = 0.0;
static float g_CenterZ = 0.0;

pcl::PointCloud<pcl::PointXYZ> g_InputCloud;
std::vector<CPoint> g_OriginalPoints;
std::vector<CPoint> g_FeaturePoints;
std::vector<std::vector<CPoint>> g_FeatureLines;

void loadOriginalPointsPLY(const char* filename)
{
	float maxX = (float)-0xFFFFF;
	float minX = (float)0xFFFFF;
	float maxY = (float)-0xFFFFF;
	float minY = (float)0xFFFFF;
	float maxZ = (float)-0xFFFFF;
	float minZ = (float)0xFFFFF;

	pcl::io::loadPLYFile<pcl::PointXYZ>(filename, g_InputCloud);
	g_OriginalPoints.clear();
	for (int i = 0; i < g_InputCloud.size(); i++)
	{
		float x = g_InputCloud.points[i].x;
		float y = g_InputCloud.points[i].y;
		float z = g_InputCloud.points[i].z;

		if (x > maxX)
			maxX = x;
		if (x < minX)
			minX = x;
		if (y > maxY)
			maxY = y;
		if (y < minY)
			minY = y;
		if (z > maxZ)
			maxZ = z;
		if (z < minZ)
			minZ = z;

		CPoint point(x, y, z);
		g_OriginalPoints.push_back(point);
	}

	g_CenterX = (maxX + minX) / 2.0;
	g_CenterY = (maxY + minY) / 2.0;
	g_CenterZ = (maxZ + minZ) / 2.0;
}

void copyPointsFromPointCloudNormal(pcl::PointCloud<pcl::PointNormal>::Ptr pIn,
	std::vector<CPoint>& out)
{
	out.clear();
	if (pIn == nullptr)
		return;
	for (int i = 0; i < pIn->size(); i++)
	{
		float x = pIn->points[i].x;
		float y = pIn->points[i].y;
		float z = pIn->points[i].z;
		CPoint point(x, y, z);
		out.push_back(point);
	}
}

void extractFeatureLine()
{
	loadOriginalPointsPLY("tripod_sampled.ply");
	g_FeaturePoints.clear();
	g_FeatureLines.clear();
	Extractor extractor;
	extractor.setInputCloud(g_InputCloud);

	extractor.mlsCalculation(0.03f);
	extractor.filterFeaturePoints(0.025f);
	extractor.laplacianSmoothing(25, 3, 5);
	extractor.emstCalculation(4, 0.1f);

	pcl::PointCloud<pcl::PointNormal>::Ptr
		pFeaturePointCloud(new pcl::PointCloud<pcl::PointNormal>);
	extractor.getFeaturePointCloud(pFeaturePointCloud);
	copyPointsFromPointCloudNormal(pFeaturePointCloud, g_FeaturePoints);

	extractor.getFeatureLines(g_FeatureLines);
}

void drawPoints(std::vector<CPoint>& points)
{
	glPointSize(1.0);
	glBegin(GL_POINTS);
	for (int i = 0; i < points.size(); i++)
		glVertex3f(points[i].x(), points[i].y(), points[i].z());
	glEnd();
}

void drawLines(std::vector<std::vector<CPoint>>& lines)
{
	for (int i = 0; i < g_FeatureLines.size(); i++)
	{
		glBegin(GL_LINES);
		for (int j = 1; j < g_FeatureLines[i].size(); j++)
		{
			glVertex3f(g_FeatureLines[i][j].x(),
				g_FeatureLines[i][j].y(), g_FeatureLines[i][j].z());
			glVertex3f(g_FeatureLines[i][j - 1].x(),
				g_FeatureLines[i][j - 1].y(), g_FeatureLines[i][j - 1].z());
		}
		glEnd();
	}
}

void drawFrame()
{
	glColor3f(0.0, 0.6, 0.6);
	drawPoints(g_OriginalPoints);
	glColor3f(1.0, 1.0, 1.0);
	//drawPoints(g_FeaturePoints);
	drawLines(g_FeatureLines);
}

void onRenderFrame()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	double eyeX = g_ViewRadius * cos(M_PI / 180.0 * g_VerticalAng) *
		cos(M_PI / 180.0 * g_HorizonAng);
	double eyeY = g_ViewRadius * sin(M_PI / 180.0 * g_VerticalAng);
	double eyeZ = g_ViewRadius * cos(M_PI / 180.0 * g_VerticalAng) *
		sin(M_PI / 180.0 * g_HorizonAng);

	eyeX += g_CenterX;
	eyeY += g_CenterY;
	eyeZ += g_CenterZ;

	gluLookAt(eyeX, eyeY, eyeZ, g_CenterX, g_CenterY, g_CenterZ, 0.0, 1.0, 0.0);

	drawFrame();
	glutSwapBuffers();
}

void onMouseClick(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
	{
		g_MousePreX = x;
		g_MousePreY = y;
	}

	if (button == GLUT_LEFT_BUTTON)
		g_Status = STATUS_ROTATE;
	else if (button == GLUT_RIGHT_BUTTON)
		g_Status = STATUS_MOVE;
	else if (button == GLUT_MIDDLE_BUTTON)
		g_Status = STATUS_ZOOM;
	else
		g_Status = 0;

	if (state == GLUT_UP)
		g_Status = 0;
}

void onMouseMove(int x, int y)
{
	if (g_Status == STATUS_ROTATE)
	{
		g_HorizonAng += g_RotateInc * (x - g_MousePreX);
		if (g_HorizonAng > 360)
			g_HorizonAng = 0;
		if (g_HorizonAng < -360)
			g_HorizonAng = 0;
		g_VerticalAng += g_RotateInc * (y - g_MousePreY);
		if (g_VerticalAng > 89.9)
			g_VerticalAng = 89.9;
		if (g_VerticalAng < -89.9)
			g_VerticalAng = -89.9;
	}
	else if (g_Status == STATUS_MOVE)
	{
		g_CenterX += (g_MousePreX - x) * g_MoveInc * sin(M_PI / 180.0 * g_HorizonAng);
		g_CenterX += (g_MousePreY - y) * g_MoveInc * sin(M_PI / 180.0 * g_VerticalAng) *
			cos(M_PI / 180.0 * g_HorizonAng);
		g_CenterY += (y - g_MousePreY) * g_MoveInc * cos(M_PI / 180.0 * g_VerticalAng);
		g_CenterZ += (x - g_MousePreX) * g_MoveInc * cos(M_PI / 180.0 * g_HorizonAng);
		g_CenterZ += (g_MousePreY - y) * g_MoveInc * sin(M_PI / 180.0 * g_VerticalAng) *
			sin(M_PI / 180.0 * g_HorizonAng);
	}
	else if (g_Status == STATUS_ZOOM)
	{
		g_ViewRadius += (g_MousePreY - y) * g_ZoomInc;
		if (g_ViewRadius < 0.1)
			g_ViewRadius = 0.1;
	}
	g_MousePreX = x;
	g_MousePreY = y;
}

void onReshape(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(75.0, (float) w / h, 0.1, 1000.0);
	glMatrixMode(GL_MODELVIEW);
}

void showViewer()
{
	char *argv[] = { "arg1", "arg2" };
	int argc = 2;
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(1024, 768);
	glutCreateWindow("Cloud Viewer");

	glEnable(GL_DEPTH_TEST);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glEnable(GL_POINT_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_POLYGON_SMOOTH);

	glutReshapeFunc(onReshape);
	glutDisplayFunc(onRenderFrame);
	glutIdleFunc(onRenderFrame);
	glutMouseFunc(onMouseClick);
	glutMotionFunc(onMouseMove);

	glutMainLoop();
}

int APIENTRY _tWinMain(HINSTANCE hInstance,
					   HINSTANCE hPrevInstance,
					   LPTSTR lpCmdLine,
					   int nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	extractFeatureLine();
	showViewer();

	return 0;
}
