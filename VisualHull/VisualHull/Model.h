#pragma once
#pragma warning(disable:4244)

#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <limits>
#include <queue>
#include <iostream>
#include <fstream>
using namespace std;

class Point
{
public:
	Point (bool inside = false, bool visited = false, bool enqueued = false):
		inside(inside), visited(visited), enqueued(false){}

	Point(const Point &p)
	{
		inside = p.inside; visited = p.visited; enqueued = p.enqueued; x = p.x; y = p.y; z = p.z;
	}
	bool inside;//是否在visualHull内部
	bool visited;//是否访问过
	bool enqueued;//是否在BFS中入队过
	int x, y, z;
	void setIndex(int index_x, int index_y, int index_z)
	{
		x = index_x;
		y = index_y;
		z = index_z;
	}
};
class Line
{
public:
	Line(){}
	Line(int size, Point point):
		m_points(vector<Point>(size, point))
	{
	}
	Point& operator [](int i) 
	{
		return m_points[i];
	}
	
	std::vector<Point> m_points;
};
//typedef std::vector<std::vector<bool>> Pixel;
typedef std::vector<Line> Pixel;
typedef std::vector<Pixel> Voxel;
//typedef std::vector<std::vector<std::vector<bool>>> Visited;

// 用于判断投影是否在visual hull内部
struct Projection
{
	Eigen::Matrix<float, 3, 4> m_projMat;
	cv::Mat m_image;
	const uint m_threshold = 125;

	bool outOfRange(int x, int max)
	{
		return x < 0 || x >= max;
	}

	bool checkRange(double x, double y, double z)
	{
		Eigen::Vector3f vec3 = m_projMat * Eigen::Vector4f(x, y, z, 1);
		int indX = vec3[1] / vec3[2];
		int indY = vec3[0] / vec3[2];

		if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
			return false;
		//判断是否为白色点，白色表示在VisualHull的内部
		return m_image.at<uchar>((uint)(vec3[1] / vec3[2]), (uint)(vec3[0] / vec3[2])) > m_threshold;
	}
};

// 用于index和实际坐标之间的转换
struct CoordinateInfo
{
	int m_resolution;
	double m_min;
	double m_max;

	double index2coor(int index)
	{
		return m_min + index * (m_max - m_min) / m_resolution;
	}

	CoordinateInfo(int resolution = 10, double min = 0.0, double max = 100.0)
		: m_resolution(resolution)
		, m_min(min)
		, m_max(max)
	{
	}
};

class Model
{
public:

	Model(int resX = 100, int resY = 100, int resZ = 100);
	~Model();

	void saveModel(const char* pFileName);
	void saveModelWithNormal(const char* pFileName);
	void loadMatrix(const char* pFileName);
	void loadImage(const char* pDir, const char* pPrefix, const char* pSuffix);
	void getModel();
	void getSurface();
	Eigen::Vector3f getNormal(int indX, int indY, int indZ);


private:
	bool outOfRange(int indexX, int indexY, int indexZ);
	bool insideHull(int indexX, int indexY, int indexZ);
	void BFS(int indexX, int indexY, int indexZ);
	void DFS(int indexX, int indexY, int indexZ);

	CoordinateInfo m_corrX;
	CoordinateInfo m_corrY;
	CoordinateInfo m_corrZ;

	int m_neiborSize;

	std::vector<Projection> m_projectionList;

	Voxel m_voxel;
	Voxel m_surface;
};
