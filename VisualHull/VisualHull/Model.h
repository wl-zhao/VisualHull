#pragma once
#pragma warning(disable:4244)

#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <limits>
#include <queue>
#include <list>
#include <iostream>
#include <fstream>
using namespace std;

class Point
{
public:
	Point(int indexX = 0, int indexY = 0, int indexZ = 0):x(indexX), y(indexY), z(indexZ){}
	const Point& operator+(const Point &p)
	{ return Point(x+p.x, y+p.y, z+p.z); }
	int x, y, z;
	friend ofstream& operator<<(ofstream& os, Point& p);
};


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
	typedef std::vector<std::vector<bool>> Pixel;
	typedef std::vector<Pixel> Voxel;

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
	//偏移                 前后左右上下 0-5			八个角块6-13				12个棱块14-25								
	const int dx[26] = { -1, 0, 0, 0, 0, 1,   1, 1, 1, 1,-1,-1,-1,-1,    1, 1,-1,-1, 0, 0, 0, 0, 1, 1,-1,-1 };
	const int dy[26] = { 0, 1, -1, 0, 0, 0,   1, 1,-1,-1,-1,-1, 1, 1,    0, 0, 0, 0, 1, 1, 1, 1, 1,-1,-1, 1 };
	const int dz[26] = { 0, 0, 0, 1, -1, 0,   1,-1, 1,-1, 1,-1, 1,-1,    1,-1, 1,-1, 1,-1, 1,-1, 0, 0, 0, 0 };
	Point dp[26];


	bool outOfRange(int indexX, int indexY, int indexZ);
	bool outOfRange(const Point &p) 
	{ return outOfRange(p.x, p.y, p.z); }
	bool insideHull(int indexX, int indexY, int indexZ);
	bool insideHull(const Point &p) 
	{ return insideHull(p.x, p.y, p.z); }
	bool totalInside(const Point &p);


	void BFS(Point p);

	bool voxel(const Point &p) 
	{ return m_voxel[p.x][p.y][p.z]; }
	bool surface(const Point &p) 
	{ return m_surface[p.x][p.y][p.z]; }
	bool visited(const Point &p) 
	{ return m_visited[p.x][p.y][p.z]; }
	bool enqueued(const Point &p) 
	{ return m_enqueued[p.x][p.y][p.z]; }


	void setVoxel(const Point &p, bool v = true) 
	{ m_voxel[p.x][p.y][p.z] = v; }
	void setSurface(const Point &p, bool v = true) 
	{ m_surface[p.x][p.y][p.z] = v; }	
	void setVisited(const Point &p, bool v = true) 
	{ m_visited[p.x][p.y][p.z] = v; }
	void setEnqueued(const Point &p, bool v = true) 
	{ m_enqueued[p.x][p.y][p.z] = v; }

	CoordinateInfo m_corrX;
	CoordinateInfo m_corrY;
	CoordinateInfo m_corrZ;

	int m_neiborSize;

	std::vector<Projection> m_projectionList;
	vector<Point> surfacePoints;

	Voxel m_voxel;
	Voxel m_surface;
	Voxel m_visited;//是否访问过
	Voxel m_enqueued;//是否入队过
};
