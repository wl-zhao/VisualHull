#include "Model.h"

Model::Model(int resX, int resY, int resZ)
	: m_corrX(resX, -5, 5)
	, m_corrY(resY, -10, 10)
	, m_corrZ(resZ, 15, 30)
{
	if (resX > 100)
		m_neiborSize = resX / 100;
	else
		m_neiborSize = 1;
	m_voxel = Voxel(m_corrX.m_resolution, Pixel(m_corrY.m_resolution, vector<bool>(m_corrZ.m_resolution, true)));
	m_surface = Voxel(m_corrX.m_resolution, Pixel(m_corrY.m_resolution, vector<bool>(m_corrZ.m_resolution, false)));
	m_enqueued = Voxel(m_corrX.m_resolution, Pixel(m_corrY.m_resolution, vector<bool>(m_corrZ.m_resolution, false)));
	m_visited = Voxel(m_corrX.m_resolution, Pixel(m_corrY.m_resolution, vector<bool>(m_corrZ.m_resolution, false)));
	for (int i = 0; i < 26; i++)
		dp[i] = Point(dx[i], dy[i], dz[i]);
	surfacePoints.clear();
}

Model::~Model()
{
}

//可用一个list存放所有的满足条件的m_surface信息
void Model::saveModel(const char* pFileName)
{
	std::ofstream fout(pFileName);
	double coorX, coorY, coorZ;
	//for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
	//	for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
	//		for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
	//			if (m_surface[indexX][indexY][indexZ])
	//			{
	//				double coorX = m_corrX.index2coor(indexX);
	//				double coorY = m_corrY.index2coor(indexY);
	//				double coorZ = m_corrZ.index2coor(indexZ);
	//				fout << coorX << ' ' << coorY << ' ' << coorZ << std::endl;
	//			}
	for (auto &p : surfacePoints)
	{
		coorX = m_corrX.index2coor(p.x);
		coorY = m_corrY.index2coor(p.y);
		coorZ = m_corrZ.index2coor(p.z);
		fout << coorX << ' ' << coorY << ' ' << coorZ << std::endl;
	}
}

void Model::saveModelWithNormal(const char* pFileName)
{
	std::ofstream fout(pFileName);

	double midX = m_corrX.index2coor(m_corrX.m_resolution / 2);
	double midY = m_corrY.index2coor(m_corrY.m_resolution / 2);
	double midZ = m_corrZ.index2coor(m_corrZ.m_resolution / 2);
	double coorX, coorY, coorZ;

	for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
		for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
			for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
				if (m_surface[indexX][indexY][indexZ])
				{
					double coorX = m_corrX.index2coor(indexX);
					double coorY = m_corrY.index2coor(indexY);
					double coorZ = m_corrZ.index2coor(indexZ);
					fout << coorX << ' ' << coorY << ' ' << coorZ << ' ';

					Eigen::Vector3f nor = getNormal(indexX, indexY, indexZ);
					fout << nor(0) << ' ' << nor(1) << ' ' << nor(2) << std::endl;
				}
	//for (auto & p : surfacePoints)
	//{
	//	coorX = m_corrX.index2coor(p.x);
	//	coorY = m_corrY.index2coor(p.y);
	//	coorZ = m_corrZ.index2coor(p.z);
	//	fout << coorX << ' ' << coorY << ' ' << coorZ << std::endl;

	//	Eigen::Vector3f nor = getNormal(p.x, p.y, p.z);
	//	fout << nor(0) << ' ' << nor(1) << ' ' << nor(2) << std::endl;
	//}
}

void Model::loadMatrix(const char* pFileName)
{
	std::ifstream fin(pFileName);

	int num;
	Eigen::Matrix<float, 3, 3> matInt;
	Eigen::Matrix<float, 3, 4> matExt;
	Projection projection;
	while (fin >> num)
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				fin >> matInt(i, j);

		double temp;
		fin >> temp;
		fin >> temp;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				fin >> matExt(i, j);

		projection.m_projMat = matInt * matExt;
		m_projectionList.push_back(projection);
	}
}

void Model::loadImage(const char* pDir, const char* pPrefix, const char* pSuffix)
{
	int fileCount = m_projectionList.size();
	std::string fileName(pDir);
	fileName += '/';
	fileName += pPrefix;
	for (int i = 0; i < fileCount; i++)
	{
		//std::cout << fileName + std::to_string(i) + pSuffix << std::endl;
		m_projectionList[i].m_image = cv::imread(fileName + std::to_string(i) + pSuffix, CV_8UC1);
	}
}

void Model::getModel()
{
	////*遍历空间中的所有点，判断该点是否位于visualhull内部
	//for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
	//	for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
	//		for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
	//			for (int i = 0; i < prejectionCount; i++)
	//			{
	//				double coorX = m_corrX.index2coor(indexX);
	//				double coorY = m_corrY.index2coor(indexY);
	//				double coorZ = m_corrZ.index2coor(indexZ);
	//				m_voxel[indexX][indexY][indexZ] = m_voxel[indexX][indexY][indexZ] && m_projectionList[i].checkRange(coorX, coorY, coorZ);
	//			}
	//int indexX, indexY, indexZ;
	//double coorX, coorY, coorZ;
	//bool inside;
	//bool flag = true;
	//const int th = 30000;
	//queue<Point> q;
	////找到一个内部的点，作为开始
	//for (indexX = 0; indexX < m_corrX.m_resolution && flag; indexX++)
	//{
	//	for (indexY = 0; indexY < m_corrY.m_resolution && flag; indexY++)
	//	{
	//		for (indexZ = 0; indexZ < m_corrZ.m_resolution && flag; indexZ++)
	//		{
	//			inside = insideHull(indexX, indexY, indexZ);
	//			m_voxel[indexX][indexY][indexZ].setIndex(indexX, indexY, indexZ);
	//			m_voxel[indexX][indexY][indexZ].visited = true;
	//			if (inside)
	//			{
	//				m_voxel[indexX][indexY][indexZ].inside = true;
	//				//q.push(m_voxel[indexX][indexY][indexZ]);
	//				//if (q.size() == th)
	//				//flag = false;
	//			}
	//			else
	//				m_voxel[indexX][indexY][indexZ].inside = false;
	//		}
	//	}
	//}
}

void Model::getSurface()
{

	//for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
	//	for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
	//		for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
	//		{
	//			Point p(indexX, indexY, indexZ);
	//			insideHull(p);
	//			if (voxel(p))
	//			{
	//				bool ans = false;
	//				for (int i = 0; i < 6; i++)
	//				{
	//					Point _p = p + dp[i];
	//					if (!visited(_p) && !outOfRange(indexX + dx[i], indexY + dy[i], indexZ + dz[i]))
	//					{
	//						insideHull(_p);
	//					}
	//					ans = ans || outOfRange(_p)
	//						|| !voxel(_p);
	//				}
	//				setSurface(p, ans);
	//				if (ans)
	//				{
	//					//cout << p.x << " " << p.y << " " << p.z << endl;
	//					BFS(p);
	//					surfacePoints.push_back(p);
	//					return;
	//				}
	//			}
	//		}

	int midx = m_corrX.m_resolution / 2;
	int midy = m_corrY.m_resolution / 2;
	int midz = m_corrZ.m_resolution / 2;

	int indexX, indexY, indexZ;
	int countx = 0, county = 0, countz = 0;
	int sign[3] = { 1, 1, 1 };

	int Mx = 4 * (m_corrX.m_resolution / 4) + 7;
	int My = 4 * (m_corrY.m_resolution / 4) + 7;
	int Mz = 4 * (m_corrZ.m_resolution / 4) + 7;

	while (countx < Mx)
	{
		indexX = (midx + sign[0] * (countx / 2) *(countx / 2)) % Mx;
		indexX = (indexX < 0) ? indexX + Mx : indexX;
		while (county < My)
		{
			indexY = (midy + sign[1] * (county / 2) * (county / 2)) % My;
			indexY = (indexY < 0) ? indexY + My : indexY;
			while (countz < Mz)
			{
				indexZ = (midz + sign[2] * (countz / 2) * (countz / 2)) % Mz;
				indexZ = (indexZ < 0) ? indexZ + Mz : indexZ;
				Point p(indexX, indexY, indexZ);
				insideHull(p);
				if (voxel(p))
				{
					bool ans = false;
					for (int i = 0; i < 6; i++)
					{
						Point _p = p + dp[i];
						if (!visited(_p) && !outOfRange(indexX + dx[i], indexY + dy[i], indexZ + dz[i]))
						{
							insideHull(_p);
						}
						ans = ans || outOfRange(_p)
							|| !voxel(_p);
					}
					setSurface(p, ans);
					if (ans)
					{
						BFS(p);
						surfacePoints.push_back(p);
						return;
					}
				}
				sign[2] *= -1;
				countz += 1;
			}
			sign[1] *= -1;
			county += 1;
		}
		sign[0] *= -1;
		countx += 1;
	}

}

//use BFS to find all the surface points
void Model::BFS(Point p)
{

	setEnqueued(p);
	queue<Point> s;
	s.push(p);
	bool ans;

	while (!s.empty())
	{
		p = s.front(); s.pop();
		Point temp = p;
		ans = false;
		for (int i = 0; i < 6; i++)
		{
			Point _p = dp[i] + p;
			if (!outOfRange(_p))
			{
				if (insideHull(_p))
				{
					if (!enqueued(_p) && !totalInside(_p))
					{
						setEnqueued(_p);
						s.push(_p);
					}
				}
			}
			ans = ans || outOfRange(_p) ||
				!voxel(_p);
		}
		setSurface(p, ans);
		if (ans)
			surfacePoints.push_back(p);
	}
}

Eigen::Vector3f Model::getNormal(int indX, int indY, int indZ)
{
	std::vector<Eigen::Vector3f> neiborList;
	std::vector<Eigen::Vector3f> innerList;

	for (int dX = -m_neiborSize; dX <= m_neiborSize; dX++)
		for (int dY = -m_neiborSize; dY <= m_neiborSize; dY++)
			for (int dZ = -m_neiborSize; dZ <= m_neiborSize; dZ++)
			{
				if (!dX && !dY && !dZ)
					continue;
				int neiborX = indX + dX;
				int neiborY = indY + dY;
				int neiborZ = indZ + dZ;
				if (!outOfRange(neiborX, neiborY, neiborZ))
				{
					float coorX = m_corrX.index2coor(neiborX);
					float coorY = m_corrY.index2coor(neiborY);
					float coorZ = m_corrZ.index2coor(neiborZ);
					if (!m_visited[neiborX][neiborY][neiborZ])
						insideHull(neiborX, neiborY, neiborZ);
					if (m_surface[neiborX][neiborY][neiborZ])
						neiborList.push_back(Eigen::Vector3f(coorX, coorY, coorZ));
					else if (m_voxel[neiborX][neiborY][neiborZ])
						innerList.push_back(Eigen::Vector3f(coorX, coorY, coorZ));
				}
			}

	Eigen::Vector3f point(m_corrX.index2coor(indX), m_corrY.index2coor(indY), m_corrZ.index2coor(indZ));

	//PCA
	Eigen::MatrixXf matA(3, neiborList.size());
	for (int i = 0; i < neiborList.size(); i++)
		matA.col(i) = neiborList[i] - point;
	//计算特征值
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(matA * matA.transpose());
	Eigen::Vector3f eigenValues = eigenSolver.eigenvalues();

	//选取最小的一个
	int indexEigen = 0;
	if (abs(eigenValues[1]) < abs(eigenValues[indexEigen]))
		indexEigen = 1;
	if (abs(eigenValues[2]) < abs(eigenValues[indexEigen]))
		indexEigen = 2;
	Eigen::Vector3f normalVector = eigenSolver.eigenvectors().col(indexEigen);

	Eigen::Vector3f innerCenter = Eigen::Vector3f::Zero();
	for (auto const& vec : innerList)
		innerCenter += vec;
	innerCenter /= innerList.size();

	if (normalVector.dot(point - innerCenter) < 0)
		normalVector *= -1;
	return normalVector;
}

bool Model::outOfRange(int indexX, int indexY, int indexZ)
{
	return indexX < 0 || indexY < 0 || indexZ < 0
		|| indexX >= m_corrX.m_resolution
		|| indexY >= m_corrY.m_resolution
		|| indexZ >= m_corrZ.m_resolution;
}

bool Model::insideHull(int indexX, int indexY, int indexZ)
{
	if (m_visited[indexX][indexY][indexZ])
		return m_voxel[indexX][indexY][indexZ];
	int prejectionCount = m_projectionList.size();
	double coorX, coorY, coorZ;
	for (int i = 0; i < prejectionCount; i++)
	{
		coorX = m_corrX.index2coor(indexX);
		coorY = m_corrY.index2coor(indexY);
		coorZ = m_corrZ.index2coor(indexZ);
		m_voxel[indexX][indexY][indexZ] =
			m_voxel[indexX][indexY][indexZ] &&
			m_projectionList[i].checkRange(coorX, coorY, coorZ);
	}
	m_visited[indexX][indexY][indexZ] = true;
	return m_voxel[indexX][indexY][indexZ];
}

bool Model::totalInside(const Point & p)
{
	if (!insideHull(p))
		return false;

	Point _p;
	for (int i = 6; i < 13; i++)
	{
		_p = dp[i] + p;
		if (!insideHull(_p))
			return false;
	}
	return true;
}

ofstream & operator<<(ofstream & os, Point & p)
{
	os << p.x << " " << p.y << " " << p.z << endl;
	return os;
}
