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
	m_voxel = Voxel(m_corrX.m_resolution, Pixel(m_corrY.m_resolution, Line(m_corrZ.m_resolution, Point(true))));
	m_surface = Voxel(m_corrX.m_resolution, Pixel(m_corrY.m_resolution, Line(m_corrZ.m_resolution, Point(false))));
	//m_status 表示是否入队，这里用enqueue
	//m_status2 表示是否访问，这里用visited
}

Model::~Model()
{
}

//可用一个list存放所有的满足条件的m_surface信息
void Model::saveModel(const char* pFileName)
{
	std::ofstream fout(pFileName);

	for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
		for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
			for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
				if (m_surface[indexX][indexY][indexZ].inside)
				{
					double coorX = m_corrX.index2coor(indexX);
					double coorY = m_corrY.index2coor(indexY);
					double coorZ = m_corrZ.index2coor(indexZ);
					fout << coorX << ' ' << coorY << ' ' << coorZ << std::endl;
				}
}

void Model::saveModelWithNormal(const char* pFileName)
{
	std::ofstream fout(pFileName);

	double midX = m_corrX.index2coor(m_corrX.m_resolution / 2);
	double midY = m_corrY.index2coor(m_corrY.m_resolution / 2);
	double midZ = m_corrZ.index2coor(m_corrZ.m_resolution / 2);

	for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
		for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
			for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
				if (m_surface[indexX][indexY][indexZ].inside)
				{
					double coorX = m_corrX.index2coor(indexX);
					double coorY = m_corrY.index2coor(indexY);
					double coorZ = m_corrZ.index2coor(indexZ);
					fout << coorX << ' ' << coorY << ' ' << coorZ << ' ';

					Eigen::Vector3f nor = getNormal(indexX, indexY, indexZ);
					fout << nor(0) << ' ' << nor(1) << ' ' << nor(2) << std::endl;
				}
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
	//int prejectionCount = m_projectionList.size();

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
	// 邻域：上、下、左、右、前、后。
	int dx[6] = { -1, 0, 0, 0, 0, 1 };
	int dy[6] = { 0, 1, -1, 0, 0, 0 };
	int dz[6] = { 0, 0, 0, 1, -1, 0 };

	// lambda表达式，用于判断某个点是否在Voxel的范围内
	//auto outOfRange = [&](int indexX, int indexY, int indexZ) {
	//	return indexX < 0 || indexY < 0 || indexZ < 0
	//		|| indexX >= m_corrX.m_resolution
	//		|| indexY >= m_corrY.m_resolution
	//		|| indexZ >= m_corrZ.m_resolution;
	//};
	int visitcount = 0;

	for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
		for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
			for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
			{
				//m_voxel[indexX][indexY][indexZ].visited = true;
				visitcount++;
				insideHull(indexX, indexY, indexZ);
				if (m_voxel[indexX][indexY][indexZ].inside)
				{
					//	m_surface[indexX][indexY][indexZ] = false;
					bool ans = false;
					for (int i = 0; i < 6; i++)
					{
						if (!m_voxel[indexX + dx[i]][indexY + dy[i]][indexZ + dz[i]].visited &&
							!outOfRange(indexX + dx[i], indexY + dy[i], indexZ + dz[i]))
						{
							insideHull(indexX + dx[i], indexY + dy[i], indexZ + dz[i]);
							m_voxel[indexX + dx[i]][indexY + dy[i]][indexZ + dz[i]].visited = true;
							visitcount++;
						}
						ans = ans || outOfRange(indexX + dx[i], indexY + dy[i], indexZ + dz[i])
							|| !m_voxel[indexX + dx[i]][indexY + dy[i]][indexZ + dz[i]].inside;
					}
					m_surface[indexX][indexY][indexZ] = ans;
					if (ans)
					{
						cout << indexX << " " << indexY << " " << indexZ << " " << endl;
						BFS(indexX, indexY, indexZ);
						cout << "visitcount" << visitcount << endl;
						return;
					}
				}
			}
}

//use BFS to find all the surface points
void Model::BFS(int indexX, int indexY, int indexZ)
{
	//int dx[6] = { -1, 0, 0, 0, 0, 1 };
	//int dy[6] = { 0, 1, -1, 0, 0, 0 };
	//int dz[6] = { 0, 0, 0, 1, -1, 0 };

	int dx[18] = { -1, 0, 0, 0, 0, 1 ,1, 1, -1, -1, 0, 0, 0, 0, 1, 1, -1, -1 };
	int dy[18] = { 0, 1, -1, 0, 0, 0 ,1, -1, 1, -1, 1, -1, 1,-1, 0, 0, 0, 0 };
	int dz[18] = { 0, 0, 0, 1, -1, 0 ,0, 0, 0, 0, 1, 1, -1, -1, 1, -1, 1,-1 };

	//queue<Point*> q;
	//Point *p = &m_voxel[indexX][indexY][indexZ];
	//Point *temp;
	//bool inside;
	//bool surface;
	//bool oor;//out of range
	//p->setIndex(indexX, indexY, indexZ);
	//p->enqueued = true;
	//q.push(p);
	//while (!q.empty())
	//{
	//	p = q.front();
	//	q.pop();
	//	surface = false;
	//	for (int i = 0; i < 18; i++)
	//	{
	//		oor = outOfRange(p->x + dx[i], p->y + dy[i], p->z + dz[i]);
	//		if (!oor)
	//		{
	//			temp = &m_voxel[p->x + dx[i]][p->y + dy[i]][p->z + dz[i]];
	//			if (!temp->visited)
	//			{
	//				inside = insideHull(p->x + dx[i], p->y + dy[i], p->z + dz[i]);
	//				temp->visited = true;
	//			}
	//			if (inside)
	//			{
	//				if (!temp->enqueued)
	//				{
	//					temp->enqueued = true;
	//					temp->setIndex(p->x + dx[i], p->y + dy[i], p->z + dz[i]);
	//					q.push(temp);
	//				}
	//			}
	//		}
	//		if (i < 6)
	//			surface = surface || oor || !temp->inside;
	//		if (i == 6)
	//		{
	//			m_surface[p->x][p->y][p->z] = surface;
	//			if (!surface)
	//				break;
	//		}
	//	}
	//}
	Point p = m_voxel[indexX][indexY][indexZ];
	m_voxel[indexX][indexY][indexZ].setIndex(indexX, indexY, indexZ);
	m_voxel[indexX][indexY][indexZ].enqueued = true;
	queue<Point> s;
	s.push(m_voxel[indexX][indexY][indexZ]);

	while (!s.empty())
	{
		Point temp = s.front(); s.pop();
		bool ans = false;
		int count = 0;
		for (int i = 0; i < 18; i++)
		{
			if (!outOfRange(temp.x + dx[i], temp.y + dy[i], temp.z + dz[i]))
			{
				if (!m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]].visited)
				{
					insideHull(temp.x + dx[i], temp.y + dy[i], temp.z + dz[i]);
					m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]].visited = true;
				}
				if (m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]].inside)
				{
					if (!m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]].enqueued)
					{
						m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]].enqueued = true;
						m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]]\
							.setIndex(temp.x + dx[i], temp.y + dy[i], temp.z + dz[i]);
						s.push(m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]]);
					}
				}
			}
			if (i < 6)
				ans = ans || outOfRange(temp.x + dx[i], temp.y + dy[i], temp.z + dz[i]) ||
				!m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]].inside;
			if (i == 6)
			{
				m_surface[temp.x][temp.y][temp.z] = ans;
				if (!ans)
				{
					break;
				}
			}
		}
	}
}

void Model::DFS(int indexX, int indexY, int indexZ)
{
	int dx[18] = { -1, 0, 0, 0, 0, 1 ,1, 1, -1, -1, 0, 0, 0, 0, 1, 1, -1, -1 };
	int dy[18] = { 0, 1, -1, 0, 0, 0 ,1, -1, 1, -1, 1, -1, 1,-1, 0, 0, 0, 0 };
	int dz[18] = { 0, 0, 0, 1, -1, 0 ,0, 0, 0, 0, 1, 1, -1, -1, 1, -1, 1,-1 };
	//std::cout<<"DFS ing..."<<std::endl;
	std::queue<Point> s;
	Point p = m_voxel[indexX][indexY][indexZ];
	p.setIndex(indexX, indexY, indexZ);
	s.push(p);
	m_voxel[p.x][p.y][p.z].enqueued = true;
	std::vector<Point> buffer;
	int ct = 0;
	while (!s.empty())
	{
		ct++;
		Point temp = s.front(); s.pop();
		//std::cout<<s.size()<<std::endl;
		bool ans = false;
		//for (int i = 0; i < 6; i++)
		//{
		//	if (!m_status2[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]])
		//	{
		//		updateVoxel(Point(temp.x + dx[i],temp.y + dy[i],temp.z + dz[i]));
		//		m_status2[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]]=true;
		//	}
		//	ans = ans || myOutOfRange(temp.x + dx[i], temp.y + dy[i], temp.z + dz[i])
		//		|| !m_voxel[temp.x+ dx[i]][temp.y + dy[i]][temp.z + dz[i]];
		//}
		//m_surface[temp.x][temp.y][temp.z]=ans;
		int count = 0;
		buffer.clear();
		//surfacePoint.push_back(temp);
		for (int i = 0; i<18; i++)
		{
			if (!outOfRange(temp.x + dx[i], temp.y + dy[i], temp.z + dz[i]))
			{
				if (!m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]].visited)
				{
					insideHull(temp.x + dx[i], temp.y + dy[i], temp.z + dz[i]);
					m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]].visited = true;
				}

				if (m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]].inside)//inner
				{
					if (!m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]].enqueued)
					{
						m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]].enqueued = true;
						s.push(m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]]);
					}
				}
			}
			if (i<6)
				ans = ans || outOfRange(temp.x + dx[i], temp.y + dy[i], temp.z + dz[i])
				|| !m_voxel[temp.x + dx[i]][temp.y + dy[i]][temp.z + dz[i]].inside;
			if (i == 6)
			{
				m_surface[temp.x][temp.y][temp.z] = ans;
				//surfacePoint.push_back(temp);
				if (!ans)
				{
					break;
					//cout << "buffer count" << buffer.size()<<endl;
					////surfacePoint.push_back(temp);
					//for (int j = 0; j<buffer.size(); j++)
					//{
					//	m_status[buffer[j].x][buffer[j].y][buffer[j].z] = true;
					//	//s.push(buffer[j]);
					//}
				}
				else
					break;
			}
		}
	}
	cout << "ct : " << ct << endl;
}

Eigen::Vector3f Model::getNormal(int indX, int indY, int indZ)
{
	auto outOfRange = [&](int indexX, int indexY, int indexZ) {
		return indexX < 0 || indexY < 0 || indexZ < 0
			|| indexX >= m_corrX.m_resolution
			|| indexY >= m_corrY.m_resolution
			|| indexZ >= m_corrZ.m_resolution;
	};

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
					if (m_surface[neiborX][neiborY][neiborZ].inside)
						neiborList.push_back(Eigen::Vector3f(coorX, coorY, coorZ));
					else if (m_voxel[neiborX][neiborY][neiborZ].inside)
						innerList.push_back(Eigen::Vector3f(coorX, coorY, coorZ));
				}
			}

	Eigen::Vector3f point(m_corrX.index2coor(indX), m_corrY.index2coor(indY), m_corrZ.index2coor(indZ));

	Eigen::MatrixXf matA(3, neiborList.size());
	for (int i = 0; i < neiborList.size(); i++)
		matA.col(i) = neiborList[i] - point;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(matA * matA.transpose());
	Eigen::Vector3f eigenValues = eigenSolver.eigenvalues();
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
	int prejectionCount = m_projectionList.size();
	double coorX, coorY, coorZ;
	for (int i = 0; i < prejectionCount; i++)
	{
		coorX = m_corrX.index2coor(indexX);
		coorY = m_corrY.index2coor(indexY);
		coorZ = m_corrZ.index2coor(indexZ);
		m_voxel[indexX][indexY][indexZ].inside  =
			m_voxel[indexX][indexY][indexZ].inside &&
			m_projectionList[i].checkRange(coorX, coorY, coorZ);
	}
	return m_voxel[indexX][indexY][indexZ].inside;
}

