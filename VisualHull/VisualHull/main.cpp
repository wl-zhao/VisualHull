#pragma warning(disable:4819)
#pragma warning(disable:4244)
#pragma warning(disable:4267)

#include <time.h>
#include <iostream>
#include <fstream>
#include <string>
#include "Model.h"
using namespace std;


int main(int argc, char** argv)
{
	ofstream fout("timelog.md", ios_base::app);
	string comment;
	float sum = 0;
	cout << "describe this version" << endl;
	cin >> comment;
	fout << "----------------" << endl;
	if (comment == "s")
		fout << "Skip the description" << endl;
	else
		fout << comment << endl;

	clock_t t = clock();
	clock_t last = clock();
	fout << "|步骤|耗时|" << endl;
	fout << "|-------|------|" << endl;

	// 分别设置xyz方向的Voxel分辨率
	Model model(300, 300, 300);

	// 读取相机的内外参数
	model.loadMatrix("../../calibParamsI.txt");

	// 读取投影图片
	model.loadImage("../../wd_segmented", "WD2_", "_00020_segmented.png");

	last = clock() - last;
	std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	fout << "|load file|" << (float(last) / CLOCKS_PER_SEC) << "|" << endl;
	sum += (float(last) / CLOCKS_PER_SEC);
	last = clock();

	// 得到Voxel模型
	model.getModel();
	std::cout << "get model done\n";

	last = clock() - last;
	std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	fout << "|get model|" << (float(last) / CLOCKS_PER_SEC) << "|" << endl;
	sum += (float(last) / CLOCKS_PER_SEC);
	last = clock();

	// 获得Voxel模型的表面
	model.getSurface();
	std::cout << "get surface done\n";

	last = clock() - last;
	std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	fout << "|get surface|" << (float(last) / CLOCKS_PER_SEC) << "|" << endl;
	sum += (float(last) / CLOCKS_PER_SEC);
	last = clock();

	// 将模型导出为xyz格式（不含法向信息）
	model.saveModel("../../WithoutNormal.xyz");
	std::cout << "save without normal done\n";

	last = clock() - last;
	std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	fout << "|save without normal|" << (float(last) / CLOCKS_PER_SEC) << "|" << endl;
	sum += (float(last) / CLOCKS_PER_SEC);
	last = clock();

	// 将模型导出为xyz格式（含法向信息）
	model.saveModelWithNormal("../../WithNormal.xyz");
	std::cout << "save with normal done\n";

	last = clock() - last;
	std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	fout << "|save with normal|" << (float(last) / CLOCKS_PER_SEC) << "|" << endl;
	sum += (float(last) / CLOCKS_PER_SEC);

	//泊松重建算法
	last = clock();
	system("PoissonRecon.x64 --in ../../WithNormal.xyz --out ../../mesh.ply");
	std::cout << "save mesh.ply done\n";


	last = clock() - last;
	std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	fout << "|save mesh.ply|" << (float(last) / CLOCKS_PER_SEC) << "|" << endl;
	sum += (float(last) / CLOCKS_PER_SEC);
	last = clock();
	t = clock() - t;
	std::cout << "time: " << (float(t) / CLOCKS_PER_SEC) << "seconds\n";
	fout << "|total|" << sum << "|" << endl;
	fout << "---------------" << endl;
	return (0);
}