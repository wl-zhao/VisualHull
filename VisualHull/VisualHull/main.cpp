#pragma warning(disable:4819)
#pragma warning(disable:4244)
#pragma warning(disable:4267)

#include <time.h>
#include <iostream>
#include <fstream>
#include "Model.h"


int main(int argc, char** argv)
{
	clock_t t = clock();
	clock_t last = clock();

	// 分别设置xyz方向的Voxel分辨率
	Model model(300, 300, 300);

	// 读取相机的内外参数
	model.loadMatrix("../../calibParamsI.txt");

	// 读取投影图片
	model.loadImage("../../wd_segmented", "WD2_", "_00020_segmented.png");

	last = clock() - last;
	std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	last = clock();
	// 得到Voxel模型
	model.getModel();
	std::cout << "get model done\n";

	last = clock() - last;
	std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	last = clock();
	// 获得Voxel模型的表面
	model.getSurface();
	std::cout << "get surface done\n";

	last = clock() - last;
	std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	last = clock();
	// 将模型导出为xyz格式
	model.saveModel("../../WithoutNormal.xyz");
	std::cout << "save without normal done\n";

	last = clock() - last;
	std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	last = clock();
	model.saveModelWithNormal("../../WithNormal.xyz");
	std::cout << "save with normal done\n";

	last = clock() - last;
	std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	last = clock();
	system("PoissonRecon.x64 --in ../../WithNormal.xyz --out ../../mesh.ply");
	std::cout << "save mesh.ply done\n";

	last = clock() - last;
	std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	last = clock();
	t = clock() - t;
	std::cout << "time: " << (float(t) / CLOCKS_PER_SEC) << "seconds\n";

	return (0);
}