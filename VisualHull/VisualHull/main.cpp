#pragma warning(disable:4819)
#pragma warning(disable:4244)
#pragma warning(disable:4267)

#include <time.h>
#include <iostream>
#include <fstream>
#include <thread>
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
	model.loadImage("../../wd_segmented", "WD2_", "_00020_segmented.png", "../../wd_data","WD2_" ,"_00020.png" );

	last = clock() - last;
	std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	fout << "|load file|" << (float(last) / CLOCKS_PER_SEC) << "|" << endl;
	sum += (float(last) / CLOCKS_PER_SEC);
	last = clock();

	// 得到Voxel模型
	//model.getModel();
	//std::cout << "get model done\n";
	//last = clock() - last;
	//std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	//fout << "|get model|" << (float(last) / CLOCKS_PER_SEC) << "|" << endl;
	//sum += (float(last) / CLOCKS_PER_SEC);
	//last = clock();

	// 获得Voxel模型的表面
	model.getSurface();
	std::cout << "get surface done\n";

	last = clock() - last;
	std::cout << "time: " << (float(last) / CLOCKS_PER_SEC) << "seconds\n";
	fout << "|get surface|" << (float(last) / CLOCKS_PER_SEC) << "|" << endl;
	sum += (float(last) / CLOCKS_PER_SEC);

	int sum1 = 0;
	int sum2 = 0;

	auto without_normal = [&]() {
		// 将模型导出为xyz格式（不含法向信息）
		cout << "without normal" << endl;
		clock_t la = clock();

		model.saveModel("../../WithoutNormal.xyz");
		std::cout << "save without normal done\n";
		la = clock() - la;
		std::cout << "time: " << (float(la) / CLOCKS_PER_SEC) << "seconds\n";
		fout << "|save without normal|" << (float(la) / CLOCKS_PER_SEC) << "|" << endl;
		sum1 += (float(la) / CLOCKS_PER_SEC);
	};

	auto with_normal = [&]() {
		// 将模型导出为xyz格式（含法向信息）
		cout << "with normal" << endl;
		clock_t la = clock();
		model.saveModelWithNormal("../../WithNormal.xyz");
		std::cout << "save with normal done\n";
		fout << "|save with normal|" << (float(la) / CLOCKS_PER_SEC) << "|" << endl;
		sum2 += (float(la) / CLOCKS_PER_SEC);
		la = clock() - la;
		std::cout << "time: " << (float(la) / CLOCKS_PER_SEC) << "seconds\n";

		la = clock();
		model.getColor();
		model.savePly("../../WithNormal.ply");
		std::cout << "save with color done\n";

		la = clock() - la;
		std::cout << "time: " << (float(la) / CLOCKS_PER_SEC) << "seconds\n";
		fout << "|save with color|" << (float(la) / CLOCKS_PER_SEC) << "|" << endl;
		sum2 += (float(la) / CLOCKS_PER_SEC);

		//泊松重建算法
		la = clock();
		//system("PoissonRecon.x64 --in ../../WithNormal.xyz --out ../../mesh.ply");
		//system("PoissonRecon_color --in ../../WithNormal.ply --out ../../mesh.ply");
		system("PoissonRecon_color --in ../../WithNormal.ply --out ../mesh.ply --color 16");
		std::cout << "save mesh.ply done\n";
		la = clock() - la;
		std::cout << "time: " << (float(la) / CLOCKS_PER_SEC) << "seconds\n";
		fout << "|save mesh.ply|" << (float(la) / CLOCKS_PER_SEC) << "|" << endl;
		sum2 += (float(la) / CLOCKS_PER_SEC);
	};

	thread with_nm(with_normal);
	thread wout_nm(without_normal);
	with_nm.join();
	wout_nm.join();

	sum = sum + sum1 + sum2;
	fout << "|total|" << sum << "|" << endl;
	fout << "---------------" << endl;
	t = clock() - t;// cout << "real time cost" << float(clock() - t) / CLOCKS_PER_SEC << endl;
	cout << "sum time" << sum << endl;
	std::cout << "program time: " << (float(t) / CLOCKS_PER_SEC) << "seconds\n";
	return (0);
}

//void cvtest()
//{
//	cv::Vec3f data = cv::Vec3b(255, 255, 255);
//	for (int i = 0; i < 10000; i++)
//	{
//		//cout << data;
//		data += cv::Vec3b(1, 1, 1);
//	}
//	cout << data;
//	//cv::Mat mat = cv::imread("test.png");
//	//for (int i = 0; i < mat.size().height; i++)
//	//{
//	//	for (int j = 0; j < mat.size().width; j++)
//	//	{
//	//		data = mat.at<cv::Vec3b>(i, j);
//	//		if (data != cv::Vec3b(255, 255, 255))
//	//		cout << "pos: i = " << i << "j = " << j << endl << "data = " << data << endl;
//	//	}
//	//}
//}
//int main()
//{
//	cvtest();
//}