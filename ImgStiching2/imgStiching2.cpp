#include "FeaturesMatcher1.hpp"
#include "LoadImgFromPath.hpp"
#include "CELLTimestamp.hpp"
void readTxt(string file, vector<string>&filename)
{
	ifstream infile;
	infile.open(file.data());   //将文件流对象与文件连接起来 
	assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行 

	string s;

	while (getline(infile, s))
	{
		filename.push_back(s);
	}
	infile.close();             //关闭文件输入流 
}
vector<string>getFileName(string filename)
{

	vector<string>fileinfo;
	//1.获取不带路径的文件名
	string::size_type iPos = filename.find_last_of('\\') + 1;
	string filename1 = filename.substr(iPos, filename.length() - iPos);
	//2.获取不带后缀的文件名
	string name = filename1.substr(0, filename1.rfind("."));
	fileinfo.push_back(name);
	//3.获取后缀名
	string suffix_str = filename1.substr(filename1.find_last_of('.') + 1);
	fileinfo.push_back(suffix_str);
	return fileinfo;
}

int main()
{
	CELLTimestamp _time;
	string path = "E:\\daspatial\\lasdata\\02";
	string resultPath =  "img\\12_t.jpg";
	
	vector<String>filename;
	glob(path, filename);
	vector<Mat> imgs;
	for (int i = 0; i < filename.size(); i++)
	{
		imgs.push_back(imread(filename[i]));
	}
	ImgProcess *LIFP = new ImgProcess();
	LIFP->GetFiles(path);
	int NUM_IMAGES = imgs.size();
	//表示图像特征
	vector<ImageFeatures> features(NUM_IMAGES);
	//表示特征匹配信息变量
	vector<MatchesInfo> pairwise_matches;
	//图片特征点检测与匹配
	LIFP->featureMatch(features, pairwise_matches);
	//相机参数评估
	vector<CameraParams> cameras;
	LIFP->GetCameraParmas(features, pairwise_matches, cameras);
	LIFP->GetStichingResult(cameras, resultPath);
	std::cout << _time.getElapsedSecond() << std::endl;
	getchar();
	return 0;
}