#include "FeaturesMatcher1.hpp"
#include "LoadImgFromPath.hpp"
#include "CELLTimestamp.hpp"
void readTxt(string file, vector<string>&filename)
{
	ifstream infile;
	infile.open(file.data());   //���ļ����������ļ��������� 
	assert(infile.is_open());   //��ʧ��,�����������Ϣ,����ֹ�������� 

	string s;

	while (getline(infile, s))
	{
		filename.push_back(s);
	}
	infile.close();             //�ر��ļ������� 
}
vector<string>getFileName(string filename)
{

	vector<string>fileinfo;
	//1.��ȡ����·�����ļ���
	string::size_type iPos = filename.find_last_of('\\') + 1;
	string filename1 = filename.substr(iPos, filename.length() - iPos);
	//2.��ȡ������׺���ļ���
	string name = filename1.substr(0, filename1.rfind("."));
	fileinfo.push_back(name);
	//3.��ȡ��׺��
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
	//��ʾͼ������
	vector<ImageFeatures> features(NUM_IMAGES);
	//��ʾ����ƥ����Ϣ����
	vector<MatchesInfo> pairwise_matches;
	//ͼƬ����������ƥ��
	LIFP->featureMatch(features, pairwise_matches);
	//�����������
	vector<CameraParams> cameras;
	LIFP->GetCameraParmas(features, pairwise_matches, cameras);
	LIFP->GetStichingResult(cameras, resultPath);
	std::cout << _time.getElapsedSecond() << std::endl;
	getchar();
	return 0;
}