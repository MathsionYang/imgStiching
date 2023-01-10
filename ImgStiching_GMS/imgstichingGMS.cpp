//#include "LoadImgFromPath.hpp"
//#include "FeatureDetectAndMatch.hpp"
//int main()
//{
//	string path = "C:\\Users\\Think\\Desktop\\RS2\\v";
//	////��ȡӰ���ļ�
//	vector<Mat> imgs;
//	LoadImgFromPath LIFP;
//	imgs = LIFP.GetFiles(path);
//	int NUM_IMAGES = imgs.size();
//	cout << "�ļ�����:" << imgs.size() << endl;
//
//	////��������ȡ��ƥ��
//	vector<ImageFeatures> features;
//	vector<MatchesInfo> pairwise_matches;    //��ʾ����ƥ����Ϣ����
//
//	FeatureDetectAndMatch *FDAM1 = new FeatureDetectAndMatch();
//	FDAM1->GetImgs(imgs);
//	features = FDAM1->GetFeauture();
//	return 0;
//}
#include "CommonHeade.h"
#include "CELLTimestamp.hpp"
using namespace std;
using namespace cv;
using namespace cv::detail;
using namespace cv::xfeatures2d;


void OptimizeSeam(Mat& img1, Mat& trans, Mat& dst);

typedef struct
{
	Point2f left_top;
	Point2f left_bottom;
	Point2f right_top;
	Point2f right_bottom;
}four_corners_t;

four_corners_t corners;

void CalcCorners(const Mat& H, const Mat& src)
{
	double v2[] = { 0, 0, 1 };//���Ͻ�
	double v1[3];//�任�������ֵ
	Mat V2 = Mat(3, 1, CV_64FC1, v2);  //������
	Mat V1 = Mat(3, 1, CV_64FC1, v1);  //������

	V1 = H * V2;
	//���Ͻ�(0,0,1)
	
	corners.left_top.x = v1[0] / v1[2];
	corners.left_top.y = v1[1] / v1[2];

	//���½�(0,src.rows,1)
	v2[0] = 0;
	v2[1] = src.rows;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //������
	V1 = Mat(3, 1, CV_64FC1, v1);  //������
	V1 = H * V2;
	corners.left_bottom.x = v1[0] / v1[2];
	corners.left_bottom.y = v1[1] / v1[2];

	//���Ͻ�(src.cols,0,1)
	v2[0] = src.cols;
	v2[1] = 0;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //������
	V1 = Mat(3, 1, CV_64FC1, v1);  //������
	V1 = H * V2;
	corners.right_top.x = v1[0] / v1[2];
	corners.right_top.y = v1[1] / v1[2];

	//���½�(src.cols,src.rows,1)
	v2[0] = src.cols;
	v2[1] = src.rows;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //������
	V1 = Mat(3, 1, CV_64FC1, v1);  //������
	V1 = H * V2;
	corners.right_bottom.x = v1[0] / v1[2];
	corners.right_bottom.y = v1[1] / v1[2];

}
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
void ReadKeyPtFile(string savePtPath, vector<Point2f>&pt1, vector<Point2f>&pt2)
{
	int nSize;
	FILE *fp1 = fopen(const_cast<char*>(savePtPath.c_str()), "r");
	fscanf(fp1, "%d\n", &nSize);
	for (int i = 0; i < nSize; i++)
	{
		float fx1, fy1;
		float fx2, fy2;
		fscanf(fp1, "%f	%f %f %f\n", &fx1, &fy1, &fx2, &fy2);
		Point2f kpt1,kpt2;
		kpt1.x = fx1;	kpt1.y = fy1;
		kpt2.x = fx2;	kpt2.y = fy2;
		pt1.push_back(kpt1);
		pt2.push_back(kpt2);
	}
	std::fclose(fp1);

}

int main(int argc, char *argv[])
{
	
	/*string path = "F:\\Mysql\\uploadimg\\CParmars\\text_stiching.txt";
	vector<string>filename;
	readTxt(path, filename);*/
	CELLTimestamp _time;
	Mat image01 = cv::imread("E:\\daspatial\\colorchange\\colorchange\\image\\change.jpg");
	Mat image02 = cv::imread("E:\\daspatial\\colorchange\\colorchange\\image\\srcMat.jpg");
	//���ý���ļ��� name = src_img1.name_src_img2.name.Ex;
	/*vector<string>name1 = getFileName(filename[0]);
	vector<string>name2 = getFileName(filename[1]);*/
	string resultPath = "E:\\daspatial\\colorchange\\colorchange\\image\\match.jpg";

	////�Ҷ�ͼת��  
	Mat image1, image2;
	cvtColor(image01, image1, CV_RGB2GRAY);
	cvtColor(image02, image2, CV_RGB2GRAY);


	////��ȡ������    
	Ptr<FeatureDetector> Detector = cv::xfeatures2d::SURF::create(1000);
	vector<KeyPoint> keyPoint1, keyPoint2;
	Detector->detect(image1, keyPoint1);
	Detector->detect(image2, keyPoint2);

	//������������Ϊ�±ߵ�������ƥ����׼��    
	
	Mat imageDesc1, imageDesc2;
	Detector->compute(image1, keyPoint1, imageDesc1);
	Detector->compute(image2, keyPoint2, imageDesc2);

	FlannBasedMatcher matcher;
	vector<vector<DMatch> > matchePoints;
	vector<DMatch> GoodMatchePoints;

	vector<Mat> train_desc(1, imageDesc1);
	matcher.add(train_desc);
	matcher.train();

	matcher.knnMatch(imageDesc2, matchePoints, 2);

	// Lowe's algorithm,��ȡ����ƥ���
	for (int i = 0; i < matchePoints.size(); i++)
	{
		if (matchePoints[i][0].distance < 0.75 * matchePoints[i][1].distance)
		{
			GoodMatchePoints.push_back(matchePoints[i][0]);
		}
	}

	Mat outImg;

	drawMatches(image02, keyPoint2, image01,keyPoint1,GoodMatchePoints,outImg);
	imwrite(resultPath, outImg);

	//
	vector<Point2f> imagePoints1, imagePoints2;

	for (int i = 0; i < GoodMatchePoints.size(); i++)
	{
		imagePoints2.push_back(keyPoint2[GoodMatchePoints[i].queryIdx].pt);
		imagePoints1.push_back(keyPoint1[GoodMatchePoints[i].trainIdx].pt);
	}



	////��ȡͼ��1��ͼ��2��ͶӰӳ����� �ߴ�Ϊ3*3  
	//
	//////Ҳ����ʹ��getPerspectiveTransform�������͸�ӱ任���󣬲���Ҫ��ֻ����4���㣬Ч���Բ�  
	////Mat   homo=getPerspectiveTransform(imagePoints1,imagePoints2);  
	//

 //  //������׼ͼ���ĸ���������
	/*string txtpath = "E:\\daspatial\\lasdata\\02\\match.txt";
	vector<Point2f>pt1, pt2;
	ReadKeyPtFile(txtpath, pt1, pt2);*/

	//Mat homo = findHomography(imagePoints2, imagePoints1, CV_RANSAC);
	//CalcCorners(homo, image01);
	//

	////ͼ����׼  
	//Mat imageTransform1, imageTransform2;
	//warpPerspective(image01, imageTransform1, homo, Size(MAX(corners.right_top.x, corners.right_bottom.x), image02.rows));
	////warpPerspective(image01, imageTransform2, adjustMat*homo, Size(image02.cols*1.3, image02.rows*1.8));
	//


	////����ƴ�Ӻ��ͼ,����ǰ����ͼ�Ĵ�С
	//int dst_width = imageTransform1.cols;  //ȡ���ҵ�ĳ���Ϊƴ��ͼ�ĳ���
	//int dst_height = image02.rows;

	//Mat dst(dst_height, dst_width, CV_8UC3);
	//dst.setTo(0);

	//imageTransform1.copyTo(dst(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
	//image02.copyTo(dst(Rect(0, 0, image02.cols, image02.rows)));

	//imwrite(resultPath, dst);

	//OptimizeSeam(image02, imageTransform1, dst);
	//string resultPath1 = "E:\\daspatial\\lasdata\\02\\finall.jpg";
	//imwrite(resultPath1, dst);

	std::cout << _time.getElapsedSecond() << std::endl;

	return 0;
}


//�Ż���ͼ�����Ӵ���ʹ��ƴ����Ȼ
void OptimizeSeam(Mat& img1, Mat& trans, Mat& dst)
{
	int start =abs( MIN(corners.left_top.x, corners.left_bottom.x));//��ʼλ�ã����ص��������߽�  

	double processWidth = img1.cols - start;//�ص�����Ŀ��  
	int rows = dst.rows;
	int cols = img1.cols; //ע�⣬������*ͨ����
	double alpha = 1;//img1�����ص�Ȩ��  
	for (int i = 0; i < rows; i++)
	{
		uchar* p = img1.ptr<uchar>(i);  //��ȡ��i�е��׵�ַ
		uchar* t = trans.ptr<uchar>(i);
		uchar* d = dst.ptr<uchar>(i);
		for (int j = start; j < cols; j++)
		{
			//�������ͼ��trans�������صĺڵ㣬����ȫ����img1�е�����
			if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
			{
				alpha = 1;
			}
			else
			{
				//img1�����ص�Ȩ�أ��뵱ǰ�������ص�������߽�ľ�������ȣ�ʵ��֤�������ַ���ȷʵ��  
				alpha = (processWidth - (j - start)) / processWidth;
			}

			d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
			d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
			d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);

		}
	}

}