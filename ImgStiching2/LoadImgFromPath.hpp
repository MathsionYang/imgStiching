#ifndef _LoadImgFromPath_hpp_
#define _LoadImgFromPath_hpp_
#include "CommonHeade.hpp"
class ImgProcess
{
public:
	vector<Mat>  GetFiles(string path);
	void featureMatch(vector<ImageFeatures> &features, vector<MatchesInfo> &pairwise_matches);
	void GetCameraParmas(vector<ImageFeatures> features, vector<MatchesInfo> pairwise_matches, vector<CameraParams> &cameras);
	void GetStichingResult(vector<CameraParams> cameras, string savepath);
private:
	vector<Mat> imgs;
	vector<String> files;
};
vector<Mat> ImgProcess::GetFiles(string path)
{
	glob(path + "/*", files, false);
	if (files.size() > 0)
	{
		for (int i = 0; i < files.size(); i++)
		{
			Mat img = imread(files[i]);
			Mat res;
			resize(img, res, Size(img.cols, img.rows), 0, 0, INTER_LINEAR);// X Y����Сһ��
			imgs.push_back(res);
		}
		return imgs;
	}
}
void ImgProcess::featureMatch(vector<ImageFeatures> &features, vector<MatchesInfo> &pairwise_matches)
{

	Ptr<OrbFeaturesFinder> finder;    //��������Ѱ����
	finder = new  OrbFeaturesFinder(Size(3,1),600);    //Ӧ��ORB����Ѱ������

	for (int i = 0; i < imgs.size(); i++)
		(*finder)(imgs[i], features[i]);    //�������

	BestOf2NearestMatcher1 matcher(false, 0.3f, 6, 6);    //��������ƥ������2NN����
	matcher(features, pairwise_matches);    //��������ƥ��
}
void ImgProcess::GetCameraParmas(vector<ImageFeatures> features, vector<MatchesInfo> pairwise_matches, vector<CameraParams> &cameras)
{
	//�������������
	HomographyBasedEstimator estimator;    
	//��ʾ�������  ���������������
	estimator(features, pairwise_matches, cameras);  
	//ת�������ת��������������
	for (size_t i = 0; i < cameras.size(); ++i)    
	{
		Mat R;
		cameras[i].R.convertTo(R, CV_32F);
		cameras[i].R = R;
	}
	//����ƽ�����ȷ�������
	Ptr<detail::BundleAdjusterBase> adjuster;
	//���߷�ɢ����
	adjuster = new detail::BundleAdjusterRay();    
	//����ƥ�����Ŷȣ���ֵ��Ϊ1
	adjuster->setConfThresh(1);  
	//��ȷ�����������
	(*adjuster)(features, pairwise_matches, cameras);   
}
void ImgProcess::GetStichingResult(vector<CameraParams> cameras,string savepath)
{
	int NUM_IMAGES = imgs.size();
	vector<Point> corners(NUM_IMAGES);    //��ʾӳ��任��ͼ������Ͻ�����
	vector<UMat> masks_warped(NUM_IMAGES);    //��ʾӳ��任���ͼ������
	vector<UMat> images_warped(NUM_IMAGES);    //��ʾӳ��任���ͼ��
	vector<Size> sizes(NUM_IMAGES);    //��ʾӳ��任���ͼ��ߴ�
	vector<Mat> masks(NUM_IMAGES);    //��ʾԴͼ������

	for (int i = 0; i < NUM_IMAGES; ++i)    //��ʼ��Դͼ������
	{
		masks[i].create(imgs[i].size(), CV_8U);    //����ߴ��С
		masks[i].setTo(Scalar::all(255));    //ȫ����ֵΪ255����ʾԴͼ����������ʹ��
	}

	Ptr<WarperCreator> warper_creator;    //����ͼ��ӳ��任������
	warper_creator = new cv::CylindricalWarper();    //����ͶӰ
	//����ͼ��ӳ��任��������ӳ��ĳ߶�Ϊ����Ľ��࣬��������Ľ��඼��ͬ
	Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(cameras[0].focal));
	for (int i = 0; i < NUM_IMAGES; ++i)
	{
		Mat_<float> K;
		cameras[i].K().convertTo(K, CV_32F);    //ת������ڲ�������������
		//�Ե�ǰͼ����ͶӰ�任���õ��任���ͼ���Լ���ͼ������Ͻ�����
		corners[i] = warper->warp(imgs[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
		sizes[i] = images_warped[i].size();    //�õ��ߴ�
		//�õ��任���ͼ������
		warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
	}
	imgs.clear();    //�����
	masks.clear();

	//�����عⲹ������Ӧ�����油������
	Ptr<ExposureCompensator> compensator =ExposureCompensator::createDefault(ExposureCompensator::GAIN);
	compensator->feed(corners, images_warped, masks_warped);    //�õ��عⲹ����
	for (int i = 0; i < NUM_IMAGES; ++i)    //Ӧ���عⲹ��������ͼ������عⲹ��
	{
		compensator->apply(i, corners[i], images_warped[i], masks_warped[i]);
	}

	//�ں��棬���ǻ���Ҫ�õ�ӳ��任ͼ������masks_warped���������Ϊ�ñ������һ������masks_seam
	vector<UMat> masks_seam(NUM_IMAGES);
	for (int i = 0; i < NUM_IMAGES; i++)
		masks_warped[i].copyTo(masks_seam[i]);
	Ptr<SeamFinder> seam_finder;    //����ӷ���Ѱ����
	//ͼ�
	seam_finder = new GraphCutSeamFinder(GraphCutSeamFinder::COST_COLOR);
	//seam_finder = new GraphCutSeamFinder(GraphCutSeamFinder::COST_COLOR_GRAD);
	vector<UMat> images_warped_f(NUM_IMAGES);
	for (int i = 0; i < NUM_IMAGES; ++i)    //ͼ����������ת��
		images_warped[i].convertTo(images_warped_f[i], CV_32F);
	//�õ��ӷ��ߵ�����ͼ��masks_warped
	seam_finder->find(images_warped_f, corners, masks_warped);
	vector<Mat> images_warped_s(NUM_IMAGES);
	Ptr<Blender> blender;    //����ͼ���ں���
	//blender = Blender::createDefault(Blender::MULTI_BAND, false);    //��Ƶ���ں�
	//MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(static_cast<Blender*>(blender));

	blender = Blender::createDefault(Blender::NO, false);    //���ںϷ���
	//���ںϷ���
	blender = Blender::createDefault(Blender::FEATHER, false);
	FeatherBlender* fb = dynamic_cast<FeatherBlender*>(static_cast<Blender*>(blender));
	fb->setSharpness(5);    //���������
	//cout << sizes[0] << endl;
	//cout << sizes[1] << endl;
	blender->prepare(corners, sizes);    //����ȫ��ͼ������

	//���ںϵ�ʱ������Ҫ�����ڽӷ���������д�������һ����Ѱ�ҽӷ��ߺ�õ�������ı߽���ǽӷ��ߴ���������ǻ���Ҫ�ڽӷ������࿪��һ�����������ںϴ�����һ������̶��𻯷�����Ϊ�ؼ�
	//Ӧ�������㷨��С�������
	vector<Mat> dilate_img(NUM_IMAGES);
	Mat element = getStructuringElement(MORPH_RECT, Size(20, 20));    //����ṹԪ��
	vector<Mat> a(NUM_IMAGES);
	vector<Mat> b(NUM_IMAGES);
	vector<Mat> c(NUM_IMAGES);
	for (int k = 0; k < NUM_IMAGES; k++)
	{
		images_warped_f[k].convertTo(images_warped_s[k], CV_16S);    //�ı���������
		dilate(masks_seam[k], masks_seam[k], element);    //��������
		masks_seam[k].copyTo(a[k]);
		masks_warped[k].copyTo(b[k]);
		c[k] = a[k] & b[k];
		c[k].copyTo(masks_seam[k]);
		blender->feed(images_warped_s[k], masks_seam[k], corners[k]);    //��ʼ������
	}

	//���ڴ�
	masks_seam.clear();
	images_warped_s.clear();
	masks_warped.clear();
	images_warped_f.clear();

	Mat result, result_mask;
	//����ںϲ������õ�ȫ��ͼ��result����������result_mask
	blender->blend(result, result_mask);

	imwrite(savepath, result);    //�洢ȫ��ͼ��
}

#endif