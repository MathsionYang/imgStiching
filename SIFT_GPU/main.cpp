#include "SiftGPU2.h"
#include "gms_matcher.h"
#include "CommonHeader.hpp"
#include "CELLTimestamp.hpp"
#include<io.h>
using namespace std;
using namespace cv;

int main()
{
	string path = "E:/Papertest2/ImageStitching/SIFT_GPU/test";
	vector<String> files;
	glob(path+"/*",files,false);

	vector<Mat> imgs;
	for (int i = 0; i < files.size(); i++)
	{
		Mat img = imread(files[i]);
		imgs.push_back(img);
	}

	int num_images = imgs.size();    //ͼ������
	cout << "ͼ������Ϊ:" << num_images << endl;
	//��ʾͼ������
	vector<ImageFeatures> features(num_images);

	Ptr<FeaturesFinder> finder;    //��������Ѱ����
	finder = new SurfFeaturesFinder();    //Ӧ��SURF����Ѱ������
	for (int i = 0; i < num_images; i++)
		(*finder)(imgs[i], features[i]);    //�������
	cout << "������ȡ���" << endl;

	vector<MatchesInfo> pairwise_matches;    //��ʾ����ƥ����Ϣ����
	BestOf2NearestMatcher matcher(false, 0.35f, 6, 6);    //��������ƥ������2NN����
	matcher(features, pairwise_matches);    //��������ƥ��


	//GMSɸѡ
	//gms_matcher gms(kp1, image1.size(), kp2, image2.size(), matches);
	cout << "����ƥ�����" << endl;

	HomographyBasedEstimator estimator;    //�������������
	vector<CameraParams> cameras;    //��ʾ����������ڲμ����
	estimator(features, pairwise_matches, cameras);    //�������������
	for (size_t i = 0; i < cameras.size(); ++i)    //ת�������ת��������������
	{
		Mat R;
		cameras[i].R.convertTo(R, CV_32F);
		cameras[i].R = R;
	}
	cout << "�������Ԥ�����" << endl;

	for (size_t i = 0; i < cameras.size(); ++i)
	{
		cout << "��" << i << "����Ϊ" << cameras[i].focal << endl;
	}
	// ��һ�����Լ�����ӳ������취���������������
	Ptr<detail::BundleAdjusterBase> adjuster;    //����ƽ�����ȷ�������
	//adjuster->setRefinementMask();
	adjuster = new detail::BundleAdjusterReproj();    //��ӳ������
	//adjuster = new detail::BundleAdjusterRay();    //���߷�ɢ����

	adjuster->setConfThresh(1.0f);    //����ƥ�����Ŷȣ���ֵ��Ϊ1
	(*adjuster)(features, pairwise_matches, cameras);    //��ȷ�����������

	vector<Mat> rmats;
	for (size_t i = 0; i < cameras.size(); ++i)    //�����������ת����
		rmats.push_back(cameras[i].R.clone());
	waveCorrect(rmats, WAVE_CORRECT_HORIZ);    //���в���У��
	for (size_t i = 0; i < cameras.size(); ++i)    //���������ֵ
		cameras[i].R = rmats[i];
	rmats.clear();    //�����

	cout << "���ù���ƽ���������������" << endl;

	vector<Point> corners(num_images);    //��ʾӳ��任��ͼ������Ͻ�����
	vector<UMat> masks_warped(num_images);    //��ʾӳ��任���ͼ������
	vector<UMat> images_warped(num_images);    //��ʾӳ��任���ͼ��
	vector<Size> sizes(num_images);    //��ʾӳ��任���ͼ��ߴ�
	vector<UMat> masks(num_images);    //��ʾԴͼ������

	for (int i = 0; i < num_images; ++i)    //��ʼ��Դͼ������
	{
		masks[i].create(imgs[i].size(), CV_8U);    //����ߴ��С
		masks[i].setTo(Scalar::all(255));    //ȫ����ֵΪ255����ʾԴͼ����������ʹ��
	}

	Ptr<WarperCreator> warper_creator;    //����ͼ��ӳ��任������
	warper_creator = new cv::SphericalWarper();
	//warper_creator = makePtr<cv::PlaneWarper>();     //ƽ��ͶӰ
	//warper_creator = new cv::CylindricalWarper();    //����ͶӰ
	//warper_creator = new cv::SphericalWarper();    //����ͶӰ
	//warper_creator = new cv::FisheyeWarper();    //����ͶӰ
	//warper_creator = new cv::StereographicWarper();    //������ͶӰ

	 //����ͼ��ӳ��任��������ӳ��ĳ߶�Ϊ����Ľ��࣬��������Ľ��඼��ͬ
	vector<double> focals;
	for (size_t i = 0; i < cameras.size(); ++i)
	{
		cout << "��" << i << "����Ϊ" << cameras[i].focal << endl;
		focals.push_back(cameras[i].focal);
	}
	sort(focals.begin(), focals.end());
	float warped_image_scale;
	if (focals.size() % 2 == 1)
		warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
	else
		warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;
	Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale));
	for (int i = 0; i < num_images; ++i)
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
	cout << "ͼ��ӳ�����" << endl;
	//�����عⲹ������Ӧ�����油������
	Ptr<ExposureCompensator> compensator =
		ExposureCompensator::createDefault(ExposureCompensator::GAIN);
	compensator->feed(corners, images_warped, masks_warped);    //�õ��عⲹ����
	for (int i = 0; i < num_images; ++i)    //Ӧ���عⲹ��������ͼ������عⲹ��
	{
		compensator->apply(i, corners[i], images_warped[i], masks_warped[i]);
	}
	cout << "ͼ���ع����" << endl;

	//�ں��棬���ǻ���Ҫ�õ�ӳ��任ͼ������masks_warped���������Ϊ�ñ������һ������masks_seam
	vector<UMat> masks_seam(num_images);
	for (int i = 0; i < num_images; i++)
		masks_warped[i].copyTo(masks_seam[i]);

	Ptr<SeamFinder> seam_finder;    //����ӷ���Ѱ����
	//seam_finder = new NoSeamFinder();    //����Ѱ�ҽӷ���
	//seam_finder = new VoronoiSeamFinder();    //��㷨
	//seam_finder = new DpSeamFinder(DpSeamFinder::COLOR);    //��̬�淶��
	//seam_finder = new DpSeamFinder(DpSeamFinder::COLOR_GRAD);
	//ͼ�
	seam_finder = new GraphCutSeamFinder(GraphCutSeamFinder::COST_COLOR);
	//seam_finder = new GraphCutSeamFinder(GraphCutSeamFinder::COST_COLOR_GRAD);

	vector<UMat> images_warped_f(num_images);
	for (int i = 0; i < num_images; ++i)    //ͼ����������ת��
		images_warped[i].convertTo(images_warped_f[i], CV_32F);

	images_warped.clear();    //���ڴ�
	//�õ��ӷ��ߵ�����ͼ��masks_seam
	seam_finder->find(images_warped_f, corners, masks_seam);
	/*for (size_t i = 0; i < num_images; i++)
	{
		namedWindow("mask_cut", WINDOW_NORMAL);
		imshow("mask_cut", masks_seam[i]);
		waitKey(0);
	}*/


	cout << "ƴ���Ż����" << endl;

	vector<Mat> images_warped_s(num_images);
	Ptr<Blender> blender;    //����ͼ���ں���

	blender = Blender::createDefault(Blender::NO, false);    //���ںϷ���
	//���ںϷ���
//    blender = Blender::createDefault(Blender::FEATHER, false);
//    //dynamic_cast��̬ǿ������ת��ʱ��ʹ��
//    FeatherBlender* fb = dynamic_cast<FeatherBlender*>(static_cast<Blender*>(blender));
//    fb->setSharpness(0.005);    //���������

//    blender = Blender::createDefault(Blender::MULTI_BAND, false);    //��Ƶ���ں�
 //   MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(static_cast<Blender*>(blender));
 //   mb->setNumBands(8);   //����Ƶ������������������

	blender->prepare(corners, sizes);    //����ȫ��ͼ������
	cout << "����ȫ��ͼ������" << endl;
	vector<Mat> dilate_img(num_images);
	vector<Mat> masks_seam_new(num_images);
	Mat tem;
	Mat element = getStructuringElement(MORPH_RECT, Size(20, 20));    //����ṹԪ��
	for (int k = 0; k < num_images; k++)
	{
		images_warped_f[k].convertTo(images_warped_s[k], CV_16S);    //�ı���������
		dilate(masks_seam[k], masks_seam_new[k], element);    //��������
		//ӳ��任ͼ����������ͺ�������ࡰ�롱���Ӷ�ʹ��չ������������ڽӷ������࣬�����߽紦����Ӱ��
		masks_warped[k].copyTo(tem);
		masks_seam_new[k] = masks_seam_new[k] & tem;
		blender->feed(images_warped_s[k], masks_seam_new[k], corners[k]);    //��ʼ������
		cout << "�������" << k << "ͼƬ" << endl;
	}

	masks_seam.clear();    //���ڴ�
	images_warped_s.clear();
	masks_warped.clear();
	images_warped_f.clear();


	Mat result, result_mask;
	//����ںϲ������õ�ȫ��ͼ��result����������result_mask
	blender->blend(result, result_mask);
	imwrite("result.jpg", result);    //�洢ȫ��ͼ��


#pragma region SIFTGPU
	//GpuFeatureDetector fp;
	//InitStatus initState = fp.create();
	//if (initState != InitStatus::INIT_OK)
	//{
	//	exit(0);
	//}
	//Mat image1, image2, des1, des2;
	//vector<KeyPoint> kp1, kp2;
	//vector<DMatch> matches, matches_gms;
	//string imgPath1 = "E:\\Papertest2\\GMS_Line2\\image\\boat\\img1.png";
	//string imgPath2 = "E:\\Papertest2\\GMS_Line2\\image\\boat\\img2.png";
	//image1 = imread(imgPath1);
	//image2 = imread(imgPath2);
	//kp1.clear();
	//kp2.clear();
	//matches.clear();
	//fp.detectAndCompute(image1, des1, kp1);
	//fp.detectAndCompute(image2, des2, kp2);
	//fp.gpuMatch(des1, des2, matches);
	//gms_matcher gms(kp1, image1.size(), kp2, image2.size(), matches);
	//std::vector<bool>vbInliers;
	//int num_inliers = gms.GetInlierMask(vbInliers, false, false);
	//for (size_t i = 0; i < vbInliers.size(); i++)
	//{
	//	if (vbInliers[i] == true)
	//	{
	//		matches_gms.push_back(matches[i]);
	//	}
	//}
	//Mat matchImg;
	//drawMatches(image1, kp1, image2, kp2, matches_gms, matchImg);
	//waitKey(10);
#pragma endregion

	return 0;
}


