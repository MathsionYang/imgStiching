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

	int num_images = imgs.size();    //图像数量
	cout << "图像数量为:" << num_images << endl;
	//表示图像特征
	vector<ImageFeatures> features(num_images);

	Ptr<FeaturesFinder> finder;    //定义特征寻找器
	finder = new SurfFeaturesFinder();    //应用SURF方法寻找特征
	for (int i = 0; i < num_images; i++)
		(*finder)(imgs[i], features[i]);    //特征检测
	cout << "特征提取完毕" << endl;

	vector<MatchesInfo> pairwise_matches;    //表示特征匹配信息变量
	BestOf2NearestMatcher matcher(false, 0.35f, 6, 6);    //定义特征匹配器，2NN方法
	matcher(features, pairwise_matches);    //进行特征匹配


	//GMS筛选
	//gms_matcher gms(kp1, image1.size(), kp2, image2.size(), matches);
	cout << "特征匹配完毕" << endl;

	HomographyBasedEstimator estimator;    //定义参数评估器
	vector<CameraParams> cameras;    //表示相机参数，内参加外参
	estimator(features, pairwise_matches, cameras);    //进行相机参数评
	for (size_t i = 0; i < cameras.size(); ++i)    //转换相机旋转参数的数据类型
	{
		Mat R;
		cameras[i].R.convertTo(R, CV_32F);
		cameras[i].R = R;
	}
	cout << "相机参数预测完毕" << endl;

	for (size_t i = 0; i < cameras.size(); ++i)
	{
		cout << "第" << i << "焦距为" << cameras[i].focal << endl;
	}
	// 在一部可以计算重映射误差，想办法让他可以输出出来
	Ptr<detail::BundleAdjusterBase> adjuster;    //光束平差法，精确相机参数
	//adjuster->setRefinementMask();
	adjuster = new detail::BundleAdjusterReproj();    //重映射误差方法
	//adjuster = new detail::BundleAdjusterRay();    //射线发散误差方法

	adjuster->setConfThresh(1.0f);    //设置匹配置信度，该值设为1
	(*adjuster)(features, pairwise_matches, cameras);    //精确评估相机参数

	vector<Mat> rmats;
	for (size_t i = 0; i < cameras.size(); ++i)    //复制相机的旋转参数
		rmats.push_back(cameras[i].R.clone());
	waveCorrect(rmats, WAVE_CORRECT_HORIZ);    //进行波形校正
	for (size_t i = 0; i < cameras.size(); ++i)    //相机参数赋值
		cameras[i].R = rmats[i];
	rmats.clear();    //清变量

	cout << "利用光束平差法进行相机矩阵更新" << endl;

	vector<Point> corners(num_images);    //表示映射变换后图像的左上角坐标
	vector<UMat> masks_warped(num_images);    //表示映射变换后的图像掩码
	vector<UMat> images_warped(num_images);    //表示映射变换后的图像
	vector<Size> sizes(num_images);    //表示映射变换后的图像尺寸
	vector<UMat> masks(num_images);    //表示源图的掩码

	for (int i = 0; i < num_images; ++i)    //初始化源图的掩码
	{
		masks[i].create(imgs[i].size(), CV_8U);    //定义尺寸大小
		masks[i].setTo(Scalar::all(255));    //全部赋值为255，表示源图的所有区域都使用
	}

	Ptr<WarperCreator> warper_creator;    //定义图像映射变换创造器
	warper_creator = new cv::SphericalWarper();
	//warper_creator = makePtr<cv::PlaneWarper>();     //平面投影
	//warper_creator = new cv::CylindricalWarper();    //柱面投影
	//warper_creator = new cv::SphericalWarper();    //球面投影
	//warper_creator = new cv::FisheyeWarper();    //鱼眼投影
	//warper_creator = new cv::StereographicWarper();    //立方体投影

	 //定义图像映射变换器，设置映射的尺度为相机的焦距，所有相机的焦距都相同
	vector<double> focals;
	for (size_t i = 0; i < cameras.size(); ++i)
	{
		cout << "第" << i << "焦距为" << cameras[i].focal << endl;
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
		cameras[i].K().convertTo(K, CV_32F);    //转换相机内参数的数据类型
		//对当前图像镜像投影变换，得到变换后的图像以及该图像的左上角坐标
		corners[i] = warper->warp(imgs[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
		sizes[i] = images_warped[i].size();    //得到尺寸
		//得到变换后的图像掩码
		warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
	}
	imgs.clear();    //清变量
	masks.clear();
	cout << "图像映射完毕" << endl;
	//创建曝光补偿器，应用增益补偿方法
	Ptr<ExposureCompensator> compensator =
		ExposureCompensator::createDefault(ExposureCompensator::GAIN);
	compensator->feed(corners, images_warped, masks_warped);    //得到曝光补偿器
	for (int i = 0; i < num_images; ++i)    //应用曝光补偿器，对图像进行曝光补偿
	{
		compensator->apply(i, corners[i], images_warped[i], masks_warped[i]);
	}
	cout << "图像曝光完毕" << endl;

	//在后面，我们还需要用到映射变换图的掩码masks_warped，因此这里为该变量添加一个副本masks_seam
	vector<UMat> masks_seam(num_images);
	for (int i = 0; i < num_images; i++)
		masks_warped[i].copyTo(masks_seam[i]);

	Ptr<SeamFinder> seam_finder;    //定义接缝线寻找器
	//seam_finder = new NoSeamFinder();    //无需寻找接缝线
	//seam_finder = new VoronoiSeamFinder();    //逐点法
	//seam_finder = new DpSeamFinder(DpSeamFinder::COLOR);    //动态规范法
	//seam_finder = new DpSeamFinder(DpSeamFinder::COLOR_GRAD);
	//图割法
	seam_finder = new GraphCutSeamFinder(GraphCutSeamFinder::COST_COLOR);
	//seam_finder = new GraphCutSeamFinder(GraphCutSeamFinder::COST_COLOR_GRAD);

	vector<UMat> images_warped_f(num_images);
	for (int i = 0; i < num_images; ++i)    //图像数据类型转换
		images_warped[i].convertTo(images_warped_f[i], CV_32F);

	images_warped.clear();    //清内存
	//得到接缝线的掩码图像masks_seam
	seam_finder->find(images_warped_f, corners, masks_seam);
	/*for (size_t i = 0; i < num_images; i++)
	{
		namedWindow("mask_cut", WINDOW_NORMAL);
		imshow("mask_cut", masks_seam[i]);
		waitKey(0);
	}*/


	cout << "拼缝优化完毕" << endl;

	vector<Mat> images_warped_s(num_images);
	Ptr<Blender> blender;    //定义图像融合器

	blender = Blender::createDefault(Blender::NO, false);    //简单融合方法
	//羽化融合方法
//    blender = Blender::createDefault(Blender::FEATHER, false);
//    //dynamic_cast多态强制类型转换时候使用
//    FeatherBlender* fb = dynamic_cast<FeatherBlender*>(static_cast<Blender*>(blender));
//    fb->setSharpness(0.005);    //设置羽化锐度

//    blender = Blender::createDefault(Blender::MULTI_BAND, false);    //多频段融合
 //   MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(static_cast<Blender*>(blender));
 //   mb->setNumBands(8);   //设置频段数，即金字塔层数

	blender->prepare(corners, sizes);    //生成全景图像区域
	cout << "生成全景图像区域" << endl;
	vector<Mat> dilate_img(num_images);
	vector<Mat> masks_seam_new(num_images);
	Mat tem;
	Mat element = getStructuringElement(MORPH_RECT, Size(20, 20));    //定义结构元素
	for (int k = 0; k < num_images; k++)
	{
		images_warped_f[k].convertTo(images_warped_s[k], CV_16S);    //改变数据类型
		dilate(masks_seam[k], masks_seam_new[k], element);    //膨胀运算
		//映射变换图的掩码和膨胀后的掩码相“与”，从而使扩展的区域仅仅限于接缝线两侧，其他边界处不受影响
		masks_warped[k].copyTo(tem);
		masks_seam_new[k] = masks_seam_new[k] & tem;
		blender->feed(images_warped_s[k], masks_seam_new[k], corners[k]);    //初始化数据
		cout << "处理完成" << k << "图片" << endl;
	}

	masks_seam.clear();    //清内存
	images_warped_s.clear();
	masks_warped.clear();
	images_warped_f.clear();


	Mat result, result_mask;
	//完成融合操作，得到全景图像result和它的掩码result_mask
	blender->blend(result, result_mask);
	imwrite("result.jpg", result);    //存储全景图像


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


