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
			resize(img, res, Size(img.cols, img.rows), 0, 0, INTER_LINEAR);// X Y各缩小一半
			imgs.push_back(res);
		}
		return imgs;
	}
}
void ImgProcess::featureMatch(vector<ImageFeatures> &features, vector<MatchesInfo> &pairwise_matches)
{

	Ptr<OrbFeaturesFinder> finder;    //定义特征寻找器
	finder = new  OrbFeaturesFinder(Size(3,1),600);    //应用ORB方法寻找特征

	for (int i = 0; i < imgs.size(); i++)
		(*finder)(imgs[i], features[i]);    //特征检测

	BestOf2NearestMatcher1 matcher(false, 0.3f, 6, 6);    //定义特征匹配器，2NN方法
	matcher(features, pairwise_matches);    //进行特征匹配
}
void ImgProcess::GetCameraParmas(vector<ImageFeatures> features, vector<MatchesInfo> pairwise_matches, vector<CameraParams> &cameras)
{
	//定义参数评估器
	HomographyBasedEstimator estimator;    
	//表示相机参数  进行相机参数评估
	estimator(features, pairwise_matches, cameras);  
	//转换相机旋转参数的数据类型
	for (size_t i = 0; i < cameras.size(); ++i)    
	{
		Mat R;
		cameras[i].R.convertTo(R, CV_32F);
		cameras[i].R = R;
	}
	//光束平差法，精确相机参数
	Ptr<detail::BundleAdjusterBase> adjuster;
	//射线发散误差方法
	adjuster = new detail::BundleAdjusterRay();    
	//设置匹配置信度，该值设为1
	adjuster->setConfThresh(1);  
	//精确评估相机参数
	(*adjuster)(features, pairwise_matches, cameras);   
}
void ImgProcess::GetStichingResult(vector<CameraParams> cameras,string savepath)
{
	int NUM_IMAGES = imgs.size();
	vector<Point> corners(NUM_IMAGES);    //表示映射变换后图像的左上角坐标
	vector<UMat> masks_warped(NUM_IMAGES);    //表示映射变换后的图像掩码
	vector<UMat> images_warped(NUM_IMAGES);    //表示映射变换后的图像
	vector<Size> sizes(NUM_IMAGES);    //表示映射变换后的图像尺寸
	vector<Mat> masks(NUM_IMAGES);    //表示源图的掩码

	for (int i = 0; i < NUM_IMAGES; ++i)    //初始化源图的掩码
	{
		masks[i].create(imgs[i].size(), CV_8U);    //定义尺寸大小
		masks[i].setTo(Scalar::all(255));    //全部赋值为255，表示源图的所有区域都使用
	}

	Ptr<WarperCreator> warper_creator;    //定义图像映射变换创造器
	warper_creator = new cv::CylindricalWarper();    //柱面投影
	//定义图像映射变换器，设置映射的尺度为相机的焦距，所有相机的焦距都相同
	Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(cameras[0].focal));
	for (int i = 0; i < NUM_IMAGES; ++i)
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

	//创建曝光补偿器，应用增益补偿方法
	Ptr<ExposureCompensator> compensator =ExposureCompensator::createDefault(ExposureCompensator::GAIN);
	compensator->feed(corners, images_warped, masks_warped);    //得到曝光补偿器
	for (int i = 0; i < NUM_IMAGES; ++i)    //应用曝光补偿器，对图像进行曝光补偿
	{
		compensator->apply(i, corners[i], images_warped[i], masks_warped[i]);
	}

	//在后面，我们还需要用到映射变换图的掩码masks_warped，因此这里为该变量添加一个副本masks_seam
	vector<UMat> masks_seam(NUM_IMAGES);
	for (int i = 0; i < NUM_IMAGES; i++)
		masks_warped[i].copyTo(masks_seam[i]);
	Ptr<SeamFinder> seam_finder;    //定义接缝线寻找器
	//图割法
	seam_finder = new GraphCutSeamFinder(GraphCutSeamFinder::COST_COLOR);
	//seam_finder = new GraphCutSeamFinder(GraphCutSeamFinder::COST_COLOR_GRAD);
	vector<UMat> images_warped_f(NUM_IMAGES);
	for (int i = 0; i < NUM_IMAGES; ++i)    //图像数据类型转换
		images_warped[i].convertTo(images_warped_f[i], CV_32F);
	//得到接缝线的掩码图像masks_warped
	seam_finder->find(images_warped_f, corners, masks_warped);
	vector<Mat> images_warped_s(NUM_IMAGES);
	Ptr<Blender> blender;    //定义图像融合器
	//blender = Blender::createDefault(Blender::MULTI_BAND, false);    //多频段融合
	//MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(static_cast<Blender*>(blender));

	blender = Blender::createDefault(Blender::NO, false);    //简单融合方法
	//羽化融合方法
	blender = Blender::createDefault(Blender::FEATHER, false);
	FeatherBlender* fb = dynamic_cast<FeatherBlender*>(static_cast<Blender*>(blender));
	fb->setSharpness(5);    //设置羽化锐度
	//cout << sizes[0] << endl;
	//cout << sizes[1] << endl;
	blender->prepare(corners, sizes);    //生成全景图像区域

	//在融合的时候，最重要的是在接缝线两侧进行处理，而上一步在寻找接缝线后得到的掩码的边界就是接缝线处，因此我们还需要在接缝线两侧开辟一块区域用于融合处理，这一处理过程对羽化方法尤为关键
	//应用膨胀算法缩小掩码面积
	vector<Mat> dilate_img(NUM_IMAGES);
	Mat element = getStructuringElement(MORPH_RECT, Size(20, 20));    //定义结构元素
	vector<Mat> a(NUM_IMAGES);
	vector<Mat> b(NUM_IMAGES);
	vector<Mat> c(NUM_IMAGES);
	for (int k = 0; k < NUM_IMAGES; k++)
	{
		images_warped_f[k].convertTo(images_warped_s[k], CV_16S);    //改变数据类型
		dilate(masks_seam[k], masks_seam[k], element);    //膨胀运算
		masks_seam[k].copyTo(a[k]);
		masks_warped[k].copyTo(b[k]);
		c[k] = a[k] & b[k];
		c[k].copyTo(masks_seam[k]);
		blender->feed(images_warped_s[k], masks_seam[k], corners[k]);    //初始化数据
	}

	//清内存
	masks_seam.clear();
	images_warped_s.clear();
	masks_warped.clear();
	images_warped_f.clear();

	Mat result, result_mask;
	//完成融合操作，得到全景图像result和它的掩码result_mask
	blender->blend(result, result_mask);

	imwrite(savepath, result);    //存储全景图像
}

#endif