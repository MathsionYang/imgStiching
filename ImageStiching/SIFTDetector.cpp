#include "SIFTDetector.h"
#include "gdal_priv.h"  
#include "ogrsf_frmts.h"  
#include "gdalwarper.h"  


Ptr<Feature2D> detector = xfeatures2d::SURF::create();

typedef struct
{
	Point2f left_top;
	Point2f left_bottom;
	Point2f right_top;
	Point2f right_bottom;
}four_corners_t;
four_corners_t corners;
bool DetectorKeyPoint(Mat img_1, Mat img_2,vector<KeyPoint> &keypoints_1, vector<KeyPoint>&  keypoints_2)
{
	if (img_1.empty()) cout << "空影像";
	detector->detect(img_1, keypoints_1);
	detector->detect(img_2, keypoints_2);
	if (!keypoints_1.empty() && !keypoints_2.empty()) return true;
	else return false;
}

bool ComputeDescriptor(Mat img_1, Mat img_2, vector<KeyPoint>& keypoints_1, vector<KeyPoint>& keypoints_2,Mat& descriptors_1, Mat& descriptors_2) {
	detector->compute(img_1, keypoints_1, descriptors_1);
	detector->compute(img_2, keypoints_2, descriptors_2);
	if (!keypoints_1.empty() && !keypoints_2.empty()) return true;
	else return false;
}

int FeatureMatch(vector<vector<DMatch> > &matchePoints, Mat& descriptors_1, Mat& descriptors_2)
{
	FlannBasedMatcher matcher;
	vector<Mat> train_desc(1, descriptors_2);
	matcher.add(train_desc);
	matcher.train();
	matcher.knnMatch(descriptors_1, matchePoints, 2);
	int size = matchePoints.size();
	cout << "匹配的关键点个数为：" << matchePoints.size() << endl;
	return size;
}

/*首先根据最近邻(NN)和第二近邻(SCN)的距离之比(NN / SCN), 选择那些可靠性较高的匹配点对来计算几何约束模型*/
vector<DMatch> NNSCNCheck(vector<vector<DMatch> > Matches,float threshold) {
	vector<DMatch> GoodMatchePoints;
	for (int i = 0; i < Matches.size(); i++)
	{
		if (Matches[i][0].distance < threshold * Matches[i][1].distance)
		{
			GoodMatchePoints.push_back(Matches[i][0]);
		}
	}
	cout << "NN/SCN的阈值为："<< threshold <<"	筛选后的点数为："<<GoodMatchePoints.size() << endl;
	return GoodMatchePoints;
	
}


int RANSACCheck( vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, Mat descriptors_1, Mat descriptors_2, vector<DMatch> Matches, string model, vector<KeyPoint> &RR_keypoint01, vector<KeyPoint> &RR_keypoint02, vector<DMatch> &RR_matches,double param1,double param2 ){
	if (!(model == string("Fundamental") || model == string("Homography")))
		CV_Error(CV_StsBadArg, "RANSAC模块图像变换类型输入错误！");
	// 分配空间     
	int ptCount = (int)Matches.size();

	if (ptCount < 40)
	{
		CV_Error(CV_StsBadArg, "没有足够的特征点！");
	}
	Mat p1(ptCount, 2, CV_32F);
	Mat p2(ptCount, 2, CV_32F);

	//取出匹配的特征点
	vector<KeyPoint> R_keypoint01, R_keypoint02;
	for (int i = 0; i < Matches.size(); i++)
	{
		R_keypoint01.push_back(keypoints_1[Matches[i].queryIdx]);
		R_keypoint02.push_back(keypoints_2[Matches[i].trainIdx]);
	}
	// 把Keypoint转换为Mat     
	Point2f pt;
	for (int i = 0; i < ptCount; i++)
	{
		pt = keypoints_1[Matches[i].queryIdx].pt;
		p1.at<float>(i, 0) = pt.x;
		p1.at<float>(i, 1) = pt.y;

		pt = keypoints_2[Matches[i].trainIdx].pt;
		p2.at<float>(i, 0) = pt.x;
		p2.at<float>(i, 1) = pt.y;
	}
	// 用RANSAC方法计算
	vector<uchar> m_RANSACStatus;       // 这个变量用于存储RANSAC后每个点的状态
	if (model == string("Fundamental")) {
		Mat m_Fundamental = findFundamentalMat(p1, p2, m_RANSACStatus, FM_RANSAC,param1, param2);
	}
	if (model == string("Homography")) {
		Mat m_Homography = findHomography(p1, p2, m_RANSACStatus, RANSAC, param1);
	}
	int index = 0;
	for (size_t i = 0; i < Matches.size(); i++)
	{
		if (m_RANSACStatus[i] != 0)
		{
			RR_keypoint01.push_back(R_keypoint01[i]);
			RR_keypoint02.push_back(R_keypoint02[i]);
			Matches[i].queryIdx = index;
			Matches[i].trainIdx = index;
			RR_matches.push_back(Matches[i]);
			index++;
		}
		//cout << m_RANSACStatus[i];
	}
	cout << "RANSAC使用的矩阵为：" << model << "	筛选后的点数为：" << RR_matches.size() << endl;
	return RR_matches.size();
}

int DistributedCheck(Mat img_2, vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, vector<DMatch> Matches,vector<KeyPoint>& RR_keypoint01, vector<KeyPoint>& RR_keypoint02, vector<DMatch>& RR_matches,int distance)
{
	int Y = img_2.rows;
	int X = img_2.cols;
	int index = 0;
	for (int i = 0; i < Y+ distance; i += distance)
	{
		for (int j = 0; j < X+ distance; j += distance)
		{

			for (int k =0; k < Matches.size(); k++)
			{
				//cout << keypoints_2[Matches[k].trainIdx].pt.x << endl;
				if (keypoints_2[Matches[k].trainIdx].pt.x >= j && keypoints_2[Matches[k].trainIdx].pt.x < j + distance && 
					keypoints_2[Matches[k].trainIdx].pt.y >= i && keypoints_2[Matches[k].trainIdx].pt.y < i + distance) 
				{
					RR_keypoint01.push_back(keypoints_1[k]);
					RR_keypoint02.push_back(keypoints_2[k]);
					Matches[k].queryIdx = index;
					Matches[k].trainIdx = index;
					RR_matches.push_back(Matches[k]);
					index++;
					break;
				}
			}
		}
	}
	cout << "在" << distance <<"*" << distance <<"的中只取一个点" << "	筛选后的点数为：" << RR_matches.size() << endl;
	return RR_matches.size();
}

Mat WarpImage(Mat img_1, Mat img_2, vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, vector<DMatch> Matches)
{
	int ptCount = (int)Matches.size();
	Mat p1(ptCount, 2, CV_32F);
	Mat p2(ptCount, 2, CV_32F);
	//取出匹配的特征点
	vector<KeyPoint> R_keypoint01, R_keypoint02;
	for (int i = 0; i < Matches.size(); i++)
	{
		R_keypoint01.push_back(keypoints_1[Matches[i].queryIdx]);
		R_keypoint02.push_back(keypoints_2[Matches[i].trainIdx]);
	}
	// 把Keypoint转换为Mat     
	Point2f pt;
	for (int i = 0; i < ptCount; i++)
	{
		pt = keypoints_1[Matches[i].queryIdx].pt;
		p1.at<float>(i, 0) = pt.x;
		p1.at<float>(i, 1) = pt.y;

		pt = keypoints_2[Matches[i].trainIdx].pt;
		p2.at<float>(i, 0) = pt.x;
		p2.at<float>(i, 1) = pt.y;
	}
	Mat Homography = findHomography(p1, p2, CV_RANSAC);
	//图像配准
	CalcCorners(Homography, img_1);
	Mat imageTransform;
	warpPerspective(img_1, imageTransform, Homography, Size(img_1.cols, img_1.rows));
	return imageTransform;


//	Mat Homography = findHomography(p1, p2, CV_RANSAC);
////图像配准
//	CalcCorners(Homography, img_1);
//	Mat imageTransform;
//	warpPerspective(img_1, imageTransform, Homography, Size(MAX(corners.right_top.x, corners.right_bottom.x), img_2.rows));
//	int dst_width = imageTransform.cols;  //取最右点的长度为拼接图的长度
//	int dst_height = img_2.rows;
//	Mat dst(dst_height, dst_width, CV_8UC3);
//	dst.setTo(0);
//
//	imageTransform.copyTo(dst(Rect(0, 0, imageTransform.cols, imageTransform.rows)));
//	img_2.copyTo(dst(Rect(0, 0, img_2.cols, img_2.rows)));
//	OptimizeSeam(img_2, imageTransform, dst);
//	return dst;
}
void OptimizeSeam(Mat& img1, Mat& trans, Mat& dst)
{
	int start = MIN(corners.left_top.x, corners.left_bottom.x);//开始位置，即重叠区域的左边界  

	double processWidth = img1.cols - start;//重叠区域的宽度  
	int rows = dst.rows;
	int cols = img1.cols; //注意，是列数*通道数
	double alpha = 1;//img1中像素的权重  
	for (int i = 0; i < rows; i++)
	{
		uchar* p = img1.ptr<uchar>(i);  //获取第i行的首地址
		uchar* t = trans.ptr<uchar>(i);
		uchar* d = dst.ptr<uchar>(i);
		for (int j = start; j < cols; j++)
		{
			//如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
			if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
			{
				alpha = 1;
			}
			else
			{
				//img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好  
				alpha = (processWidth - (j - start)) / processWidth;
			}

			d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
			d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
			d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);

		}
	}

}
void CalcCorners(const Mat& H, const Mat& src)
{
	double v2[] = { 0, 0, 1 };//左上角
	double v1[3];//变换后的坐标值
	Mat V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	Mat V1 = Mat(3, 1, CV_64FC1, v1);  //列向量

	V1 = H * V2;
	//左上角(0,0,1)
	cout << "V2: " << V2 << endl;
	cout << "V1: " << V1 << endl;
	corners.left_top.x = v1[0] / v1[2];
	corners.left_top.y = v1[1] / v1[2];

	//左下角(0,src.rows,1)
	v2[0] = 0;
	v2[1] = src.rows;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
	V1 = H * V2;
	corners.left_bottom.x = v1[0] / v1[2];
	corners.left_bottom.y = v1[1] / v1[2];

	//右上角(src.cols,0,1)
	v2[0] = src.cols;
	v2[1] = 0;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
	V1 = H * V2;
	corners.right_top.x = v1[0] / v1[2];
	corners.right_top.y = v1[1] / v1[2];

	//右下角(src.cols,src.rows,1)
	v2[0] = src.cols;
	v2[1] = src.rows;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
	V1 = H * V2;
	corners.right_bottom.x = v1[0] / v1[2];
	corners.right_bottom.y = v1[1] / v1[2];

}

/***
* 遥感影像重采样
* @param pszSrcFile        输入文件的路径
* @param pszOutFile        写入的结果图像的路径
* @param eResample         采样模式，有五种，具体参见GDALResampleAlg定义，默认为双线性内插
							GRA_NearestNeighbour=0      最近邻法，算法简单并能保持原光谱信息不变；缺点是几何精度差，灰度不连续，边缘会出现锯齿状
							GRA_Bilinear=1              双线性法，计算简单，图像灰度具有连续性且采样精度比较精确；缺点是会丧失细节；
							GRA_Cubic=2                 三次卷积法，计算量大，图像灰度具有连续性且采样精度高；
							GRA_CubicSpline=3           三次样条法，灰度连续性和采样精度最佳；
							GRA_Lanczos=4               分块兰索斯法，由匈牙利数学家、物理学家兰索斯法创立，实验发现效果和双线性接近；
* @param fResX             X转换采样比，默认大小为1.0，大于1图像变大，小于1表示图像缩小。数值上等于采样后图像的宽度和采样前图像宽度的比
* @param fResY             Y转换采样比，默认大小为1.0，大于1图像变大，小于1表示图像缩小。数值上等于采样后图像的高度和采样前图像高度的比
* @retrieve     0   成功
* @retrieve     -1  打开源文件失败
* @retrieve     -2  创建新文件失败
* @retrieve     -3  处理过程中出错
*/
int ResampleGDAL(const char* pszSrcFile, const char* pszOutFile, float fResX = 1.0, float fResY = 1.0, GDALResampleAlg eResample = GRA_Bilinear)
{
	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
	GDALDataset *pDSrc = (GDALDataset *)GDALOpen(pszSrcFile, GA_ReadOnly);
	if (pDSrc == NULL)
	{
		return -1;
	}


	GDALDriver *pDriver = GetGDALDriverManager()->GetDriverByName("GTiff");
	if (pDriver == NULL)
	{
		GDALClose((GDALDatasetH)pDSrc);
		return -2;
	}
	int width = pDSrc->GetRasterXSize();
	int height = pDSrc->GetRasterYSize();
	int nBandCount = pDSrc->GetRasterCount();
	GDALDataType dataType = pDSrc->GetRasterBand(1)->GetRasterDataType();


	char *pszSrcWKT = NULL;
	pszSrcWKT = const_cast<char *>(pDSrc->GetProjectionRef());


	double dGeoTrans[6] = { 0 };
	int nNewWidth = width, nNewHeight = height;
	pDSrc->GetGeoTransform(dGeoTrans);


	bool bNoGeoRef = false;
	double dOldGeoTrans0 = dGeoTrans[0];
	//如果没有投影，人为设置一个    
	if (strlen(pszSrcWKT) <= 0)
	{
		//OGRSpatialReference oSRS;  
		//oSRS.SetUTM(50,true); //北半球  东经120度  
		//oSRS.SetWellKnownGeogCS("WGS84");  
		//oSRS.exportToWkt(&pszSrcWKT);  
		//pDSrc->SetProjection(pszSrcWKT);  


		//  
		dGeoTrans[0] = 1.0;
		pDSrc->SetGeoTransform(dGeoTrans);
		//  


		bNoGeoRef = true;
	}


	//adfGeoTransform[0] /* top left x */  
	//adfGeoTransform[1] /* w-e pixel resolution */  
	//adfGeoTransform[2] /* rotation, 0 if image is "north up" */  
	//adfGeoTransform[3] /* top left y */  
	//adfGeoTransform[4] /* rotation, 0 if image is "north up" */  
	//adfGeoTransform[5] /* n-s pixel resolution */  


	dGeoTrans[1] = dGeoTrans[1] / fResX;
	dGeoTrans[5] = dGeoTrans[5] / fResY;
	nNewWidth = static_cast<int>(nNewWidth*fResX + 0.5);
	nNewHeight = static_cast<int>(nNewHeight*fResY + 0.5);


	//创建结果数据集  
	GDALDataset *pDDst = pDriver->Create(pszOutFile, nNewWidth, nNewHeight, nBandCount, dataType, NULL);
	if (pDDst == NULL)
	{
		GDALClose((GDALDatasetH)pDSrc);
		return -2;
	}

	pDDst->SetProjection(pszSrcWKT);
	pDDst->SetGeoTransform(dGeoTrans);


	void *hTransformArg = NULL;
	hTransformArg = GDALCreateGenImgProjTransformer2((GDALDatasetH)pDSrc, (GDALDatasetH)pDDst, NULL); //GDALCreateGenImgProjTransformer((GDALDatasetH) pDSrc,pszSrcWKT,(GDALDatasetH) pDDst,pszSrcWKT,FALSE,0.0,1);  


	if (hTransformArg == NULL)
	{
		GDALClose((GDALDatasetH)pDSrc);
		GDALClose((GDALDatasetH)pDDst);
		return -3;
	}

	GDALWarpOptions *psWo = GDALCreateWarpOptions();


	psWo->papszWarpOptions = CSLDuplicate(NULL);
	psWo->eWorkingDataType = dataType;
	psWo->eResampleAlg = eResample;


	psWo->hSrcDS = (GDALDatasetH)pDSrc;
	psWo->hDstDS = (GDALDatasetH)pDDst;


	psWo->pfnTransformer = GDALGenImgProjTransform;
	psWo->pTransformerArg = hTransformArg;


	psWo->nBandCount = nBandCount;
	psWo->panSrcBands = (int *)CPLMalloc(nBandCount * sizeof(int));
	psWo->panDstBands = (int *)CPLMalloc(nBandCount * sizeof(int));
	for (int i = 0; i < nBandCount; i++)
	{
		psWo->panSrcBands[i] = i + 1;
		psWo->panDstBands[i] = i + 1;
	}


	GDALWarpOperation oWo;
	if (oWo.Initialize(psWo) != CE_None)
	{
		GDALClose((GDALDatasetH)pDSrc);
		GDALClose((GDALDatasetH)pDDst);
		return -3;
	}

	oWo.ChunkAndWarpImage(0, 0, nNewWidth, nNewHeight);


	GDALDestroyGenImgProjTransformer(hTransformArg);
	GDALDestroyWarpOptions(psWo);
	if (bNoGeoRef)
	{
		dGeoTrans[0] = dOldGeoTrans0;
		pDDst->SetGeoTransform(dGeoTrans);
		//pDDst->SetProjection("");  
	}
	GDALFlushCache(pDDst);
	GDALClose((GDALDatasetH)pDSrc);
	GDALClose((GDALDatasetH)pDDst);
	return 0;
}







