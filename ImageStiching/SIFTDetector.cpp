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
	if (img_1.empty()) cout << "��Ӱ��";
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
	cout << "ƥ��Ĺؼ������Ϊ��" << matchePoints.size() << endl;
	return size;
}

/*���ȸ��������(NN)�͵ڶ�����(SCN)�ľ���֮��(NN / SCN), ѡ����Щ�ɿ��Խϸߵ�ƥ���������㼸��Լ��ģ��*/
vector<DMatch> NNSCNCheck(vector<vector<DMatch> > Matches,float threshold) {
	vector<DMatch> GoodMatchePoints;
	for (int i = 0; i < Matches.size(); i++)
	{
		if (Matches[i][0].distance < threshold * Matches[i][1].distance)
		{
			GoodMatchePoints.push_back(Matches[i][0]);
		}
	}
	cout << "NN/SCN����ֵΪ��"<< threshold <<"	ɸѡ��ĵ���Ϊ��"<<GoodMatchePoints.size() << endl;
	return GoodMatchePoints;
	
}


int RANSACCheck( vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, Mat descriptors_1, Mat descriptors_2, vector<DMatch> Matches, string model, vector<KeyPoint> &RR_keypoint01, vector<KeyPoint> &RR_keypoint02, vector<DMatch> &RR_matches,double param1,double param2 ){
	if (!(model == string("Fundamental") || model == string("Homography")))
		CV_Error(CV_StsBadArg, "RANSACģ��ͼ��任�����������");
	// ����ռ�     
	int ptCount = (int)Matches.size();

	if (ptCount < 40)
	{
		CV_Error(CV_StsBadArg, "û���㹻�������㣡");
	}
	Mat p1(ptCount, 2, CV_32F);
	Mat p2(ptCount, 2, CV_32F);

	//ȡ��ƥ���������
	vector<KeyPoint> R_keypoint01, R_keypoint02;
	for (int i = 0; i < Matches.size(); i++)
	{
		R_keypoint01.push_back(keypoints_1[Matches[i].queryIdx]);
		R_keypoint02.push_back(keypoints_2[Matches[i].trainIdx]);
	}
	// ��Keypointת��ΪMat     
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
	// ��RANSAC��������
	vector<uchar> m_RANSACStatus;       // ����������ڴ洢RANSAC��ÿ�����״̬
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
	cout << "RANSACʹ�õľ���Ϊ��" << model << "	ɸѡ��ĵ���Ϊ��" << RR_matches.size() << endl;
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
	cout << "��" << distance <<"*" << distance <<"����ֻȡһ����" << "	ɸѡ��ĵ���Ϊ��" << RR_matches.size() << endl;
	return RR_matches.size();
}

Mat WarpImage(Mat img_1, Mat img_2, vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, vector<DMatch> Matches)
{
	int ptCount = (int)Matches.size();
	Mat p1(ptCount, 2, CV_32F);
	Mat p2(ptCount, 2, CV_32F);
	//ȡ��ƥ���������
	vector<KeyPoint> R_keypoint01, R_keypoint02;
	for (int i = 0; i < Matches.size(); i++)
	{
		R_keypoint01.push_back(keypoints_1[Matches[i].queryIdx]);
		R_keypoint02.push_back(keypoints_2[Matches[i].trainIdx]);
	}
	// ��Keypointת��ΪMat     
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
	//ͼ����׼
	CalcCorners(Homography, img_1);
	Mat imageTransform;
	warpPerspective(img_1, imageTransform, Homography, Size(img_1.cols, img_1.rows));
	return imageTransform;


//	Mat Homography = findHomography(p1, p2, CV_RANSAC);
////ͼ����׼
//	CalcCorners(Homography, img_1);
//	Mat imageTransform;
//	warpPerspective(img_1, imageTransform, Homography, Size(MAX(corners.right_top.x, corners.right_bottom.x), img_2.rows));
//	int dst_width = imageTransform.cols;  //ȡ���ҵ�ĳ���Ϊƴ��ͼ�ĳ���
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
	int start = MIN(corners.left_top.x, corners.left_bottom.x);//��ʼλ�ã����ص��������߽�  

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
void CalcCorners(const Mat& H, const Mat& src)
{
	double v2[] = { 0, 0, 1 };//���Ͻ�
	double v1[3];//�任�������ֵ
	Mat V2 = Mat(3, 1, CV_64FC1, v2);  //������
	Mat V1 = Mat(3, 1, CV_64FC1, v1);  //������

	V1 = H * V2;
	//���Ͻ�(0,0,1)
	cout << "V2: " << V2 << endl;
	cout << "V1: " << V1 << endl;
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

/***
* ң��Ӱ���ز���
* @param pszSrcFile        �����ļ���·��
* @param pszOutFile        д��Ľ��ͼ���·��
* @param eResample         ����ģʽ�������֣�����μ�GDALResampleAlg���壬Ĭ��Ϊ˫�����ڲ�
							GRA_NearestNeighbour=0      ����ڷ����㷨�򵥲��ܱ���ԭ������Ϣ���䣻ȱ���Ǽ��ξ��Ȳ�ҶȲ���������Ե����־��״
							GRA_Bilinear=1              ˫���Է�������򵥣�ͼ��ҶȾ����������Ҳ������ȱȽϾ�ȷ��ȱ���ǻ�ɥʧϸ�ڣ�
							GRA_Cubic=2                 ���ξ��������������ͼ��ҶȾ����������Ҳ������ȸߣ�
							GRA_CubicSpline=3           �������������Ҷ������ԺͲ���������ѣ�
							GRA_Lanczos=4               �ֿ�����˹��������������ѧ�ҡ�����ѧ������˹��������ʵ�鷢��Ч����˫���Խӽ���
* @param fResX             Xת�������ȣ�Ĭ�ϴ�СΪ1.0������1ͼ����С��1��ʾͼ����С����ֵ�ϵ��ڲ�����ͼ��Ŀ�ȺͲ���ǰͼ���ȵı�
* @param fResY             Yת�������ȣ�Ĭ�ϴ�СΪ1.0������1ͼ����С��1��ʾͼ����С����ֵ�ϵ��ڲ�����ͼ��ĸ߶ȺͲ���ǰͼ��߶ȵı�
* @retrieve     0   �ɹ�
* @retrieve     -1  ��Դ�ļ�ʧ��
* @retrieve     -2  �������ļ�ʧ��
* @retrieve     -3  ��������г���
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
	//���û��ͶӰ����Ϊ����һ��    
	if (strlen(pszSrcWKT) <= 0)
	{
		//OGRSpatialReference oSRS;  
		//oSRS.SetUTM(50,true); //������  ����120��  
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


	//����������ݼ�  
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







