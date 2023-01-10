#include "FileFormatChange.h"
//GDAL读取影像信息
cv::Mat FileFormatChange::GDALGetFileFromImg(const string  src_path, MyStruct &St)
{
	GDALAllRegister();
	GDALDataset *poDataset = (GDALDataset *)GDALOpen(src_path.c_str(), GA_ReadOnly);
	St.Xsize = poDataset->GetRasterXSize();
	St.Ysize = poDataset->GetRasterYSize();
	St.nbands = poDataset->GetRasterCount();
	poDataset->GetGeoTransform(St.tmpadfGeoTransform);
	St.proj = poDataset->GetProjectionRef();
	St.iDataType = poDataset->GetRasterBand(1)->GetRasterDataType();
	cv::Mat img = GDAL2Mat(poDataset, St);
	GDALClose((GDALDatasetH)poDataset);
	return img;
}
//GDAL数据转opencv的Mat格式
cv::Mat FileFormatChange::GDAL2Mat(GDALDataset *poDataset, MyStruct &St)  //const QString fileName
{
	//将GDAL的数据类型与Mat的数据类型对应起来
	auto MdataType = NULL;
	auto MdataTypes = NULL;
	if (St.iDataType == GDT_Byte)
	{
		MdataType = CV_MAKETYPE(CV_8U, 1);
		MdataTypes = CV_MAKETYPE(CV_8U, St.nbands);
	}
	if (St.iDataType == GDT_UInt16)
	{
		MdataType = CV_MAKETYPE(CV_16U, 1);
		MdataTypes = CV_MAKETYPE(CV_16U, St.nbands);
	}
	vector<cv::Mat> imgMat;// 每个波段
	float *pafScan = new float[St.Xsize*St.Ysize];   // 存储数据

	for (int i = 0; i < St.nbands; i++)
	{
		GDALRasterBand *pBand = poDataset->GetRasterBand(i + 1);
		//pafScan = new float[tmpCols*tmpRows];
		pBand->RasterIO(GF_Read, 0, 0, St.Xsize, St.Ysize, pafScan,
			St.Xsize, St.Ysize, St.iDataType, 0, 0); //GDT_Float32
		cv::Mat tmpMat = cv::Mat(St.Ysize, St.Xsize, MdataType, pafScan);//CV_32FC1
		imgMat.push_back(tmpMat.clone());
	}
	delete[]pafScan;
	pafScan = NULL;

	cv::Mat img;
	img.create(St.Ysize, St.Xsize, MdataTypes); //CV_32FC(tmpBandSize)
	cv::merge(imgMat, img);
	//释放内存
	imgMat.clear();
	return imgColorCodeChange(St, img);
}
//BGR转换为RGB.将图像格式转为8位
cv::Mat FileFormatChange::imgColorCodeChange(MyStruct St, cv::Mat img)
{
	std::vector<cv::Mat> imgMat(St.nbands);
	std::vector<cv::Mat> tempMat(St.nbands);
	cv::split(img, imgMat);
	tempMat.at(0) = (imgMat.at(2));//BGR-->RGB
	tempMat.at(1) = (imgMat.at(1));
	tempMat.at(2) = (imgMat.at(0));
	cv::Mat img1, img_1;
	cv::merge(tempMat, img1);
	imgMat.clear();
	tempMat.clear();
	////16位转8位
	cv::normalize(img1, img_1, 0, 255, cv::NORM_MINMAX, CV_8UC3);
	return img_1;
}
