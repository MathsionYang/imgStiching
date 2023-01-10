#pragma once
#include <gdal.h>
#include <gdal_priv.h>  
//创建一个存放影像信息的structure
typedef struct
{
	//GDALDataset *poDataset = NULL;
	int Xsize = 0;
	int Ysize = 0;
	int nbands = 0;
	double *tmpadfGeoTransform = new double[6]; //存储地理6参数
	const char* proj = NULL;//存储投影
	GDALDataType iDataType = GDT_Byte;
}MyStruct;