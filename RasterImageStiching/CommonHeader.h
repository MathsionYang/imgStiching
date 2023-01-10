#pragma once
#include <gdal.h>
#include <gdal_priv.h>  
//����һ�����Ӱ����Ϣ��structure
typedef struct
{
	//GDALDataset *poDataset = NULL;
	int Xsize = 0;
	int Ysize = 0;
	int nbands = 0;
	double *tmpadfGeoTransform = new double[6]; //�洢����6����
	const char* proj = NULL;//�洢ͶӰ
	GDALDataType iDataType = GDT_Byte;
}MyStruct;