#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <gdal_priv.h>
#include <vector>
#include <string>
#include <math.h>
#include <iostream>
using namespace cv;
using namespace std;

/***********************************************************/
// GCDataType:GDAL��OpenCV��������ת�����м��ʽ
// GC_Byte   =======  GDT_Byte   =======  CV_8U  =======  unsigned char
// GC_UInt16 =======  GDT_UInt16 =======  CV_16U =======  unsigned short
// GC_Int16  =======  GDT_Int16  =======  CV_16S =======  short int
// GC_UInt32 =======  GDT_UInt32 =======  ȱʧ   =======  unsigned long
// GC_Int32  =======  GDT_Int32  =======  CV_32S =======  long
// GC_Float32=======  GDT_Float32=======  CV_32F =======  float
// GC_Float64=======  GDT_Float64=======  CV_64F =======  double
/***********************************************************/
typedef enum {
	GC_Byte = 0,
	GC_UInt16 = 1,
	GC_Int16 = 2,
	GC_UInt32 = 3,
	GC_Int32 = 4,
	GC_Float32 = 5,
	GC_Float64 = 6,
	GC_ERRType = 7
} GCDataType;

class GDALOpenCV
{
	/***********************************************************/
	// PatchIndex:�洢����Ӱ��ֿ���Ϣ
	// iPatch �����1----�����߽��غ�    2----�����߽��غ�   3----�ĸ��߽��غ�
	//  11 12 13 14     21 22 23 24    31
	// =======================
	//   11 =    21    =  12
	// =======================
	//   24 =    31    =  22
	// =======================
	//   14 =    23    =  13
	// =======================
	/***********************************************************/
	typedef struct {
		int iPatch;      //  �ڼ���
		int row_begin;   // Ӱ������ʼ��
		int col_begin;   // Ӱ������ʼ��
		int width;       // Ӱ���Ŀ��
		int heigth;     // Ӱ���ĸ߶�
	} PatchIndex;


public:
	GDALOpenCV(const std::string fileName);  // Ψһ���캯��
	~GDALOpenCV(void);   // ��������

public:
	void Initialization();   // ��ʼ��  ʵ����Ϊ��ȡӰ���ʽ;
	cv::Size SetPatchSize(const int r, const int c)  // ���÷ֿ��С
	{
		m_patchSize.width = c;  m_patchSize.height = r;
		return m_patchSize;
	};

	void SetOverlappedPixel(const int num)
	{
		m_overlappedPixel = num;
	};


	bool GDAL2Mat(cv::Mat &img);  // Ӱ���ȡΪMat��ʽ  ���ֿ�
	bool Mat2File(const std::string outFileName, cv::Mat &img, const int flag = 1);  // Mat�ļ����ΪӰ��
	//  flag = Ĭ��Ϊ1  ���TIFF   ���⻹֧��ENVI �� ARDAS���ݸ�ʽ

	int GetImgToPatchNum();  // ����Ӱ��ֿ���  ��  ��ȡӰ��ֿ���Ϣ
	void GetROIFromPatchIndex(const int, cv::Mat &);  //  ��ȡ��Ӧ���ŵ�Ӱ��
	bool SetROIMatToFileByIndex(const std::string outFile, cv::Mat &img,
		const int index, const int flag = 1); // Ӱ��ֿ�д��  �д��Ľ�  ����ϸ���е���ȶ 

	GCDataType GDALType2GCType(const GDALDataType ty); // GDAL Type ==========> GDALOpenCV Type
	GDALDataType GCType2GDALType(const GCDataType ty); //  GDALOpenCV Type ==========> GDAL Type
	GCDataType OPenCVType2GCType(const int ty); // OPenCV Type ==========> GDALOpenCV Type
	int GCType2OPenCVType(const GCDataType ty); // GDALOpenCV Type ==========> OPenCV Type

private:
	void* AllocateMemory(const GCDataType lDataType, const long long lSize);   // ���ܷ����ڴ�
	void* SetMemCopy(void *dst, const void *src, const GCDataType lDataType, const long long lSize);

public:
	GCDataType m_dataType;  // ��������
	int m_imgWidth; // Ӱ����   ����
	int m_imgHeigth; // Ӱ��߶�   ����
	int m_bandNum; // Ӱ�񲨶���
private:
	//GDALDataType m_gdalType;
	GDALDataset *m_poDataSet;  // ����������
	GDALDataset *m_outPoDataSet;
	cv::Size m_patchSize;// �ֿ�ͼ���С
	//std::string m_fileName; // �ļ���  ��
	vector<PatchIndex> *m_patchIndex;//�ֿ��ʶ
	int m_overlappedPixel;
};

