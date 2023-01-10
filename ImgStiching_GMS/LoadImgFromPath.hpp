#ifndef _LoadImgFromPath_hpp_
#define _LoadImgFromPath_hpp_
#include "CommonHeade.h"
class LoadImgFromPath
{
public:
	LoadImgFromPath() 
	{
		scale = 2;
		dst = 0;
		dst_cvsize.width = 0;
		dst_cvsize.height = 0;
	}

	vector<Mat>  GetFiles(string path);
private:
	double scale;
	CvSize dst_cvsize;
	IplImage *dst ;
	vector<Mat> imgs;
	vector<String> files;
};
vector<Mat> LoadImgFromPath::GetFiles(string path)
{
	glob(path + "/*", files, false);
	if (files.size() > 0)
	{
	
		for (int i = 0; i < files.size(); i++)
		{
			Mat img = imread(files[i]);
			IplImage* ipl_img = 0;
			ipl_img = &IplImage(img);

			dst_cvsize.width = (int)((ipl_img->width) / scale);
			dst_cvsize.height = (int)((ipl_img->height) / scale);

			dst = cvCreateImage(dst_cvsize, ipl_img->depth, ipl_img->nChannels);
			cvResize(ipl_img, dst, CV_INTER_NN);//
			Mat mat_img;
			mat_img = cvarrToMat(dst);//3.x°æ±¾
			imgs.push_back(mat_img);
		}
		return imgs;
	}
}

#endif