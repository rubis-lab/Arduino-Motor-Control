#include "laneDetect.h"

using namespace std;

CvCapture* pCapture;
sPoint _vp;
IplImage* pDraw;

//#ifndef M_PI
#define M_PI		3.14159265358979323846	// pi 
#define M_PIf		3.14159265358979323846f	// pi 
#define M_PI_2		1.57079632679489661923 	// pi/2 
//#endif

#define _RAD2DEG	(180./M_PI)
#define _DEG2RAD	(M_PI/180.)

bool lessY(const sPoint& lhs, const sPoint& rhs)
{
	return lhs.y < rhs.y;
}

bool moreY(const sPoint& lhs, const sPoint& rhs)
{
	return lhs.y > rhs.y;
}

inline double DeltaRad (double ang1, double ang2)
{
	double da = ang1 - ang2;
	if(-M_PI < da && da < M_PI)
	{
		return da;
	}
	else
	{
		da = fmod(da, 2*M_PI);
		if(M_PI <= da)
		{
			return da - 2*M_PI;
		}
		else if(da <= -M_PI)
		{
			return da + 2*M_PI;
		}
		else
		{
			return da;
		}
	}
	return da;
}

inline float DeltaRad (float ang1, float ang2)
{
	float da = ang1 - ang2;
	if(-M_PIf < da && da < M_PIf)
	{
		return da;
	}
	else
	{
		da = fmod (da, 2.f*M_PIf);
		if(M_PIf <= da)
		{
			return da - 2.f*M_PIf;
		}
		else if(da <= -M_PIf)
		{
			return da + 2.f*M_PIf;
		}
		else
		{
			return da;
		}
	}
	return da;
}

inline int INT_ (const double a)
{
	return (0 < a)? (int)(a + 0.5) : (int)(a - 0.5);
}


sPoint VanishingPoint(vector<sLine> &cand, vector<sLine> *inlier)
{
	if (inlier) {
		inlier->reserve (cand.size());

		for (unsigned int i=0; i<cand.size(); i++) { 
			inlier->push_back (cand[i]);
		}
	}

	return _vp;
}

void LaneCandidate(IplImage *mag, IplImage *ori, IplImage *dst)
{
#define pPixel(img,y,x)	(img->imageData + (y)*img->widthStep + img->nChannels*img->depth/8*(x))
	IplImage *mask = cvCreateImage (cvGetSize(mag), IPL_DEPTH_8U, 1);
	cvZero (mask);

	//CvPoint pts[3] = { {0, mag->height}, {mag->width/2, 0}, {mag->width, mag->height} };
	//cvFillConvexPoly (mask, &pts[0], 3, cvScalar(255));
	CvPoint pts[4] = { {0, mag->height}, {mag->width/9, 0}, {mag->width*8/9, 0}, {mag->width, mag->height} };
	cvFillConvexPoly (mask, &pts[0], 4, cvScalar(255));

	vector<sLaneCand> tmp;
	tmp.reserve (mag->width);

	for(int h = 0; h < mag->height; h++)
	{
		tmp.clear();

		float *m = (float *)pPixel(mag,  h, 0);
		float *o = (float *)pPixel(ori,  h, 0);
		char  *d = (char  *)pPixel(dst,  h, 0);
		char  *k = (char  *)pPixel(mask, h, 0);

		for(int w = 1, nw = mag->width-1; w < nw; w++) 
		{
			if(!k[w])
			{
				const double margin = 0.5;

				if((M_PI*2-margin < o[w] || o[w] < +margin) ||
						(M_PI-margin < o[w] && o[w] < M_PI+margin) ) 
				{
					continue;
				}
			}
			
			//const double threshold = 0.15;
			const double threshold = 0.1;

			if (threshold < m[w] && m[w-1] < m[w] && m[w] > m[w+1])
			{
				tmp.push_back (sLaneCand(w, h, m[w], o[w]));
			}
		}

		const double lane_width_max = 50.;
		double lane_width = h*lane_width_max/mag->height;
		int lane_width_lo = -3 + (int)(lane_width*0.2);
		int lane_width_hi = 3 + (int)(lane_width*1.8);

		const float mag_threshold = 0.2f;
		const float ori_threshold = 0.5f;

		for(int i = 0; i < (int)tmp.size()-1; i++)
		{
			if (M_PI/2 < tmp[i].ori && tmp[i].ori < M_PI*3/2)
			{
				continue;
			}

			for(int j = i+1; j < (int)tmp.size(); j++)
			{
				int width = tmp[j].x - tmp[i].x;

				if(lane_width_lo <= width && width <= lane_width_hi)
				{
					float mag_err = (tmp[j].mag - tmp[i].mag)/(tmp[j].mag + tmp[i].mag);
					float ori_err = DeltaRad (DeltaRad (tmp[j].ori, tmp[i].ori), M_PIf);

					if ((-mag_threshold < mag_err && mag_err < +mag_threshold) && 
							(-ori_threshold < ori_err && ori_err < +ori_threshold))
					{
						int x_mid = (tmp[j].x + tmp[i].x)/2;
						float mag_avg = (tmp[j].mag + tmp[i].mag)/2.f;
						float ori_avg = (tmp[j].ori + tmp[i].ori)/2.f + M_PIf; 

						d[x_mid] = (mag_avg <= 1.) ? (int)(mag_avg*255) : 255;

						cvCircle (pDraw, cvPoint(x_mid, h), 2, CV_RGB(0,0,255));
					}
				}
				else if (width > lane_width_hi)
				{
					break;
				}
			}
		}
	}

	cvReleaseImage (&mask);
}

int nCount, gAvgX, gAvgY, prevX, prevY;

int LaneDetect(IplImage* pSrc, IplImage* pDst)
{
	int ret = 0;
	pDraw = pDst;

	uchar p[3];

	for(int y = 0; y < pSrc->height; y++)
	{
		for(int x = 0; x < pSrc->width; x++) 
		{
			p[0] = pSrc->imageData[pSrc->widthStep * y + x * 3];   // B 
			p[1] = pSrc->imageData[pSrc->widthStep * y + x * 3 + 1];   // G 
			p[2] = pSrc->imageData[pSrc->widthStep * y + x * 3 + 2]; // R 

			pSrc->imageData[pSrc->widthStep * y + x * 3] = 255 - p[0];
			pSrc->imageData[pSrc->widthStep * y + x * 3 + 1] = 255;
			pSrc->imageData[pSrc->widthStep * y + x * 3 + 2] = 255 - p[2];
		}
	}

	IplImage* pGray = cvCreateImage(cvGetSize(pSrc), IPL_DEPTH_8U, 1);
	cvCvtColor(pSrc, pGray, CV_RGB2GRAY);

	IplImage* pImg_32f = cvCreateImage(cvGetSize(pGray), IPL_DEPTH_32F, 1);
	cvConvertScale(pGray, pImg_32f, 1.0 / 255.0, 0);
	cvSmooth(pImg_32f, pImg_32f, CV_GAUSSIAN, 5);

	IplImage* pDiff_x = cvCreateImage(cvGetSize(pImg_32f), IPL_DEPTH_32F, 1);
	IplImage* pDiff_y = cvCreateImage(cvGetSize(pImg_32f), IPL_DEPTH_32F, 1);
	cvSobel(pImg_32f, pDiff_x, 1, 0, 3);
	cvSobel(pImg_32f, pDiff_y, 0, 1, 3);

	IplImage* pMagnitude = cvCreateImage(cvGetSize(pImg_32f), IPL_DEPTH_32F, 1);
	IplImage* pOrientation = cvCreateImage(cvGetSize(pImg_32f), IPL_DEPTH_32F, 1);
	cvCartToPolar(pDiff_x, pDiff_y, pMagnitude, pOrientation, 0);

	IplImage* pMagnitude2 = cvCreateImage(cvGetSize(pMagnitude), IPL_DEPTH_8U, 1);
	cvZero(pMagnitude2);

	LaneCandidate(pMagnitude, pOrientation, pMagnitude2);

	vector<sLine> cand;
	cand.reserve(1000);

	CvMemStorage* storage = cvCreateMemStorage(0);

	//CvSeq* lines = cvHoughLines2(pMagnitude2, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 5, 2, 10);
	CvSeq* lines = cvHoughLines2(pMagnitude2, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 50, 50, 30);
	for(int i = 0; i < lines->total; i++)
	{
		CvPoint* line = (CvPoint*)cvGetSeqElem(lines, i);
		cand.push_back (sLine(line[0].x, line[0].y, line[1].x, line[1].y));
		//cvLine(pDraw, line[0], line[1], CV_RGB(255,0,0), 3, 8);
	}

	cvReleaseMemStorage(&storage);

	vector<sLine> inlier;
	sPoint vp = VanishingPoint (cand, &inlier);

	CvMemStorage* storage2 = cvCreateMemStorage(0);
	CvPoint pt0;
	CvSeq* ptseq = cvCreateSeq(CV_SEQ_KIND_GENERIC | CV_32SC2, sizeof(CvContour), sizeof(CvPoint), storage2);
	for (unsigned int i=0; i<inlier.size(); i++) { 
		cvLine (pDraw, cvPoint(inlier[i].sx, inlier[i].sy), cvPoint(inlier[i].ex, inlier[i].ey), CV_RGB(0,255,0), 3, 8);
		pt0.x = inlier[i].sx; pt0.y = inlier[i].sy;
		cvSeqPush(ptseq, &pt0);
		pt0.x = inlier[i].ex; pt0.y = inlier[i].ey;
		cvSeqPush(ptseq, &pt0);
	}

	if(ptseq->total)
	{
		vector<sPoint> cPoint;		

		CvSeq* hull = cvConvexHull2(ptseq, 0, CV_COUNTER_CLOCKWISE, 0);
		int hullcount = hull->total;

		CvFont Font;
		cvInitFont(&Font, CV_FONT_HERSHEY_PLAIN, 1.0f, 1.0f);
		char temp[256] = {0, };
		pt0 = **CV_GET_SEQ_ELEM(CvPoint*, hull, hullcount-1);
		for (int i = 0; i < hullcount; i++)
		{			
			cPoint.push_back(sPoint(pt0.x, pt0.y));
			
			CvPoint pt = **CV_GET_SEQ_ELEM(CvPoint*, hull, i);
			cvLine (pDraw, pt0, pt, CV_RGB(255, 0, 0));		
			pt0 = pt;
		}

		if(cPoint.size() > 2 && lines->total >= 2)
		{
			//cSPoint : 4 rectangular points;
			// index 0 : top-left
			// index 1 : top-right
			// index 2 : bottom-right
			// index 3 : bottom-left
			vector<sPoint> cSPoint = vector<sPoint>(4);

			int avgX = 0;
			int avgY = 0;
			int minX = 640;
			int minX2 = 640;
			int maxX = 0;
			int maxX2 = 0;

			int bottomLeftIdx = 0;
			int bottomRightIdx = 0;
			int topLeftIdx = 0;
			int topRightIdx = 0;

	
			for(int i = 0; i < cPoint.size(); i++)
			{
				// find average points
				avgX += cPoint[i].x;
				avgY += cPoint[i].y;

				// find two mininum x pionts
				if (cPoint[i].x < minX2)
				{
					if (cPoint[i].x < minX)
					{
						minX2 = minX;
						minX = cPoint[i].x;
						topLeftIdx = bottomLeftIdx;
						bottomLeftIdx = i;
					}
					else
					{
						minX2 = cPoint[i].x;
						topLeftIdx = i;
					}
				}

				// find two maximum x points
				if (cPoint[i].x > maxX2)
				{
					if (cPoint[i].x > maxX)
					{
						maxX2 = maxX;
						maxX = cPoint[i]. x;
						topRightIdx = bottomRightIdx;
						bottomRightIdx = i;
					}
					else
					{
						maxX2 = cPoint[i].x;
						topRightIdx = i;
					}
				}
			}
			//cvCircle (pDst, cvPoint((cPoint[0].x+cPoint[1].x)/2, (cPoint[0].y+cPoint[1].y)/2), 5, CV_RGB(255,255,255), 3, 8);
			//ret = 320 - (cPoint[0].x+cPoint[1].x)/2;
			avgX = avgX / cPoint.size();
			avgY = avgY / cPoint.size();
			//int avgX = (cPoint[0].x+cPoint[1].x)/2;
			//int avgY = (cPoint[0].y+cPoint[1].y)/2;	
			//printf("avgX : %d, avgY : %d\n", avgX, avgY);

			// determine top bottom points
			if (cPoint[topLeftIdx].y > cPoint[bottomLeftIdx].y)
			{
				int temp = topLeftIdx;
				topLeftIdx = bottomLeftIdx;
				bottomLeftIdx = temp;
			}
			if (cPoint[topRightIdx].y > cPoint[bottomRightIdx].y)
			{
				int temp = topRightIdx;
				topRightIdx = bottomRightIdx;
				bottomRightIdx = temp;
			}
	
			//top-left0
			cSPoint[0] = cPoint[topLeftIdx];
			//top-right1
			cSPoint[1] = cPoint[topRightIdx];
			//bottom-right2
			cSPoint[2] = cPoint[bottomRightIdx];
			//bottom-left3
			cSPoint[3] = cPoint[bottomLeftIdx];

			// draw detected points
			for (int i = 0; i < cSPoint.size() ;i++)
			{
				sprintf(temp, "%d.(%d, %d)", i, cSPoint[i].x, cSPoint[i].y);
				cvPutText(pDst, temp, cvPoint(cSPoint[i].x, cSPoint[i].y), &Font, cvScalar(0, 0, 255));
			}

			// calculate centerpoint
			double dXL = cSPoint[0].x - cSPoint[3].x;
			double dYL = cSPoint[0].y - cSPoint[3].y;
			double dXR = cSPoint[1].x - cSPoint[2].x;
			double dYR = cSPoint[1].y - cSPoint[2].y;

			if(dYL && dYR)
			{
				double xL = (dXL*280 - dXL*cSPoint[3].y + dYL*cSPoint[3].x) / dYL;
				double xR = (dXR*280 - dXR*cSPoint[2].y + dYR*cSPoint[2].x) / dYR;
				cvCircle (pDst, cvPoint(int((xL+xR)/2), 280), 5, CV_RGB(0,0,0), 3, 8);
				ret = 320 -((xL+xR)/2);
			}

			//sort(cPoint.begin(), cPoint.end(), lessY);
	
			if(abs(avgX - prevX) > 30 || abs(avgY - prevY) > 30)
			{
				prevX = avgX;
				prevY = avgY;
			}
			cvCircle (pDst, cvPoint(prevX, prevY), 5, CV_RGB(255,255,255), 3, 8);	
			//ret = 320 - prevX;		
			//cvCircle (pDst, cvPoint(avgX, avgY), 5, CV_RGB(255,255,255), 3, 8);			
			//ret = 320 - avgX;
		}

		cvReleaseMemStorage (&storage2);
	}
	cvShowImage("result", pDst);

	cvReleaseImage(&pGray);
	cvReleaseImage(&pImg_32f);
	cvReleaseImage(&pDiff_x);
	cvReleaseImage(&pDiff_y);
	cvReleaseImage(&pMagnitude2);
	cvReleaseImage(&pMagnitude);
	cvReleaseImage(&pOrientation);
	cvWaitKey(10);

	return ret;
}

int initLD(void)
{
	pCapture = cvCaptureFromCAM(0);
	if(pCapture == NULL)
	{
		printf("Can't open CAMERA\n");
		return -1;
	}
	nCount = 0;
	gAvgX = 0;
	gAvgY = 0;
	prevX = -100;
	prevY = -100;
	return 0;
}

int laneDetect(void)
{
	IplImage* pSrc = cvQueryFrame(pCapture);
	if(pSrc)
	{
		CvRect ROI = cvRect(0, 0, 640, 480);
		cvSetImageROI(pSrc, ROI);
		IplImage* pSrcROI = cvCreateImage(cvSize(ROI.width, ROI.height), pSrc->depth, pSrc->nChannels);
		cvCopy(pSrc, pSrcROI);
		cvResetImageROI(pSrc);
		
		IplImage* pDst = cvCreateImage(cvGetSize(pSrcROI), IPL_DEPTH_8U, 3);
		cvCopy(pSrcROI, pDst, 0);
		int ret = LaneDetect(pSrcROI, pDst);
		
		cvReleaseImage(&pSrcROI);
		cvReleaseImage(&pDst);
		return ret;
	}

	return -1;
}

#if 0
void* DoLaneDetect(void* arg)
{
	/*
	cvGrapFrame(pCapture);
	IplImage* pSrc = cvRetrieveFrame(pCapture);
	*/

	IplImage* pSrc = cvQueryFrame(pCapture);
	if(pSrc)
	{
		CvRect ROI = cvRect(0, 100, 640, 330);
		cvSetImageROI(pSrc, ROI);
		IplImage* pSrcROI = cvCreateImage(cvSize(ROI.width, ROI.height), pSrc->depth, pSrc->nChannels);
		cvCopy(pSrc, pSrcROI);
		cvResetImageROI(pSrc);

		IplImage* pDst = cvCreateImage(cvGetSize(pSrcROI), IPL_DEPTH_8U, 3);
		cvCopy(pSrcROI, pDst, 0);
		LaneDetect(pSrcROI, pDst);

		cvReleaseImage(&pSrcROI);
		cvReleaseImage(&pDst);
	}
}

int main(void)
{
	pCapture = cvCaptureFromCAM(0);
	
	int status;
	pthread_t th;

	status = pthread_create(&th, NULL, DoLaneDetect, NULL);

	pthread_join(th, NULL);

	return 0;
}
#endif
