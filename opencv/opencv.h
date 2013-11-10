// Copyright 2011 <chaishushan@gmail.com>. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.
//
// OpenCV Homepage: http://code.opencv.org

#ifndef _GO_OPENCV_BINDING_H_
#define _GO_OPENCV_BINDING_H_

#if defined(__linux) || defined(__linux__)
//  OpenCV 2.0.x
#   include <opencv/cv.h>
#   include <opencv/highgui.h>
//#   include <opencv2/photo/photo_c.h>
//#   include <opencv2/imgproc/imgproc_c.h>
#elif defined(WIN32) || defined(_WIN32)
//  OpenCV 2.4.x
#   include <opencv/cv.h>
#   include <opencv/highgui.h>
#   include <opencv2/photo/photo_c.h>
#   include <opencv2/imgproc/imgproc_c.h>
#else   // Mac OS X ?
//  OpenCV 2.4.x ?
#   include <opencv/cv.h>
#   include <opencv/highgui.h>
#   include <opencv2/photo/photo_c.h>
#   include <opencv2/imgproc/imgproc_c.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

  void cvPhaseCorrelate();

// Trackbar
int GoOpenCV_CreateTrackbar(
	char* trackbar_name, char* window_name,
	int value, int count
);
void GoOpenCV_DestroyTrackbar(
	char* trackbar_name, char* window_name
);

// mouse callback
void GoOpenCV_SetMouseCallback(
	const char* window_name
);

// video writer args
unsigned GoOpenCV_FOURCC_(
	int c1, int c2, int c3, int c4
);

unsigned short cvmGet16U(const CvMat* mat, int row, int col);
void cvmSet16U(const CvMat* mat, int row, int col, unsigned short val);

unsigned char cvmGet8U(const CvMat* mat, int row, int col);
void cvmSet8U(const CvMat* mat, int row, int col, unsigned char val);

short cvmGet16S(const CvMat* mat, int row, int col);

char cvmGet8S(const CvMat* mat, int row, int col);

int cvmGet32S(const CvMat* mat, int row, int col);

float cvmGet32F(const CvMat* mat, int row, int col);
void cvmSet32F(const CvMat* mat, int row, int col, float val);

#ifdef __cplusplus
}
#endif

#endif // _GO_OPENCV_BINDING_H_


