// Copyright 2011 <chaishushan@gmail.com>. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "opencv.h"
#include <opencv2/imgproc/imgproc.hpp> 
#include "_cgo_export.h"

#include <stdlib.h>
#include <string.h>

//-----------------------------------------------------------------------------
// trackbar
//-----------------------------------------------------------------------------

// trackbar data
struct TrackbarUserdata {
	char* win_name;
	char* bar_name;
	int value;
};
static struct TrackbarUserdata *trackbar_list[1000];
static int trackbar_list_len = 0;

static void trackbarCallback(int pos, void* userdata) {
	struct TrackbarUserdata *arg = (struct TrackbarUserdata*)userdata;
}

extern "C" {

  void cvPhaseCorrelate(){
    cv::Mat x,y,z;
    cv::phaseCorrelate(x,y,z);
  }

int GoOpenCV_CreateTrackbar(
	char* trackbar_name, char* window_name,
	int value, int count
) {
  cv::Mat xx;
	struct TrackbarUserdata *userdata = (struct TrackbarUserdata*)malloc(sizeof(*userdata));
	trackbar_list[trackbar_list_len++] = userdata;

	userdata->win_name = (char*)window_name;
	userdata->bar_name = (char*)trackbar_name;
	userdata->value = value;

	return cvCreateTrackbar2(trackbar_name, window_name,
		&(userdata->value), count,
		trackbarCallback, userdata
	);
}
void GoOpenCV_DestroyTrackbar(char* trackbar_name, char* window_name) {
	int i;
	for(i = 0; i < trackbar_list_len; ++i) {
		if(strcmp((char*)trackbar_list[i]->win_name, window_name)) continue;
		if(strcmp((char*)trackbar_list[i]->bar_name, trackbar_name)) continue;

		free(trackbar_list[i]);
		trackbar_list[i] = trackbar_list[--trackbar_list_len];
		break;
	}
}

//-----------------------------------------------------------------------------
// mouse callback
//-----------------------------------------------------------------------------

static void mouseCallback(int event, int x, int y, int flags, void* param) {
	char* name = (char*)param;
}
void GoOpenCV_SetMouseCallback(const char* window_name) {
	cvSetMouseCallback(window_name, mouseCallback, (void*)window_name);
}

//-----------------------------------------------------------------------------

// video writer args
unsigned GoOpenCV_FOURCC_(int c1, int c2, int c3, int c4) {
	return (unsigned)CV_FOURCC(c1,c2,c3,c4);
}

void cvmSet16U(const CvMat* mat, int row, int col, unsigned short val) {
  *(unsigned short*)(mat->data.ptr+row*(size_t)mat->step+2*col)=val;
}

unsigned short cvmGet16U(const CvMat* mat, int row, int col) {
  return *(unsigned short*)(mat->data.ptr+row*(size_t)mat->step+2*col);
}

unsigned char cvmGet8U(const CvMat* mat, int row, int col) {
  return *(unsigned char*)(mat->data.ptr+row*(size_t)mat->step+col);
}

void cvmSet8U(const CvMat* mat, int row, int col, unsigned char val) {
  *(unsigned char*)(mat->data.ptr+row*(size_t)mat->step+col) = val;
}


short cvmGet16S(const CvMat* mat, int row, int col) {
  return *(short*)(mat->data.ptr+row*(size_t)mat->step+2*col);
}

char cvmGet8S(const CvMat* mat, int row, int col) {
  return *(char*)(mat->data.ptr+row*(size_t)mat->step+col);
}

int cvmGet32S(const CvMat* mat, int row, int col) {
  return *(int*)(mat->data.ptr+row*(size_t)mat->step+4*col);
}

float cvmGet32F(const CvMat* mat, int row, int col) {
  return *(float*)(mat->data.ptr+row*(size_t)mat->step+4*col);
}

void cvmSet32F(const CvMat* mat, int row, int col, float val) {
  *(float*)(mat->data.ptr+row*(size_t)mat->step+4*col) = val;
}
}


//-----------------------------------------------------------------------------

