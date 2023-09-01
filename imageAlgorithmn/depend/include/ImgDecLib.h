#pragma once
#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_
#include "fftw3.h"

typedef		unsigned char			INT8U;
typedef		unsigned short int		INT16U;
typedef		unsigned long int		INT32U;
typedef		unsigned long long int	INT64U;

typedef		signed char				INT8S;
typedef 	signed short int		INT16S;
typedef 	signed long int			INT32S;
typedef 	signed long long int	INT64S;

typedef		float					FP32;
typedef		double					FP64;
typedef		bool					BLEAN;

#define PI 3.14159265	

#define IN
#define OUT
#define IMG_FAIL	1		
#define IMG_OK		0
#define TRUE		1
#define FALSE		0
#define B_Gb_Gr_R	1
#define R_Gr_Gb_B	2
#define Yr  		77
#define Yg 			150
#define Yb 			29
#define Y_plus 		128
#define Y_dvd 		256
#define Y_dvd_shift 10

#define getGrayData(r,g,b) ((r*Yr + g*Yg + b*Yb + Y_plus) >> Y_dvd_shift)

#define MAXI(a,b) ((a) > (b) ? (a) : (b))
#define MINI(a,b) ((a) < (b) ? (a) : (b))

#define ABS(X) ((X)>=0 ? (X) : (-(X)))

#define DUMP_FFT_DATA		1
#define DUMP_FILTERD_DATA	2
#define DUMP_IFFT_DATA		3

typedef struct _stDumpFFT {
	BLEAN blDump;
	INT8U dumpDATA;
}stDumpFFT;

typedef struct _stBoundery {
	BLEAN flag;
	INT16U x;
	INT16U y;
}stBoundery;

__declspec(dllexport) INT8U getImgSize(IN char* fileName, INT16U* height, INT16U *width);
__declspec(dllexport) INT16U ** imgMalloc(IN INT16U height, IN INT16U  width, INT16U *&MARTRIX);
__declspec(dllexport) void writeData(IN INT16U **data, INT16U width, INT16U height);
__declspec(dllexport) void writeFFTData(fftw_complex *data, INT16U width, INT16U height);
__declspec(dllexport) INT8S bandFilter(IN OUT INT16U** grayData, IN INT16U height, IN INT16U width, IN FP32 d_low, IN FP32 d_high,IN stDumpFFT *pstDumpFFT);
__declspec(dllexport) INT8S bandFilter2(IN OUT INT16U** grayData, IN INT16U height, IN INT16U width, IN FP32 d_low, IN FP32 d_high, IN stDumpFFT *pstDumpFFT);
__declspec(dllexport) INT8S bandFilter3(IN OUT INT16U** grayData, IN INT16U height, IN INT16U width, IN FP32 d_low, IN FP32 d_high, IN stDumpFFT *pstDumpFFT);
__declspec(dllexport) INT8U getRawData(IN char* fileName, IN INT16U width, IN INT16U height, OUT INT16U** grayData);
__declspec(dllexport) INT8U raw2Gray(IN INT16U** grayData, OUT INT16U** ulGrayData, IN INT16U width, IN INT16U height, IN INT8U colorRange);
__declspec(dllexport) INT8U gaussFilter(IN INT16U height, IN INT16U width, IN INT16U** grayData, OUT INT16U** fpGrayData);
__declspec(dllexport) INT16U getThValue(IN INT16U ** grayData, IN INT16U height, IN INT16U width);
__declspec(dllexport) INT8S getSobelAndSupressNonMax(IN OUT INT16U **grayData, IN INT16U height, IN INT16U width);
__declspec(dllexport) INT8S supressIsolatePoint(IN OUT INT16U **grayData, IN INT16U height, IN INT16U width, IN INT16U thValue);
__declspec(dllexport) INT32U getBoundery(IN OUT INT16U **grayData, IN INT16U height, IN INT16U width, IN INT16U thValue,  IN INT8U outputFlag);

#endif

