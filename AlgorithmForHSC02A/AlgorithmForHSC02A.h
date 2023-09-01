#include "EvtCISAlgorithm.h"
// successful softbin of image
#define SUCCESS_SB 2

ALGAPI EVT_ALG_RET Deinit();

ALGAPI EVT_ALG_RET Init(char* pAddr);

ALGAPI EVT_ALG_RET Dark_DPCOn(
    unsigned short* rawData,
    unsigned char* frameData,
    int frameWidth,
    int frameHeight,
    int frameCnt,
    EVT_IMG_FORMAT frameFormat,
    int pixelDepth,
    unsigned int siteNum,
    SaveImageCallBack saveCallBack,
    int argc,
    char** argv);

ALGAPI EVT_ALG_RET Dark_DPCOff(
    unsigned short* rawData,
    unsigned char* frameData,
    int frameWidth,
    int frameHeight,
    int frameCnt,
    EVT_IMG_FORMAT frameFormat,
    int pixelDepth,
    unsigned int siteNum,
    SaveImageCallBack saveCallBack,
    int argc,
    char** argv);

ALGAPI EVT_ALG_RET TempNoise(
    unsigned short* rawData,
    unsigned char* frameData,
    int frameWidth,
    int frameHeight,
    int frameCnt,
    EVT_IMG_FORMAT frameFormat,
    int pixelDepth,
    unsigned int siteNum,
    SaveImageCallBack saveCallBack,
    int argc,
    char** argv);

EVT_ALG_RET test_demo(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv
);

ALGAPI EVT_ALG_RET DarkFPN(
    unsigned short* rawData,
    unsigned char* frameData,
    int frameWidth,
    int frameHeight,
    int frameCnt,
    EVT_IMG_FORMAT frameFormat,
    int pixelDepth,
    unsigned int siteNum,
    SaveImageCallBack saveCallBack,
    int argc,
    char** argv);

ALGAPI EVT_ALG_RET PRNU40(
    unsigned short* rawData,
    unsigned char* frameData,
    int frameWidth,
    int frameHeight,
    int frameCnt,
    EVT_IMG_FORMAT frameFormat,
    int pixelDepth,
    unsigned int siteNum,
    SaveImageCallBack saveCallBack,
    int argc,
    char** argv);

ALGAPI EVT_ALG_RET PRNU80(
    unsigned short* rawData,
    unsigned char* frameData,
    int frameWidth,
    int frameHeight,
    int frameCnt,
    EVT_IMG_FORMAT frameFormat,
    int pixelDepth,
    unsigned int siteNum,
    SaveImageCallBack saveCallBack,
    int argc,
    char** argv);

ALGAPI EVT_ALG_RET CG_PRNU40(
    unsigned short* rawData,
    unsigned char* frameData,
    int frameWidth,
    int frameHeight,
    int frameCnt,
    EVT_IMG_FORMAT frameFormat,
    int pixelDepth,
    unsigned int siteNum,
    SaveImageCallBack saveCallBack,
    int argc,
    char** argv);

ALGAPI EVT_ALG_RET CG_PRNU80(
    unsigned short* rawData,
    unsigned char* frameData,
    int frameWidth,
    int frameHeight,
    int frameCnt,
    EVT_IMG_FORMAT frameFormat,
    int pixelDepth,
    unsigned int siteNum,
    SaveImageCallBack saveCallBack,
    int argc,
    char** argv);

ALGAPI EVT_ALG_RET PRNU(
    unsigned int rsrcID,
    unsigned int siteNum,
    int argc,
    char** argv);

ALGAPI EVT_ALG_RET CG(
    unsigned int rsrcID,
    unsigned int siteNum,
    int argc,
    char** argv);

ALGAPI EVT_ALG_RET FWC(
    unsigned short* rawData,
    unsigned char* frameData,
    int frameWidth,
    int frameHeight,
    int frameCnt,
    EVT_IMG_FORMAT frameFormat,
    int pixelDepth,
    unsigned int siteNum,
    SaveImageCallBack saveCallBack,
    int argc,
    char** argv);

ALGAPI EVT_ALG_RET Light(
    unsigned short* rawData,
    unsigned char* frameData,
    int frameWidth,
    int frameHeight,
    int frameCnt,
    EVT_IMG_FORMAT frameFormat,
    int pixelDepth,
    unsigned int siteNum,
    SaveImageCallBack saveCallBack,
    int argc,
    char** argv);

ALGAPI EVT_ALG_RET TGP_ColorBar(
    unsigned short* rawData,
    unsigned char* frameData,
    int frameWidth,
    int frameHeight,
    int frameCnt,
    EVT_IMG_FORMAT frameFormat,
    int pixelDepth,
    unsigned int siteNum,
    SaveImageCallBack saveCallBack,
    int argc,
    char** argv);

ALGAPI EVT_ALG_RET GetTestResult(
    unsigned int rsrcID,
    unsigned int siteNum,
    int argc,
    char** argv);

// Dont modify this function define
ALGAPI EVT_ALG_RET preprocess(
    unsigned char* pImageBuffer,
    uint16_t* width,
    uint16_t* height,
    unsigned int* frameCount,
    uint8_t* average,
    const char* testName);

ALGAPI EVT_ALG_RET DarkCurrent(
	unsigned int rsrcID,
	unsigned int siteNum,
	int argc,
	char** argv);

ALGAPI EVT_ALG_RET DarkFrame1(
	unsigned short* rawData,
	unsigned char* frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv);

ALGAPI EVT_ALG_RET DarkFrame2(
	unsigned short* rawData,
	unsigned char* frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv);