#pragma once

#ifdef HSC02IMAGEALGORITHM_EXPORTS
#define HSC02IMAGEALGORITHM_API extern "C" __declspec(dllexport)
#else
#define HSC02IMAGEALGORITHM_API extern "C" __declspec(dllimport)
#endif

enum ImgErrorCode
{
    Img_Success = 0,
    Img_Invalid_Ptr,
    Img_Invalid_Size,
    Img_Empty_Mat,
    Img_Init_Fail,
    Img_OpenFile_Fail,
    Img_CV_Exception
};

enum EClusterType
{
    Cluster_Cluster5 = 0,
    Cluster_4T1,
    Cluster_4T2,
    Cluster_4T3,
    Cluster_4T4,
    Cluster_3X1,
    Cluster_1X3,
    Cluster_2X1,
    Cluster_1X2,
    Cluster_2X1X1,
    Cluster_1X2X1,
    Cluster_2X2,
    Cluster_2D1,
    Cluster_2D2,
    Cluster_2D3,
    Cluster_2D4,
    Cluster_1X1,
    Cluster_Big,
    Cluster_MAX
};

enum EROIType
{
    ROI_1 = 0,
    ROI_2,
    ROI_3,
    ROI_4,
    ROI_5,
    ROI_6,
    ROI_7,
    ROI_8,
    ROI_9,
    ROI_MAX,
};

enum ImgPixArray {
    R_GR_GB_B,
    B_GB_GR_R,
    GB_B_R_GR,
    GR_R_B_GB
};

struct ImgCondition_Dark_DPCOn
{
    ImgPixArray ThisPixArray = B_GB_GR_R;

    double DeadLineThreshold = 16; // DN
    double BadPointThreshold = 64; // DN
    double ClusterThreshold = 64; // DN
    int ClusterCountKernelSize = 5; // Pixel
};

struct ImgResult_Dark_DPCOn
{
    size_t BadPixelCount;

    size_t ClusterCount;
    size_t EachClustersSize[Cluster_MAX];

    size_t Horizontal_DeadLine_Length;

    size_t Vertical_DeadLine_Length;

    double Mean;
    double Std;
};

struct ImgCondition_Dark_DPCOff
{
    ImgPixArray ThisPixArray = B_GB_GR_R;
    double BadPointThreshold1_Min = 64, BadPointThreshold1_Max = 254; // DN
    double BadPointThreshold2_Min = 255, BadPointThreshold2_Max = 1025; // DN
};

struct ImgCondition_Dark_TmpNoise
{
    ImgPixArray ThisPixArray = B_GB_GR_R;
    size_t FrameCount = 8;
	// ROI
    int LeftTop_X = 0;
    int LeftTop_Y = 0;
    int ROI_Width = 480;
    int ROI_Height = 190;
	int Gain_x1=1;
	int Gain_x16=16;
};

struct ImgResult_Dark_TmpNoise
{

    double TmpNoise;

};

struct ImgResult_Dark_DPCOff
{
    size_t BadPixel1_Count;
    size_t BadPixel2_Count;

    double Mean;
    double Std;
};

struct ImgCondition_DarkFPN
{
    ImgPixArray ThisPixArray = B_GB_GR_R;
    size_t FrameCount = 100;

    // ROI
    int LeftTop_X = 0;
    int LeftTop_Y = 0;
    int ROI_Width = 480;
    int ROI_Height = 190;
};

struct ImgResult_DarkFPN
{
    double AFPN;
    double HFPN;
    double VFPN;
};

struct ImgCondition_Light
{
    ImgPixArray ThisPixArray = B_GB_GR_R;

    double DeadLineThreshold = 32; // DN

    int BadPointKernelSize = 7; // Pixel
    double BlackPointThreshold = -0.3f; // DN
    double WhitePointThreshold = 0.3f; // DN

    int ClusterBadPointKernelSize = 7; // Pixel
    int ClusterCountKernelSize = 5; // Pixel
    double BlackClusterThreshold = -0.26f; // DN
    double WhiteClusterThreshold = 0.26f; // DN
};

struct ImgResult_Light
{
    // 坏线统计 ok
    size_t R_HorizontalDeadLineCount;
    size_t Gr_HorizontalDeadLineCount;
    size_t Gb_HorizontalDeadLineCount;
    size_t B_HorizontalDeadLineCount;

    size_t R_VerticalDeadLineCount;
    size_t Gr_VerticalDeadLineCount;
    size_t Gb_VerticalDeadLineCount;
    size_t B_VerticalDeadLineCount;

    // 坏点统计 ok
    size_t R_BlackPixelCount;
    size_t Gr_BlackPixelCount;
    size_t Gb_BlackPixelCount;
    size_t B_BlackPixelCount;
    size_t R_WhitePixelCount;
    size_t Gr_WhitePixelCount;
    size_t Gb_WhitePixelCount;
    size_t B_WhitePixelCount;

    // 簇统计
    size_t ClusterCount;
    size_t EachClustersSize[Cluster_MAX];

    // Ratio ok
    double R_B;
    double Gr_Gb;
    double R_Gr;
    double B_Gb;

    // Mean ok
    double Mean;
    double Mean_R;
    double Mean_Gr;
    double Mean_Gb;
    double Mean_B;

    // Std ok
    double Std;
    double Std_R;
    double Std_Gr;
    double Std_Gb;
    double Std_B;

    //////////////////////////////////////////////////// ROI ///////////////////////////////////////////////
    // Luminance Uniformity (Shading) with ROI
    // ROI_X(mean) / ROI_5(mean)
    double LUShadings[ROI_MAX]; // ok

    // Color Uniformity (Balance) with ROI
    // ROI_X(R mean / B mean) / ROI_5(R mean / B mean)
    double CUBalances_RB[ROI_MAX]; // ok
    // MAX(ROI_X(R mean / B mean)) without ROI5/ROI5
    double CUBalances_RB_Max; // ok
    // ROI_X(Gr mean / Gb mean) / ROI_5(Gr mean / Gb mean)
    double CUBalances_GrGb[ROI_MAX]; // ok
    // MAX(ROI_X(R mean / B mean)) without ROI5/ROI5
    double CUBalances_GrGb_Max; // ok

    // Color shading
    double RG_ROI19, RG_ROI37, RG_ROI28, RG_ROI46;
    double BG_ROI19, BG_ROI37, BG_ROI28, BG_ROI46;

    // Ratio ROI5 ok
    double ROI5_R_B;
    double ROI5_Gr_Gb;
    double ROI5_R_Gr;
    double ROI5_B_Gb;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Blemish
    int BlemishCount;

    // Scratch
    int ScratchCount;
};

struct ImgCondition_PRNU
{
    ImgPixArray ThisPixArray = B_GB_GR_R;
    unsigned int Depth = 10;
    size_t FrameCount = 2;

    // ROI
    int LeftTop_X = 214;
    //int LeftTop_Y = 69;
    int LeftTop_Y = 70;
    int ROI_Width = 50;
    int ROI_Height = 50;
};

struct ImgCondition_CG
{
    ImgPixArray ThisPixArray = B_GB_GR_R;
    unsigned int Depth = 10;
    size_t FrameCount = 2;

    // ROI
    int LeftTop_X = 214;
    int LeftTop_Y = 70;
    int ROI_Width = 50;
    int ROI_Height = 50;
};

struct ImgResult_PRNU
{
    double PRNU40_R;
    double PRNU80_R;
    double PRNU_R;
    double PRNU40_Gr;
    double PRNU80_Gr;
    double PRNU_Gr;
    double PRNU40_Gb;
    double PRNU80_Gb;
    double PRNU_Gb;
    double PRNU40_B;
    double PRNU80_B;
    double PRNU_B;

	//add in 2023/06/30     Sensitivity
	double Diff_R;
	double Diff_Gr;
	double Diff_Gb;
	double Diff_B;
};

struct ImgResult_CG
{
    double CG_B;
    double CG_Gr;
    double CG_Gb;
    double CG_R;
	double CG;
};


struct ImageResult_FWC
{
    double Mean_R;
    double Mean_Gr;
    double Mean_Gb;
    double Mean_B;

    double Std_R;
    double Std_Gr;
    double Std_Gb;
    double Std_B;
};

struct ImgCondition_DarkCurrent
{
	ImgPixArray ThisPixArray = B_GB_GR_R;
	unsigned int Depth = 10;
	size_t FrameCount = 1;

	// ROI
	int LeftTop_X = 0;
	//int LeftTop_Y = 69;
	int LeftTop_Y = 0;
	int ROI_Width = 480;
	int ROI_Height = 190;
};

struct ImgResult_DarkCurrent
{
	double darkcurrent;
};


// 暗图不分channel
HSC02IMAGEALGORITHM_API ImgErrorCode DarkImageWithDPCOnTest(unsigned short* RawImage, unsigned int ImageHeight, unsigned int ImageWidth, ImgCondition_Dark_DPCOn Condition, ImgResult_Dark_DPCOn* Res);
HSC02IMAGEALGORITHM_API ImgErrorCode DarkImageWithDPCOffTest(unsigned short* RawImage, unsigned int ImageHeight, unsigned int ImageWidth, ImgCondition_Dark_DPCOff Condition, ImgResult_Dark_DPCOff* Res);
HSC02IMAGEALGORITHM_API ImgErrorCode DarkFPNTest(unsigned short* RawImage, unsigned int ImageHeight, unsigned int ImageWidth, ImgCondition_DarkFPN Condition, ImgResult_DarkFPN* Res);
HSC02IMAGEALGORITHM_API ImgErrorCode DarkTmpNoise(unsigned short* RawImage, unsigned int ImageHeight, unsigned int ImageWidth, ImgCondition_Dark_TmpNoise Condition, ImgResult_Dark_TmpNoise * Res);
// 亮图分channel
HSC02IMAGEALGORITHM_API ImgErrorCode TestLightImage(unsigned short* RawImage, unsigned int ImageHeight, unsigned int ImageWidth, ImgCondition_Light Condition, ImgResult_Light* Res);

HSC02IMAGEALGORITHM_API ImgErrorCode TestRPNU(unsigned short* RawImage40, unsigned short* RawImage80, unsigned ImageHeight, unsigned int ImageWidth, ImgCondition_PRNU Condition, ImgResult_PRNU* Res);

HSC02IMAGEALGORITHM_API ImgErrorCode TestCG(unsigned short* RawImage40, unsigned short* RawImage80, unsigned ImageHeight, unsigned int ImageWidth, ImgCondition_CG Condition, ImgResult_CG* Res);

HSC02IMAGEALGORITHM_API ImgErrorCode FWCTest(unsigned short* RawImage, unsigned ImageHeight, unsigned int ImageWidth, ImgCondition_Light Condition, ImageResult_FWC* Res);

HSC02IMAGEALGORITHM_API ImgErrorCode DarkCurrentTest(unsigned short* RawImageFrame1, unsigned short* RawImageFrame2, unsigned ImageHeight, unsigned int ImageWidth, ImgCondition_DarkCurrent Condition, ImgResult_DarkCurrent* Res);