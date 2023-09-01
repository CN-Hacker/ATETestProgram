#pragma once
#include <vector>

struct ImageResult
{
    char testName[32];
    unsigned char pData[1024];
};

using ImageResults = std::vector<ImageResult>;

struct DarkImgResultWithDPCOn
{
    size_t badPixelCount;

    // 簇统计
    size_t clusterCount;

    size_t cluster_Cluster5;
    size_t cluster_4T1;
    size_t cluster_4T2;
    size_t cluster_4T3;
    size_t cluster_4T4;
    size_t cluster_3X1;
    size_t cluster_1X3;
    size_t cluster_2X1;
    size_t cluster_1X2;
    size_t cluster_2X1X1;
    size_t cluster_1X2X1;
    size_t cluster_2X2;
    size_t cluster_2D1;
    size_t cluster_2D2;
    size_t cluster_2D3;
    size_t cluster_2D4;
    size_t cluster_1X1;
    size_t cluster_Big;

    size_t hDeadLineSize;

    size_t vDeadLineSize;

    double mean;
    double std;
};

struct DarkImgResultWithDPCOff
{
    size_t badPixelCount1;
    size_t badPixelCount2;

    double mean;
    double std;
};

struct LightImgResult
{
    // 坏线统计
    size_t hDeadLineCount_R;
    size_t hDeadLineCount_Gr;
    size_t hDeadLineCount_Gb;
    size_t hDeadLineCount_B;

    size_t vDeadLineCount_R;
    size_t vDeadLineCount_Gr;
    size_t vDeadLineCount_Gb;
    size_t vDeadLineCount_B;

    // 坏点统计
    size_t blackPixelCount_R;
    size_t blackPixelCount_Gr;
    size_t blackPixelCount_Gb;
    size_t blackPixelCount_B;
    size_t whitePixelCount_R;
    size_t whitePixelCount_Gr;
    size_t whitePixelCount_Gb;
    size_t whitePixelCount_B;

    // 簇统计
    size_t clusterCount;

    size_t cluster_Cluster5;
	size_t cluster_4T1;
	size_t cluster_4T2;
	size_t cluster_4T3;
	size_t cluster_4T4;
	size_t cluster_3X1;
	size_t cluster_1X3;
	size_t cluster_2X1;
	size_t cluster_1X2;
	size_t cluster_2X1X1;
	size_t cluster_1X2X1;
	size_t cluster_2X2;
	size_t cluster_2D1;
	size_t cluster_2D2;
	size_t cluster_2D3;
	size_t cluster_2D4;
	size_t cluster_1X1;
	size_t cluster_Big;

    // Radio
    double radio_R_B;
    double radio_Gr_Gb;
    double radio_R_Gr;
    double radio_B_Gb;

    // Mean
    double mean;
    double mean_R;
    double mean_Gr;
    double mean_Gb;
    double mean_B;

    // Std
    double std_R;
    double std_Gr;
    double std_Gb;
    double std_B;

    // Luminance Uniformity (Shading) with ROI
    // ROI_X(mean) / ROI_5(mean)
    double shading_ROI1;
    double shading_ROI2;
    double shading_ROI3;
    double shading_ROI4;
    double shading_ROI6;
    double shading_ROI7;
    double shading_ROI8;
    double shading_ROI9;

    // Color Uniformity (Balance) with ROI
    // ROI_X(R mean / B mean) / ROI_5(R mean / B mean)
    double balance_RB_ROI1;
    double balance_RB_ROI2;
    double balance_RB_ROI3;
    double balance_RB_ROI4;
    double balance_RB_ROI6;
    double balance_RB_ROI7;
    double balance_RB_ROI8;
    double balance_RB_ROI9;
    // MAX(ROI_X(R mean / B mean)) without ROI5/ROI5
    double balances_RB_Max;
    // ROI_X(Gr mean / Gb mean) / ROI_5(Gr mean / Gb mean)
    double balance_GrGb_ROI1;
    double balance_GrGb_ROI2;
    double balance_GrGb_ROI3;
    double balance_GrGb_ROI4;
    double balance_GrGb_ROI6;
    double balance_GrGb_ROI7;
    double balance_GrGb_ROI8;
    double balance_GrGb_ROI9;
    // MAX(ROI_X(R mean / B mean)) without ROI5/ROI5
    double balances_GrGb_Max;

    // Color shading
    double RG_ROI19, RG_ROI37, RG_ROI28, RG_ROI46;
    double BG_ROI19, BG_ROI37, BG_ROI28, BG_ROI46;

    // Ratio ROI5
    double ratio_ROI5_R_B;
    double ratio_ROI5_Gr_Gb;
    double ratio_ROI5_R_Gr;
    double ratio_ROI5_B_Gb;


    // Blemish
    double blemish;

    // Scratch
    double scratch;
};

struct DarkFPNResult
{
    double AFPN;
    double HFPN;
    double VFPN;
};

struct DarkCurrentResult
{
	double darkcurrent;
};
struct FWCResult
{
	double R_mean;
	double Gr_mean;
	double Gb_mean;
	double B_mean;

	double R_std;
	double Gr_std;
	double Gb_std;
	double B_std;
};
struct PRNUResult
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

	//add in 2023/06/30
	double Diff_R;
	double Diff_Gr;
	double Diff_Gb;
	double Diff_B;
};

struct CGResult
{
    double CG_R;
    double CG_B;
    double CG_Gr;
    double CG_Gb;
	double CG;
};

struct TempNoiseResult
{
	double TempNoise;
};