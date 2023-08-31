// HSC02_Image_Algorithm.cpp : 定义 DLL 应用程序的导出函数。
//

#include "HSC02_Image_Algorithm.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <map>
#include <stack>
#include <numeric>
#include "../../depend\include\ImgDecLib.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "opencv2/opencv.hpp"
#include <Windows.h>

struct MaxMin
{
    unsigned short MinMatValue;
    unsigned short MaxMatValue;
};

enum EPixType
{
    R = 0x10,
    Gr = 0x20,
    Gb = 0x40,
    B = 0x80,
    PixType_Max = 4
};

const std::map<
    ImgPixArray, 
    std::array<EPixType, PixType_Max>
> PixArray2PixType
{
    {R_GR_GB_B, {R, Gr, Gb, B}},
    {B_GB_GR_R, {B, Gb, Gr, R}},
    {GB_B_R_GR, {Gb, B, R, Gr}},
    {GR_R_B_GB, {Gr, R, B, Gb}},
};

struct MatAllMean
{
    double Mean_All;
    double Mean_R;
    double Mean_Gr;
    double Mean_Gb;
    double Mean_B;
};

struct MatAllStd
{
    double Std_All;
    double Std_R;
    double Std_Gr;
    double Std_Gb;
    double Std_B;
};

using TMatROIS = std::array<cv::Mat, ROI_MAX>;
using TMeanROIS = std::array<MatAllMean, ROI_MAX>;
using TStdROIS = std::array<MatAllStd, ROI_MAX>;

const std::array<cv::Mat_<ushort>, EClusterType::Cluster_MAX - 1> ClusterKindMats
{
    (cv::Mat_<ushort>(3,3) << 0,0xF0,0,0xF0,0xF0,0xF0,0,0xF0,0), // Cluster_Cluster5 = 0,
    (cv::Mat_<ushort>(2,3) << 0,0xF0,0,0xF0,0xF0,0xF0), // Cluster_4T1,
    (cv::Mat_<ushort>(3,2) << 0xF0,0,0xF0,0xF0,0xF0,0), // Cluster_4T2,
    (cv::Mat_<ushort>(2,3) << 0xF0,0xF0,0xF0,0,0xF0,0), // Cluster_4T3,
    (cv::Mat_<ushort>(3,2) << 0,0xF0,0xF0,0xF0,0,0xF0), // Cluster_4T4,
    (cv::Mat_<ushort>(1,3) << R,Gr,R), // Cluster_3X1,
    (cv::Mat_<ushort>(3,1) << B,Gr,B), // Cluster_1X3,
    (cv::Mat_<ushort>(1,2) << 0xF0,0xF0), // Cluster_2X1,
    (cv::Mat_<ushort>(2,1) << B,Gr), // Cluster_1X2,
    (cv::Mat_<ushort>(1,3) << R,0,R), // Cluster_2X1X1,
    (cv::Mat_<ushort>(3,1) << B,0,B), // Cluster_1X2X1,
    (cv::Mat_<ushort>(2,2) << R,Gr,Gb,B), // Cluster_2X2,
    (cv::Mat_<ushort>(2,2) << 0,B,R,0), // Cluster_2D1,
    (cv::Mat_<ushort>(2,2) << B,0,0,R), // Cluster_2D2,
    (cv::Mat_<ushort>(3,3) << 0,0,B,0,0,0,B,0,0), // Cluster_2D3,
    (cv::Mat_<ushort>(3,3) << B,0,0,0,0,0,0,0,B), // Cluster_2D4,
    (cv::Mat_<ushort>(1,1) << Gr), // Cluster_1X1,
    //CClusterKindMat{1, (cv::Mat_<ushort>(0,0))}, // Cluster_Big,
};

// For print debug information
// for EClusterType to String
const std::map<EClusterType, std::string> Enum2String_ClusterType
{
    {Cluster_Cluster5,"Cluster_Cluster5"},
    {Cluster_4T1,"Cluster_4T1"},
    {Cluster_4T2,"Cluster_4T2"},
    {Cluster_4T3,"Cluster_4T3"},
    {Cluster_4T4,"Cluster_4T4"},
    {Cluster_3X1,"Cluster_3X1"},
    {Cluster_1X3,"Cluster_1X3"},
    {Cluster_2X1,"Cluster_2X1"},
    {Cluster_1X2,"Cluster_1X2"},
    {Cluster_2X1X1,"Cluster_2X1X1"},
    {Cluster_1X2X1,"Cluster_1X2X1"},
    {Cluster_2X2,"Cluster_2X2"},
    {Cluster_2D1,"Cluster_2D1"},
    {Cluster_2D2,"Cluster_2D2"},
    {Cluster_2D3,"Cluster_2D3"},
    {Cluster_2D4,"Cluster_2D4"},
    {Cluster_1X1,"Cluster_1X1"},
    {Cluster_Big,"Cluster_Big"},
    {Cluster_MAX,"Cluster_MAX"},
};
// debug flag
bool BPrint = false;
int Count = 0;
std::vector<std::pair<cv::Rect, EClusterType>> ClusterRectsVector;
std::vector<int> DeadLineRowVector;
std::vector<int> DeadLineColVector;

//initializer_list<pair<string, Gender>> 
template<typename T>
T MaxValue(std::initializer_list<T> ValueList)
{
    static_assert(std::is_arithmetic<T>::value != 0, "Must be arithmetic type");
    return *std::max_element(ValueList.begin(), ValueList.end());
}

template<typename T>
T MinValue(std::initializer_list<T> ValueList)
{
    static_assert(std::is_arithmetic<T>::value != 0, "Must be arithmetic type");
    return *std::min_element(ValueList.begin(), ValueList.end());
}

//bool operator<(const cv::Point& PointLeft, const cv::Point& PointRight)
//{
//    return (PointLeft.x+ PointLeft.y)
//}

unsigned int PointDistance_Rect(const cv::Point& Start, const cv::Point& End)
{
    return std::max(std::abs(End.x - Start.x), std::abs(End.y - Start.y));
}

//EPixType AnalysePixType(const cv::Point& PixPoint)
//{
//    bool bIsRight = PixPoint.x & 0b1;
//    bool bIsBottom = ;
//
//    uchar PointPlace = ((PixPoint.y & 0b1 << 1) | (PixPoint.x & 0b1)) & 0b11;
//
//    EPixType PixType = EPixType::PixType_Max;
//    switch (ThisPixArray)
//    {
//    case R_GR_GB_B:PixType = static_cast<EPixType>(((~bIsRight) << 1 | (bIsBottom)) & 0b11);
//    case B_GB_GR_R:PixType = static_cast<EPixType>(((~bIsRight) << 1 | (bIsBottom)) & 0b11);
//    case GB_B_R_GR:PixType = static_cast<EPixType>(((~bIsRight) << 1 | (bIsBottom)) & 0b11);
//    case GR_R_B_GB:PixType = static_cast<EPixType>(((~bIsRight) << 1 | (bIsBottom)) & 0b11);
//    default:
//        break;
//    }
//
//    return PixType;
//}

void SetPrint(bool bPrint);

template<typename T>
void SaveMatToCsv(const cv::Mat_<T>& Mat, const std::string& FileName)
{
    std::stringstream ss;
    for (int Row = 0; Row < Mat.rows; Row++)
    {
        for (int Col = 0; Col < Mat.cols; Col++)
        {
            ss << Mat(Row, Col) << ",";
        }
        ss << "\n";
    }
    std::ofstream ofs(FileName, std::ios::trunc);
    if (ofs.is_open())
    {
        ofs << ss.str();
    }
    ofs.close();
}

cv::Mat GetChannelMat(ImgPixArray PixArray, cv::Size MatSize)
{
    cv::Mat ChannelMat(MatSize, CV_16UC1);
    for (int Row = 0; Row < ChannelMat.rows; Row++)
    {
        for (int Col = 0; Col < ChannelMat.cols; Col++)
        {
            ChannelMat.at<ushort>(Row, Col) = PixArray2PixType.at(PixArray).at((Row % 2 == 0 ? 0 : 2) + (Col % 2 == 0 ? 0 : 1));
        }
    }

    return ChannelMat;
}

EClusterType CalCulateClusterType(const cv::Mat_<ushort>& ClusterChannelMat)
{
    EClusterType ClusterType = Cluster_Big;
    for (int CurrentClusterType = 0; CurrentClusterType < ClusterKindMats.size(); CurrentClusterType++)
    {
        const cv::Mat_<ushort>& ClusterKindMat = ClusterKindMats.at(CurrentClusterType);
     
        if (ClusterKindMat.size != ClusterChannelMat.size) continue;

        cv::Size Size = ClusterKindMat.size();
        for (int Row = 0; Row < Size.height; Row++)
        {
            for (int Col = 0; Col < Size.width; Col++)
            {
                if (ClusterKindMat(Row, Col) == 0xF0)
                {
                    if (ClusterChannelMat(Row, Col) == 0)
                        goto NEXT;
                }
                else if (ClusterChannelMat(Row, Col) == 0xF0)
                {
                    if (ClusterKindMat(Row, Col) == 0)
                        goto NEXT;
                }
                else
                {
                    if (ClusterChannelMat(Row, Col) != ClusterKindMat(Row, Col))
                        goto NEXT;
                }
            }
        }
        ClusterType = static_cast<EClusterType>(CurrentClusterType);
        break;

    NEXT:
        continue;
    }

    return ClusterType;
}

ImgErrorCode GetMeanAndStd(const cv::Mat& InputMat, double& Mean, double& StdDev)
{
    if (InputMat.empty())
    {
        return Img_Empty_Mat;
    }
    cv::Mat meanMat, stdDevMat;
    cv::meanStdDev(InputMat, meanMat, stdDevMat);
    Mean = meanMat.at<double>(0, 0);
    StdDev = stdDevMat.at<double>(0, 0);

    return Img_Success;
}

ImgErrorCode SplitRawMat(const cv::Mat& SrcMat, ImgPixArray PixArray, cv::Mat& R_Mat, cv::Mat& Gr_Mat, cv::Mat& Gb_Mat, cv::Mat& B_Mat)
{
    if (SrcMat.empty())
    {
        return Img_Empty_Mat;
    }

    // 获取长宽的一半
    int Rows = SrcMat.rows >> 1;
    int Cols = SrcMat.cols >> 1;

    // 一个像素点的四个区域
    cv::Mat TopLeftCorner = cv::Mat::zeros(Rows, Cols, SrcMat.type());
    cv::Mat TopRightCorner = cv::Mat::zeros(Rows, Cols, SrcMat.type());
    cv::Mat BottomLeftCorner = cv::Mat::zeros(Rows, Cols, SrcMat.type());
    cv::Mat BottomRightCorner = cv::Mat::zeros(Rows, Cols, SrcMat.type());

    ushort TopLeftMaxValue = 0, TopLeftMinValue = -1;
    ushort TopRightMaxValue = 0, TopRightMinValue = -1;
    ushort BottomLeftMaxValue = 0, BottomLeftMinValue = -1;
    ushort BottomRightMaxValue = 0, BottomRightMinValue = -1;

    for (int Row = 0; Row < Rows; Row++)
    {
        for (int Col = 0; Col < Cols; Col++)
        {
            ushort TopLeftValue = SrcMat.at<ushort>(2 * Row, 2 * Col);
            ushort TopRightValue = SrcMat.at<ushort>(2 * Row, 2 * Col + 1);
            ushort BottomLeftValue = SrcMat.at<ushort>(2 * Row + 1, 2 * Col);
            ushort BottomRightValue = SrcMat.at<ushort>(2 * Row + 1, 2 * Col + 1);

            TopLeftCorner.at<ushort>(Row, Col) = TopLeftValue;
            TopRightCorner.at<ushort>(Row, Col) = TopRightValue;
            BottomLeftCorner.at<ushort>(Row, Col) = BottomLeftValue;
            BottomRightCorner.at<ushort>(Row, Col) = BottomRightValue;

            TopLeftMinValue = std::min(TopLeftMinValue, TopLeftValue);
            TopRightMinValue = std::min(TopRightMinValue, TopRightValue);
            BottomLeftMinValue = std::min(BottomLeftMinValue, BottomLeftValue);
            BottomRightMinValue = std::min(BottomRightMinValue, BottomRightValue);

            TopLeftMaxValue = std::max(TopLeftMaxValue, TopLeftValue);
            TopRightMaxValue = std::max(TopRightMaxValue, TopRightValue);
            BottomLeftMaxValue = std::max(BottomLeftMaxValue, BottomLeftValue);
            BottomRightMaxValue = std::max(BottomRightMaxValue, BottomRightValue);
        }
    }

    switch (PixArray)
    {
    case R_GR_GB_B:
        R_Mat = TopLeftCorner;
        Gr_Mat = TopRightCorner;
        Gb_Mat = BottomLeftCorner;
        B_Mat = BottomRightCorner;
        break;
    case B_GB_GR_R:
        B_Mat = TopLeftCorner;
        Gb_Mat = TopRightCorner;
        Gr_Mat = BottomLeftCorner;
        R_Mat = BottomRightCorner;
        break;
    case GB_B_R_GR:
        Gb_Mat = TopLeftCorner;
        B_Mat = TopRightCorner;
        R_Mat = BottomLeftCorner;
        Gr_Mat = BottomRightCorner;
        break;
    case GR_R_B_GB:
        Gr_Mat = TopLeftCorner;
        R_Mat = TopRightCorner;
        B_Mat = BottomLeftCorner;
        Gb_Mat = BottomRightCorner;
        break;
    default:
        break;
    }
    // B_GB_GR_R


    return Img_Success;
}

ImgErrorCode JoinRawMat(ImgPixArray PixArray, const cv::Mat& R_Mat, const cv::Mat& Gr_Mat, const cv::Mat& Gb_Mat, const cv::Mat& B_Mat, cv::Mat& JoinMat)
{
    if (R_Mat.empty() || Gr_Mat.empty() || Gb_Mat.empty() || B_Mat.empty())
    {
        return Img_Empty_Mat;
    }

    if (!(R_Mat.rows == Gr_Mat.rows && B_Mat.rows == Gb_Mat.rows && R_Mat.rows == B_Mat.rows) ||
        !(R_Mat.cols == Gr_Mat.cols && B_Mat.cols == Gb_Mat.cols && R_Mat.cols == B_Mat.cols)
    )
    {
        return Img_Invalid_Size;
    }

    if (!(R_Mat.type() == Gr_Mat.type() && B_Mat.type() == Gb_Mat.type() && R_Mat.type() == B_Mat.type()))
    {
        return Img_Invalid_Size;
    }

    // 获取长宽的一半
    int Rows = R_Mat.rows;
    int Cols = R_Mat.cols;
    int Type = R_Mat.type();

    JoinMat = cv::Mat::zeros(2 * Rows, 2 * Cols, Type);

    for (int Row = 0; Row < Rows; Row++)
    {
        for (int Col = 0; Col < Cols; Col++)
        {    
            // B_GB_GR_R
            ushort TopLeftValue    ;
            ushort TopRightValue   ;
            ushort BottomLeftValue ;
            ushort BottomRightValue;

            switch (PixArray)
            {
            case R_GR_GB_B:
                TopLeftValue = R_Mat.at<ushort>(Row, Col);
                TopRightValue = Gr_Mat.at<ushort>(Row, Col);
                BottomLeftValue = Gb_Mat.at<ushort>(Row, Col);
                BottomRightValue = B_Mat.at<ushort>(Row, Col);
                break;
            case B_GB_GR_R:
                TopLeftValue = B_Mat.at<ushort>(Row, Col);
                TopRightValue = Gb_Mat.at<ushort>(Row, Col);
                BottomLeftValue = Gr_Mat.at<ushort>(Row, Col);
                BottomRightValue = R_Mat.at<ushort>(Row, Col);
                break;
            case GB_B_R_GR:
                TopLeftValue = Gb_Mat.at<ushort>(Row, Col);
                TopRightValue = B_Mat.at<ushort>(Row, Col);
                BottomLeftValue = R_Mat.at<ushort>(Row, Col);
                BottomRightValue = Gr_Mat.at<ushort>(Row, Col);
                break;
            case GR_R_B_GB:
                TopLeftValue = Gr_Mat.at<ushort>(Row, Col);
                TopRightValue = R_Mat.at<ushort>(Row, Col);
                BottomLeftValue = B_Mat.at<ushort>(Row, Col);
                BottomRightValue = Gb_Mat.at<ushort>(Row, Col);
                break;
            default:
                break;
            }

            JoinMat.at<short>(2 * Row, 2 * Col) = TopLeftValue;
            JoinMat.at<short>(2 * Row, 2 * Col + 1) = TopRightValue;
            JoinMat.at<short>(2 * Row + 1, 2 * Col) = BottomLeftValue;
            JoinMat.at<short>(2 * Row + 1, 2 * Col + 1) = BottomRightValue;
        }
    }

    return Img_Success;
}

ImgErrorCode SplitRawMatWithROI(const cv::Mat& SrcMat, ImgPixArray PixArray, TMatROIS& MatROIs, TMeanROIS& MeanROIs, TStdROIS& StdROIs)
{
    if (SrcMat.empty())
    {
        return Img_Empty_Mat;
    }

    const std::array<cv::Rect, ROI_MAX> RectROIs{
        cv::Rect(cv::Point(0  , 0  ), cv::Point(159, 63 )),
        cv::Rect(cv::Point(160, 0  ), cv::Point(319, 63 )),
        cv::Rect(cv::Point(320, 0  ), cv::Point(479, 63 )),
        cv::Rect(cv::Point(0  , 64 ), cv::Point(159, 126)),
        cv::Rect(cv::Point(214, 70 ), cv::Point(264, 120)),
        cv::Rect(cv::Point(320, 64 ), cv::Point(479, 126)),
        cv::Rect(cv::Point(0  , 127), cv::Point(159, 189)),
        cv::Rect(cv::Point(160, 127), cv::Point(319, 189)),
        cv::Rect(cv::Point(320, 127), cv::Point(479, 189))
    };


    for (int ROI_Type = 0; ROI_Type < ROI_MAX; ROI_Type++)
    {
        cv::Mat R_Mat, Gr_Mat, Gb_Mat, B_Mat;
        MatROIs[ROI_Type] = SrcMat(RectROIs[ROI_Type]).clone();
        SplitRawMat(MatROIs[ROI_Type], PixArray, R_Mat, Gr_Mat, Gb_Mat, B_Mat);

        GetMeanAndStd(MatROIs[ROI_Type], MeanROIs[ROI_Type].Mean_All, StdROIs[ROI_Type].Std_All);
        GetMeanAndStd(R_Mat, MeanROIs[ROI_Type].Mean_R, StdROIs[ROI_Type].Std_R);
        GetMeanAndStd(Gr_Mat, MeanROIs[ROI_Type].Mean_Gr, StdROIs[ROI_Type].Std_Gr);
        GetMeanAndStd(Gb_Mat, MeanROIs[ROI_Type].Mean_Gb, StdROIs[ROI_Type].Std_Gb);
        GetMeanAndStd(B_Mat, MeanROIs[ROI_Type].Mean_B, StdROIs[ROI_Type].Std_B);
    }

    return Img_Success;
}

// 边缘镜像补充
ImgErrorCode GetDeadLineFillMirror(cv::Mat& InputMat, double Threshold, size_t& HorizontalCount, size_t& VerticalCount)
{
    if (InputMat.empty())
    {
        return Img_Empty_Mat;
    }
    /*
                 1:
        +------+ X
        |      | X
        | IMG  | X
        |      | X
        +------+ X
      0:YYYYYYYY
    */
    cv::Mat HorizontalMeanMat, VerticalMeanMat;
    cv::reduce(InputMat, HorizontalMeanMat, 1, cv::REDUCE_AVG, CV_64FC1);
    cv::reduce(InputMat, VerticalMeanMat, 0, cv::REDUCE_AVG, CV_64FC1);

    HorizontalCount = 0;
    int Rows = HorizontalMeanMat.rows;
    if (BPrint) DeadLineRowVector.clear();
    for (int Row = 0; Row < Rows; Row++)
    {
        double CurrentRowMean = HorizontalMeanMat.at<double>(Row, 0);
        double LastRowMean1 = HorizontalMeanMat.at<double>(std::abs(Row - 1), 0);
        double LastRowMean2 = HorizontalMeanMat.at<double>(std::abs(Row - 2), 0);
        double LastRowMean3 = HorizontalMeanMat.at<double>(std::abs(Row - 3), 0);
        double LastRowMean4 = HorizontalMeanMat.at<double>(std::abs(Row - 4), 0);
        double LastRowMean = (LastRowMean1 + LastRowMean2 + LastRowMean3 + LastRowMean4) / 4.f;

        double NextRowMean1 = HorizontalMeanMat.at<double>((Rows - 1) - std::abs((Rows - 1) - (Row + 1)), 0);
        double NextRowMean2 = HorizontalMeanMat.at<double>((Rows - 1) - std::abs((Rows - 1) - (Row + 2)), 0);
        double NextRowMean3 = HorizontalMeanMat.at<double>((Rows - 1) - std::abs((Rows - 1) - (Row + 3)), 0);
        double NextRowMean4 = HorizontalMeanMat.at<double>((Rows - 1) - std::abs((Rows - 1) - (Row + 4)), 0);
        double NextRowMean = (NextRowMean1 + NextRowMean2 + NextRowMean3 + NextRowMean4) / 4.f;

        double LastDeltaMean = std::abs(CurrentRowMean - LastRowMean);
        double NextDeltaMean = std::abs(CurrentRowMean - NextRowMean);
        if (LastDeltaMean > Threshold && NextDeltaMean > Threshold)
        {
            HorizontalCount++;
            if (BPrint)
            {
                DeadLineRowVector.emplace_back(Row);
            }
        }
    }

    VerticalCount = 0;
    int Cols = VerticalMeanMat.cols;
    if (BPrint) DeadLineColVector.clear();
    for (int Col = 0; Col < Cols; Col++)
    {
        double CurrentColMean = VerticalMeanMat.at<double>(0, Col);
        double LastColMean1 = VerticalMeanMat.at<double>(0, std::abs(Col - 1));
        double LastColMean2 = VerticalMeanMat.at<double>(0, std::abs(Col - 2));
        double LastColMean3 = VerticalMeanMat.at<double>(0, std::abs(Col - 3));
        double LastColMean4 = VerticalMeanMat.at<double>(0, std::abs(Col - 4));
        double LastColMean = (LastColMean1 + LastColMean2 + LastColMean3 + LastColMean4) / 4.f;

        double NextColMean1 = VerticalMeanMat.at<double>(0, (Cols - 1) - std::abs((Cols - 1) - (Col + 1)));
        double NextColMean2 = VerticalMeanMat.at<double>(0, (Cols - 1) - std::abs((Cols - 1) - (Col + 2)));
        double NextColMean3 = VerticalMeanMat.at<double>(0, (Cols - 1) - std::abs((Cols - 1) - (Col + 3)));
        double NextColMean4 = VerticalMeanMat.at<double>(0, (Cols - 1) - std::abs((Cols - 1) - (Col + 4)));
        double NextColMean = (NextColMean1 + NextColMean2 + NextColMean3 + NextColMean4) / 4.f;

        double LastDeltaMean = std::abs(CurrentColMean - LastColMean);
        double NextDeltaMean = std::abs(CurrentColMean - NextColMean);
        if (LastDeltaMean > Threshold && NextDeltaMean > Threshold)
        {
            VerticalCount++;
            if (BPrint)
            {
                DeadLineColVector.emplace_back(Col);
            }
        }
    }

    return Img_Success;
}

// 边缘整体mean值补充
ImgErrorCode GetDeadLineFillAllMean(cv::Mat& InputMat, double Mean, double Threshold, size_t& HorizontalCount, size_t& VerticalCount)
{
    if (InputMat.empty())
    {
        return Img_Empty_Mat;
    }
    /*
                 1:
        +------+ X
        |      | X
        | IMG  | X
        |      | X
        +------+ X
      0:YYYYYYYY
    */
    cv::Mat HorizontalMeanMat, VerticalMeanMat;
    cv::reduce(InputMat, HorizontalMeanMat, 1, cv::REDUCE_AVG, CV_64FC1);
    cv::reduce(InputMat, VerticalMeanMat, 0, cv::REDUCE_AVG, CV_64FC1);

    HorizontalCount = 0;
    int Rows = HorizontalMeanMat.rows;
    if (BPrint) DeadLineRowVector.clear();
    for (int Row = 0; Row < Rows; Row++)
    {
        double CurrentRowMean = HorizontalMeanMat.at<double>(Row, 0);
        double LastRowMean1 = Row - 1 < 0 ? Mean : HorizontalMeanMat.at<double>(Row - 1, 0);
        double LastRowMean2 = Row - 2 < 0 ? Mean : HorizontalMeanMat.at<double>(Row - 2, 0);
        double LastRowMean3 = Row - 3 < 0 ? Mean : HorizontalMeanMat.at<double>(Row - 3, 0);
        double LastRowMean4 = Row - 4 < 0 ? Mean : HorizontalMeanMat.at<double>(Row - 4, 0);
        double LastRowMean = (LastRowMean1 + LastRowMean2 + LastRowMean3 + LastRowMean4) / 4.f;

        double NextRowMean1 = Row + 1 > Rows - 1 ? Mean : HorizontalMeanMat.at<double>(Row + 1, 0);
        double NextRowMean2 = Row + 2 > Rows - 1 ? Mean : HorizontalMeanMat.at<double>(Row + 2, 0);
        double NextRowMean3 = Row + 3 > Rows - 1 ? Mean : HorizontalMeanMat.at<double>(Row + 3, 0);
        double NextRowMean4 = Row + 4 > Rows - 1 ? Mean : HorizontalMeanMat.at<double>(Row + 4, 0);
        double NextRowMean = (NextRowMean1 + NextRowMean2 + NextRowMean3 + NextRowMean4) / 4.f;

        double LastDeltaMean = std::abs(CurrentRowMean - LastRowMean);
        double NextDeltaMean = std::abs(CurrentRowMean - NextRowMean);
        if (LastDeltaMean > Threshold && NextDeltaMean > Threshold)
        {
            HorizontalCount++;
            if (BPrint)
            {
                DeadLineRowVector.emplace_back(Row);
            }
        }
    }

    VerticalCount = 0;
    int Cols = VerticalMeanMat.cols;
    if (BPrint) DeadLineColVector.clear();
    for (int Col = 0; Col < Cols; Col++)
    {
        double CurrentColMean = VerticalMeanMat.at<double>(0, Col);
        double LastColMean1 = Col - 1 < 0 ? Mean : VerticalMeanMat.at<double>(0, Col - 1);
        double LastColMean2 = Col - 2 < 0 ? Mean : VerticalMeanMat.at<double>(0, Col - 2);
        double LastColMean3 = Col - 3 < 0 ? Mean : VerticalMeanMat.at<double>(0, Col - 3);
        double LastColMean4 = Col - 4 < 0 ? Mean : VerticalMeanMat.at<double>(0, Col - 4);
        double LastColMean = (LastColMean1 + LastColMean2 + LastColMean3 + LastColMean4) / 4.f;

        double NextColMean1 = Col + 1 > Cols - 1 ? Mean : VerticalMeanMat.at<double>(0, Col + 1);
        double NextColMean2 = Col + 2 > Cols - 1 ? Mean : VerticalMeanMat.at<double>(0, Col + 2);
        double NextColMean3 = Col + 3 > Cols - 1 ? Mean : VerticalMeanMat.at<double>(0, Col + 3);
        double NextColMean4 = Col + 4 > Cols - 1 ? Mean : VerticalMeanMat.at<double>(0, Col + 4);
        double NextColMean = (NextColMean1 + NextColMean2 + NextColMean3 + NextColMean4) / 4.f;

        double LastDeltaMean = std::abs(CurrentColMean - LastColMean);
        double NextDeltaMean = std::abs(CurrentColMean - NextColMean);
        if (LastDeltaMean > Threshold && NextDeltaMean > Threshold)
        {
            VerticalCount++;
            if (BPrint)
            {
                DeadLineColVector.emplace_back(Col);
            }
        }
    }

    return Img_Success;
}

// 边缘X行/列mean值补充
ImgErrorCode GetDeadLineFillMarginMean(cv::Mat& InputMat, int MarginWidth, double Threshold, size_t& HorizontalCount, size_t& VerticalCount, std::vector<double>* TBLR = nullptr)
{
    if (InputMat.empty())
    {
        return Img_Empty_Mat;
    }
    /*
                 1:
        +------+ X
        |      | X
        | IMG  | X
        |      | X
        +------+ X
      0:YYYYYYYY
    */
    cv::Mat HorizontalMeanMat, VerticalMeanMat;
    cv::reduce(InputMat, HorizontalMeanMat, 1, cv::REDUCE_AVG, CV_64FC1);
    cv::reduce(InputMat, VerticalMeanMat, 0, cv::REDUCE_AVG, CV_64FC1);
    int Rows = HorizontalMeanMat.rows;
    int Cols = VerticalMeanMat.cols;
    MarginWidth = MarginWidth <0 || MarginWidth>Rows || MarginWidth > Cols ? 1: MarginWidth;

    double LeftMean = cv::mean(VerticalMeanMat(cv::Rect(0, 0, MarginWidth, 1)))[0];
    double RightMean = cv::mean(VerticalMeanMat(cv::Rect(Cols - MarginWidth, 0, MarginWidth, 1)))[0];
    double TopMean = cv::mean(HorizontalMeanMat(cv::Rect(0, 0, 1, MarginWidth)))[0];
    double BottomMean = cv::mean(HorizontalMeanMat(cv::Rect(0, Rows - MarginWidth, 1, MarginWidth)))[0];

    if (TBLR != nullptr)
    {
        auto& vTBLR = *TBLR;
        vTBLR.clear();
        vTBLR.push_back(TopMean);
        vTBLR.push_back(BottomMean);
        vTBLR.push_back(LeftMean);
        vTBLR.push_back(RightMean);
    }

    HorizontalCount = 0;
    if (BPrint) DeadLineRowVector.clear();
    for (int Row = 0; Row < Rows; Row++)
    {
        double CurrentRowMean = HorizontalMeanMat.at<double>(Row, 0);
        double LastRowMean1 = Row - 1 < 0 ? TopMean : HorizontalMeanMat.at<double>(Row - 1, 0);
        double LastRowMean2 = Row - 2 < 0 ? TopMean : HorizontalMeanMat.at<double>(Row - 2, 0);
        double LastRowMean3 = Row - 3 < 0 ? TopMean : HorizontalMeanMat.at<double>(Row - 3, 0);
        double LastRowMean4 = Row - 4 < 0 ? TopMean : HorizontalMeanMat.at<double>(Row - 4, 0);
        double LastRowMean = (LastRowMean1 + LastRowMean2 + LastRowMean3 + LastRowMean4) / 4.f;

        double NextRowMean1 = Row + 1 > Rows - 1 ? BottomMean : HorizontalMeanMat.at<double>(Row + 1, 0);
        double NextRowMean2 = Row + 2 > Rows - 1 ? BottomMean : HorizontalMeanMat.at<double>(Row + 2, 0);
        double NextRowMean3 = Row + 3 > Rows - 1 ? BottomMean : HorizontalMeanMat.at<double>(Row + 3, 0);
        double NextRowMean4 = Row + 4 > Rows - 1 ? BottomMean : HorizontalMeanMat.at<double>(Row + 4, 0);
        double NextRowMean = (NextRowMean1 + NextRowMean2 + NextRowMean3 + NextRowMean4) / 4.f;

        double LastDeltaMean = std::abs(CurrentRowMean - LastRowMean);
        double NextDeltaMean = std::abs(CurrentRowMean - NextRowMean);
        if (LastDeltaMean > Threshold && NextDeltaMean > Threshold)
        {
            HorizontalCount++;
            if (BPrint)
            {
                DeadLineRowVector.emplace_back(Row);
            }
        }
    }

    VerticalCount = 0;
    if (BPrint) DeadLineColVector.clear();
    for (int Col = 0; Col < Cols; Col++)
    {
        double CurrentColMean = VerticalMeanMat.at<double>(0, Col);
        double LastColMean1 = Col - 1 < 0 ? LeftMean : VerticalMeanMat.at<double>(0, Col - 1);
        double LastColMean2 = Col - 2 < 0 ? LeftMean : VerticalMeanMat.at<double>(0, Col - 2);
        double LastColMean3 = Col - 3 < 0 ? LeftMean : VerticalMeanMat.at<double>(0, Col - 3);
        double LastColMean4 = Col - 4 < 0 ? LeftMean : VerticalMeanMat.at<double>(0, Col - 4);
        double LastColMean = (LastColMean1 + LastColMean2 + LastColMean3 + LastColMean4) / 4.f;

        double NextColMean1 = Col + 1 > Cols - 1 ? RightMean : VerticalMeanMat.at<double>(0, Col + 1);
        double NextColMean2 = Col + 2 > Cols - 1 ? RightMean : VerticalMeanMat.at<double>(0, Col + 2);
        double NextColMean3 = Col + 3 > Cols - 1 ? RightMean : VerticalMeanMat.at<double>(0, Col + 3);
        double NextColMean4 = Col + 4 > Cols - 1 ? RightMean : VerticalMeanMat.at<double>(0, Col + 4);
        double NextColMean = (NextColMean1 + NextColMean2 + NextColMean3 + NextColMean4) / 4.f;

        double LastDeltaMean = std::abs(CurrentColMean - LastColMean);
        double NextDeltaMean = std::abs(CurrentColMean - NextColMean);
        if (LastDeltaMean > Threshold && NextDeltaMean > Threshold)
        {
            VerticalCount++;
            if (BPrint)
            {
                DeadLineColVector.emplace_back(Col);
            }
        }
    }

    return Img_Success;
}

ImgErrorCode GetDarkBadPoints(cv::Mat& InputMat, double Mean, cv::Mat& OutputMat, double Threshold, int& BadPointCount)
{
    if (InputMat.empty())
    {
        return Img_Empty_Mat;
    }            
    cv::Mat InputConvertMat;
    InputMat.convertTo(InputConvertMat, CV_64FC1);
    BadPointCount = 0;
    cv::Mat SubMeanMat = cv::abs(InputConvertMat - Mean);
    cv::Mat ThresholdMat;
    cv::threshold(SubMeanMat, ThresholdMat, Threshold, 1, cv::THRESH_BINARY);
    BadPointCount = static_cast<int>(cv::sum(ThresholdMat)[0]);
    ThresholdMat.convertTo(OutputMat, CV_16UC1);

    return Img_Success;
}

ImgErrorCode GetLightBadPoints(cv::Mat& InputMat, double Mean, cv::Mat* PtrBlackPointtMat, cv::Mat* PtrWhitePointtMat, int BadPointKernelSize,
    double BlackPointThreshold, size_t& BlackPointCount, double WhitePointThreshold, size_t& WhitePointCount)
{
    if (InputMat.empty())
    {
        return Img_Empty_Mat;
    }            
    
    cv::Mat InputConvertMat;
    InputMat.convertTo(InputConvertMat, CV_64FC1);
    cv::Mat BlurMat = cv::Mat::zeros(InputMat.rows, InputMat.cols, CV_64FC1);
    // 均值滤波
    cv::blur(InputConvertMat, BlurMat, cv::Size(BadPointKernelSize, BadPointKernelSize)); // 边缘镜像
    BlurMat = (BlurMat * (BadPointKernelSize * BadPointKernelSize) - InputConvertMat) / (BadPointKernelSize*BadPointKernelSize - 1); // 去除自身
    cv::Mat SubMeanMat = (InputConvertMat - BlurMat) / BlurMat;

    cv::Mat BlackThresholdMat, WhiteThresholdMat;
    cv::threshold(SubMeanMat, BlackThresholdMat, BlackPointThreshold, 1, cv::THRESH_BINARY_INV);
    BlackPointCount = static_cast<int>(cv::sum(BlackThresholdMat)[0]);
    if (PtrBlackPointtMat != nullptr)
    {
        BlackThresholdMat.convertTo(*PtrBlackPointtMat, CV_16UC1);
    }
    cv::threshold(SubMeanMat, WhiteThresholdMat, WhitePointThreshold, 1, cv::THRESH_BINARY);
    WhitePointCount = static_cast<int>(cv::sum(WhiteThresholdMat)[0]);
    if (PtrWhitePointtMat != nullptr)
    {
        WhiteThresholdMat.convertTo(*PtrWhitePointtMat, CV_16UC1);
    }

    return Img_Success;
}

ImgErrorCode GetCluster(const cv::Mat& BadPointMat, int ClusterKernelSize, ImgPixArray PixArray, std::array<size_t, Cluster_MAX>& ClusterCountArray)
{
    if (BadPointMat.empty())
    {
        return Img_Empty_Mat;
    }

    // 提取坏点坐标
    std::vector<cv::Point> BadPoints;
    cv::Mat_<ushort> BadPointMat_(BadPointMat);
    for (int Row = 0; Row < BadPointMat_.rows; Row++)
    {
        for (int Col = 0; Col < BadPointMat_.cols; Col++)
        {
            if (BadPointMat_(Row, Col) != 0)
            {
                BadPoints.push_back(cv::Point(Col, Row));
            }
        }
    }

    // 坏点的邻接矩阵
    cv::Mat PointDistanceMat(BadPoints.size(), BadPoints.size(), CV_16U);
    for (int Row = 0; Row < PointDistanceMat.rows; Row++)
    {
        cv::Point StartPoint = BadPoints[Row];
        for (int Col = 0; Col < PointDistanceMat.cols; Col++)
        {
            cv::Point EndPoint = BadPoints[Col];

            PointDistanceMat.at<uint16_t>(Row, Col) = PointDistance_Rect(StartPoint, EndPoint);
        }
    }

    // 深度遍历邻接矩阵，点点之间小于ClusterKernelSize的数组
    std::vector<bool> BadPointIsUsed(BadPoints.size(), false);
    std::function<void(int, std::vector<int>&)> CalculateBadPointIndexes;
    const int ClusterKernelRadius = ClusterKernelSize / 2;
    CalculateBadPointIndexes =
        [ClusterKernelRadius, &BadPointIsUsed, &PointDistanceMat = std::as_const(PointDistanceMat), &CalculateBadPointIndexes](int CurrentRow, std::vector<int>& BadPointIndexes /* By R_BadPoints */)
    {
        BadPointIndexes.push_back(CurrentRow);
        BadPointIsUsed[CurrentRow] = true;

        std::stack<int> s;
        s.push(CurrentRow);
        while (!s.empty())
        {
            int tempRow = s.top();

            bool bFind = false;
            for (int Col = 0; Col < PointDistanceMat.cols; Col++)
            {
                // 跳过自己
                if (Col == tempRow) continue;

                // 跳过被选择的点
                if (BadPointIsUsed[Col] == true) continue;

                if (PointDistanceMat.at<uint16_t>(tempRow, Col) <= ClusterKernelRadius)
                {
                    s.push(Col);
                    BadPointIndexes.push_back(Col);
                    BadPointIsUsed[Col] = true; // 标记被访问
                    bFind = true;
                    break;
                }
            }
            if (bFind == false)
            {
                s.pop();
            }
        }
    };

    // 获得ClusterAreas
    std::vector<cv::Rect> ClusterAreas;
    for (int Row = 0; Row < PointDistanceMat.rows; Row++)
    {
        if (BadPointIsUsed[Row] == false)
        {
            std::vector<int> BadPointIndexes;

            // 递归获得组成坏簇的坏点下表
            CalculateBadPointIndexes(Row, BadPointIndexes);

            // 结果判断
            if (BadPointIndexes.size() != 0)
            {
                std::vector<int> PointXs, PointYs;
                for (int Index = 0; Index < BadPointIndexes.size(); Index++)
                {
                    cv::Point BadPoint = BadPoints.at(BadPointIndexes[Index]);
                    PointXs.push_back(BadPoint.x);
                    PointYs.push_back(BadPoint.y);
                }
                auto PointX_MinMax = std::minmax_element(PointXs.begin(), PointXs.end());
                auto PointY_MinMax = std::minmax_element(PointYs.begin(), PointYs.end());

                cv::Rect ClusterRect;
                ClusterRect.x = *PointX_MinMax.first;
                ClusterRect.y = *PointY_MinMax.first;
                ClusterRect.width = *PointX_MinMax.second - *PointX_MinMax.first + 1;
                ClusterRect.height = *PointY_MinMax.second - *PointY_MinMax.first + 1;

                ClusterAreas.push_back(ClusterRect);
            }
        }
    }

    // 获取channelMat
    const cv::Mat ChannelMat = GetChannelMat(PixArray, cv::Size(BadPointMat.cols, BadPointMat.rows));

    // 截取坏簇的(最小)矩形,与截取出来的ChannelMat对应位的乘积,完成带有RGrGbB的坏簇矩阵
    if (BPrint)
    {
        ClusterRectsVector.clear();
    }
    ClusterCountArray.fill(0);
    for (int Index = 0; Index < ClusterAreas.size(); Index++)
    {
        cv::Rect ClusterArea = ClusterAreas[Index];
        cv::Mat ClusterPointMat, ClusterChannelMat;
        BadPointMat(ClusterArea).convertTo(ClusterPointMat, CV_64FC1);
        ChannelMat(ClusterArea).convertTo(ClusterChannelMat, CV_64FC1);
        cv::Mat ClusterMat = ClusterChannelMat.mul(ClusterPointMat);

        EClusterType ClusterType = CalCulateClusterType(ClusterMat);
        ClusterCountArray[ClusterType]++;
        if (BPrint)
        {
            ClusterRectsVector.emplace_back(std::make_pair(ClusterArea, ClusterType));
        }
    }

    return Img_Success;
}

ImgErrorCode GetFPN(const cv::Mat& InputMat, double& HorizontalFPN, double& VerticalFPN)
{
    if (InputMat.empty())
    {
        return Img_Empty_Mat;
    }

    cv::Mat HorizontalMeanMat, VerticalMeanMat;
    cv::reduce(InputMat, HorizontalMeanMat, 1, cv::REDUCE_AVG, CV_64FC1);
    cv::reduce(InputMat, VerticalMeanMat, 0, cv::REDUCE_AVG, CV_64FC1);

    double Mean;
    GetMeanAndStd(HorizontalMeanMat, Mean, HorizontalFPN);
    GetMeanAndStd(VerticalMeanMat, Mean, VerticalFPN);

    return Img_Success;
}

void RawMat2GrayMat(const cv::Mat& InputMat, cv::Mat& OutputMat)
{
    INT16U **srcData;
    INT16U **grayData;
    INT16U *pSrc = NULL;
    INT16U *pGray = NULL;

    srcData = imgMalloc(InputMat.rows, InputMat.cols, pSrc);
    grayData = imgMalloc(InputMat.rows, InputMat.cols, pGray);

    for (uint i = 0; i < InputMat.rows; ++i)
    {
        for (uint j = 0; j < InputMat.cols; ++j)
        {
            srcData[i][j] = InputMat.at<ushort>(i, j);
        }
    }

    raw2Gray(srcData, grayData, InputMat.cols, InputMat.rows, B_Gb_Gr_R);

    OutputMat = cv::Mat::zeros(InputMat.rows, InputMat.cols, InputMat.type());

    for (int row = 0; row < InputMat.rows; row++) {
        for (int col = 0; col < InputMat.cols; col++) {
            OutputMat.at<ushort>(row, col) = grayData[row][col];
        }
    }

    free((void*)srcData);
    free((void*)grayData);

    free(pSrc);
    free(pGray);
}

ImgErrorCode GetBlemish(const cv::Mat& InputMat, int &numOfBoundery)
{
    if (InputMat.empty())
    {
        return Img_Empty_Mat;
    }
    
    cv::Mat GraySrcMat;
    RawMat2GrayMat(InputMat, GraySrcMat);

    uint matRows = GraySrcMat.rows;
    uint matCols = GraySrcMat.cols;

    INT16U WIDTH = matCols;
    INT16U HEIGHT = matRows;

    //INT16U **grayData;
    INT16U *pGray = NULL;
    stDumpFFT stDumpFFT;
    INT16U thValue;

    INT16U *pSrc = NULL;
    INT16U *poutGray = NULL;

    INT16U **srcData = imgMalloc(GraySrcMat.rows, GraySrcMat.cols, pSrc);
    INT16U **grayData = imgMalloc(GraySrcMat.rows, GraySrcMat.cols, poutGray);

    for (uint i = 0; i < GraySrcMat.rows; ++i)
    {
        for (uint j = 0; j < GraySrcMat.cols; ++j)
        {
            INT16U temp = GraySrcMat.at<ushort>(i, j);
            //std::cout << temp << std::endl;
            srcData[i][j] = temp;
        }
    }

    INT16U width = matCols;
    INT16U height = matRows;

    gaussFilter(matRows, matCols, srcData, grayData);

    stDumpFFT.blDump = FALSE;
    stDumpFFT.dumpDATA = DUMP_FFT_DATA;
    bandFilter3(grayData, HEIGHT, WIDTH, (FP32)0.009, (FP32)0.09, &stDumpFFT);//0.004 0.04

    {
        INT16U h, w;
        INT64U sumGrads, sumGrayGrads;
        INT64U grands;
        INT32S xGrads, yGrads;

        sumGrads = 0;
        sumGrayGrads = 0;
        for (h = 2; h < height - 2; h++)
        {
            for (w = 2; w < width - 2; w++)
            {
                xGrads = grayData[h - 1][w] - grayData[h + 1][w];
                yGrads = grayData[h][w - 1] - grayData[h][w + 1];
                grands = MAXI(ABS(xGrads), ABS(yGrads));
                sumGrads += grands;
                sumGrayGrads += (grands * grayData[h][w]);
            }
        }

        if (sumGrayGrads == 0)
        {
            thValue = 0;
        }
        else if (sumGrads != 0)
        {
            thValue = ((INT16U)(sumGrayGrads / sumGrads));
        }
        else
        {
            thValue = 512;
            time_t currentTime = time(0);
            tm ctm;
            localtime_s(&ctm, &currentTime);
            char stime[256];
            memset(stime, 0, 256);
            std::strftime(stime, 256, "%Y%m%d%H%M%S", &ctm);
            std::string failRawName = "FailImage_THValue";
            failRawName = failRawName + __FUNCDNAME__ + stime + ".raw";
            std::ofstream ofs(failRawName, std::ios::trunc | std::ios::binary);
            ofs.write((char*)InputMat.data, InputMat.rows * InputMat.cols * 2);
            ofs.close();
        }
    }
    // thValue = getThValue(grayData, HEIGHT, WIDTH);

    getSobelAndSupressNonMax(grayData, HEIGHT, WIDTH);

    supressIsolatePoint(grayData, HEIGHT, WIDTH, thValue);

    numOfBoundery = getBoundery(grayData, HEIGHT, WIDTH, thValue, 1);

    free(pGray);
    free(pSrc);
    free(poutGray);

    free((void*)srcData);
    free((void*)grayData);

    return Img_Success;
}

/////////////////////////////////////////////////////////// Test Image /////////////////////////////////////////////////////////////
//ELogImageType ThisLogImageType = ELogImageType::Log_ImageNone;
//ELogPrintType ThisLogPrintType = ELogPrintType::Log_PrintNone;
//ELogOutputType ThisLogOutputType = ELogOutputType::Log_OutputNone;

ImgErrorCode DarkImageWithDPCOnTest(unsigned short* RawImage, unsigned int ImageHeight, unsigned int ImageWidth, ImgCondition_Dark_DPCOn Condition, ImgResult_Dark_DPCOn* Res)
{
    if (Res == nullptr)
    {
        return Img_Init_Fail;
    }

    const uint TestImageHeight = ImageHeight;
    const uint TestImageWidth = ImageWidth;
    const int TestImageType = CV_16UC1;

    cv::Mat SrcMat(TestImageHeight, TestImageWidth, TestImageType, RawImage);

    // SplitRawMat Variable
//    cv::Mat R_Mat, Gr_Mat, Gb_Mat, B_Mat;
//    MaxMin Src_MaxMinMatValue;
//    MaxMin R_MaxMinMatValue, Gr_MaxMinMatValue, Gb_MaxMinMatValue, B_MaxMinMatValue;
//#pragma region SplitRawMat
//    SplitRawMat(
//        SrcMat, R_Mat, Gr_Mat, Gb_Mat, B_Mat, 
//        Src_MaxMinMatValue, R_MaxMinMatValue, Gr_MaxMinMatValue, Gb_MaxMinMatValue, B_MaxMinMatValue
//    );
//#pragma endregion

    double Src_Mean;
    double Src_StdDev;
    GetMeanAndStd(SrcMat, Src_Mean, Src_StdDev);
    Res->Mean = Src_Mean;
    Res->Std = Src_StdDev;
	
    const double DeadLineThreshold = Condition.DeadLineThreshold;
    size_t HorizontalDeadLineCount;
    size_t VerticalDeadLineCount;
    GetDeadLineFillMirror(SrcMat, DeadLineThreshold, HorizontalDeadLineCount, VerticalDeadLineCount);
    Res->Horizontal_DeadLine_Length = HorizontalDeadLineCount;
    Res->Vertical_DeadLine_Length = VerticalDeadLineCount;

    const double BadPointThreshold = Condition.BadPointThreshold;
    cv::Mat BadPointMat;
    int BadPointCount;
    GetDarkBadPoints(SrcMat, Src_Mean, BadPointMat, BadPointThreshold, BadPointCount);
    Res->BadPixelCount = BadPointCount;
	
    std::array<size_t, Cluster_MAX> ClusterCountArray;
    GetCluster(BadPointMat, Condition.ClusterCountKernelSize, Condition.ThisPixArray, ClusterCountArray);
    Res->ClusterCount = std::accumulate(ClusterCountArray.begin(), ClusterCountArray.end(), size_t(0));
    for (int Index = 0; Index < Cluster_MAX; Index++)
    {
        Res->EachClustersSize[Index] = ClusterCountArray.at(Index);
    }

    return Img_Success;
}

ImgErrorCode DarkImageWithDPCOffTest(unsigned short* RawImage, unsigned int ImageHeight, unsigned int ImageWidth, ImgCondition_Dark_DPCOff Condition, ImgResult_Dark_DPCOff * Res)
{
    if (Res == nullptr)
    {
        return Img_Init_Fail;
    }

    const uint TestImageHeight = ImageHeight;
    const uint TestImageWidth = ImageWidth;
    const int TestImageType = CV_16UC1;

    cv::Mat SrcMat(TestImageHeight, TestImageWidth, TestImageType, RawImage);

    double Src_Mean;
    double Src_StdDev;
    GetMeanAndStd(SrcMat, Src_Mean, Src_StdDev);
    Res->Mean = Src_Mean;
    Res->Std = Src_StdDev;

    const double BadPointThreshold1 = Condition.BadPointThreshold1_Min;
    const double BadPointThreshold2 = Condition.BadPointThreshold2_Min;
    cv::Mat BadPointMat1, BadPointMat2;
    int BadPointCount1, BadPointCount2;
    GetDarkBadPoints(SrcMat, Src_Mean, BadPointMat1, BadPointThreshold1, BadPointCount1);
    GetDarkBadPoints(SrcMat, Src_Mean, BadPointMat2, BadPointThreshold2, BadPointCount2);
    Res->BadPixel1_Count = BadPointCount1 - BadPointCount2;
    Res->BadPixel2_Count = BadPointCount2;

    return Img_Success;
}

ImgErrorCode DarkTmpNoise(unsigned short* RawImage, unsigned int ImageHeight, unsigned int ImageWidth, ImgCondition_Dark_TmpNoise Condition, ImgResult_Dark_TmpNoise * Res)
{
    if (Res == nullptr)
    {
        return Img_Init_Fail;
    }

    const uint TestImageHeight = ImageHeight;
    const uint TestImageWidth = ImageWidth;
    const int TestImageType = CV_16UC1;
	const cv::Rect ROIRect(Condition.LeftTop_X, Condition.LeftTop_Y, Condition.ROI_Width, Condition.ROI_Height);
    //cv::Mat SrcMat(TestImageHeight, TestImageWidth, TestImageType, RawImage);

	cv::Mat DarkFrame1, DarkFrame2, DarkFrame3,DarkFrame4,DarkFrame5,DarkFrame6,DarkFrame7,DarkFrame8;
	cv::Mat DarkDiffFrame1, DarkDiffFrame2, DarkDiffFrame3,DarkDiffFrame4,DarkDiffFrame5,DarkDiffFrame6,DarkDiffFrame7,DarkDiffFrame8;
	cv::Mat DarkMeanFrame;
	//原始8张图的数据
	DarkFrame1 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage)(ROIRect);
    DarkFrame2 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage + TestImageWidth * TestImageHeight)(ROIRect);
    DarkFrame3 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage + TestImageWidth * TestImageHeight*2)(ROIRect);
	DarkFrame4 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage + TestImageWidth * TestImageHeight*3)(ROIRect);
	DarkFrame5 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage + TestImageWidth * TestImageHeight*4)(ROIRect);
	DarkFrame6 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage + TestImageWidth * TestImageHeight*5)(ROIRect);
	DarkFrame7 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage + TestImageWidth * TestImageHeight*6)(ROIRect);
	DarkFrame8 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage + TestImageWidth * TestImageHeight*7)(ROIRect);
	//8张图求平均
	DarkMeanFrame=(DarkFrame1+DarkFrame2+DarkFrame3+DarkFrame4+DarkFrame5+DarkFrame6+DarkFrame7+DarkFrame8)/8;
	//8张图和平均图做差
	DarkDiffFrame1=(DarkFrame1-DarkMeanFrame)+512;
	DarkDiffFrame2=(DarkFrame2-DarkMeanFrame)+512;
	DarkDiffFrame3=(DarkFrame3-DarkMeanFrame)+512;
	DarkDiffFrame4=(DarkFrame4-DarkMeanFrame)+512;
	DarkDiffFrame5=(DarkFrame5-DarkMeanFrame)+512;
	DarkDiffFrame6=(DarkFrame6-DarkMeanFrame)+512;
	DarkDiffFrame7=(DarkFrame7-DarkMeanFrame)+512;
	DarkDiffFrame8=(DarkFrame8-DarkMeanFrame)+512;
	//算出8张Diff图的mean和std
    
	double Diff_Mean [8];
	double Diff_StdDev [8];
	//double Diff_Mean1,Diff_Mean2,Diff_Mean3,Diff_Mean4,Diff_Mean5,Diff_Mean6,Diff_Mean7,Diff_Mean8;
	//double Diff_StdDev1,Diff_StdDev2,Diff_StdDev3,Diff_StdDev4,Diff_StdDev5,Diff_StdDev6,Diff_StdDev7,Diff_StdDev8;
    GetMeanAndStd(DarkDiffFrame1, Diff_Mean[0], Diff_StdDev[0]);
	GetMeanAndStd(DarkDiffFrame2, Diff_Mean[1], Diff_StdDev[1]);
	GetMeanAndStd(DarkDiffFrame3, Diff_Mean[2], Diff_StdDev[2]);
	GetMeanAndStd(DarkDiffFrame4, Diff_Mean[3], Diff_StdDev[3]);
	GetMeanAndStd(DarkDiffFrame5, Diff_Mean[4], Diff_StdDev[4]);
	GetMeanAndStd(DarkDiffFrame6, Diff_Mean[5], Diff_StdDev[5]);
	GetMeanAndStd(DarkDiffFrame7, Diff_Mean[6], Diff_StdDev[6]);
	GetMeanAndStd(DarkDiffFrame8, Diff_Mean[7], Diff_StdDev[7]);
	//TmpNoise
	double Sum_Sqrt_StdDev=0;
	double Sqrt_StdDev [8];
	for (int i = 0; i < Condition.FrameCount; i++)
	{
		Sqrt_StdDev[i]=Diff_StdDev[i]*Diff_StdDev[i];
	}
	for (int i = 0; i < Condition.FrameCount; i++)
	{
		Sum_Sqrt_StdDev+=Sqrt_StdDev[i];
	}

	Res->TmpNoise=sqrt((8/7)*(Sum_Sqrt_StdDev/8))/Condition.Gain_x16;//Condition.Gain_x16代表16


    return Img_Success;
}

ImgErrorCode DarkFPNTest(unsigned short* RawImage, unsigned int ImageHeight, unsigned int ImageWidth, ImgCondition_DarkFPN Condition, ImgResult_DarkFPN* Res)
{
    if (Res == nullptr)
    {
        return Img_Init_Fail;
    }

    const uint TestImageHeight = ImageHeight;
    const uint TestImageWidth = ImageWidth;
    const int TestImageType = CV_16UC1;
    const cv::Rect ROIRect(Condition.LeftTop_X, Condition.LeftTop_Y, Condition.ROI_Width, Condition.ROI_Height);

    cv::Mat FPNSrcMat = cv::Mat::zeros(Condition.FrameCount, 1, CV_64FC1);

    std::string fileName = "Light_BadPixel_" + std::to_string(Count) + ".txt";
    std::ofstream ofs(fileName, std::ios::trunc);
    
    

    double hfpn = 0.0, vfpn = 0.0;
    for (int Index = 0; Index < Condition.FrameCount; Index++)
    {
        double hfpn_t, vfpn_t;
        cv::Mat SrcMat;
        cv::Mat(TestImageHeight, TestImageWidth, TestImageType, RawImage + Index * TestImageHeight * TestImageWidth)(ROIRect).copyTo(SrcMat);
        cv::Mat temp = SrcMat.clone();
        FPNSrcMat.at<double>(Index, 0) = cv::mean(SrcMat)[0];
        ofs << "%d:" << Index << cv::mean(SrcMat)[0] << "\n";
        GetFPN(temp, hfpn_t, vfpn_t);
        hfpn += hfpn_t;
        vfpn += vfpn_t;
    }

    cv::Mat TempMat, FPNMat;
    cv::meanStdDev(FPNSrcMat, TempMat, FPNMat);

    ofs.close();

    Res->AFPN = FPNMat.at<double>(0, 0);
    Res->HFPN = hfpn / Condition.FrameCount;
    Res->VFPN = vfpn / Condition.FrameCount;

    return ImgErrorCode::Img_Success;
}

ImgErrorCode TestLightImage(unsigned short* RawImage, unsigned int ImageHeight, unsigned int ImageWidth, ImgCondition_Light Condition, ImgResult_Light * Res)
{
    if (Res == nullptr)
    {
        return Img_Init_Fail;
    }

	SetPrint(BPrint);

    if (BPrint == true)
    {
        Count++;
    }

    const uint TestImageHeight = ImageHeight;
    const uint TestImageWidth = ImageWidth;
    const int TestImageType = CV_16UC1;

    cv::Mat SrcMat(TestImageHeight, TestImageWidth, TestImageType, RawImage);

    TMatROIS  ROI_Mats;
    TMeanROIS ROI_Means;
    TStdROIS  ROI_Stds;
#pragma region ROI
    SplitRawMatWithROI(SrcMat, Condition.ThisPixArray, ROI_Mats, ROI_Means, ROI_Stds);
    // ROI_5 data
    double CenterMean = ROI_Means[ROI_5].Mean_All;
    double CenterRB = ROI_Means[ROI_5].Mean_R / ROI_Means[ROI_5].Mean_B;
    double CenterGrGb = ROI_Means[ROI_5].Mean_Gr / ROI_Means[ROI_5].Mean_Gb;
    // Luminance/Color Uniformity
    Res->CUBalances_RB_Max = 0.f;
    Res->CUBalances_GrGb_Max = 0.f;
    for (int ROI_Type = 0; ROI_Type < ROI_MAX; ROI_Type++)
    {
        Res->LUShadings[ROI_Type] = ROI_Means[ROI_Type].Mean_All / CenterMean;

        double Corners_RB = (ROI_Means[ROI_Type].Mean_R / ROI_Means[ROI_Type].Mean_B);
        double Corners_GrGb = (ROI_Means[ROI_Type].Mean_Gr / ROI_Means[ROI_Type].Mean_Gb);
        
        Res->CUBalances_RB[ROI_Type] = Corners_RB / CenterRB;
        Res->CUBalances_GrGb[ROI_Type] = Corners_GrGb / CenterGrGb;
        Res->CUBalances_RB_Max = std::max(Res->CUBalances_RB_Max, Corners_RB);
        Res->CUBalances_GrGb_Max = std::max(Res->CUBalances_GrGb_Max, Corners_GrGb);
    }
    Res->ROI5_R_B = ROI_Means[ROI_5].Mean_R / ROI_Means[ROI_5].Mean_B;
    Res->ROI5_Gr_Gb = ROI_Means[ROI_5].Mean_Gr / ROI_Means[ROI_5].Mean_Gb;
    Res->ROI5_R_Gr = ROI_Means[ROI_5].Mean_R / ROI_Means[ROI_5].Mean_Gr;
    Res->ROI5_B_Gb = ROI_Means[ROI_5].Mean_B / ROI_Means[ROI_5].Mean_Gb;

    Res->RG_ROI19 = (ROI_Means[ROI_1].Mean_R / ROI_Means[ROI_1].Mean_Gr) / (ROI_Means[ROI_9].Mean_R / ROI_Means[ROI_9].Mean_Gr);
    Res->RG_ROI28 = (ROI_Means[ROI_2].Mean_R / ROI_Means[ROI_2].Mean_Gr) / (ROI_Means[ROI_8].Mean_R / ROI_Means[ROI_8].Mean_Gr);
    Res->RG_ROI37 = (ROI_Means[ROI_3].Mean_R / ROI_Means[ROI_3].Mean_Gr) / (ROI_Means[ROI_7].Mean_R / ROI_Means[ROI_7].Mean_Gr);
    Res->RG_ROI46 = (ROI_Means[ROI_4].Mean_R / ROI_Means[ROI_4].Mean_Gr) / (ROI_Means[ROI_6].Mean_R / ROI_Means[ROI_6].Mean_Gr);

    Res->BG_ROI19 = (ROI_Means[ROI_1].Mean_B / ROI_Means[ROI_1].Mean_Gb) / (ROI_Means[ROI_9].Mean_B / ROI_Means[ROI_9].Mean_Gb);
    Res->BG_ROI28 = (ROI_Means[ROI_2].Mean_B / ROI_Means[ROI_2].Mean_Gb) / (ROI_Means[ROI_8].Mean_B / ROI_Means[ROI_8].Mean_Gb);
    Res->BG_ROI37 = (ROI_Means[ROI_3].Mean_B / ROI_Means[ROI_3].Mean_Gb) / (ROI_Means[ROI_7].Mean_B / ROI_Means[ROI_7].Mean_Gb);
    Res->BG_ROI46 = (ROI_Means[ROI_4].Mean_B / ROI_Means[ROI_4].Mean_Gb) / (ROI_Means[ROI_6].Mean_B / ROI_Means[ROI_6].Mean_Gb);

#pragma endregion
    
    cv::Mat R_Mat, Gr_Mat, Gb_Mat, B_Mat;
#pragma region MeanStd

    //SplitRawMat(SrcMat, Condition.ThisPixArray, R_Mat, Gr_Mat, Gb_Mat, B_Mat);

    double R_Mean, Gr_Mean, Gb_Mean, B_Mean, Src_Mean;
    double R_StdDev, Gr_StdDev, Gb_StdDev, B_StdDev, Src_StdDev;
    GetMeanAndStd(ROI_Mats[ROI_5], Src_Mean, Src_StdDev);
    SplitRawMat(ROI_Mats[ROI_5], Condition.ThisPixArray, R_Mat, Gr_Mat, Gb_Mat, B_Mat);
    //GetMeanAndStd(SrcMat, Src_Mean, Src_StdDev);
    GetMeanAndStd(R_Mat, R_Mean, R_StdDev);
    GetMeanAndStd(Gr_Mat, Gr_Mean, Gr_StdDev);
    GetMeanAndStd(Gb_Mat, Gb_Mean, Gb_StdDev);
    GetMeanAndStd(B_Mat, B_Mean, B_StdDev);

    Res->Mean = Src_Mean; //ROI_Means[ROI_5].Mean_All;
    Res->Mean_R = R_Mean; //ROI_Means[ROI_5].Mean_R;
    Res->Mean_Gr = Gr_Mean; //ROI_Means[ROI_5].Mean_Gr;
    Res->Mean_Gb = Gb_Mean; //ROI_Means[ROI_5].Mean_Gb;
    Res->Mean_B = B_Mean; //ROI_Means[ROI_5].Mean_B;
    Res->Std = Src_StdDev;
    Res->Std_R = R_StdDev;
    Res->Std_Gr = Gr_StdDev;
    Res->Std_Gb = Gb_StdDev;
    Res->Std_B = B_StdDev;
    // Ratio
    Res->R_B = R_Mean / B_Mean;
    Res->Gr_Gb = Gr_Mean / Gb_Mean;
    Res->R_Gr = R_Mean / Gr_Mean;
    Res->B_Gb = B_Mean / Gb_Mean;
#pragma endregion

#pragma region GetDeadLine
    const double DeadLineThreshold = Condition.DeadLineThreshold;
    if (BPrint == false)
    {
/*        GetDeadLineFillMirror(R_Mat, DeadLineThreshold, Res->R_HorizontalDeadLineCount, Res->R_VerticalDeadLineCount);
        GetDeadLineFillMirror(Gr_Mat, DeadLineThreshold, Res->Gr_HorizontalDeadLineCount, Res->Gr_VerticalDeadLineCount);
        GetDeadLineFillMirror(Gb_Mat, DeadLineThreshold, Res->Gb_HorizontalDeadLineCount, Res->Gb_VerticalDeadLineCount);
        GetDeadLineFillMirror(B_Mat, DeadLineThreshold, Res->B_HorizontalDeadLineCount, Res->B_VerticalDeadLineCount);   */   
       /* GetDeadLineFillAllMean(R_Mat,  R_Mean,  DeadLineThreshold, Res->R_HorizontalDeadLineCount, Res->R_VerticalDeadLineCount);
        GetDeadLineFillAllMean(Gr_Mat, Gr_Mean, DeadLineThreshold, Res->Gr_HorizontalDeadLineCount, Res->Gr_VerticalDeadLineCount);
        GetDeadLineFillAllMean(Gb_Mat, Gb_Mean, DeadLineThreshold, Res->Gb_HorizontalDeadLineCount, Res->Gb_VerticalDeadLineCount);
        GetDeadLineFillAllMean(B_Mat,  B_Mean,  DeadLineThreshold, Res->B_HorizontalDeadLineCount, Res->B_VerticalDeadLineCount);*/
        GetDeadLineFillMarginMean(R_Mat,  10,  DeadLineThreshold, Res->R_HorizontalDeadLineCount, Res->R_VerticalDeadLineCount);
        GetDeadLineFillMarginMean(Gr_Mat, 10, DeadLineThreshold, Res->Gr_HorizontalDeadLineCount, Res->Gr_VerticalDeadLineCount);
        GetDeadLineFillMarginMean(Gb_Mat, 10, DeadLineThreshold, Res->Gb_HorizontalDeadLineCount, Res->Gb_VerticalDeadLineCount);
        GetDeadLineFillMarginMean(B_Mat,  10,  DeadLineThreshold, Res->B_HorizontalDeadLineCount, Res->B_VerticalDeadLineCount);
    }
    else
    {
        auto ConvertFoo = [](EPixType type, int rc, int line)->int
        {
            int l = line;
            if (rc == 0) // horizontal
            {
                switch (type)
                {
                case EPixType::Gb:
                case EPixType::B: l = l * 2; break;
                case EPixType::R:
                case EPixType::Gr:l = l * 2 + 1; break;
                default:
                    break;
                }
            }
            else if (rc == 1) // vertical
            {
                switch (type)
                {
                case EPixType::B: 
                case EPixType::Gr: l = l * 2; break;
                case EPixType::Gb:
                case EPixType::R:l = l * 2 + 1; break;
                default:
                    break;
                }
            }
            return l;
        };
        auto DebugFoo = [&](std::ostream& s, const std::string& typeString, const EPixType type)
        {        
            for (auto DeadRow : DeadLineRowVector)
            {
                s << typeString << " DeadLine Row=" << ConvertFoo(type, 0, DeadRow) << "\n";
            }
            for (auto DeadCol : DeadLineColVector)
            {
                s << typeString << " DeadLine Col=" << ConvertFoo(type, 1, DeadCol) << "\n";
            }
        };
        std::string fileName = "Light_Dealline_" + std::to_string(Count) + ".txt";
        std::ofstream ofs(fileName, std::ios::trunc);
        //{
        //    GetDeadLineFillMirror(R_Mat, DeadLineThreshold, Res->R_HorizontalDeadLineCount, Res->R_VerticalDeadLineCount);
        //    DebugFoo(ofs, "R", EPixType::R);
        //    GetDeadLineFillMirror(Gr_Mat, DeadLineThreshold, Res->Gr_HorizontalDeadLineCount, Res->Gr_VerticalDeadLineCount);
        //    DebugFoo(ofs, "Gr", EPixType::Gr);
        //    GetDeadLineFillMirror(Gb_Mat, DeadLineThreshold, Res->Gb_HorizontalDeadLineCount, Res->Gb_VerticalDeadLineCount);
        //    DebugFoo(ofs, "Gb", EPixType::Gb);
        //    GetDeadLineFillMirror(B_Mat, DeadLineThreshold, Res->B_HorizontalDeadLineCount, Res->B_VerticalDeadLineCount);
        //    DebugFoo(ofs, "B", EPixType::B);
        //}
        //{
        //    GetDeadLineFillAllMean(R_Mat, R_Mean, DeadLineThreshold, Res->R_HorizontalDeadLineCount, Res->R_VerticalDeadLineCount);
        //    DebugFoo(ofs, "R", EPixType::R);
        //    GetDeadLineFillAllMean(Gr_Mat, Gr_Mean, DeadLineThreshold, Res->Gr_HorizontalDeadLineCount, Res->Gr_VerticalDeadLineCount);
        //    DebugFoo(ofs, "Gr", EPixType::Gr);
        //    GetDeadLineFillAllMean(Gb_Mat, Gb_Mean, DeadLineThreshold, Res->Gb_HorizontalDeadLineCount, Res->Gb_VerticalDeadLineCount);
        //    DebugFoo(ofs, "Gb", EPixType::Gb);
        //    GetDeadLineFillAllMean(B_Mat, B_Mean, DeadLineThreshold, Res->B_HorizontalDeadLineCount, Res->B_VerticalDeadLineCount);
        //    DebugFoo(ofs, "B", EPixType::B);
        //}
        {
            std::vector<double> TBLR;
            int MarginWidth = 10;
            ofs << "Margin Width = " << MarginWidth << std::endl;
            GetDeadLineFillMarginMean(R_Mat, 10, DeadLineThreshold, Res->R_HorizontalDeadLineCount, Res->R_VerticalDeadLineCount, &TBLR);
            ofs << "R :Top Mean = " << TBLR[0] << ", Bottom Mean = " << TBLR[1] << ", Left Mean = " << TBLR[2] << ", Right Mean = " << TBLR[3] << std::endl;
            DebugFoo(ofs, "R", EPixType::R);
            GetDeadLineFillMarginMean(Gr_Mat, 10, DeadLineThreshold, Res->Gr_HorizontalDeadLineCount, Res->Gr_VerticalDeadLineCount, &TBLR);
            ofs << "Gr :Top Mean = " << TBLR[0] << ", Bottom Mean = " << TBLR[1] << ", Left Mean = " << TBLR[2] << ", Right Mean = " << TBLR[3] << std::endl;
            DebugFoo(ofs, "Gr", EPixType::Gr);
            GetDeadLineFillMarginMean(Gb_Mat, 10, DeadLineThreshold, Res->Gb_HorizontalDeadLineCount, Res->Gb_VerticalDeadLineCount, &TBLR);
            ofs << "Gb :Top Mean = " << TBLR[0] << ", Bottom Mean = " << TBLR[1] << ", Left Mean = " << TBLR[2] << ", Right Mean = " << TBLR[3] << std::endl;
            DebugFoo(ofs, "Gb", EPixType::Gb);
            GetDeadLineFillMarginMean(B_Mat, 10, DeadLineThreshold, Res->B_HorizontalDeadLineCount, Res->B_VerticalDeadLineCount, &TBLR);
            ofs << "B :Top Mean = " << TBLR[0] << ", Bottom Mean = " << TBLR[1] << ", Left Mean = " << TBLR[2] << ", Right Mean = " << TBLR[3] << std::endl;
            DebugFoo(ofs, "B", EPixType::B);
        }
        ofs.close();
    }
#pragma endregion

    const int BadPointKernelSize = Condition.BadPointKernelSize;
#pragma region GetBadPoints
    if (BadPointKernelSize % 2 != 1) return Img_Init_Fail;
    const double BlackPointThreshold = Condition.BlackPointThreshold;
    const double WhitePointThreshold = Condition.WhitePointThreshold;
    if (BPrint == false)
    {
        GetLightBadPoints(R_Mat, R_Mean, nullptr, nullptr, BadPointKernelSize, BlackPointThreshold, Res->R_BlackPixelCount, WhitePointThreshold, Res->R_WhitePixelCount);
        GetLightBadPoints(Gr_Mat, Gr_Mean, nullptr, nullptr, BadPointKernelSize, BlackPointThreshold, Res->Gr_BlackPixelCount, WhitePointThreshold, Res->Gr_WhitePixelCount);
        GetLightBadPoints(Gb_Mat, Gb_Mean, nullptr, nullptr, BadPointKernelSize, BlackPointThreshold, Res->Gb_BlackPixelCount, WhitePointThreshold, Res->Gb_WhitePixelCount);
        GetLightBadPoints(B_Mat, B_Mean, nullptr, nullptr, BadPointKernelSize, BlackPointThreshold, Res->B_BlackPixelCount, WhitePointThreshold, Res->B_WhitePixelCount);
    }
    else
    {
        cv::Mat R_BlackPointMat, R_WhitePointMat;
        cv::Mat Gr_BlackPointMat, Gr_WhitePointMat;
        cv::Mat Gb_BlackPointMat, Gb_WhitePointMat;
        cv::Mat B_BlackPointMat, B_WhitePointMat;
        GetLightBadPoints(R_Mat, R_Mean, &R_BlackPointMat, &R_WhitePointMat, BadPointKernelSize, BlackPointThreshold, Res->R_BlackPixelCount, WhitePointThreshold, Res->R_WhitePixelCount);
        GetLightBadPoints(Gr_Mat, Gr_Mean, &Gr_BlackPointMat, &Gr_WhitePointMat, BadPointKernelSize, BlackPointThreshold, Res->Gr_BlackPixelCount, WhitePointThreshold, Res->Gr_WhitePixelCount);
        GetLightBadPoints(Gb_Mat, Gb_Mean, &Gb_BlackPointMat, &Gb_WhitePointMat, BadPointKernelSize, BlackPointThreshold, Res->Gb_BlackPixelCount, WhitePointThreshold, Res->Gb_WhitePixelCount);
        GetLightBadPoints(B_Mat, B_Mean, &B_BlackPointMat, &B_WhitePointMat, BadPointKernelSize, BlackPointThreshold, Res->B_BlackPixelCount, WhitePointThreshold, Res->B_WhitePixelCount);

        cv::Mat BlackPointMat, WhitePointMat;
        JoinRawMat(Condition.ThisPixArray, R_BlackPointMat, Gr_BlackPointMat, Gb_BlackPointMat, B_BlackPointMat, BlackPointMat);
        JoinRawMat(Condition.ThisPixArray, R_WhitePointMat, Gr_WhitePointMat, Gb_WhitePointMat, B_WhitePointMat, WhitePointMat);
        cv::Mat BadMat = BlackPointMat + WhitePointMat;
        std::string fileName = "Light_BadPixel_" + std::to_string(Count) + ".txt";
        std::ofstream ofs(fileName, std::ios::trunc);
        for (int row = 0; row < BadMat.rows; ++row)
        {
            for (int col = 0; col < BadMat.cols; ++col)
            {
                if (BadMat.at<ushort>(row, col) != 0)
                {
                    ofs << "Row=" << row << ", Col=" << col << "\n";
                }
            }
        }
        ofs.close();
    }
#pragma endregion

#pragma region GetCluster
    // 获得26%坏点矩阵
    cv::Mat R_BlackPointMat, R_WhitePointMat;
    cv::Mat Gr_BlackPointMat, Gr_WhitePointMat;
    cv::Mat Gb_BlackPointMat, Gb_WhitePointMat;
    cv::Mat B_BlackPointMat, B_WhitePointMat;
    const double BlackClusterThreshold = Condition.BlackClusterThreshold;
    const double WhiteClusterThreshold = Condition.WhiteClusterThreshold;
    size_t TempCount;
    GetLightBadPoints(R_Mat, R_Mean, &R_BlackPointMat, &R_WhitePointMat, BadPointKernelSize, BlackClusterThreshold, TempCount, WhiteClusterThreshold, TempCount);
    GetLightBadPoints(Gr_Mat, Gr_Mean, &Gr_BlackPointMat, &Gr_WhitePointMat, BadPointKernelSize, BlackClusterThreshold, TempCount, WhiteClusterThreshold, TempCount);
    GetLightBadPoints(Gb_Mat, Gb_Mean, &Gb_BlackPointMat, &Gb_WhitePointMat, BadPointKernelSize, BlackClusterThreshold, TempCount, WhiteClusterThreshold, TempCount);
    GetLightBadPoints(B_Mat, B_Mean, &B_BlackPointMat, &B_WhitePointMat, BadPointKernelSize, BlackClusterThreshold, TempCount, WhiteClusterThreshold, TempCount);
    // 组合分channel的矩阵
    cv::Mat BlackPointMat, WhitePointMat;
    JoinRawMat(Condition.ThisPixArray, R_BlackPointMat, Gr_BlackPointMat, Gb_BlackPointMat, B_BlackPointMat, BlackPointMat);
    JoinRawMat(Condition.ThisPixArray, R_WhitePointMat, Gr_WhitePointMat, Gb_WhitePointMat, B_WhitePointMat, WhitePointMat);
    // 如果黑白坏点要一起计算的话,可以通过cv::bitwise_or将矩阵组合到一起
    //...
    // 分开计算黑白坏点的cluster
    std::array<size_t, Cluster_MAX> BlackClusterCountArray, WhiteClusterCountArray;
    GetCluster(BlackPointMat, Condition.ClusterCountKernelSize, Condition.ThisPixArray, BlackClusterCountArray);
    if (BPrint)
    {
        std::string fileName = "Light_BlackClusterRect_" + std::to_string(Count) + ".txt";
        std::ofstream ofs(fileName, std::ios::trunc);
        for (auto p : ClusterRectsVector)
        {
            auto& ClusterArea = p.first;
            auto ClusterType = p.second;
            ofs << "Rect: x= " << ClusterArea.x << ",y= " << ClusterArea.y << ",w= " << ClusterArea.width
                << ",h= " << ClusterArea.height << ",type=" << Enum2String_ClusterType.at(ClusterType) << std::endl;;
        }
        ofs.close();
    }
    GetCluster(WhitePointMat, Condition.ClusterCountKernelSize, Condition.ThisPixArray, WhiteClusterCountArray);
    if (BPrint)
    {
        std::string fileName = "Light_WhiteClusterRect_" + std::to_string(Count) + ".txt";
        std::ofstream ofs(fileName, std::ios::trunc);
        for (auto p : ClusterRectsVector)
        {
            auto& ClusterArea = p.first;
            auto ClusterType = p.second;
            ofs << "Rect: x= " << ClusterArea.x << ",y= " << ClusterArea.y << ",w= " << ClusterArea.width
                << ",h= " << ClusterArea.height << ",type=" << Enum2String_ClusterType.at(ClusterType) << std::endl;;
        }
        ofs.close();
    }

    // 对结果赋值
    Res->ClusterCount = 
        std::accumulate(BlackClusterCountArray.begin(), BlackClusterCountArray.end(), size_t(0)) +
        std::accumulate(WhiteClusterCountArray.begin(), WhiteClusterCountArray.end(), size_t(0));
    for (int Index = 0; Index < Cluster_MAX; Index++)
    {
        Res->EachClustersSize[Index] = BlackClusterCountArray.at(Index) + WhiteClusterCountArray.at(Index);
    }
#pragma endregion


#pragma region FPN
    //double HorizontalFPN, VerticalFPN;
    //GetFPN(SrcMat, HorizontalFPN, VerticalFPN);
    //Res->HorizontalFPN = HorizontalFPN;
    //Res->VerticalFPN = VerticalFPN;
#pragma endregion

#pragma region BlemishCount
    int numOfBoundery;
    GetBlemish(SrcMat, numOfBoundery);
    Res->BlemishCount = numOfBoundery;
#pragma endregion
	Res->ScratchCount = 0;
    return Img_Success;
}

ImgErrorCode TestRPNU(unsigned short* RawImage40, unsigned short* RawImage80, unsigned ImageHeight, unsigned int ImageWidth, ImgCondition_PRNU Condition, ImgResult_PRNU * Res)
{
    if (Res == nullptr)
    {
        return Img_Init_Fail;
    }
    if (Condition.FrameCount != 2 || Condition.Depth != 10)
    {
        return Img_Init_Fail;
    }

    const uint TestImageHeight = ImageHeight;
    const uint TestImageWidth = ImageWidth;
    const int TestImageType = CV_16UC1;
    const cv::Rect ROIRect(Condition.LeftTop_X, Condition.LeftTop_Y, Condition.ROI_Width, Condition.ROI_Height);
    const uint TestImageDepth = Condition.Depth;

    // 求40的DiffFrame
    cv::Mat Image40Frame1, Image40Frame2, Image40DiffFrame;
    Image40Frame1 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage40)(ROIRect);
    Image40Frame2 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage40 + TestImageWidth * TestImageHeight)(ROIRect);
    Image40DiffFrame = Image40Frame1 - Image40Frame2 + 512;

    // 求80的DiffFrame
    cv::Mat Image80Frame1, Image80Frame2, Image80DiffFrame;
    Image80Frame1 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage80)(ROIRect);
    Image80Frame2 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage80 + TestImageWidth * TestImageHeight)(ROIRect);
    Image80DiffFrame = Image80Frame1 - Image80Frame2 + 512;

    auto TwoFrameMeanStd = [](const cv::Mat& Frame1Mat, const cv::Mat& FrameDiffMat, double& Frame1Mean, double& Frame1Std, double& FrameDiffMean, double& FrameDiffStd)
    {
        cv::Mat MatFrame1Mean, MatFrameDiffMean;
        cv::Mat MatFrame1Std, MatFrameDiffStd;
        cv::meanStdDev(Frame1Mat, MatFrame1Mean, MatFrame1Std);
        cv::meanStdDev(FrameDiffMat, MatFrameDiffMean, MatFrameDiffStd);
        Frame1Mean = MatFrame1Mean.at<double>(0, 0);
        Frame1Std = MatFrame1Std.at<double>(0, 0);
        FrameDiffMean = MatFrameDiffMean.at<double>(0, 0);
        FrameDiffStd = MatFrameDiffStd.at<double>(0, 0);
    };

    // Split
    cv::Mat Image40Frame1_R, Image40Frame1_Gr, Image40Frame1_Gb, Image40Frame1_B;
    cv::Mat Image40FrameDiff_R, Image40FrameDiff_Gr, Image40FrameDiff_Gb, Image40FrameDiff_B;
    cv::Mat Image80Frame1_R, Image80Frame1_Gr, Image80Frame1_Gb, Image80Frame1_B;
    cv::Mat Image80FrameDiff_R, Image80FrameDiff_Gr, Image80FrameDiff_Gb, Image80FrameDiff_B;
    SplitRawMat(Image40Frame1, Condition.ThisPixArray, Image40Frame1_R, Image40Frame1_Gr, Image40Frame1_Gb, Image40Frame1_B);
    SplitRawMat(Image40DiffFrame, Condition.ThisPixArray, Image40FrameDiff_R, Image40FrameDiff_Gr, Image40FrameDiff_Gb, Image40FrameDiff_B);
    SplitRawMat(Image80Frame1, Condition.ThisPixArray, Image80Frame1_R, Image80Frame1_Gr, Image80Frame1_Gb, Image80Frame1_B);
    SplitRawMat(Image80DiffFrame, Condition.ThisPixArray, Image80FrameDiff_R, Image80FrameDiff_Gr, Image80FrameDiff_Gb, Image80FrameDiff_B);

    // Get mean and std
    double Image40Frame1Mean_R, Image40Frame1Std_R, Image40FrameDiffMean_R, Image40FrameDiffStd_R;
    TwoFrameMeanStd(Image40Frame1_R, Image40FrameDiff_R, Image40Frame1Mean_R, Image40Frame1Std_R, Image40FrameDiffMean_R, Image40FrameDiffStd_R);
    double Image40Frame1Mean_Gr, Image40Frame1Std_Gr, Image40FrameDiffMean_Gr, Image40FrameDiffStd_Gr;
    TwoFrameMeanStd(Image40Frame1_Gr, Image40FrameDiff_Gr, Image40Frame1Mean_Gr, Image40Frame1Std_Gr, Image40FrameDiffMean_Gr, Image40FrameDiffStd_Gr);
    double Image40Frame1Mean_Gb, Image40Frame1Std_Gb, Image40FrameDiffMean_Gb, Image40FrameDiffStd_Gb;
    TwoFrameMeanStd(Image40Frame1_Gb, Image40FrameDiff_Gb, Image40Frame1Mean_Gb, Image40Frame1Std_Gb, Image40FrameDiffMean_Gb, Image40FrameDiffStd_Gb);
    double Image40Frame1Mean_B, Image40Frame1Std_B, Image40FrameDiffMean_B, Image40FrameDiffStd_B;
    TwoFrameMeanStd(Image40Frame1_B, Image40FrameDiff_B, Image40Frame1Mean_B, Image40Frame1Std_B, Image40FrameDiffMean_B, Image40FrameDiffStd_B);
    double Image80Frame1Mean_R, Image80Frame1Std_R, Image80FrameDiffMean_R, Image80FrameDiffStd_R;
    TwoFrameMeanStd(Image80Frame1_R, Image80FrameDiff_R, Image80Frame1Mean_R, Image80Frame1Std_R, Image80FrameDiffMean_R, Image80FrameDiffStd_R);
    double Image80Frame1Mean_Gr, Image80Frame1Std_Gr, Image80FrameDiffMean_Gr, Image80FrameDiffStd_Gr;
    TwoFrameMeanStd(Image80Frame1_Gr, Image80FrameDiff_Gr, Image80Frame1Mean_Gr, Image80Frame1Std_Gr, Image80FrameDiffMean_Gr, Image80FrameDiffStd_Gr);
    double Image80Frame1Mean_Gb, Image80Frame1Std_Gb, Image80FrameDiffMean_Gb, Image80FrameDiffStd_Gb;
    TwoFrameMeanStd(Image80Frame1_Gb, Image80FrameDiff_Gb, Image80Frame1Mean_Gb, Image80Frame1Std_Gb, Image80FrameDiffMean_Gb, Image80FrameDiffStd_Gb);
    double Image80Frame1Mean_B, Image80Frame1Std_B, Image80FrameDiffMean_B, Image80FrameDiffStd_B;
    TwoFrameMeanStd(Image80Frame1_B, Image80FrameDiff_B, Image80Frame1Mean_B, Image80Frame1Std_B, Image80FrameDiffMean_B, Image80FrameDiffStd_B);

	//Mean_80-Mean_40      Sensitivity
	//add in 2023/06/30
	double diff_R = Image80Frame1Mean_R - Image40Frame1Mean_R;
	double diff_Gr = Image80Frame1Mean_Gr - Image40Frame1Mean_Gr;
	double diff_Gb = Image80Frame1Mean_Gb - Image40Frame1Mean_Gb;
	double diff_B = Image80Frame1Mean_B - Image40Frame1Mean_B;

    // calculate PRUN with each channel and 40/80
    double PRNU40_R = std::sqrt(Image40Frame1Std_R * Image40Frame1Std_R - Image40FrameDiffStd_R * Image40FrameDiffStd_R / 2.f);
    double PRNU40_Gr = std::sqrt(Image40Frame1Std_Gr * Image40Frame1Std_Gr - Image40FrameDiffStd_Gr * Image40FrameDiffStd_Gr / 2.f);
    double PRNU40_Gb = std::sqrt(Image40Frame1Std_Gb * Image40Frame1Std_Gb - Image40FrameDiffStd_Gb * Image40FrameDiffStd_Gb / 2.f);
    double PRNU40_B = std::sqrt(Image40Frame1Std_B * Image40Frame1Std_B - Image40FrameDiffStd_B * Image40FrameDiffStd_B / 2.f);
    double PRNU80_R = std::sqrt(Image80Frame1Std_R * Image80Frame1Std_R - Image80FrameDiffStd_R * Image80FrameDiffStd_R / 2.f);
    double PRNU80_Gr = std::sqrt(Image80Frame1Std_Gr * Image80Frame1Std_Gr - Image80FrameDiffStd_Gr * Image80FrameDiffStd_Gr / 2.f);
    double PRNU80_Gb = std::sqrt(Image80Frame1Std_Gb * Image80Frame1Std_Gb - Image80FrameDiffStd_Gb * Image80FrameDiffStd_Gb / 2.f);
    double PRNU80_B = std::sqrt(Image80Frame1Std_B * Image80Frame1Std_B - Image80FrameDiffStd_B * Image80FrameDiffStd_B / 2.f);

    double PRNU_R = 1.f * (PRNU80_R - PRNU40_R) / (Image80Frame1Mean_R - Image40Frame1Mean_R);
    double PRNU_Gr = 1.f * (PRNU80_Gr - PRNU40_Gr) / (Image80Frame1Mean_Gr - Image40Frame1Mean_Gr);
    double PRNU_Gb = 1.f * (PRNU80_Gb - PRNU40_Gb) / (Image80Frame1Mean_Gb - Image40Frame1Mean_Gb);
    double PRNU_B = 1.f * (PRNU80_B - PRNU40_B) / (Image80Frame1Mean_B - Image40Frame1Mean_B);

    Res->PRNU40_R = PRNU40_R;
    Res->PRNU80_R = PRNU80_R;
    Res->PRNU_R = PRNU_R;
    Res->PRNU40_Gr = PRNU40_Gr;
    Res->PRNU80_Gr = PRNU80_Gr;
    Res->PRNU_Gr = PRNU_Gr;
    Res->PRNU40_Gb = PRNU40_Gb;
    Res->PRNU80_Gb = PRNU80_Gb;
    Res->PRNU_Gb = PRNU_Gb;
    Res->PRNU40_B = PRNU40_B;
    Res->PRNU80_B = PRNU80_B;
    Res->PRNU_B = PRNU_B;

	//add in 2023/06/30    Sensitivity
	Res->Diff_R = diff_R;
	Res->Diff_Gr = diff_Gr;
	Res->Diff_Gb = diff_Gb;
	Res->Diff_B = diff_B;

    return Img_Success;
}

ImgErrorCode TestCG(unsigned short* RawImage40, unsigned short* RawImage80, unsigned ImageHeight, unsigned int ImageWidth, ImgCondition_CG Condition, ImgResult_CG* Res)
{

    std::string fileName = "FPN_1.txt";
    std::ofstream ofs(fileName, std::ios::trunc);

    ofs << 1 << std::endl;

    ofs.close();
    if (Res == nullptr)
    {
        return Img_Init_Fail;
    }
    if (Condition.FrameCount != 2 || Condition.Depth != 10)
    {
        return Img_Init_Fail;
    }

    
    ofs.open(fileName, std::ios::trunc);

    ofs << 2 << std::endl;

    ofs.close();

    const uint TestImageHeight = ImageHeight;
    const uint TestImageWidth = ImageWidth;
    const int TestImageType = CV_16UC1;
    const cv::Rect ROIRect(Condition.LeftTop_X, Condition.LeftTop_Y, Condition.ROI_Width, Condition.ROI_Height);
    const uint TestImageDepth = Condition.Depth;

    // 求40的DiffFrame
    cv::Mat Image40Frame1, Image40Frame2, Image40DiffFrame;
    Image40Frame1 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage40)(ROIRect);
    Image40Frame2 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage40 + TestImageWidth * TestImageHeight)(ROIRect);
    Image40DiffFrame = Image40Frame1 - Image40Frame2 + 512;

    // 求80的DiffFrame
    cv::Mat Image80Frame1, Image80Frame2, Image80DiffFrame;
    Image80Frame1 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage80)(ROIRect);
    Image80Frame2 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImage80 + TestImageWidth * TestImageHeight)(ROIRect);
    Image80DiffFrame = Image80Frame1 - Image80Frame2 + 512;

    auto TwoFrameMeanStd = [](const cv::Mat& Frame1Mat, const cv::Mat& FrameDiffMat, double& Frame1Mean, double& Frame1Std, double& FrameDiffMean, double& FrameDiffStd)
    {
        cv::Mat MatFrame1Mean, MatFrameDiffMean;
        cv::Mat MatFrame1Std, MatFrameDiffStd;
        cv::meanStdDev(Frame1Mat, MatFrame1Mean, MatFrame1Std);
        cv::meanStdDev(FrameDiffMat, MatFrameDiffMean, MatFrameDiffStd);
        Frame1Mean = MatFrame1Mean.at<double>(0, 0);
        Frame1Std = MatFrame1Std.at<double>(0, 0);
        FrameDiffMean = MatFrameDiffMean.at<double>(0, 0);
        FrameDiffStd = MatFrameDiffStd.at<double>(0, 0);
    };

    // Split
    cv::Mat Image40Frame1_R, Image40Frame1_Gr, Image40Frame1_Gb, Image40Frame1_B;
    cv::Mat Image40FrameDiff_R, Image40FrameDiff_Gr, Image40FrameDiff_Gb, Image40FrameDiff_B;
    cv::Mat Image80Frame1_R, Image80Frame1_Gr, Image80Frame1_Gb, Image80Frame1_B;
    cv::Mat Image80FrameDiff_R, Image80FrameDiff_Gr, Image80FrameDiff_Gb, Image80FrameDiff_B;
    SplitRawMat(Image40Frame1, Condition.ThisPixArray, Image40Frame1_R, Image40Frame1_Gr, Image40Frame1_Gb, Image40Frame1_B);
    SplitRawMat(Image40DiffFrame, Condition.ThisPixArray, Image40FrameDiff_R, Image40FrameDiff_Gr, Image40FrameDiff_Gb, Image40FrameDiff_B);
    SplitRawMat(Image80Frame1, Condition.ThisPixArray, Image80Frame1_R, Image80Frame1_Gr, Image80Frame1_Gb, Image80Frame1_B);
    SplitRawMat(Image80DiffFrame, Condition.ThisPixArray, Image80FrameDiff_R, Image80FrameDiff_Gr, Image80FrameDiff_Gb, Image80FrameDiff_B);

    // Get mean and std
	double Image40Frame1Mean, Image40Frame1Std, Image40FrameDiffMean, Image40FrameDiffStd;
	TwoFrameMeanStd(Image40Frame1, Image40DiffFrame, Image40Frame1Mean, Image40Frame1Std, Image40FrameDiffMean, Image40FrameDiffStd);
    double Image40Frame1Mean_R, Image40Frame1Std_R, Image40FrameDiffMean_R, Image40FrameDiffStd_R;
    TwoFrameMeanStd(Image40Frame1_R, Image40FrameDiff_R, Image40Frame1Mean_R, Image40Frame1Std_R, Image40FrameDiffMean_R, Image40FrameDiffStd_R);
    double Image40Frame1Mean_Gr, Image40Frame1Std_Gr, Image40FrameDiffMean_Gr, Image40FrameDiffStd_Gr;
    TwoFrameMeanStd(Image40Frame1_Gr, Image40FrameDiff_Gr, Image40Frame1Mean_Gr, Image40Frame1Std_Gr, Image40FrameDiffMean_Gr, Image40FrameDiffStd_Gr);
    double Image40Frame1Mean_Gb, Image40Frame1Std_Gb, Image40FrameDiffMean_Gb, Image40FrameDiffStd_Gb;
    TwoFrameMeanStd(Image40Frame1_Gb, Image40FrameDiff_Gb, Image40Frame1Mean_Gb, Image40Frame1Std_Gb, Image40FrameDiffMean_Gb, Image40FrameDiffStd_Gb);
    double Image40Frame1Mean_B, Image40Frame1Std_B, Image40FrameDiffMean_B, Image40FrameDiffStd_B;
    TwoFrameMeanStd(Image40Frame1_B, Image40FrameDiff_B, Image40Frame1Mean_B, Image40Frame1Std_B, Image40FrameDiffMean_B, Image40FrameDiffStd_B);

	double Image80Frame1Mean, Image80Frame1Std, Image80FrameDiffMean, Image80FrameDiffStd;
    TwoFrameMeanStd(Image80Frame1, Image80DiffFrame, Image80Frame1Mean, Image80Frame1Std, Image80FrameDiffMean, Image80FrameDiffStd);
    double Image80Frame1Mean_R, Image80Frame1Std_R, Image80FrameDiffMean_R, Image80FrameDiffStd_R;
    TwoFrameMeanStd(Image80Frame1_R, Image80FrameDiff_R, Image80Frame1Mean_R, Image80Frame1Std_R, Image80FrameDiffMean_R, Image80FrameDiffStd_R);
    double Image80Frame1Mean_Gr, Image80Frame1Std_Gr, Image80FrameDiffMean_Gr, Image80FrameDiffStd_Gr;
    TwoFrameMeanStd(Image80Frame1_Gr, Image80FrameDiff_Gr, Image80Frame1Mean_Gr, Image80Frame1Std_Gr, Image80FrameDiffMean_Gr, Image80FrameDiffStd_Gr);
    double Image80Frame1Mean_Gb, Image80Frame1Std_Gb, Image80FrameDiffMean_Gb, Image80FrameDiffStd_Gb;
    TwoFrameMeanStd(Image80Frame1_Gb, Image80FrameDiff_Gb, Image80Frame1Mean_Gb, Image80Frame1Std_Gb, Image80FrameDiffMean_Gb, Image80FrameDiffStd_Gb);
    double Image80Frame1Mean_B, Image80Frame1Std_B, Image80FrameDiffMean_B, Image80FrameDiffStd_B;
    TwoFrameMeanStd(Image80Frame1_B, Image80FrameDiff_B, Image80Frame1Mean_B, Image80Frame1Std_B, Image80FrameDiffMean_B, Image80FrameDiffStd_B);

    // calculate PRUN with each channel and 40/80
	double PRNU40 = std::sqrt(Image40Frame1Std * Image40Frame1Std - Image40FrameDiffStd * Image40FrameDiffStd / 2.f);
    double PRNU40_R = std::sqrt(Image40Frame1Std_R * Image40Frame1Std_R - Image40FrameDiffStd_R * Image40FrameDiffStd_R / 2.f);
    double PRNU40_Gr = std::sqrt(Image40Frame1Std_Gr * Image40Frame1Std_Gr - Image40FrameDiffStd_Gr * Image40FrameDiffStd_Gr / 2.f);
    double PRNU40_Gb = std::sqrt(Image40Frame1Std_Gb * Image40Frame1Std_Gb - Image40FrameDiffStd_Gb * Image40FrameDiffStd_Gb / 2.f);
    double PRNU40_B = std::sqrt(Image40Frame1Std_B * Image40Frame1Std_B - Image40FrameDiffStd_B * Image40FrameDiffStd_B / 2.f);

	double PRNU80 = std::sqrt(Image80Frame1Std * Image80Frame1Std - Image80FrameDiffStd * Image80FrameDiffStd / 2.f);
    double PRNU80_R = std::sqrt(Image80Frame1Std_R * Image80Frame1Std_R - Image80FrameDiffStd_R * Image80FrameDiffStd_R / 2.f);
    double PRNU80_Gr = std::sqrt(Image80Frame1Std_Gr * Image80Frame1Std_Gr - Image80FrameDiffStd_Gr * Image80FrameDiffStd_Gr / 2.f);
    double PRNU80_Gb = std::sqrt(Image80Frame1Std_Gb * Image80Frame1Std_Gb - Image80FrameDiffStd_Gb * Image80FrameDiffStd_Gb / 2.f);
    double PRNU80_B = std::sqrt(Image80Frame1Std_B * Image80Frame1Std_B - Image80FrameDiffStd_B * Image80FrameDiffStd_B / 2.f);

	double CG = 0.5f * (Image80FrameDiffStd* Image80FrameDiffStd - Image40FrameDiffStd* Image40FrameDiffStd) / (Image80Frame1Mean - Image40Frame1Mean);
    double CG_R = 0.5f * (Image80FrameDiffStd_R* Image80FrameDiffStd_R - Image40FrameDiffStd_R* Image40FrameDiffStd_R) / (Image80Frame1Mean_R - Image40Frame1Mean_R);
    double CG_Gr = 0.5f * (Image80FrameDiffStd_Gr * Image80FrameDiffStd_Gr - Image40FrameDiffStd_Gr * Image40FrameDiffStd_Gr) / (Image80Frame1Mean_Gr - Image40Frame1Mean_Gr);
    double CG_Gb = 0.5f * (Image80FrameDiffStd_Gb * Image80FrameDiffStd_Gb - Image40FrameDiffStd_Gb * Image40FrameDiffStd_Gb) / (Image80Frame1Mean_Gb - Image40Frame1Mean_Gb);
    double CG_B = 0.5f * (Image80FrameDiffStd_B * Image80FrameDiffStd_B - Image40FrameDiffStd_B * Image40FrameDiffStd_B) / (Image80Frame1Mean_B - Image40Frame1Mean_B);


 
    ofs.open(fileName, std::ios::trunc);
    
        ofs << "Rect: x= " << CG_B << ",y= " << CG_Gb << ",w= " << CG_Gr
            << ",h= " << CG_R << std::endl;
   
    ofs.close();

	Res->CG=CG;
    Res->CG_R = CG_R;
    Res->CG_Gr = CG_Gr;
    Res->CG_Gb = CG_Gb;
    Res->CG_B = CG_B;


    return Img_Success;
}

ImgErrorCode FWCTest(unsigned short* RawImage, unsigned ImageHeight, unsigned int ImageWidth, ImgCondition_Light Condition, ImageResult_FWC* Res)
{
	if (Res == nullptr)
	{
		return Img_Init_Fail;
	}

	const uint TestImageHeight = ImageHeight;
	const uint TestImageWidth = ImageWidth;
	const int TestImageType = CV_16UC1;

	cv::Mat SrcMat(TestImageHeight, TestImageWidth, TestImageType, RawImage);


#pragma region MeanStd
	cv::Mat R_Mat, Gr_Mat, Gb_Mat, B_Mat;
	SplitRawMat(SrcMat, Condition.ThisPixArray, R_Mat, Gr_Mat, Gb_Mat, B_Mat);

	double R_Mean, Gr_Mean, Gb_Mean, B_Mean, Src_Mean;
	double R_StdDev, Gr_StdDev, Gb_StdDev, B_StdDev, Src_StdDev;
	GetMeanAndStd(SrcMat, Src_Mean, Src_StdDev);
	GetMeanAndStd(R_Mat, R_Mean, R_StdDev);
	GetMeanAndStd(Gr_Mat, Gr_Mean, Gr_StdDev);
	GetMeanAndStd(Gb_Mat, Gb_Mean, Gb_StdDev);
	GetMeanAndStd(B_Mat, B_Mean, B_StdDev);



	Res->Mean_R = R_Mean;
	Res->Mean_Gr = Gr_Mean;
	Res->Mean_Gb = Gb_Mean;
	Res->Mean_B = B_Mean;

	Res->Std_R = R_StdDev;
	Res->Std_Gr = Gr_StdDev;
	Res->Std_Gb = Gb_StdDev;
	Res->Std_B = B_StdDev;


#pragma endregion
    return Img_Success;
}

void SetPrint(bool bPrint)
{
	char iniName[MAX_PATH];
	GetCurrentDirectoryA(MAX_PATH, iniName);
	snprintf(iniName, MAX_PATH, "%s\\AlgConfig.ini", iniName);
	try
	{
		boost::property_tree::ptree pt;
		boost::property_tree::read_ini(iniName, pt);
		if (pt.get<int>("bPrint") == 1)
		{
			bPrint = true;
		}
	}
	catch (const boost::exception& e)
	{
		bPrint = false;
	}
    BPrint = bPrint;
    Count = 0;
}

ImgErrorCode DarkCurrentTest(unsigned short* RawImageFrame1, unsigned short* RawImageFrame2, unsigned ImageHeight, unsigned int ImageWidth, ImgCondition_DarkCurrent Condition, ImgResult_DarkCurrent* Res)
{
	if (Res == nullptr)
	{
		return Img_Init_Fail;
	}
	if (Condition.FrameCount != 1 || Condition.Depth != 10)
	{
		return Img_Init_Fail;
	}

	const uint TestImageHeight = ImageHeight;
	const uint TestImageWidth = ImageWidth;
	const int TestImageType = CV_16UC1;
	const cv::Rect ROIRect(Condition.LeftTop_X, Condition.LeftTop_Y, Condition.ROI_Width, Condition.ROI_Height);
	const uint TestImageDepth = Condition.Depth;


	double currentMean[2];
	double currentStd[2];

	cv::Mat ImageCurrentFrame1;
	ImageCurrentFrame1 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImageFrame1)(ROIRect);
	GetMeanAndStd(ImageCurrentFrame1, currentMean[0], currentStd[0]);

	cv::Mat ImageCurrentFrame2;
	ImageCurrentFrame2 = cv::Mat(cv::Size(TestImageWidth, TestImageHeight), CV_16UC1, RawImageFrame2)(ROIRect);
	GetMeanAndStd(ImageCurrentFrame2, currentMean[1], currentStd[1]);

	double DC = (currentMean[0] - currentMean[1]) / (16 * 0.4375);

	Res->darkcurrent = DC;


	return Img_Success;
}