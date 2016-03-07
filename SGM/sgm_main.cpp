/*

Project name :SGM

Author: MingChao_Sun

Date:2016/3/6 

*/


#include <string>

#include <windows.h>
#undef min
#undef max

#include <algorithm>
#include <stack>
#include "png.hpp"




// Default parameters
const int SGMSTEREO_DEFAULT_DISPARITY_TOTAL = 256;
const double SGMSTEREO_DEFAULT_DISPARITY_FACTOR = 256;
const int SGMSTEREO_DEFAULT_SOBEL_CAP_VALUE = 15;
const int SGMSTEREO_DEFAULT_CENSUS_WINDOW_RADIUS = 2;
const double SGMSTEREO_DEFAULT_CENSUS_WEIGHT_FACTOR = 1.0 / 6.0;
const int SGMSTEREO_DEFAULT_AGGREGATION_WINDOW_RADIUS = 2;
const int SGMSTEREO_DEFAULT_SMOOTHNESS_PENALTY_SMALL = 100;
const int SGMSTEREO_DEFAULT_SMOOTHNESS_PENALTY_LARGE = 1600;
const int SGMSTEREO_DEFAULT_CONSISTENCY_THRESHOLD = 1;

/*
* SGM计算
*/
void compute(const png::image<png::rgb_pixel>& leftImage, const png::image<png::rgb_pixel>& rightImage,
	float* disparityImage);

/*
* SGM初始化
*/
void initialize(const png::image<png::rgb_pixel>& leftImage, const png::image<png::rgb_pixel>& rightImage);

/*
* 计算代价
*/
void computeCostImage(const png::image<png::rgb_pixel>& leftImage, const png::image<png::rgb_pixel>& rightImage);

/*
* 计算LeftCostImage()
*/
void computeLeftCostImage(const unsigned char* leftGrayscaleImage, const unsigned char* rightGrayscaleImage);

/*
* 计算RightCostImage()
*/
void computeRightCostImage();

/*
* 使用Sobel导数滤波器计算x导数图像 上下界为0-30
*/
void computeCappedSobelImage(const unsigned char* image, const bool horizontalFlip, unsigned char* sobelImage);

/*
* 输入左右censusTransform,计算海明距离得出Census Transform 项的Cost （每一行）分为 1-256 256-width 两部分，每部分采用不同策略计算
*/
void computeCensusImage(const unsigned char* image, int* censusImage);

/*
* 计算上边缘的Cost (代价聚集 聚集窗口半径 2)
*/
void calcTopRowCost(unsigned char*& leftSobelRow, int*& leftCensusRow,
	unsigned char*& rightSobelRow, int*& rightCensusRow,
	unsigned short* costImageRow);

/*
* 计算所有行的Cost (代价聚集 聚集窗口半径 2)
*/
void calcRowCosts(unsigned char*& leftSobelRow, int*& leftCensusRow,
	unsigned char*& rightSobelRow, int*& rightCensusRow,
	unsigned short* costImageRow);

/*
* 计算极线方向的梯度造成的Cost
*/
void calcPixelwiseSAD(const unsigned char* leftSobelRow, const unsigned char* rightSobelRow);

/*
* 计算一行中每个像素与其左平均，右平均三者之间的极值 用于与左图像素计算Cost
*/
void calcHalfPixelRight(const unsigned char* rightSobelRow);

/*
* 输入左右censusTransform图,计算海明距离得出Census Transform 项的Cost
*/
void addPixelwiseHamming(const int* leftCensusRow, const int* rightCensusRow);

/*
* 执行SGM 使得Cost最小
*/
void performSGM(unsigned short* costImage, unsigned short* disparityImage);

/*
* 斑点滤波器
*/
void speckleFilter(const int maxSpeckleSize, const int maxDifference, unsigned short* image);

/*
* 保持左右一致 输入左右DisparityImage，输出
*/
void enforceLeftRightConsistency(unsigned short* leftDisparityImage, unsigned short* rightDisparityImage);

/*
* SGM释放内存
*/
void freeDataBuffer();

/*
* 保存图片
*/
void SaveFlattenCharImage(unsigned char* FlattenCharImage, std::string outputDisparityImageFilename);


//定义图片尺寸
int width_;
int height_;

//定义存储SGM的DisparityImage
float* initialDisparityImage_;

/*
*SGM相关变量
*/

// Parameter
int disparityTotal_ = SGMSTEREO_DEFAULT_DISPARITY_TOTAL;				       //要求:大于0且能被16整除
double disparityFactor_ = SGMSTEREO_DEFAULT_DISPARITY_FACTOR;                  //leftDisparityImage的像素值的缩小比例
int sobelCapValue_ = SGMSTEREO_DEFAULT_SOBEL_CAP_VALUE;                        //sobel导数限制 （0-30）
int censusWindowRadius_ = SGMSTEREO_DEFAULT_CENSUS_WINDOW_RADIUS;              //CT变换窗口半径   default = 2
double censusWeightFactor_ = SGMSTEREO_DEFAULT_CENSUS_WEIGHT_FACTOR;           //Census Cost 占有的权重
int aggregationWindowRadius_ = SGMSTEREO_DEFAULT_AGGREGATION_WINDOW_RADIUS;    //代价聚集窗口半径 default = 2
int smoothnessPenaltySmall_ = SGMSTEREO_DEFAULT_SMOOTHNESS_PENALTY_SMALL;      //用于performSGM
int smoothnessPenaltyLarge_ = SGMSTEREO_DEFAULT_SMOOTHNESS_PENALTY_LARGE;      //用于performSGM
int consistencyThreshold_ = SGMSTEREO_DEFAULT_CONSISTENCY_THRESHOLD;		   //例如左图中的点al(x,y)通过视差计算其在右图中的对应点ar(x + leftDisparityValue,y),若leftDisparityValue与ar在右图中的视差 差值大于该值，则判断为异常点



// Data
int widthStep_;

unsigned short* leftCostImage_;
unsigned short* rightCostImage_;
unsigned char* pixelwiseCostRow_;
unsigned short* rowAggregatedCost_;
unsigned char* halfPixelRightMin_;
unsigned char* halfPixelRightMax_;

int pathRowBufferTotal_;
int disparitySize_;
int pathTotal_;
int pathDisparitySize_;
int costSumBufferRowSize_;
int costSumBufferSize_;
int pathMinCostBufferSize_;
int pathCostBufferSize_;
int totalBufferSize_;

short* sgmBuffer_;

int main(int argc, char* argv[]) {

	//从命令行获取图片路径（参数少于3则输出提示）
	if (argc < 3) {
		std::cerr << "usage: sgmstereo left right" << std::endl;
		exit(1);
	}

	//　时间点1
	DWORD dwTime1 = GetTickCount();

	std::string leftImageFilename = argv[1];
	std::string rightImageFilename = argv[2];

	std::cout << "readFile... \n  Left:" << leftImageFilename << "\n  Right:" << rightImageFilename << std::endl;

	//读取 png Image
	png::image<png::rgb_pixel> leftImage(leftImageFilename);

	png::image<png::rgb_pixel> rightImage(rightImageFilename);

	//　时间点2
	DWORD dwTime2 = GetTickCount();

	std::cout << "Read Finished , Time_Use :" << dwTime2 - dwTime1 << "ms" << std::endl;

	std::cout << "Init..." << std::endl;

	//获取图片尺寸
	width_ = static_cast<int>(leftImage.get_width());
	height_ = static_cast<int>(leftImage.get_height());

	if (rightImage.get_width() != width_ || rightImage.get_height() != height_) {
		throw std::invalid_argument("[SGMStereo::setImageSize] sizes of left and right images are different");
	}


	//创建PNG对象
	png::image<png::gray_pixel> disparityImage;

	disparityImage.resize(width_, height_);

	//根据图像尺寸分配内存
	initialDisparityImage_ = reinterpret_cast<float*>(malloc(width_*height_*sizeof(float)));

	//　时间点3
	DWORD dwTime3 = GetTickCount();

	std::cout << "Init Finished , Time_Use :" << dwTime3 - dwTime2 << "ms" << std::endl << std::endl;


	std::cout << "SGM Compute..." << std::endl;

	//SGM-返回DisparityImage
	compute(leftImage, rightImage, initialDisparityImage_);

	//　时间点4
	DWORD dwTime4 = GetTickCount();

	std::cout << "SGM Finished , Time_Use :" << dwTime4 - dwTime3 << "ms" << std::endl << std::endl;

	std::cout << "Save Image..." << std::endl;

	//循环将视差写入图片
	for (int y = 0; y < height_; ++y) {

		for (int x = 0; x < width_; ++x) {

			if (initialDisparityImage_[width_*y + x] <= 0.0 || initialDisparityImage_[width_*y + x] > 255.0) {
				disparityImage.set_pixel(x, y, 0);
			}
			else {
				disparityImage.set_pixel(x, y, static_cast<unsigned short>(initialDisparityImage_[width_*y + x]));
			}

		}
	}

	//保存图片
	std::string outputDisparityImageFilename = "sgm_disparity.png";

	disparityImage.write(outputDisparityImageFilename);

	//　时间点5
	DWORD dwTime5 = GetTickCount();

	std::cout << "Image Saved , Time_Use :" << dwTime5 - dwTime4 << "ms" << std::endl << std::endl;

	std::cout << "All Done ,Total Time_Use :" << dwTime5 - dwTime1 << "ms" << std::endl << std::endl;
}

/*
* SGM计算
*/
void compute(const png::image<png::rgb_pixel>& leftImage, const png::image<png::rgb_pixel>& rightImage,
	float* disparityImage)
{

	//初始化SGM所需的变量 根据图片大小 分配内存
	std::cout << "  SGM_initialize" << std::endl;
	initialize(leftImage, rightImage);

	//进行Cost的计算 这里Cost由极线方向的梯度差和CensusTransform后的海明距离组成
	std::cout << "  SGM_computeCostImage" << std::endl;
	computeCostImage(leftImage, rightImage);

	//左 执行SGM
	std::cout << "  SGM_performSGM_leftDisparityImage" << std::endl;
	unsigned short* leftDisparityImage = reinterpret_cast<unsigned short*>(malloc(width_*height_*sizeof(unsigned short)));
	performSGM(leftCostImage_, leftDisparityImage);

	//右 执行SGM
	std::cout << "  SGM_performSGM_rightDisparityImage" << std::endl;
	unsigned short* rightDisparityImage = reinterpret_cast<unsigned short*>(malloc(width_*height_*sizeof(unsigned short)));
	performSGM(rightCostImage_, rightDisparityImage);
	
	//左右一致
	std::cout << "  SGM_enforceLeftRightConsistency" << std::endl;
	enforceLeftRightConsistency(leftDisparityImage, rightDisparityImage); 

	//保存写入
	std::cout << "  SGM_Write" << std::endl;
	for (int y = 0; y < height_; ++y) {
		for (int x = 0; x < width_; ++x) {
			disparityImage[width_*y + x] = static_cast<float>(leftDisparityImage[width_*y + x] / disparityFactor_);
		}
	}

	//释放内存
	std::cout << "  SGM_Free" << std::endl;
	freeDataBuffer();
	free(leftDisparityImage);
	free(rightDisparityImage);
}

/*
* 参数与内存初始化
*/
void initialize(const png::image<png::rgb_pixel>& leftImage, const png::image<png::rgb_pixel>& rightImage) {

	//计算大于等于宽度的最小的16的倍数
	widthStep_ = width_ + 15 - (width_ - 1) % 16;

	//CostImage(代价图)
	leftCostImage_ = reinterpret_cast<unsigned short*>(_mm_malloc(width_*height_*disparityTotal_*sizeof(unsigned short), 16));
	rightCostImage_ = reinterpret_cast<unsigned short*>(_mm_malloc(width_*height_*disparityTotal_*sizeof(unsigned short), 16));

	//pixelwiseCost的长度为 width个组，每组256
	int pixelwiseCostRowBufferSize = width_*disparityTotal_;

	//rowAggregatedCost_的长度
	int rowAggregatedCostBufferSize = width_*disparityTotal_*(aggregationWindowRadius_ * 2 + 2);

	//halfPixelRightMin_的长度
	int halfPixelRightBufferSize = widthStep_;

	//记录pixelwiseCost(梯度Cost以及 海明CT Cost)
	pixelwiseCostRow_ = reinterpret_cast<unsigned char*>(_mm_malloc(pixelwiseCostRowBufferSize*sizeof(unsigned char), 16));

	//用于计算 rowAggregatedCostCurrent（行聚集Cost）
	rowAggregatedCost_ = reinterpret_cast<unsigned short*>(_mm_malloc(rowAggregatedCostBufferSize*sizeof(unsigned short), 16));

	//记录左平均，右平均，与center值三者之间的极值
	halfPixelRightMin_ = reinterpret_cast<unsigned char*>(_mm_malloc(halfPixelRightBufferSize*sizeof(unsigned char), 16));
	halfPixelRightMax_ = reinterpret_cast<unsigned char*>(_mm_malloc(halfPixelRightBufferSize*sizeof(unsigned char), 16));

	//计算pathDisparitySize_
	pathRowBufferTotal_ = 2;
	disparitySize_ = disparityTotal_ + 16;
	pathTotal_ = 8;
	pathDisparitySize_ = pathTotal_*disparitySize_;

	//计算sgmBuffer_的长度
	costSumBufferRowSize_ = width_*disparityTotal_;
	costSumBufferSize_ = costSumBufferRowSize_*height_;
	pathMinCostBufferSize_ = (width_ + 2)*pathTotal_;
	pathCostBufferSize_ = pathMinCostBufferSize_*disparitySize_;
	totalBufferSize_ = (pathMinCostBufferSize_ + pathCostBufferSize_)*pathRowBufferTotal_ + costSumBufferSize_ + 16;

	//sgmBuffer_ 用于performSGM
	sgmBuffer_ = reinterpret_cast<short*>(_mm_malloc(totalBufferSize_*sizeof(short), 16));

}

/*
* 计算代价
*/
void computeCostImage(const png::image<png::rgb_pixel>& leftImage, const png::image<png::rgb_pixel>& rightImage) {

	std::cout << "    SGM_computeCostImage_Gray&Flatten" << std::endl;

	unsigned char* leftGrayscaleImage = reinterpret_cast<unsigned char*>(malloc(width_*height_*sizeof(unsigned char)));

	unsigned char* rightGrayscaleImage = reinterpret_cast<unsigned char*>(malloc(width_*height_*sizeof(unsigned char)));

	//convertToGrayscale (将左右图片灰度化并压平)
	for (int y = 0; y < height_; ++y) {
		for (int x = 0; x < width_; ++x) {
			png::rgb_pixel pix = leftImage.get_pixel(x, y);
			leftGrayscaleImage[width_*y + x] = static_cast<unsigned char>(0.299*pix.red + 0.587*pix.green + 0.114*pix.blue + 0.5);
			pix = rightImage.get_pixel(x, y);
			rightGrayscaleImage[width_*y + x] = static_cast<unsigned char>(0.299*pix.red + 0.587*pix.green + 0.114*pix.blue + 0.5);
		}
	}

	std::cout << "    SGM_computeCostImage_computeLeftCostImage" << std::endl;

	//在leftCostImage_段内存块中填充0值（最快方法）
	memset(leftCostImage_, 0, width_*height_*disparityTotal_*sizeof(unsigned short));

	//computeLeftCostImage(计算LeftCostImage)
	computeLeftCostImage(leftGrayscaleImage, rightGrayscaleImage);

	std::cout << "    SGM_computeCostImage_computeRightCostImage" << std::endl;

	//computeRightCostImage(计算RightCostImage)
	computeRightCostImage();

	std::cout << "    SGM_computeCostImage_Free" << std::endl;

	//释放左右灰度压平Image
	free(leftGrayscaleImage);
	free(rightGrayscaleImage);
}

/*
* 计算左代价
*/
void computeLeftCostImage(const unsigned char* leftGrayscaleImage, const unsigned char* rightGrayscaleImage) {

	std::cout << "      SGM_computeCostImage_computeLeftCostImage_Sobel" << std::endl;

	//使用Sobel导数滤波器计算左右两图的x导数图像
	unsigned char* leftSobelImage = reinterpret_cast<unsigned char*>(_mm_malloc(widthStep_*height_*sizeof(unsigned char), 16));
	unsigned char* rightSobelImage = reinterpret_cast<unsigned char*>(_mm_malloc(widthStep_*height_*sizeof(unsigned char), 16));

	//水平翻转 false 
	computeCappedSobelImage(leftGrayscaleImage, false, leftSobelImage);
	computeCappedSobelImage(rightGrayscaleImage, true, rightSobelImage);

	std::cout << "      SGM_computeCostImage_computeLeftCostImage_SaveSobelImage" << std::endl;

	//存储左右x导数图
	SaveFlattenCharImage(leftSobelImage, "sgm_leftSobelImage.png");
	SaveFlattenCharImage(rightSobelImage, "sgm_rightSobelImage.png");

	std::cout << "      SGM_computeCostImage_computeLeftCostImage_CensusTransform" << std::endl;

	//计算CensusTransform
	int* leftCensusImage = reinterpret_cast<int*>(malloc(width_*height_*sizeof(int)));
	int* rightCensusImage = reinterpret_cast<int*>(malloc(width_*height_*sizeof(int)));

	computeCensusImage(leftGrayscaleImage, leftCensusImage);
	computeCensusImage(rightGrayscaleImage, rightCensusImage);

	std::cout << "      SGM_computeCostImage_computeLeftCostImage_calcTopRowCost" << std::endl;

	//更换变量名
	unsigned char* leftSobelRow = leftSobelImage;
	unsigned char* rightSobelRow = rightSobelImage;

	int* leftCensusRow = leftCensusImage;
	int* rightCensusRow = rightCensusImage;

	unsigned short* costImageRow = leftCostImage_;

	//计算上边缘的Cost 代价聚集 聚集窗口半径 2 将代价存入costImageRow
	calcTopRowCost(leftSobelRow, leftCensusRow,
		rightSobelRow, rightCensusRow,
		costImageRow);
	
	//指向costImageRow的下一个存储位置
	costImageRow += width_*disparityTotal_;

	std::cout << "      SGM_computeCostImage_computeLeftCostImage_calcRowCost" << std::endl;

	//计算所有行的Cost 代价聚集 聚集窗口半径 2 将代价存入costImageRow
	calcRowCosts(leftSobelRow, leftCensusRow,
		rightSobelRow, rightCensusRow,
		costImageRow);

	std::cout << "      SGM_computeCostImage_computeLeftCostImage_Free" << std::endl;

	//释放梯度和CensusTransform
	_mm_free(leftSobelImage);
	_mm_free(rightSobelImage);
	free(leftCensusImage);
	free(rightCensusImage);

}

/*
* 计算RightCostImage()  分为 1-256 256-width 两部分，每部分采用不同策略计算 使用LeftCostImage变换得到 --------------------------------------------------------------------- 不是很理解 
*/
void computeRightCostImage() {

	//widthStepCost width_*disparityTotal_ （与pixelwiseCostRow_ 相同）
	const int widthStepCost = width_*disparityTotal_;

	//循环height次 处理所有行的代价计算
	for (int y = 0; y < height_; ++y) {

		//从leftCostImage_中逐行取leftCostRow （此时leftCostImage_已经计算完毕）
		unsigned short* leftCostRow = leftCostImage_ + widthStepCost*y;

		//rightCostImage_中逐行取rightCostRow（此时rightCostImage_还未计算）
		unsigned short* rightCostRow = rightCostImage_ + widthStepCost*y;

		//循环disparityTotal_次(256次) 到左边缘的距离d为x-1
		for (int x = 0; x < disparityTotal_; ++x) {

			//获取指向leftCostRow每行行首的指针 （对应一个图像每行的每个x） 
			unsigned short* leftCostPointer = leftCostRow + disparityTotal_*x;

			//获取指向rightCostPointer每行行首的指针 （对应一个图像每行的每个x） 
			unsigned short* rightCostPointer = rightCostRow + disparityTotal_*x;

			//循环x + 1次
			for (int d = 0; d <= x; ++d) {

				//rightCostRow 为 leftCostRow 关于对角线的对称（看做每一行disparityTotal_个元素的矩阵）
				*(rightCostPointer) = *(leftCostPointer);

				rightCostPointer -= disparityTotal_ - 1;

				++leftCostPointer;

			}
		}

		//循环 width - disparityTotal_次（width - 256 次）到边缘的距离 >256
		for (int x = disparityTotal_; x < width_; ++x) {

			//获取指向leftCostRow每行行首的指针 （对应一个图像每行的每个x）
			unsigned short* leftCostPointer = leftCostRow + disparityTotal_*x;

			//获取指向rightCostRow每行行首的指针 （对应一个图像每行的每个x） 
			unsigned short* rightCostPointer = rightCostRow + disparityTotal_*x;

			//循环disparityTotal_次
			for (int d = 0; d < disparityTotal_; ++d) {

				//rightCostRow 为 leftCostRow 关于对角线的对称（看做每一行disparityTotal_个元素的矩阵）
				*(rightCostPointer) = *(leftCostPointer);

				rightCostPointer -= disparityTotal_ - 1;

				++leftCostPointer;

			}
		}

		//与距离右边缘256处开始循环 循环 disparityTotal_次（256次）到右边缘的距离为width_ -  x （用于填充？）
		for (int x = width_ - disparityTotal_ + 1; x < width_; ++x) {

			//到右边缘的距离
			int maxDisparityIndex = width_ - x;

			//获取上一个位置数值
			unsigned short lastValue = *(rightCostRow + disparityTotal_*x + maxDisparityIndex - 1);

			//获取指向rightCostRow x行 maxDisparityIndex 处的 指针 
			unsigned short* rightCostPointer = rightCostRow + disparityTotal_*x + maxDisparityIndex;

			//循环上层循环循环的次数
			for (int d = maxDisparityIndex; d < disparityTotal_; ++d) {

				//填充数值
				*(rightCostPointer) = lastValue;

				//指针移动
				++rightCostPointer;
			}

		}
	}
}


/*
* 使用Sobel导数滤波器计算x导数图像 上下界为0-30
*/
void computeCappedSobelImage(const unsigned char* image, const bool horizontalFlip, unsigned char* sobelImage){

	//将SobelImage初始化为sobelCapValue_ = 15，长度为widthStep_*height_
	memset(sobelImage, sobelCapValue_, widthStep_*height_);

	if (horizontalFlip) {
		for (int y = 1; y < height_ - 1; ++y) {
			for (int x = 1; x < width_ - 1; ++x) {

				//Dx模板 计算x方向导数
				int sobelValue = (image[width_*(y - 1) + x + 1] + 2 * image[width_*y + x + 1] + image[width_*(y + 1) + x + 1])
					- (image[width_*(y - 1) + x - 1] + 2 * image[width_*y + x - 1] + image[width_*(y + 1) + x - 1]);

				//为导数指定上下界
				if (sobelValue > sobelCapValue_) sobelValue = 2 * sobelCapValue_;
				else if (sobelValue < -sobelCapValue_) sobelValue = 0;
				else sobelValue += sobelCapValue_;

				//反向赋值
				sobelImage[widthStep_*y + width_ - x - 1] = sobelValue;
			}
		}
	}
	else {
		for (int y = 1; y < height_ - 1; ++y) {
			for (int x = 1; x < width_ - 1; ++x) {

				//Dx模板 计算x方向导数
				int sobelValue = (image[width_*(y - 1) + x + 1] + 2 * image[width_*y + x + 1] + image[width_*(y + 1) + x + 1])
					- (image[width_*(y - 1) + x - 1] + 2 * image[width_*y + x - 1] + image[width_*(y + 1) + x - 1]);

				//为导数指定上下界
				if (sobelValue > sobelCapValue_) sobelValue = 2 * sobelCapValue_;
				else if (sobelValue < -sobelCapValue_) sobelValue = 0;
				else sobelValue += sobelCapValue_;

				//正向赋值
				sobelImage[widthStep_*y + x] = sobelValue;
			}
		}
	}
}

/*
* 对图像进行CensusTransform 窗口半径2
*/
void computeCensusImage(const unsigned char* image, int* censusImage){

	//遍历所有像素
	for (int y = 0; y < height_; ++y) {
		for (int x = 0; x < width_; ++x) {

			//依次作为中间值
			unsigned char centerValue = image[width_*y + x];

			//定义CensusCode
			int censusCode = 0;

			//窗口半径为2 遍历窗口使用位运算计算CensusCode
			for (int offsetY = -censusWindowRadius_; offsetY <= censusWindowRadius_; ++offsetY) {
				for (int offsetX = -censusWindowRadius_; offsetX <= censusWindowRadius_; ++offsetX) {
					censusCode = censusCode << 1;
					if (y + offsetY >= 0 && y + offsetY < height_
						&& x + offsetX >= 0 && x + offsetX < width_
						&& image[width_*(y + offsetY) + x + offsetX] >= centerValue) censusCode += 1;
				}
			}

			//记录返回 CensusCode
			censusImage[width_*y + x] = censusCode;
		}
	}
}

/*
* 计算上边缘的Cost 代价聚集 聚集窗口半径 2
*/
void calcTopRowCost(unsigned char*& leftSobelRow, int*& leftCensusRow,
	unsigned char*& rightSobelRow, int*& rightCensusRow,
	unsigned short* costImageRow)
{

	//循环3次 处理上边缘的代价计算
	for (int rowIndex = 0; rowIndex <= aggregationWindowRadius_; ++rowIndex) {

		int rowAggregatedCostIndex = std::min(rowIndex, height_ - 1) % (aggregationWindowRadius_ * 2 + 2);

		unsigned short* rowAggregatedCostCurrent = rowAggregatedCost_ + rowAggregatedCostIndex*width_*disparityTotal_;

		//输入左右x导数,计算某行的梯度造成的Cost 
		calcPixelwiseSAD(leftSobelRow, rightSobelRow);

		//输入左右censusTransform,计算某行海明距离得出Cost
		addPixelwiseHamming(leftCensusRow, rightCensusRow);

		//将rowAggregatedCostCurrent初始化为0，长度为 disparityTotal_*sizeof(unsigned short)
		memset(rowAggregatedCostCurrent, 0, disparityTotal_*sizeof(unsigned short));

		// 循环3次 主要完成边缘处的Cost计算 x = 0 1 2 对应 左图边缘三个像素
		for (int x = 0; x <= aggregationWindowRadius_; ++x) {
			
			// x = 0 时 scale = aggregationWindowRadius_ + 1 = 3 ,否则 x = 1
			int scale = x == 0 ? aggregationWindowRadius_ + 1 : 1;

			// 循环256次
			for (int d = 0; d < disparityTotal_; ++d) {

				//计算rowAggregatedCostCurrent第一行的值 使用pixelwiseCostRow_（记录了梯度和海明Cost）的x行的和 （前三行和为一行）并且第一行的权重为其余两行的三倍
				rowAggregatedCostCurrent[d] += static_cast<unsigned short>(pixelwiseCostRow_[disparityTotal_*x + d] * scale);
			
			}
		}

		//循环宽度 - 1次 x = 1...width-1 完成Cost的窗口聚集
		for (int x = 1; x < width_; ++x) {

			//定义addPixelwiseCost 等于pixelwiseCostRow_加上 x + 窗口半径 乘上 disparityTotal_ ,到达右边缘时用width -1乘上disparityTotal_ （用于取pixelwiseCostRow_中 的 x + 窗口半径 行）
			const unsigned char* addPixelwiseCost = pixelwiseCostRow_
				+ std::min((x + aggregationWindowRadius_)*disparityTotal_, (width_ - 1)*disparityTotal_);

			//定义subPixelwiseCost 等于pixelwiseCostRow_加上 x - 窗口半径 - 1 乘上 disparityTotal_ ，当位于左边缘时取0 （用于取pixelwiseCostRow_中 的 x - 窗口半径 - 1 行）
			const unsigned char* subPixelwiseCost = pixelwiseCostRow_
				+ std::max((x - aggregationWindowRadius_ - 1)*disparityTotal_, 0);

			// 循环256次
			for (int d = 0; d < disparityTotal_; ++d) {

				//计算rowAggregatedCostCurrent第1行到第width-1行的值 为累加上addPixelwiseCost[d] - subPixelwiseCost[d] （pixelwiseCostRow_ 多行（由窗口半径决定）的和）
				rowAggregatedCostCurrent[disparityTotal_*x + d]
					= static_cast<unsigned short>(rowAggregatedCostCurrent[disparityTotal_*(x - 1) + d]
					+ addPixelwiseCost[d] - subPixelwiseCost[d]);
			}
		}

		// rowIndex = 0 时 scale = aggregationWindowRadius_ + 1 = 3 ,否则 x = 1
		int scale = rowIndex == 0 ? aggregationWindowRadius_ + 1 : 1;

		//循环 width_*disparityTotal_次 （与pixelwiseCostRow_ 相同）
		for (int i = 0; i < width_*disparityTotal_; ++i) {

			//记录到 CostImage（行） 并且第一行的权重为其余两行的三倍
			costImageRow[i] += rowAggregatedCostCurrent[i] * scale;

		}

		//取图片下一行进行计算

		//取下一行导数
		leftSobelRow += widthStep_;
		rightSobelRow += widthStep_;
		
		//取下一行CensusTransform
		leftCensusRow += width_;
		rightCensusRow += width_;
	}
}

/*
* 计算行的Cost 代价聚集 聚集窗口半径 2
*/
void calcRowCosts(unsigned char*& leftSobelRow, int*& leftCensusRow,
	unsigned char*& rightSobelRow, int*& rightCensusRow,
	unsigned short* costImageRow)
{
	//widthStepCost width_*disparityTotal_ （与pixelwiseCostRow_ 相同）
	const int widthStepCost = width_*disparityTotal_;

	//__m128i 128字节共用体 ,使用寄存器加速
	const __m128i registerZero = _mm_setzero_si128();

	//循环height次 处理所有行的代价计算
	for (int y = 1; y < height_; ++y) {

		int addRowIndex = y + aggregationWindowRadius_;

		int addRowAggregatedCostIndex = std::min(addRowIndex, height_ - 1) % (aggregationWindowRadius_ * 2 + 2);

		unsigned short* addRowAggregatedCost = rowAggregatedCost_ + width_*disparityTotal_*addRowAggregatedCostIndex;

		//保证 y + 窗口半径 不超出图片高度
		if (addRowIndex < height_) {
			 
			//输入左右x导数,计算某行的梯度造成的Cost
			calcPixelwiseSAD(leftSobelRow, rightSobelRow);
	
			//输入左右censusTransform,计算某行海明距离得出Cost
			addPixelwiseHamming(leftCensusRow, rightCensusRow);

			//将addRowAggregatedCost初始化为0，长度为 disparityTotal_*sizeof(unsigned short)
			memset(addRowAggregatedCost, 0, disparityTotal_*sizeof(unsigned short));

			// x = 0 循环3次 主要完成边缘处的Cost计算 x = 0 1 2 对应 左图边缘三个像素
			for (int x = 0; x <= aggregationWindowRadius_; ++x) {

				// x = 0 时 scale = aggregationWindowRadius_ + 1 = 3 ,否则 x = 1
				int scale = x == 0 ? aggregationWindowRadius_ + 1 : 1;

				//循环256次
				for (int d = 0; d < disparityTotal_; ++d) {

					//计算addRowAggregatedCost第一行的值 使用pixelwiseCostRow_（记录了梯度和海明Cost）的x行的和 （前三行和为一行）并且第一行的权重为其余两行的三倍
					addRowAggregatedCost[d] += static_cast<unsigned short>(pixelwiseCostRow_[disparityTotal_*x + d] * scale);
				}
			}

			
			int subRowAggregatedCostIndex = std::max(y - aggregationWindowRadius_ - 1, 0) % (aggregationWindowRadius_ * 2 + 2);

			const unsigned short* subRowAggregatedCost = rowAggregatedCost_ + width_*disparityTotal_*subRowAggregatedCostIndex;
			
			const unsigned short* previousCostRow = costImageRow - widthStepCost;

			//循环宽度 - 1次 x = 1...width-1 完成Cost的窗口聚集
			for (int x = 1; x < width_; ++x) {
				
				//定义addPixelwiseCost 等于pixelwiseCostRow_加上 x + 窗口半径 乘上 disparityTotal_ ,到达右边缘时用width -1乘上disparityTotal_ （用于取pixelwiseCostRow_中 的 x + 窗口半径 行）
				const unsigned char* addPixelwiseCost = pixelwiseCostRow_
					+ std::min((x + aggregationWindowRadius_)*disparityTotal_, (width_ - 1)*disparityTotal_);

				//定义subPixelwiseCost 等于pixelwiseCostRow_加上 x - 窗口半径 - 1 乘上 disparityTotal_ ，当位于左边缘时取0 （用于取pixelwiseCostRow_中 的 x - 窗口半径 - 1 行）
				const unsigned char* subPixelwiseCost = pixelwiseCostRow_
					+ std::max((x - aggregationWindowRadius_ - 1)*disparityTotal_, 0);

				// 循环16次 （比边缘的256次减少了很多，并且使用寄存器加速）
				for (int d = 0; d < disparityTotal_; d += 16) {

					//获取 d处的addPixelwiseCost 放入寄存器中
					__m128i registerAddPixelwiseLow = _mm_load_si128(reinterpret_cast<const __m128i*>(addPixelwiseCost + d));

					//Interleaves the upper 8 signed or unsigned 8-bit integers in a with the upper 8 signed or unsigned 8-bit integers in b.
					__m128i registerAddPixelwiseHigh = _mm_unpackhi_epi8(registerAddPixelwiseLow, registerZero);
					//Interleaves the lower 8 signed or unsigned 8-bit integers in a with the lower 8 signed or unsigned 8-bit integers in b.
					registerAddPixelwiseLow = _mm_unpacklo_epi8(registerAddPixelwiseLow, registerZero);
					
					//获取 d处的subPixelwiseCost 放入寄存器中
					__m128i registerSubPixelwiseLow = _mm_load_si128(reinterpret_cast<const __m128i*>(subPixelwiseCost + d));

					//Interleaves the upper 8 signed or unsigned 8-bit integers in a with the upper 8 signed or unsigned 8-bit integers in b.
					__m128i registerSubPixelwiseHigh = _mm_unpackhi_epi8(registerSubPixelwiseLow, registerZero);
					//Interleaves the lower 8 signed or unsigned 8-bit integers in a with the lower 8 signed or unsigned 8-bit integers in b.
					registerSubPixelwiseLow = _mm_unpacklo_epi8(registerSubPixelwiseLow, registerZero);

					// 寄存器低位 Low 计算代价聚合
					__m128i registerAddAggregated = _mm_load_si128(reinterpret_cast<const __m128i*>(addRowAggregatedCost
						+ disparityTotal_*(x - 1) + d));

					registerAddAggregated = _mm_adds_epi16(_mm_subs_epi16(registerAddAggregated, registerSubPixelwiseLow),
						registerAddPixelwiseLow);

					__m128i registerCost = _mm_load_si128(reinterpret_cast<const __m128i*>(previousCostRow + disparityTotal_*x + d));
					registerCost = _mm_adds_epi16(_mm_subs_epi16(registerCost,
						_mm_load_si128(reinterpret_cast<const __m128i*>(subRowAggregatedCost + disparityTotal_*x + d))),
						registerAddAggregated);

					_mm_store_si128(reinterpret_cast<__m128i*>(addRowAggregatedCost + disparityTotal_*x + d), registerAddAggregated);

					//记录到 CostImage（行）
					_mm_store_si128(reinterpret_cast<__m128i*>(costImageRow + disparityTotal_*x + d), registerCost);

					// 寄存器高位 High 计算代价聚合
					registerAddAggregated = _mm_load_si128(reinterpret_cast<const __m128i*>(addRowAggregatedCost + disparityTotal_*(x - 1) + d + 8));
					
					registerAddAggregated = _mm_adds_epi16(_mm_subs_epi16(registerAddAggregated, registerSubPixelwiseHigh),
						registerAddPixelwiseHigh);

					registerCost = _mm_load_si128(reinterpret_cast<const __m128i*>(previousCostRow + disparityTotal_*x + d + 8));
					registerCost = _mm_adds_epi16(_mm_subs_epi16(registerCost,
						_mm_load_si128(reinterpret_cast<const __m128i*>(subRowAggregatedCost + disparityTotal_*x + d + 8))),
						registerAddAggregated);

					_mm_store_si128(reinterpret_cast<__m128i*>(addRowAggregatedCost + disparityTotal_*x + d + 8), registerAddAggregated);

					//记录到 CostImage（行）
					_mm_store_si128(reinterpret_cast<__m128i*>(costImageRow + disparityTotal_*x + d + 8), registerCost);

				}

			}

		}
		//取图片下一行进行计算

		//取下一行导数
		leftSobelRow += widthStep_;
		rightSobelRow += widthStep_;

		//取下一行CensusTransform
		leftCensusRow += width_;
		rightCensusRow += width_;

		//指向costImageRow的下一个存储位置
		costImageRow += widthStepCost;
	}

}

/*
* 计算极线方向的梯度造成的Cost 得到pixelwiseCostRow_ 每一行对应一高Width 宽 disparityTotal_ 的Cost图 ，分为 1-16 16-256 256 -width 三部分，每部分采用不同策略计算
*/
void calcPixelwiseSAD(const unsigned char* leftSobelRow, const unsigned char* rightSobelRow) {

	//计算右图第一行每个像素与其左右平均三者之间的极值，用于与左图像素计算Cost
	calcHalfPixelRight(rightSobelRow);

	//循环16次 处理左右图边缘
	for (int x = 0; x < 16; ++x) {

		//获取第一行x处的x方向导数值
		int leftCenterValue = leftSobelRow[x];

		// x = 0           leftHalfValue = centerValue 其余情况 leftHalfValue = centerValue与上个位置的centerValue的平均
		int leftHalfLeftValue = x > 0 ? (leftCenterValue + leftSobelRow[x - 1]) / 2 : leftCenterValue;

		//x = width_ - 1  rightHalfValue = centerValue 其余情况 rightHalfValue = centerValue与下个位置的centerValue的平均
		int leftHalfRightValue = x < width_ - 1 ? (leftCenterValue + leftSobelRow[x + 1]) / 2 : leftCenterValue;

		//取左平均与右平均与中间值的最小值
		int leftMinValue = std::min(leftHalfLeftValue, leftHalfRightValue);
		leftMinValue = std::min(leftMinValue, leftCenterValue);
		
		//取左平均与右平均与中间值的最大值
		int leftMaxValue = std::max(leftHalfLeftValue, leftHalfRightValue);
		leftMaxValue = std::max(leftMaxValue, leftCenterValue);

		//循环x+1次
		for (int d = 0; d <= x; ++d) {

			// 依次取右图右边缘（由于Sobel的时候翻转，对应左图左边缘）到对应点的所有像素的 左右中三者最大，最小，像素值 三值
			int rightCenterValue = rightSobelRow[width_ - 1 - x + d];
			int rightMinValue = halfPixelRightMin_[width_ - 1 - x + d];
			int rightMaxValue = halfPixelRightMax_[width_ - 1 - x + d];
			
			//LtoR 的Cost值为，0，左值-右最大值，右最小值-左值 中的最大值
			int costLtoR = std::max(0, leftCenterValue - rightMaxValue);
			costLtoR = std::max(costLtoR, rightMinValue - leftCenterValue);
			
			//RtoL 的Cost值为，0，右值-左最大值，左最小值-右值 中的最大值
			int costRtoL = std::max(0, rightCenterValue - leftMaxValue);
			costRtoL = std::max(costRtoL, leftMinValue - rightCenterValue);
			
			//CostValue 为LtoR 与 RtoL中的最小值
			int costValue = std::min(costLtoR, costRtoL);

			//记录Cost值 每个x 对应其与右图对应位置到边缘 x+1个点的Cost 记录到pixelwiseCostRow_ 的前16组 形如 [.     ,..    ,...    ,....    ].... （记录到对角线位置）
			pixelwiseCostRow_[disparityTotal_*x + d] = costValue;
		}

		//遍历上面循环未填充的上三角部分 即空余的 256 - x - 1 个位置 由对角线下个位置开始  
		for (int d = x + 1; d < disparityTotal_; ++d) {
			
			//前16组的空余位置填充每组Cost值的最后一个数值(左图x处像素与右图边缘像素的Cost值) 
			pixelwiseCostRow_[disparityTotal_*x + d] = pixelwiseCostRow_[disparityTotal_*x + d - 1];
	
		}
	}
	
	//循环256 - 16 次 处理 图片左边缘 到 距离边缘256 处
	for (int x = 16; x < disparityTotal_; ++x) {

		//获取第一行x处的x方向导数值
		int leftCenterValue = leftSobelRow[x];

		// x = 0           leftHalfValue = centerValue 其余情况 leftHalfValue = centerValue与上个位置的centerValue的平均     -----------------------------------------------------该处可以少一次判断
		int leftHalfLeftValue = x > 0 ? (leftCenterValue + leftSobelRow[x - 1]) / 2 : leftCenterValue;

		//x = width_ - 1  rightHalfValue = centerValue 其余情况 rightHalfValue = centerValue与下个位置的centerValue的平均
		int leftHalfRightValue = x < width_ - 1 ? (leftCenterValue + leftSobelRow[x + 1]) / 2 : leftCenterValue;

		//取左平均与右平均与中间值的最小值
		int leftMinValue = std::min(leftHalfLeftValue, leftHalfRightValue);
		leftMinValue = std::min(leftMinValue, leftCenterValue);
		
		//取左平均与右平均与中间值的最大值
		int leftMaxValue = std::max(leftHalfLeftValue, leftHalfRightValue);
		leftMaxValue = std::max(leftMaxValue, leftCenterValue);

		//__m128i 128字节共用体 ; _mm_set1_epi8 设置 16 integer值 ; 将最大最小像素值保存,使用寄存器加速
		__m128i registerLeftCenterValue = _mm_set1_epi8(static_cast<char>(leftCenterValue));
		__m128i registerLeftMinValue = _mm_set1_epi8(static_cast<char>(leftMinValue));
		__m128i registerLeftMaxValue = _mm_set1_epi8(static_cast<char>(leftMaxValue));

		
		//x每增大256循环一次，由于disparityTotal_ = 256 所以这里只进入一次 d = 0
		for (int d = 0; d < x / 16; d += 16) {

			//_mm_loadu_si128 加载 128 位值， 取右图 对应点的 左右中三者最大，最小，像素值 三值 使用寄存器加速
			__m128i registerRightCenterValue = _mm_loadu_si128(reinterpret_cast<const __m128i*>(rightSobelRow + width_ - 1 - x + d));
			__m128i registerRightMinValue = _mm_loadu_si128(reinterpret_cast<const __m128i*>(halfPixelRightMin_ + width_ - 1 - x + d));
			__m128i registerRightMaxValue = _mm_loadu_si128(reinterpret_cast<const __m128i*>(halfPixelRightMax_ + width_ - 1 - x + d));

			//LtoR 的Cost值为，左值-右最大值，右最小值-左值 中的最大值
			__m128i registerCostLtoR = _mm_max_epu8(_mm_subs_epu8(registerLeftCenterValue, registerRightMaxValue),
				_mm_subs_epu8(registerRightMinValue, registerLeftCenterValue));

			//RtoL 的Cost值为，右值-左最大值，左最小值-右值 中的最大值
			__m128i registerCostRtoL = _mm_max_epu8(_mm_subs_epu8(registerRightCenterValue, registerLeftMaxValue),
				_mm_subs_epu8(registerLeftMinValue, registerRightCenterValue));

			//CostValue 为LtoR 与 RtoL中的最小值
			__m128i registerCost = _mm_min_epu8(registerCostLtoR, registerCostRtoL);

			//记录Cost值 每个x 与其右图对应点的Cost 记录到pixelwiseCostRow_ 的 16 - 256 组的每一组的第一个位置 （d = 0）
			_mm_store_si128(reinterpret_cast<__m128i*>(pixelwiseCostRow_ + disparityTotal_*x + d), registerCost);
		}

		//d初始取值为1~15 循环 x - x/16次 
		for (int d = x / 16; d <= x; ++d) {
			
			//依次取对应点偏移x/16个点处 到 右图右边缘（由于Sobel的时候翻转，对应左图左边缘）所有像素的 左右中三者最大，最小，像素值 三值
			int rightCenterValue = rightSobelRow[width_ - 1 - x + d];
			int rightMinValue = halfPixelRightMin_[width_ - 1 - x + d];
			int rightMaxValue = halfPixelRightMax_[width_ - 1 - x + d];

			//LtoR 的Cost值为，0，左值-右最大值，右最小值-左值 中的最大值
			int costLtoR = std::max(0, leftCenterValue - rightMaxValue);
			costLtoR = std::max(costLtoR, rightMinValue - leftCenterValue);

			//RtoL 的Cost值为，0，右值-左最大值，左最小值-右值 中的最大值
			int costRtoL = std::max(0, rightCenterValue - leftMaxValue);
			costRtoL = std::max(costRtoL, leftMinValue - rightCenterValue);

			// CostValue 为LtoR 与 RtoL中的最小值
			int costValue = std::min(costLtoR, costRtoL);

			//记录Cost值 每个x 与其从对应右图点偏移x/16个点处到边缘的Cost 记录到pixelwiseCostRow_ 的 16 - 256 组 每一组从第2到第16个位置（x/16）开始记录 记录到对角线位置
			pixelwiseCostRow_[disparityTotal_*x + d] = costValue;
		}

		//遍历上面循环未填充的上三角部分 即空余的 256 - x - 1 个位置 由对角线下个位置开始 
		for (int d = x + 1; d < disparityTotal_; ++d) {

			//16组-256组的空余位置填充每组Cost值的最后一个数值(左图x处像素与右图边缘像素的Cost值) 
			pixelwiseCostRow_[disparityTotal_*x + d] = pixelwiseCostRow_[disparityTotal_*x + d - 1];
		}

	}

	//循环width - 256次 处理 距离左边缘256处 到 右边缘
	for (int x = disparityTotal_; x < width_; ++x) {

		//获取第一行x处的x方向导数值
		int leftCenterValue = leftSobelRow[x];

		// x = 0           leftHalfValue = centerValue 其余情况 leftHalfValue = centerValue与上个位置的centerValue的平均     -----------------------------------------------------该处可以少一次判断
		int leftHalfLeftValue = x > 0 ? (leftCenterValue + leftSobelRow[x - 1]) / 2 : leftCenterValue;

		//x = width_ - 1  rightHalfValue = centerValue 其余情况 rightHalfValue = centerValue与下个位置的centerValue的平均
		int leftHalfRightValue = x < width_ - 1 ? (leftCenterValue + leftSobelRow[x + 1]) / 2 : leftCenterValue;

		// 取左平均与右平均与中间值的最小值
		int leftMinValue = std::min(leftHalfLeftValue, leftHalfRightValue);
		leftMinValue = std::min(leftMinValue, leftCenterValue);

		//取左平均与右平均与中间值的最大值
		int leftMaxValue = std::max(leftHalfLeftValue, leftHalfRightValue);
		leftMaxValue = std::max(leftMaxValue, leftCenterValue);

		//__m128i 128字节共用体 ; _mm_set1_epi8 设置 16 integer值 ; 将最大最小像素值保存,使用寄存器加速
		__m128i registerLeftCenterValue = _mm_set1_epi8(static_cast<char>(leftCenterValue));
		__m128i registerLeftMinValue = _mm_set1_epi8(static_cast<char>(leftMinValue));
		__m128i registerLeftMaxValue = _mm_set1_epi8(static_cast<char>(leftMaxValue));

		int i = 1;

		//d每次递增16 直到256 共循环16次
		for (int d = 0; d < disparityTotal_; d += 16) {

			//_mm_loadu_si128 加载 128 位值， 取右图 从对应点到向边缘每间隔16取16个点 的 左右中三者最大，最小，像素值 三值 使用寄存器加速
			__m128i registerRightCenterValue = _mm_loadu_si128(reinterpret_cast<const __m128i*>(rightSobelRow + width_ - 1 - x + d));
			__m128i registerRightMinValue = _mm_loadu_si128(reinterpret_cast<const __m128i*>(halfPixelRightMin_ + width_ - 1 - x + d));
			__m128i registerRightMaxValue = _mm_loadu_si128(reinterpret_cast<const __m128i*>(halfPixelRightMax_ + width_ - 1 - x + d));

			//LtoR 的Cost值为，左值-右最大值，右最小值-左值 中的最大值
			__m128i registerCostLtoR = _mm_max_epu8(_mm_subs_epu8(registerLeftCenterValue, registerRightMaxValue),
				_mm_subs_epu8(registerRightMinValue, registerLeftCenterValue));

			//RtoL 的Cost值为，右值-左最大值，左最小值-右值 中的最大值
			__m128i registerCostRtoL = _mm_max_epu8(_mm_subs_epu8(registerRightCenterValue, registerLeftMaxValue),
				_mm_subs_epu8(registerLeftMinValue, registerRightCenterValue));

			//CostValue 为LtoR 与 RtoL中的最小值
			__m128i registerCost = _mm_min_epu8(registerCostLtoR, registerCostRtoL);

			//记录Cost值 每个x 与其右图对应点的Cost 记录到pixelwiseCostRow_ 的 256 - width 组的每一组的第1,17,33..个位置 （d = 0 16 32 ..） (16个值)
			_mm_store_si128(reinterpret_cast<__m128i*>(pixelwiseCostRow_ + disparityTotal_*x + d), registerCost);
		}
	}
}

/*
* 计算一行中每个像素与其左平均，右平均三者之间的极值 用于与左图像素计算Cost 
*/
void calcHalfPixelRight(const unsigned char* rightSobelRow) {

	//遍历第一行
	for (int x = 0; x < width_; ++x) {

		//依次取第一行x导数值
		int centerValue = rightSobelRow[x];

		// x = 0           leftHalfValue = centerValue 其余情况 leftHalfValue = centerValue与上个位置的centerValue的平均
		int leftHalfValue = x > 0 ? (centerValue + rightSobelRow[x - 1]) / 2 : centerValue;

		//x = width_ - 1  rightHalfValue = centerValue 其余情况 rightHalfValue = centerValue与下个位置的centerValue的平均
		int rightHalfValue = x < width_ - 1 ? (centerValue + rightSobelRow[x + 1]) / 2 : centerValue;

		//取左平均与右平均与中间值的最小值
		int minValue = std::min(leftHalfValue, rightHalfValue);
		minValue = std::min(minValue, centerValue);

		//取左平均与右平均与中间值的最大值
		int maxValue = std::max(leftHalfValue, rightHalfValue);
		maxValue = std::max(maxValue, centerValue);

		//记录最大最小值
		halfPixelRightMin_[x] = minValue;
		halfPixelRightMax_[x] = maxValue;
	}
}

/*
* 输入左右censusTransform,计算海明距离得出Census Transform 项的Cost （每一行）分为 1-256 256-width 两部分，每部分采用不同策略计算
*/
void addPixelwiseHamming(const int* leftCensusRow, const int* rightCensusRow) {

	//循环 256 次 
	for (int x = 0; x < disparityTotal_; ++x) {

		// 从CensusTransform后的左图中依次取CencusCode
		int leftCencusCode = leftCensusRow[x];

		// 海明距离
		int hammingDistance = 0;

		// 循环x+1次 
		for (int d = 0; d <= x; ++d) {

			// 依次取右图对应点到边缘的所有像素的CencusCode
			int rightCensusCode = rightCensusRow[x - d];

			// 计算海明距离
			hammingDistance = static_cast<int>(_mm_popcnt_u32(static_cast<unsigned int>(leftCencusCode^rightCensusCode)));

			// 记录海明距离 与权重相乘 加到pixelwiseCostRow_上 作为cost
			pixelwiseCostRow_[disparityTotal_*x + d] += static_cast<unsigned char>(hammingDistance*censusWeightFactor_);

		}

		//计算海明距离（左图x处像素与右图边缘像素的Cost值，也是填充部分的最后一个值） 
		hammingDistance = static_cast<unsigned char>(hammingDistance*censusWeightFactor_);

		//遍历上面循环未填充的上三角部分 即空余的 256 - x - 1 个位置
		for (int d = x + 1; d < disparityTotal_; ++d) {

			//前256组的上三角部分 与 左图x处像素与右图边缘像素的Cost值 相加 作为新的Cost值 
			pixelwiseCostRow_[disparityTotal_*x + d] += hammingDistance;

		}
	}

	//循环 width - 256 次 
	for (int x = disparityTotal_; x < width_; ++x) {

		//从CensusTransform后的左图中依次取CencusCode （256 - width）
		int leftCencusCode = leftCensusRow[x];
		
		// 循环256次 
		for (int d = 0; d < disparityTotal_; ++d) {

			// 依次取右图中从对应点到向边缘偏移256个点的所有像素的CencusCode
			int rightCensusCode = rightCensusRow[x - d];
			
			//计算海明距离
			int hammingDistance = static_cast<int>(_mm_popcnt_u32(static_cast<unsigned int>(leftCencusCode^rightCensusCode)));

			// 记录海明距离 与权重相乘 加到pixelwiseCostRow_上 作为cost (全填充)
			pixelwiseCostRow_[disparityTotal_*x + d] += static_cast<unsigned char>(hammingDistance*censusWeightFactor_);
		}
	
	}
}

/*
* 执行SGM 使得Cost最小 输入CostImage_ 输出DisparityImage 左右各一次
*/
void performSGM(unsigned short* costImage, unsigned short* disparityImage) {

	std::cout << "    SGM_performSGM_start" << std::endl;

	//定义CostImage的最大值为short的最大值
	const short costMax = SHRT_MAX;

	//widthStepCost width_*disparityTotal_ （与pixelwiseCostRow_ 相同）
	int widthStepCostImage = width_*disparityTotal_;

	//指向定义好的sgmBuffer_
	short* costSums = sgmBuffer_;

	//将sgmBuffer_清零
	memset(costSums, 0, costSumBufferSize_*sizeof(short));

	//pathRowBufferTotal_ = 2 定义指向指针数据的指针变量 （二维）
	short** pathCosts = new short*[pathRowBufferTotal_];
	short** pathMinCosts = new short*[pathRowBufferTotal_];

	//定义迭代次数 (实验发现迭代次数越高 边界越细腻，但是黑色区域也越多)
	const int processPassTotal = 2;

	//循环2次 得出costSums
	for (int processPassCount = 0; processPassCount < processPassTotal; ++processPassCount) {

		std::cout << "performSGM_epoch: " << processPassCount << std::endl;

		//定义 XY方向 的迭代步长 开始与结束
		int startX, endX, stepX;
		int startY, endY, stepY;

		//第一次迭代设置开始x为0 直到x = width结束 步长为1 第二次反向从照片右侧向左扫描                  ------------------------------------------------- 多次迭代: processPassCount %2 == 0 效果会更好一点
		if (processPassCount == 0) {
			startX = 0; endX = width_; stepX = 1;
			startY = 0; endY = height_; stepY = 1;
		}
		else {
			startX = width_ - 1; endX = -1; stepX = -1;
			startY = height_ - 1; endY = -1; stepY = -1;
		}

		//循环2次 为pathCosts[i]和pathMinCosts[i] 分配内存
		for (int i = 0; i < pathRowBufferTotal_; ++i) {

			pathCosts[i] = costSums + costSumBufferSize_ + pathCostBufferSize_*i + pathDisparitySize_ + 8;

			//清零
			memset(pathCosts[i] - pathDisparitySize_ - 8, 0, pathCostBufferSize_*sizeof(short));
			
			pathMinCosts[i] = costSums + costSumBufferSize_ + pathCostBufferSize_*pathRowBufferTotal_
				+ pathMinCostBufferSize_*i + pathTotal_ * 2;
			
			//清零
			memset(pathMinCosts[i] - pathTotal_, 0, pathMinCostBufferSize_*sizeof(short));

		}

		//y方向进行循环遍历
		for (int y = startY; y != endY; y += stepY) {

			//取第y行图像的CostImage
			unsigned short* pixelCostRow = costImage + widthStepCostImage*y;
			
			//取costSum中的第y行
			short* costSumRow = costSums + costSumBufferRowSize_*y;

			//每一行将Costs[0]交换给Costs[1]后进行清零
			memset(pathCosts[0] - pathDisparitySize_ - 8, 0, pathDisparitySize_*sizeof(short));
			memset(pathCosts[0] + width_*pathDisparitySize_ - 8, 0, pathDisparitySize_*sizeof(short));
			memset(pathMinCosts[0] - pathTotal_, 0, pathTotal_*sizeof(short));
			memset(pathMinCosts[0] + width_*pathTotal_, 0, pathTotal_*sizeof(short));

			//x方向进行循环遍历
			for (int x = startX; x != endX; x += stepX) {

				//pathTotal_ = 8
				int pathMinX = x*pathTotal_;

				//disparitySize_ = disparityTotal_ + 16
				int pathX = pathMinX*disparitySize_;

				//smoothnessPenaltyLarge_ = 1600
				int previousPathMin0 = pathMinCosts[0][pathMinX - stepX*pathTotal_] + smoothnessPenaltyLarge_;
				int previousPathMin2 = pathMinCosts[1][pathMinX + 2] + smoothnessPenaltyLarge_;

				short* previousPathCosts0 = pathCosts[0] + pathX - stepX*pathDisparitySize_;
				short* previousPathCosts2 = pathCosts[1] + pathX + disparitySize_ * 2;

				//costMax = SHRT_MAX;
				previousPathCosts0[-1] = previousPathCosts0[disparityTotal_] = costMax;
				previousPathCosts2[-1] = previousPathCosts2[disparityTotal_] = costMax;

				//当前x的路径Cost
				short* pathCostCurrent = pathCosts[0] + pathX;

				//取costImage中第y行的第x像素位置
				const unsigned short* pixelCostCurrent = pixelCostRow + disparityTotal_*x;

				//取costSum中的第y行第x个像素
				short* costSumCurrent = costSumRow + disparityTotal_*x;

				//定义寄存器regPenaltySmall存储惩罚值 smoothnessPenaltySmall_ = 100 
				__m128i regPenaltySmall = _mm_set1_epi16(static_cast<short>(smoothnessPenaltySmall_));

				//定义寄存器regPathMin0, regPathMin2
				__m128i regPathMin0, regPathMin2;

				regPathMin0 = _mm_set1_epi16(static_cast<short>(previousPathMin0));
				regPathMin2 = _mm_set1_epi16(static_cast<short>(previousPathMin2));
				
				//定义寄存器regNewPathMin costMax = SHRT_MAX;
				__m128i regNewPathMin = _mm_set1_epi16(costMax);

				//循环32次 间隔8进行操作
				for (int d = 0; d < disparityTotal_; d += 8) {

					//取y行x处与右图中距离对应点d距离处的Cost
					__m128i regPixelCost = _mm_load_si128(reinterpret_cast<const __m128i*>(pixelCostCurrent + d));

					//定义寄存器regPathCost0, regPathCost2;
					__m128i regPathCost0, regPathCost2;

					/*
					 *动态规划思想：p的代价想要最小，那么前提必须是邻域内的点q的代价最小，q想要代价最小，那么必须保证q的领域点m的代价最小，如此传递下去
					 *每一个点的代价聚合值是:
					 *“当前代价+min（路径相邻点的当前视差代价聚合值 + P1，路径相邻点的视差差值为1的代价聚合值 + P1，路径相邻点的视差插值大于1的最小代价聚合值 + P2）- 路径相邻点的视差插值大于1的最小代价聚合值 "
					 *其实就好比最小代价的蔓延，当前代价聚合值由当前代价和路径上一点的加了惩罚的最小代价聚合值所决定（最后那一项纯粹是为了防止数字过大）
					 */
					regPathCost0 = _mm_load_si128(reinterpret_cast<const __m128i*>(previousPathCosts0 + d));
					regPathCost2 = _mm_load_si128(reinterpret_cast<const __m128i*>(previousPathCosts2 + d));


					regPathCost0 = _mm_min_epi16(regPathCost0,
						_mm_adds_epi16(_mm_loadu_si128(reinterpret_cast<const __m128i*>(previousPathCosts0 + d - 1)),
						regPenaltySmall));

					regPathCost0 = _mm_min_epi16(regPathCost0,
						_mm_adds_epi16(_mm_loadu_si128(reinterpret_cast<const __m128i*>(previousPathCosts0 + d + 1)),
						regPenaltySmall));

					regPathCost2 = _mm_min_epi16(regPathCost2,
						_mm_adds_epi16(_mm_loadu_si128(reinterpret_cast<const __m128i*>(previousPathCosts2 + d - 1)),
						regPenaltySmall));

					regPathCost2 = _mm_min_epi16(regPathCost2,
						_mm_adds_epi16(_mm_loadu_si128(reinterpret_cast<const __m128i*>(previousPathCosts2 + d + 1)),
						regPenaltySmall));

					regPathCost0 = _mm_min_epi16(regPathCost0, regPathMin0);
					regPathCost0 = _mm_adds_epi16(_mm_subs_epi16(regPathCost0, regPathMin0), regPixelCost);

					regPathCost2 = _mm_min_epi16(regPathCost2, regPathMin2);
					regPathCost2 = _mm_adds_epi16(_mm_subs_epi16(regPathCost2, regPathMin2), regPixelCost);

					//存入pathCostCurrent
					_mm_store_si128(reinterpret_cast<__m128i*>(pathCostCurrent + d), regPathCost0);
					_mm_store_si128(reinterpret_cast<__m128i*>(pathCostCurrent + d + disparitySize_ * 2), regPathCost2);


					__m128i regMin02 = _mm_min_epi16(_mm_unpacklo_epi16(regPathCost0, regPathCost2),
						_mm_unpackhi_epi16(regPathCost0, regPathCost2));

					regMin02 = _mm_min_epi16(_mm_unpacklo_epi16(regMin02, regMin02),
						_mm_unpackhi_epi16(regMin02, regMin02));

					regNewPathMin = _mm_min_epi16(regNewPathMin, regMin02);

					__m128i regCostSum = _mm_load_si128(reinterpret_cast<const __m128i*>(costSumCurrent + d));

					regCostSum = _mm_adds_epi16(regCostSum, regPathCost0);
					regCostSum = _mm_adds_epi16(regCostSum, regPathCost2);

					//将Cost记录到costSum
					_mm_store_si128(reinterpret_cast<__m128i*>(costSumCurrent + d), regCostSum);
				}


				regNewPathMin = _mm_min_epi16(regNewPathMin, _mm_srli_si128(regNewPathMin, 8));
				_mm_storel_epi64(reinterpret_cast<__m128i*>(&pathMinCosts[0][pathMinX]), regNewPathMin);

			}

			//最后一次迭代
			if (processPassCount == processPassTotal - 1) {

				//获取disparityImage 的第y行
				unsigned short* disparityRow = disparityImage + width_*y;

				//遍历该行的每个像素
				for (int x = 0; x < width_; ++x) {

					//取costSum中的第y行第x个像素 （长度为256的costmap）
					short* costSumCurrent = costSumRow + disparityTotal_*x;
					
					//定义bestSumCost 为costSumCurrent[0 - disparityTotal_]中的最小值
					int bestSumCost = costSumCurrent[0];

					//定义bestDisparity 为bestSumCost的位置d
					int bestDisparity = 0;

					//循环disparityTotal_次 求bestSumCost以及此时的bestDisparity d
					for (int d = 1; d < disparityTotal_; ++d) {
						if (costSumCurrent[d] < bestSumCost) {
							bestSumCost = costSumCurrent[d];
							bestDisparity = d;
						}
					}

					//如果bestDisparity不是边缘值（0/255）则利用左右的Cost进行进一步的计算 如果是边缘值 则确定bestDispart
					if (bestDisparity > 0 && bestDisparity < disparityTotal_ - 1) {
					//if (false) { //--------------------------------------------------------------------------------------------------------------------------------------实验少了一些黑色区域

						//取bestpart以及其左右的Cost值
						int centerCostValue = costSumCurrent[bestDisparity];
						int leftCostValue = costSumCurrent[bestDisparity - 1];
						int rightCostValue = costSumCurrent[bestDisparity + 1];

						//如果右小于左
						if (rightCostValue < leftCostValue) {

							//计算bestDispart ----------------------------------------------------------------------------------------------------------------------------不是很明白为什么,会提高左右一致性检验的效果
							bestDisparity = static_cast<int>(bestDisparity*disparityFactor_
								+ static_cast<double>(rightCostValue - leftCostValue) / (centerCostValue - leftCostValue) / 2.0*disparityFactor_ + 0.5);

						}
						//如过左小于等于右
						else {

							//计算bestDispart
							bestDisparity = static_cast<int>(bestDisparity*disparityFactor_
								+ static_cast<double>(rightCostValue - leftCostValue) / (centerCostValue - rightCostValue) / 2.0*disparityFactor_ + 0.5);

						}
					}
					else {

						//确定bestDispart 进行缩放
						bestDisparity = static_cast<int>(bestDisparity*disparityFactor_);
					}

					//将bestDisparity记录到disparityImage
					disparityRow[x] = static_cast<unsigned short>(bestDisparity);
				}
			}

			//将01位置的值相互交换 使得0位置在下次迭代时清零使用 1位置保存结果 ----------------------------------------------------------------------------------为什么交换？多次迭代为什么值不变？
			std::swap(pathCosts[0], pathCosts[1]);
			std::swap(pathMinCosts[0], pathMinCosts[1]);


		}

	}

	//释放内存
	delete[] pathCosts;
	delete[] pathMinCosts;

	//使用speckleFilter（speckle滤波器）去除一系列斑点值
	//speckleFilter(100, static_cast<int>(2 * disparityFactor_), disparityImage);  //----------------------------------------------------------------------------------------0值
}

/*
* speckle滤波器 去除斑点噪声
*/
void speckleFilter(const int maxSpeckleSize, const int maxDifference, unsigned short* image){
	std::vector<int> labels(width_*height_, 0);
	std::vector<bool> regionTypes(1);
	regionTypes[0] = false;

	int currentLabelIndex = 0;

	for (int y = 0; y < height_; ++y) {
		for (int x = 0; x < width_; ++x) {
			int pixelIndex = width_*y + x;
			if (image[width_*y + x] != 0) {
				if (labels[pixelIndex] > 0) {
					if (regionTypes[labels[pixelIndex]]) {
						image[width_*y + x] = 0;
					}
				}
				else {
					std::stack<int> wavefrontIndices;
					wavefrontIndices.push(pixelIndex);
					++currentLabelIndex;
					regionTypes.push_back(false);
					int regionPixelTotal = 0;
					labels[pixelIndex] = currentLabelIndex;

					while (!wavefrontIndices.empty()) {
						int currentPixelIndex = wavefrontIndices.top();
						wavefrontIndices.pop();
						int currentX = currentPixelIndex%width_;
						int currentY = currentPixelIndex / width_;
						++regionPixelTotal;
						unsigned short pixelValue = image[width_*currentY + currentX];

						if (currentX < width_ - 1 && labels[currentPixelIndex + 1] == 0
							&& image[width_*currentY + currentX + 1] != 0
							&& std::abs(pixelValue - image[width_*currentY + currentX + 1]) <= maxDifference)
						{
							labels[currentPixelIndex + 1] = currentLabelIndex;
							wavefrontIndices.push(currentPixelIndex + 1);
						}

						if (currentX > 0 && labels[currentPixelIndex - 1] == 0
							&& image[width_*currentY + currentX - 1] != 0
							&& std::abs(pixelValue - image[width_*currentY + currentX - 1]) <= maxDifference)
						{
							labels[currentPixelIndex - 1] = currentLabelIndex;
							wavefrontIndices.push(currentPixelIndex - 1);
						}

						if (currentY < height_ - 1 && labels[currentPixelIndex + width_] == 0
							&& image[width_*(currentY + 1) + currentX] != 0
							&& std::abs(pixelValue - image[width_*(currentY + 1) + currentX]) <= maxDifference)
						{
							labels[currentPixelIndex + width_] = currentLabelIndex;
							wavefrontIndices.push(currentPixelIndex + width_);
						}

						if (currentY > 0 && labels[currentPixelIndex - width_] == 0
							&& image[width_*(currentY - 1) + currentX] != 0
							&& std::abs(pixelValue - image[width_*(currentY - 1) + currentX]) <= maxDifference)
						{
							labels[currentPixelIndex - width_] = currentLabelIndex;
							wavefrontIndices.push(currentPixelIndex - width_);
						}
					}

					if (regionPixelTotal <= maxSpeckleSize) {
						regionTypes[currentLabelIndex] = true;
						image[width_*y + x] = 0;
					}
				}
			}
		}
	}
}

/*
* 保持左右一致 输入左右DisparityImage，输出 (反应在图片上去除了一些细小的噪声)
*/
void enforceLeftRightConsistency(unsigned short* leftDisparityImage, unsigned short* rightDisparityImage){

	// 检查左视差图 循环遍历
	for (int y = 0; y < height_; ++y) {
		for (int x = 0; x < width_; ++x) {

			//值为0 跳出本次循环 
			if (leftDisparityImage[width_*y + x] == 0) continue;

			//否则计算左视差图(x,y)处灰度值
			int leftDisparityValue = static_cast<int>(static_cast<double>(leftDisparityImage[width_*y + x]) / disparityFactor_ + 0.5);

			//视差大于此处到边缘的距离 则将视差赋值为0 并跳出 (反应为结果左侧的部分的一系列0值)   -----------------------------------------------------------------------------------0值
			if (x - leftDisparityValue < 0) {
				
				//leftDisparityImage[width_*y + x] = 0;

				continue;
			}

			//使用视差计算对应点在右图中的视差rightDisparityValue
			int rightDisparityValue = static_cast<int>(static_cast<double>(rightDisparityImage[width_*y + x - leftDisparityValue]) / disparityFactor_ + 0.5);

			//如果rightDisparityValue为0（不存在对应点）/ 或者与对应点在右视差图中的视差差距大于阈值，则设置为0  --------------------------------------------------------------------0值
			if (rightDisparityValue == 0 || abs(leftDisparityValue - rightDisparityValue) > consistencyThreshold_) {

				//leftDisparityImage[width_*y + x] = 0;

			}
		}
	}

	// 检查右视差图 循环遍历
	for (int y = 0; y < height_; ++y) {
		for (int x = 0; x < width_; ++x) {

			//值为0 跳出本次循环 
			if (rightDisparityImage[width_*y + x] == 0)  continue;

			//否则计算右视差图(x, y)处灰度值
			int rightDisparityValue = static_cast<int>(static_cast<double>(rightDisparityImage[width_*y + x]) / disparityFactor_ + 0.5);

			//视差大于此处到边缘的距离 则将视差赋值为0 并跳出 (反应为结果左侧的部分的一系列0值)   -----------------------------------------------------------------------------------0值
			if (x + rightDisparityValue >= width_) {
				
				//rightDisparityImage[width_*y + x] = 0; 

				continue;
			}

			//使用视差计算对应点在左图中的视差rightDisparityValue
			int leftDisparityValue = static_cast<int>(static_cast<double>(leftDisparityImage[width_*y + x + rightDisparityValue]) / disparityFactor_ + 0.5);

			//如果rightDisparityValue为0（不存在对应点）/ 或者与对应点在右视差图中的视差差距大于阈值，则设置为0  --------------------------------------------------------------------0值
			if (leftDisparityValue == 0 || abs(rightDisparityValue - leftDisparityValue) > consistencyThreshold_) {

				//rightDisparityImage[width_*y + x] = 0;

			}
		}
	}
}

/*
* 保持Char*类型压平图片
*/
void SaveFlattenCharImage(unsigned char* FlattenCharImage, std::string outputDisparityImageFilename){

	png::image<png::gray_pixel> png;

	png.resize(width_, height_);

	//循环将像素写入图片
	for (int y = 0; y < height_; ++y) {

		for (int x = 0; x < width_; ++x) {

			if (FlattenCharImage[width_*y + x] <= 0.0 || FlattenCharImage[width_*y + x] > 255.0) {
				png.set_pixel(x, y, 0);
			}
			else {
				png.set_pixel(x, y, static_cast<unsigned short>(FlattenCharImage[width_*y + x]));
			}

		}
	}

	//保存图片
	png.write(outputDisparityImageFilename);

}

/*
* SGM释放内存
*/
void freeDataBuffer(){
	_mm_free(leftCostImage_);
	_mm_free(rightCostImage_);
	_mm_free(pixelwiseCostRow_);
	_mm_free(rowAggregatedCost_);
	_mm_free(halfPixelRightMin_);
	_mm_free(halfPixelRightMax_);
	_mm_free(sgmBuffer_);
}

