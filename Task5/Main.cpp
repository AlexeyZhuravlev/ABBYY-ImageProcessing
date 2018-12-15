#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "Sift.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <set>

class CPointsCompare {
public:
	bool operator()(const cv::Point& first, const cv::Point& second) const
	{
		return first.x < second.x || first.x == second.x && first.y < second.y;
	}
};

// Calculates forward dft transform of input image as a complex-valued matrix
cv::Mat forwardDftTransform(cv::Mat sourceImage) 
{
	cv::Mat planes[] = { cv::Mat_<float>(sourceImage), cv::Mat::zeros(sourceImage.size(), CV_32F) };
	cv::Mat complexI;
	merge(planes, 2, complexI);

	cv::dft(complexI, complexI);

	return complexI;
}

// Splits the complex-valued matrix into pair of it's Re part matrix and Im part matrix
std::pair<cv::Mat, cv::Mat> splitComplex(cv::Mat complex)
{
	cv::Mat first, second;
	cv::Mat planes[] = { cv::Mat::zeros(complex.size(), CV_32F), cv::Mat::zeros(complex.size(), CV_32F) };

	cv::split( complex, planes );

	return std::make_pair( planes[0], planes[1] );
}

// Calculates images correlation in fourier space: A x B* / |A x B*|
// Returns complex-valued result
cv::Mat calcFourierCorrelation(cv::Mat first, cv::Mat second)
{
	cv::Mat dftFirstRe, dftFirstIm, dftSecondRe, dftSecondIm;
	std::tie(dftFirstRe, dftFirstIm) = splitComplex(first);
	std::tie(dftSecondRe, dftSecondIm) = splitComplex(second);

	cv::Mat correlationRe = dftFirstRe.mul(dftSecondRe) + dftFirstIm.mul(dftSecondIm);
	cv::Mat correlationIm = dftFirstIm.mul(dftSecondRe) - dftFirstRe.mul(dftSecondIm);
	cv::Mat correlationMagnitude = correlationRe;
	cv::magnitude(correlationRe, correlationIm, correlationMagnitude);
	correlationRe /= correlationMagnitude;
	correlationIm /= correlationMagnitude;

	cv::Mat complexCorrelation;
	cv::Mat planes[] = { correlationRe, correlationIm };
	cv::merge(planes, 2, complexCorrelation);

	return complexCorrelation;
}

// Implementation of phase-correlation motion estimation algorithm
cv::Point phaseCorrelationMotionEstimation(cv::Mat first, cv::Mat second)
{
	// Calculate fourier tranformation of input images
	cv::Mat dftFirst = forwardDftTransform(first);
	cv::Mat dftSecond = forwardDftTransform(second);

	// Calculate correlation of two images in fourier space
	cv::Mat complexCorrelation = calcFourierCorrelation(dftFirst, dftSecond);
	// Get original image correlation by inferse fouier tranform
	cv::Mat inverseCorrelation = cv::Mat::zeros(complexCorrelation.size(), CV_32F);
	cv::idft(complexCorrelation, inverseCorrelation, cv::DFT_REAL_OUTPUT);

	// Find location of maximal value in correlation matrix
	cv::Point maxIdx;
	cv::minMaxLoc(inverseCorrelation, NULL, NULL, NULL, &maxIdx);

	// Obtain the vector of motion from  maximal value location
	int r = inverseCorrelation.rows;
	int c = inverseCorrelation.cols;
	int resX = maxIdx.x > c / 2 ? c - maxIdx.x : -maxIdx.x;
	int resY = maxIdx.y > r / 2 ? r - maxIdx.y : -maxIdx.y;

	return cv::Point(resX, resY);
}

std::vector<cv::Mat> readInputImages(int nImages)
{
	std::vector<cv::Mat> result;
	for (int i = 1; i <= nImages; i++) {
		std::stringstream ss;
		ss << "Data/" << std::setw(2) << std::setfill('0') << i << ".tif";
		cv::Mat image = cv::imread(ss.str(), CV_LOAD_IMAGE_GRAYSCALE);
		result.push_back(image);
	}
	return result;
}

std::vector<cv::Point> calcMoves(const std::vector<cv::Mat>& images)
{
	std::vector<cv::Point> moves;
	cv::Point accMove(0, 0);
	moves.push_back(accMove);
	for (int i = 1; i < images.size(); i++) {
		cv::Point locMove = phaseCorrelationMotionEstimation(images[i - 1], images[i]);
		accMove = accMove + locMove;
		moves.push_back(accMove);
	}
	return moves;
}

struct IKeyPointDetector {
	virtual std::vector<cv::Point> DetectKeyPoints(cv::Mat image) const = 0;
};

class CHarrisKeyPointDetector : public IKeyPointDetector {
public:
	CHarrisKeyPointDetector(int _blockSize, int _kSize, double _k, float _threshold) :
		blockSize(_blockSize),
		kSize(_kSize),
		k(_k),
		threshold( _threshold ) {}

	virtual std::vector<cv::Point> DetectKeyPoints(cv::Mat image) const override;
private:
	int blockSize;
	int kSize;
	double k;

	float threshold;
};

class CSiftKeyPointDetector : public IKeyPointDetector {
public:
	CSiftKeyPointDetector();
	virtual std::vector<cv::Point> DetectKeyPoints(cv::Mat image) const override;
private:
	cv::Ptr<cv::xfeatures2d::SIFT> sift;
};

class COrbKeyPointDetector : public IKeyPointDetector {
public:
	COrbKeyPointDetector();
	virtual std::vector<cv::Point> DetectKeyPoints(cv::Mat image) const override;
private:
	cv::Ptr<cv::ORB> orb;
};

std::vector<cv::Point> CHarrisKeyPointDetector::DetectKeyPoints(cv::Mat image) const
{
	cv::Mat result = cv::Mat::zeros(image.size(), CV_32F);
	cv::cornerHarris(image, result, blockSize, kSize, k);
	cv::normalize(result, result, 0, 1, cv::NORM_MINMAX);

	std::vector<cv::Point> keyPoints;
	for (int i = 0; i < result.rows; i++) {
		for (int j = 0; j < result.cols; j++) {
			if (result.at<float>(i, j) > threshold) {
				keyPoints.emplace_back(j, i);
			}
		}
	}
	return keyPoints;
}

CSiftKeyPointDetector::CSiftKeyPointDetector()
{
	sift = cv::xfeatures2d::SIFT::create();
}

std::vector<cv::Point> CSiftKeyPointDetector::DetectKeyPoints(cv::Mat image) const
{
	std::vector<cv::KeyPoint> keypoints;
	sift->detect(image, keypoints);
	std::vector<cv::Point> result;
	for (int i = 0; i < keypoints.size(); i++) {
		result.push_back(keypoints[i].pt);
	}
	return result;
}

COrbKeyPointDetector::COrbKeyPointDetector()
{
	orb = cv::ORB::create();
}

std::vector<cv::Point> COrbKeyPointDetector::DetectKeyPoints(cv::Mat image) const
{
	std::vector<cv::KeyPoint> keypoints;
	orb->detect(image, keypoints);
	std::vector<cv::Point> result;
	for (int i = 0; i < keypoints.size(); i++) {
		result.push_back(keypoints[i].pt);
	}
	return result;
}

bool hasMatch( const cv::Point& keyPoint, const std::set<cv::Point, CPointsCompare>& initialKeyPointsSet, const int possibleDelta = 5 )
{
	bool hasMatch = false;
	for (int deltaX = -possibleDelta; deltaX <= possibleDelta; deltaX++) {
		for (int deltaY = -possibleDelta; deltaY <= possibleDelta; deltaY++) {
			cv::Point delta(deltaX, deltaY);
			cv::Point originalKeyPointWithDelta = keyPoint + delta;
			if (initialKeyPointsSet.find(originalKeyPointWithDelta) != initialKeyPointsSet.end()) {
				hasMatch = true;
			}
		}
	}
	return hasMatch;
}

std::vector<float> measureReproductibility(const std::vector<cv::Mat>& images, const std::vector<cv::Point>& moves, 
	const IKeyPointDetector* detector)
{
	std::vector<float> result;
	std::vector<cv::Point> initialKeyPoints = detector->DetectKeyPoints(images[0]);
	std::set<cv::Point, CPointsCompare> initialKeyPointsSet(initialKeyPoints.begin(), initialKeyPoints.end());
	for (int i = 1; i < images.size(); i++) {
		cv::Point currentMove = moves[i];
		std::vector<cv::Point> newKeyPoints = detector->DetectKeyPoints(images[i]);
		int reproducedCount = 0;
		for (int j = 0; j < newKeyPoints.size(); j++) {
			cv::Point originalKeyPoint = newKeyPoints[j] + currentMove;
			if (hasMatch(originalKeyPoint, initialKeyPointsSet)) {
				reproducedCount++;
			}
		}
		result.push_back(static_cast<float>(reproducedCount) / newKeyPoints.size());
	}
	return result;
}

void saveResults(const std::string& name, const std::vector<float>& results)
{
	std::ofstream file(name);
	for (int i = 0; i < results.size(); i++) {
		file << i + 1 << "\t" << results[i] << std::endl;
	}
}

int main(int argc, char** argv)
{
	std::vector<cv::Mat> inputImages = readInputImages(12);

	std::vector<cv::Point> moves = calcMoves(inputImages);
	
	CHarrisKeyPointDetector harris(11, 5, 0.04, 0.5);
	std::vector<float> harrisResults = measureReproductibility(inputImages, moves, &harris);
	saveResults("Harris.txt", harrisResults);

	CSiftKeyPointDetector sift;
	std::vector<float> siftResults = measureReproductibility(inputImages, moves, &sift);
	saveResults("Sift.txt", siftResults);

	COrbKeyPointDetector orb;
	std::vector<float> orbResults = measureReproductibility(inputImages, moves, &orb);
	saveResults("Orb.txt", orbResults);

	return 0;
}
