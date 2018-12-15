#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>

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
std::pair<int, int> phaseCorrelationMotionEstimation(cv::Mat first, cv::Mat second)
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

	return std::make_pair(resX, resY);
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

std::vector<std::pair<int, int>> calcMoves(const std::vector<cv::Mat>& images) 
{
	std::vector<std::pair<int, int>> moves;
	std::pair<int, int> accMove = std::make_pair(0, 0);
	moves.push_back(accMove);
	for (int i = 1; i < images.size(); i++) {
		auto start = std::chrono::steady_clock::now();
		std::pair<int, int> locMove = phaseCorrelationMotionEstimation(images[i - 1], images[i]);
		auto end = std::chrono::steady_clock::now();

		std::cerr << i << " " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << std::endl;

		accMove = std::make_pair(accMove.first + locMove.first, accMove.second + locMove.second);
		moves.push_back(accMove);
	}
	return moves;
}

void writeResult(const std::vector<std::pair<int, int>>& moves, std::string filename)
{
	std::ofstream stream(filename);

	for (int i = 0; i < moves.size(); i++) {
		stream << moves[i].first << " " << moves[i].second << std::endl;
	}
}

int main(int argc, char** argv)
{
	std::vector<cv::Mat> inputImages = readInputImages(12);

	std::vector<std::pair<int, int>> moves = calcMoves(inputImages);
	
	writeResult(moves, "output.txt");

	return 0;
}
