#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <experimental/filesystem>
#include <time.h>

class CGrayImageWithRepresentations {
public:
	CGrayImageWithRepresentations(cv::Mat source);

	std::pair<double, double> GetMeanAndDispInWindow(int top, int left, int bottom, int right);

	unsigned char GetAt(int row, int column) { return image[row][column]; }

private:
	// Серое изображение
	std::vector< std::vector<unsigned char> > image;
	// Интегральное представление изображение: частичные суммы
	std::vector< std::vector<long long> > integralImage;
	// Частичные суммы квадратов
	std::vector< std::vector<long long> > integralSquareImage;

	void calcRepresentations(int rows, int cols);

	static double getAverageFromPartialSums(const std::vector< std::vector<long long> >& partialSums, int t, int l, int b, int r);
};

double CGrayImageWithRepresentations::getAverageFromPartialSums(const std::vector< std::vector<long long> >& partialSums, int t, int l, int b, int r)
{
	assert(l <= r);
	assert(t <= b);
	assert(t >= 0);
	assert(l >= 0);
	assert(b + 1 < partialSums.size());
	assert(r + 1 < partialSums[0].size());

	long long numerator = partialSums[b + 1][r + 1] - partialSums[b + 1][l] - partialSums[t][r + 1] + partialSums[t][l];
	long long denumerator = (b - t + 1) * (r - l + 1);

	return static_cast<double>(numerator) / denumerator;
}

std::pair<double, double> CGrayImageWithRepresentations::GetMeanAndDispInWindow(int top, int left, int bottom, int right)
{
	double mean = getAverageFromPartialSums(integralImage, top, left, bottom, right);;
	double squaresAverage = getAverageFromPartialSums(integralSquareImage, top, left, bottom, right);
	return std::make_pair(mean, squaresAverage - mean * mean);
}

CGrayImageWithRepresentations::CGrayImageWithRepresentations(cv::Mat source)
{
	image.resize(source.rows, std::vector<unsigned char>(source.cols, 0));
	for (int i = 0; i < source.rows; i++) {
		for (int j = 0; j < source.cols; j++) {
			cv::Vec3b channels = source.at<cv::Vec3b>(cv::Point(j, i));
			image[i][j] = (channels[0] + channels[1] + channels[2]) / 3;
		}
	}

	calcRepresentations(source.rows, source.cols);
}

void CGrayImageWithRepresentations::calcRepresentations(int rows, int cols)
{
	integralImage.resize(rows + 1, std::vector<long long>(cols + 1, 0));
	integralSquareImage.resize(rows + 1, std::vector<long long>(cols + 1, 0));

	for (int i = 1; i <= rows; i++) {
		for (int j = 1; j <= cols; j++) {
			integralImage[i][j] = integralImage[i - 1][j] + integralImage[i][j - 1] - integralImage[i - 1][j - 1]
				+ image[i - 1][j - 1];
			integralSquareImage[i][j] = integralSquareImage[i - 1][j] + integralSquareImage[i][j - 1]
				- integralSquareImage[i - 1][j - 1] + static_cast<int>(image[i - 1][j - 1]) * image[i - 1][j - 1];
		}
	}
}

// Sauvola binarization
cv::Mat BinarizeImage(cv::Mat source, int widowRadius = 25, double k = 0.2, double R = 128.0) {
	CGrayImageWithRepresentations image(source);
	cv::Mat result(source.rows, source.cols, CV_8UC1);

	for (int i = 0; i < source.rows; i++) {
		for (int j = 0; j < source.cols; j++) {
			int l = std::max(0, j - widowRadius);
			int r = std::min(source.cols - 1, j + widowRadius);
			int t = std::max(0, i - widowRadius);
			int b = std::min(source.rows - 1, i + widowRadius);
			std::pair<double, double> meanAndDisp = image.GetMeanAndDispInWindow(t, l, b, r);
			double mean = meanAndDisp.first;
			double disp = meanAndDisp.second;
			int treshold = static_cast<int>(std::round(mean * (1 + k * (sqrt(disp) / R - 1))));
			result.at<unsigned char>(cv::Point(j, i)) = image.GetAt(i, j) > treshold ? 255 : 0;
		}
	}

	return result;
}

std::vector<std::string> GetFilenames(std::experimental::filesystem::path path)
{
	namespace stdfs = std::experimental::filesystem;

	std::vector<std::string> filenames;
	const stdfs::directory_iterator end{};

	for (stdfs::directory_iterator iter{ path }; iter != end; ++iter)
	{
		if (stdfs::is_regular_file(*iter)) {
			filenames.push_back(iter->path().string());
		}
	}

	return filenames;
}

int main(int argc, char** argv)
{
	if (argc != 3)
	{
		std::cout << " Usage: Binarization SourceFolder ResultFolder" << std::endl;
		return -1;
	}

	std::vector<std::string> fileNames = GetFilenames( argv[1] );

	double sumTime = 0.0;

	for (int i = 0; i < fileNames.size(); i++) {

		std::string fileName = fileNames[i];
		auto pos = fileName.rfind('\\');
		std::string resultFileName = argv[2] + fileName.substr(pos) + ".tiff";

		cv::Mat sourceImage;
		sourceImage = cv::imread(fileName, cv::IMREAD_COLOR); // Read the file

		time_t start = clock();

		cv::Mat resultImage = BinarizeImage(sourceImage);

		time_t end = clock();
		double time = static_cast<double>(end - start) / CLOCKS_PER_SEC;
		sumTime += time;

		std::vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PXM_BINARY);

		if (!cv::imwrite(resultFileName, resultImage)) {
			std::cout << "Could not save result image " << resultFileName << std::endl;
			return -1;
		}
	}

	std::cout << "Average time: " << sumTime / fileNames.size() << std::endl;
	
	return 0;
}
