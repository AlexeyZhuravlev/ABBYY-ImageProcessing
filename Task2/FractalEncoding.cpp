#include <windows.h>
#include <gdiplus.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <tchar.h>
#include <time.h>
#include <vector>
#include <tuple>
#include <array>
#include <cassert>
#include <string>
#include <fstream>

using namespace Gdiplus;

static const int DomainBlockSize = 8;
static const int RangeBlockSize = DomainBlockSize / 2;
static const float DomainBlockDispTreshold = 2.0;
static const int WindowSize = 50;
static const int DecoderNumOfIterations = 30;

struct CPoint {
	int x;
	int y;

	CPoint() :
		x(0), y(0) {}
};

struct CRangeBlockTranformationParams {
	CPoint domainBlockCoordinate;
	// number of rotations allpied to block
	BYTE nCW90Rotations;
	// flips
	bool verticalFlip;
	bool horizontalFlip;
	// Luminance linear transoformation params
	float luminanceCoeff;
	float luminanceBias;

	CRangeBlockTranformationParams() :
		nCW90Rotations(0),
		verticalFlip(false),
		horizontalFlip(false),
		luminanceCoeff(0),
		luminanceBias(0) {}
};

struct CFractalCode {
	std::vector< std::vector<CRangeBlockTranformationParams> > rangeBlocksTranformations;
};

class CGrayImage {
public:
	CGrayImage(int width, int height);
	void SetRangeBlock(int row, int column, const std::vector<std::vector<BYTE>>& block);

	void CopyToBmpData(BitmapData& pData) const;

	BYTE GetAt(int row, int column) const { return image[row][column]; }

	int Height() const { return image.size(); }
	int Width() const { return image[0].size(); }

private:
	// Серое изображение
	std::vector< std::vector<BYTE> > image;
};

CGrayImage::CGrayImage(int width, int height)
{
	image.resize( height, std::vector<BYTE>(width, 128) );
}

void CGrayImage::CopyToBmpData(BitmapData& pData) const
{
	BYTE *pBuffer = (BYTE *)pData.Scan0;
	int baseAdr = 0;

	for (unsigned int y = 0; y < pData.Height; y++) {
		int pixelAdr = baseAdr;
		for (unsigned int x = 0; x < pData.Width; x++) {
			pBuffer[pixelAdr] = image[y][x];
			pixelAdr += 1;
		}
		baseAdr += pData.Stride;
	}
}

void CGrayImage::SetRangeBlock(int row, int column, const std::vector<std::vector<BYTE>>& block)
{
	for (int i = 0; i < RangeBlockSize; i++) {
		for (int j = 0; j < RangeBlockSize; j++) {
			image[row + i][column + j] = block[i][j];
		}
	}
}

class CGrayImageWithRepresentations {
public:
	CGrayImageWithRepresentations( const BitmapData& pData );

	std::pair<float, float> GetMeanAndDispInWindow(int top, int left, int bottom, int right) const;

	BYTE GetAt(int row, int column) const { return image[row][column]; }

	int Height() const { return image.size(); }
	int Width() const { return image[0].size(); }

private:
	// Серое изображение
	std::vector< std::vector<BYTE> > image;
	// Интегральное представление изображение: частичные суммы
	std::vector< std::vector<long long> > integralImage;
	// Частичные суммы квадратов
	std::vector< std::vector<long long> > integralSquareImage;

	void calcRepresentations(int rows, int cols);

	static float getAverageFromPartialSums(const std::vector< std::vector<long long> >& partialSums, int t, int l, int b, int r);
};

CGrayImageWithRepresentations::CGrayImageWithRepresentations( const BitmapData& pData )
{
	image.resize(pData.Height, std::vector<unsigned char>(pData.Width, 0));

	BYTE *pBuffer = (BYTE *)pData.Scan0;
	int baseAdr = 0;

	for( unsigned int y = 0; y < pData.Height; y++ ) {
		int pixelAdr = baseAdr;
		for( unsigned int x = 0; x < pData.Width; x++ ) {
			image[y][x] = pBuffer[pixelAdr];
			pixelAdr += 1;
		}
		baseAdr += pData.Stride;
	}

	calcRepresentations( pData.Height, pData.Width );
}

float CGrayImageWithRepresentations::getAverageFromPartialSums(const std::vector< std::vector<long long> >& partialSums, int t, int l, int b, int r)
{
	assert(l <= r);
	assert(t <= b);
	assert(t >= 0);
	assert(l >= 0);
	assert(b + 1 < partialSums.size());
	assert(r + 1 < partialSums[0].size());

	long long numerator = partialSums[b + 1][r + 1] - partialSums[b + 1][l] - partialSums[t][r + 1] + partialSums[t][l];
	long long denumerator = (b - t + 1) * (r - l + 1);

	return static_cast<float>(numerator) / denumerator;
}

std::pair<float, float> CGrayImageWithRepresentations::GetMeanAndDispInWindow(int top, int left, int bottom, int right) const
{
	float mean = getAverageFromPartialSums(integralImage, top, left, bottom, right);;
	float squaresAverage = getAverageFromPartialSums(integralSquareImage, top, left, bottom, right);
	return std::make_pair(mean, sqrtf(squaresAverage - mean * mean));
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

BYTE linearTransform(BYTE value, float coef, float bias)
{
	return static_cast<BYTE>(max(0, min(255, round(value * coef + bias))));
}

template<typename ImageType>
void applyLinearTransformAndMinimizeDomainBlock(float coef, float bias, const ImageType& image,
	int row, int col, std::vector<std::vector<BYTE>>& result)
{
	result.resize(RangeBlockSize, std::vector<BYTE>(RangeBlockSize));
	for (int i = row; i < row + DomainBlockSize; i += 2) {
		for (int j = col; j < col + DomainBlockSize; j += 2) {
			BYTE value = ( linearTransform(image.GetAt(i, j), coef, bias) 
					     + linearTransform(image.GetAt(i + 1, j), coef, bias) 
					     + linearTransform(image.GetAt(i, j + 1), coef, bias) 
					     + linearTransform(image.GetAt(i + 1, j + 1), coef, bias) ) / 4;
			result[(i - row) / 2][(j - col) / 2] = value;
		}
	}
}

int sqr(int a) {
	return a * a;
}

// Calculates L1 Metrics, matching range block with pattern
int matchBlockWithRangeBlock(const CGrayImageWithRepresentations& image, int row, int col, const std::vector<std::vector<BYTE>>& pattern)
{
	int score = 0;
	for (int i = 0; i < RangeBlockSize; i++) {
		for (int j = 0; j < RangeBlockSize; j++) {
			score += abs(image.GetAt(row + i, col + j) - pattern[i][j]);
		}
	}
	return score;
}

void flipVertical(std::vector<std::vector<BYTE>>& pattern)
{
	std::reverse(pattern.begin(), pattern.end());
}

void flipHorizontal(std::vector<std::vector<BYTE>>& pattern)
{
	for (int i = 0; i < RangeBlockSize; i++) {
		std::reverse(pattern[i].begin(), pattern[i].end());
	}
}

void rotateClockwise(std::vector<std::vector<BYTE>>& pattern)
{
	for (int i = 0; i < RangeBlockSize; i++) {
		for (int j = i + 1; j < RangeBlockSize; j++) {
			std::swap(pattern[i][j], pattern[j][i]);
		}
	}
	flipHorizontal(pattern);
}

void tryFindTranformForDomainBlock(const CGrayImageWithRepresentations& image, int rangeRow, int rangeColumn, 
	std::pair<float,float> rangeMeanAndDisp, int domainRow, int domainColumn, 
	CRangeBlockTranformationParams& result, int& best)
{
	std::pair<float, float> domainMeanAndDisp = image.GetMeanAndDispInWindow(domainRow, domainColumn, domainRow + DomainBlockSize - 1, domainColumn + DomainBlockSize - 1);
	if (domainMeanAndDisp.second < DomainBlockDispTreshold) {
		return;
	}

	float linearK = rangeMeanAndDisp.second / domainMeanAndDisp.second;
	float linearB = -domainMeanAndDisp.first * linearK + rangeMeanAndDisp.first;
	std::vector<std::vector<BYTE>> transformedDomain;
	applyLinearTransformAndMinimizeDomainBlock(linearK, linearB, image, domainRow, domainColumn, transformedDomain);
	int rotations = 0;
	for (int rotations = 0; rotations < 4; rotations++) {
		for (int verticalFlipValue = 0; verticalFlipValue < 2; verticalFlipValue++) {
			for (int horizontalFlipValue = 0; horizontalFlipValue < 2; horizontalFlipValue++) {
				int matchingScore = matchBlockWithRangeBlock(image, rangeRow, rangeColumn, transformedDomain);
				if (matchingScore < best) {
					best = matchingScore;
					result.domainBlockCoordinate.x = domainColumn;
					result.domainBlockCoordinate.y = domainRow;
					result.nCW90Rotations = rotations;
					result.luminanceCoeff = linearK;
					result.luminanceBias = linearB;
					result.horizontalFlip = (horizontalFlipValue == 1);
					result.verticalFlip = (verticalFlipValue == 1);
				}
				flipHorizontal(transformedDomain);
			}
			flipVertical(transformedDomain);
		}
		rotateClockwise(transformedDomain);
	}
}

void findTransformationParams(const CGrayImageWithRepresentations& image, int row, int column, CRangeBlockTranformationParams& result)
{
	std::pair<float, float> meanAndDisp = image.GetMeanAndDispInWindow(row, column, row + RangeBlockSize - 1, column + RangeBlockSize - 1);
	int bestMatch = 255 * 64;
	for (int i = max(0, row - WindowSize); i < min(image.Height() - DomainBlockSize, row + WindowSize) && bestMatch > 0; i++) {
		for (int j = max(0, column - WindowSize); j < min(image.Width() - DomainBlockSize, column + WindowSize) && bestMatch > 0; j++) {
			tryFindTranformForDomainBlock(image, row, column, meanAndDisp, i, j, result, bestMatch);
		}
	}
}

void encode(const CGrayImageWithRepresentations& image, CFractalCode& code )
{
	assert(image.Height() % RangeBlockSize == 0);
	assert(image.Width() % RangeBlockSize == 0);
	std::vector<std::vector<CRangeBlockTranformationParams>>& transformations = code.rangeBlocksTranformations;
	transformations.resize(image.Height() / RangeBlockSize, std::vector<CRangeBlockTranformationParams>(image.Width()));
	for (int i = 0; i < image.Height(); i += RangeBlockSize) {
		for (int j = 0; j < image.Width(); j += RangeBlockSize) {
			findTransformationParams(image, i, j, transformations[i / RangeBlockSize][j / RangeBlockSize]);
		}
	}
}

void decodeIteration(CGrayImage& image, const CFractalCode& code)
{
	for (int i = 0; i < image.Height(); i += RangeBlockSize) {
		for (int j = 0; j < image.Width(); j += RangeBlockSize) {
			const CRangeBlockTranformationParams& currentParams = code.rangeBlocksTranformations[i / RangeBlockSize][j / RangeBlockSize];
			std::vector<std::vector<BYTE>> transformedDomain;
			applyLinearTransformAndMinimizeDomainBlock(currentParams.luminanceCoeff, currentParams.luminanceBias,
				image, currentParams.domainBlockCoordinate.y, currentParams.domainBlockCoordinate.x, 
				transformedDomain);
			for (int k = 0; k < currentParams.nCW90Rotations; k++) {
				rotateClockwise(transformedDomain);
			}
			if (currentParams.verticalFlip) {
				flipVertical(transformedDomain);
			}
			if (currentParams.horizontalFlip) {
				flipHorizontal(transformedDomain);
			}
			image.SetRangeBlock(i, j, transformedDomain);
		}
	}
}

int GetEncoderClsid( const WCHAR* format, CLSID* pClsid )
{
   UINT  num = 0;          // number of image encoders
   UINT  size = 0;         // size of the image encoder array in bytes

   ImageCodecInfo* pImageCodecInfo = NULL;

   GetImageEncodersSize( &num, &size );
   if( size == 0 )
      return -1;  // Failure

   pImageCodecInfo = (ImageCodecInfo*)(malloc(size));
   if( pImageCodecInfo == NULL )
      return -1;  // Failure

   GetImageEncoders( num, size, pImageCodecInfo );

   for( UINT j = 0; j < num; j++ )
   {
      if( wcscmp( pImageCodecInfo[j].MimeType, format ) == 0 )
      {
         *pClsid = pImageCodecInfo[j].Clsid;
         free(pImageCodecInfo);
         return j;  // Success
      }    
   }

   free( pImageCodecInfo );
   return -1;  // Failure
}

double computePSNR(const CGrayImageWithRepresentations& source, const CGrayImage& approx) {
	int sum = 0;
	int maxValue = 0;
	for (int i = 0; i < source.Height(); i++) {
		for (int j = 0; j < source.Width(); j++) {
			sum += sqr(source.GetAt(i, j) - approx.GetAt(i, j) );
			maxValue = max(maxValue, source.GetAt(i, j));
		}
	}
	double mse = static_cast<double>(sum) / (source.Height() * source.Width());
	double PSNR = 10 * log(sqr(maxValue) / mse) / log(10);

	return PSNR;
}

int _tmain(int argc, _TCHAR* argv[])
{
	if( argc != 3 )
	{
		_tprintf( _T("Usage: BayerPattern <inputFile.bmp> <outputFile.bmp>\n") );
		return 0;
	}

	GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR			gdiplusToken;
	GdiplusStartup( &gdiplusToken, &gdiplusStartupInput, NULL );

	{	
		Bitmap GDIBitmap( argv[1] );

		int w = GDIBitmap.GetWidth();
		int h = GDIBitmap.GetHeight();
		BitmapData bmpData;
		Rect rc( 0, 0, w, h ); // whole image
		if( Ok != GDIBitmap.LockBits( &rc, ImageLockModeRead | ImageLockModeWrite, PixelFormat8bppIndexed, &bmpData ) )
			_tprintf( _T("Failed to lock image: %s\n"), argv[1] );
		else
			_tprintf( _T("File: %s\n"), argv[1] );

		CGrayImageWithRepresentations image(bmpData);
		CFractalCode code;

		_tprintf(_T("Encoding start\n"));
		
		encode(image, code);

		_tprintf(_T("Encoding end\n"));

		CGrayImage restoredImage(w, h);

		std::wstring csvFile = std::wstring(argv[2]) + L".csv";
		FILE* pFile = _wfopen(csvFile.c_str(), L"w");

		for (int i = 0; i < DecoderNumOfIterations; i++) {
			decodeIteration(restoredImage, code);

			double psnr = computePSNR(image, restoredImage);

			_tprintf(_T("%d: %.3f\n"), i, psnr);
			fprintf(pFile, "%d,%.3f\n", i, psnr);

			restoredImage.CopyToBmpData(bmpData);

			GDIBitmap.UnlockBits(&bmpData);

			// Save result
			CLSID clsId;
			GetEncoderClsid(_T("image/bmp"), &clsId);
			std::wstring filename = argv[2] + std::to_wstring(i) + L".bmp";
			GDIBitmap.Save(filename.c_str(), &clsId, NULL);
		}
	}

	GdiplusShutdown( gdiplusToken );

	return 0;
}
