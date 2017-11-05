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

using namespace Gdiplus;

static const int LumaRed = 9798;	//	0.299
static const int LumaGreen = 19235;	//	0.587
static const int LumaBlue = 3735;	//	0.114
static const int CoeffNormalizationBitsCount = 15;
static const int CoeffNormalization = 1 << CoeffNormalizationBitsCount;

typedef std::vector<BYTE> CRow;
typedef std::vector<CRow> CMatrix;

enum TChannelName {
	CN_Blue = 0,
	CN_Green = 1,
	CN_Red = 2
};

CMatrix getPaddedGrayMatrix( const BitmapData& pData, int padding, int fillValue )
{
	int matrixHeight = pData.Height + padding * 2;
	int matrixWidth = pData.Width + padding * 2;
	CMatrix matrix( matrixHeight, CRow( matrixWidth, fillValue ) );

	BYTE *pBuffer = (BYTE *)pData.Scan0;
	int baseAdr = 0;

	for( int y = 0; y < pData.Height; y++ ) {
		int pixelAdr = baseAdr;
		for( int x = 0; x < pData.Width; x++ ) {
			int B = pBuffer[pixelAdr]; // blue
			int G = pBuffer[pixelAdr + 1]; // green
			int R = pBuffer[pixelAdr + 2]; // red

			int Y = (LumaRed * R + LumaGreen * G + LumaBlue * B + (CoeffNormalization >> 1))
				>> CoeffNormalizationBitsCount; // luminance

			matrix[y + padding][x + padding] = Y;

			pixelAdr += 3;
		}
		baseAdr += pData.Stride;
	}

	return matrix;
}

TChannelName getCurrentBayerChannel( int x, int y )
{
	if( x % 2 == 0 && y % 2 == 1 ) {
		return CN_Green;
	}
	if( y % 2 == 0 ) {
		return CN_Red;
	} else {
		return CN_Blue;
	}
}

bool notZero( int dx, int dy )
{
	return dx != 0 || dy != 0;
}

bool isZero( int dx, int dy )
{
	return dx == 0 && dy == 0;
}

bool isNorth( int dx, int dy )
{
	return dy < 0 && abs(dx) + abs(dy) <= 2 || isZero( dx, dy );
}

bool isSouth( int dx, int dy )
{
	return dy > 0 && abs( dx ) + abs( dy ) <= 2 || isZero( dx, dy );
}

bool isWest( int dx, int dy )
{
	return dx < 0 && abs( dx ) + abs( dy ) <= 2 || isZero( dx, dy );
}

bool isEast( int dx, int dy )
{
	return dx > 0 && abs( dx ) + abs( dy ) <= 2 || isZero( dx, dy );
}

bool isNorthWest( int dx, int dy )
{
	return dx <= 0 && dy <= 0 && abs( dx - dy ) <= 1;
}

bool isNorthEast( int dx, int dy )
{
	return dx >= 0 && dy <= 0 && abs( dx + dy ) <= 1;
}

bool isSouthWest( int dx, int dy )
{
	return dx <= 0 && dy >= 0 && abs( dx + dy ) <= 1;
}

bool isSouthEast( int dx, int dy )
{
	return dx >= 0 && dy >= 0 && abs( dx - dy ) <= 1;
}

bool alwaysTrue( int dx, int dy )
{
	return true;
}

typedef bool( *DeltaCheckFunction )( int dx, int dy );

std::pair<int, int> findNearestSameColor( int x, int y, int dx, int dy )
{
	int resultX = x;
	int resultY = y;
	while( getCurrentBayerChannel( x, y ) != getCurrentBayerChannel( resultX, resultY ) ) {
		resultX += dx;
		resultY += dy;
	}
	return std::make_pair( resultX, resultY );
}

int calcDirectionGradient( const CMatrix& m, int sourceX, int sourceY, int grayPadding, DeltaCheckFunction directionCheck, int deltaX, int deltaY )
{
	const int radius = 2;
	int gradient = 0;

	for( int dx = -radius; dx <= radius; dx++ ) {
		for( int dy = -radius; dy <= radius; dy++ ) {
			if( !directionCheck( dx, dy ) && notZero( dx, dy ) ) {
				continue;
			}
			int newX, newY;
			std::tie(newX, newY) = findNearestSameColor( sourceX + dx, sourceY + dy, -deltaX, -deltaY );
			int gradientValue = abs( m[sourceY + dx + grayPadding][sourceX + dy + grayPadding] - m[newY + grayPadding][newX + grayPadding] );
			if( dx * deltaY - dy * deltaX != 0 ) {
				gradientValue /= 2;
			}
			gradient += gradientValue;
		}
	}

	return gradient;
}

std::tuple<int, int, int> calcEstimation( const CMatrix& m, int sourceX, int sourceY,
	int grayPadding, DeltaCheckFunction check )
{
	int x = sourceX + grayPadding;
	int y = sourceY + grayPadding;
	const int radius = 2;
	int blue = 0;
	int red = 0;
	int green = 0;
	int cntBlue = 0;
	int cntRed = 0;
	int cntGreen = 0;

	for( int dx = -radius; dx <= radius; dx++ ) {
		for( int dy = -radius; dy <= radius; dy++ ) {
			if( !check( dx, dy ) ) {
				continue;
			}
			TChannelName color = getCurrentBayerChannel( sourceX + dx, sourceY + dy );
			switch( color ) {
				case CN_Blue:
					blue += m[y + dy][x + dx];
					cntBlue++;
					break;
				case CN_Green:
					green += m[y + dy][x + dx];
					cntGreen++;
					break;
				case CN_Red:
					red += m[y + dy][x + dx];
					cntRed++;
					break;
			}
		}
	}

	if( cntBlue > 0 ) {
		blue /= cntBlue;
	}
	if( cntGreen > 0 ) {
		green /= cntGreen;
	}
	if( cntRed > 0 ) {
		red /= cntRed;
	}
	return std::make_tuple( blue, green, red );
}

std::tuple<int, int, int> computeVngInterpolation( const CMatrix& m, int sourceX, int sourceY, 
	int grayPadding, float k1 = 1.5f, float k2 = 0.5f )
{
	TChannelName currentChannel = getCurrentBayerChannel( sourceX, sourceY );
	int x = sourceX + grayPadding;
	int y = sourceY + grayPadding;

	const int nGradients = 8;
	const DeltaCheckFunction checkFunctions[nGradients] = {
		isNorth,
		isEast,
		isSouth,
		isWest,
		isNorthEast,
		isSouthEast,
		isNorthWest,
		isSouthWest
	};
	const int gradients[nGradients] = {
		calcDirectionGradient( m, sourceX, sourceY, grayPadding, isNorth, 0, -1 ),
		calcDirectionGradient( m, sourceX, sourceY, grayPadding, isEast, 1, 0 ),
		calcDirectionGradient( m, sourceX, sourceY, grayPadding, isSouth, 0, 1 ),
		calcDirectionGradient( m, sourceX, sourceY, grayPadding, isWest, -1, 0 ),
		calcDirectionGradient( m, sourceX, sourceY, grayPadding, isNorthEast, 1, -1 ),
		calcDirectionGradient( m, sourceX, sourceY, grayPadding, isSouthEast, 1, 1 ),
		calcDirectionGradient( m, sourceX, sourceY, grayPadding, isNorthWest, -1, -1 ),
		calcDirectionGradient( m, sourceX, sourceY, grayPadding, isSouthWest, -1, 1 )
	};
	int min = *std::min_element( gradients, gradients + nGradients );
	int max = *std::max_element( gradients, gradients + nGradients );
	int treshold = static_cast<int>( k1 * min + k2 * ( max - min ) );

	int sumR = 0;
	int sumB = 0;
	int sumG = 0;
	int nGoodGradients = 0;

	for( int i = 0; i < nGradients; i++ ) {
		if( gradients[i] >= treshold ) {
			int db, dg, dr;
			std::tie(db, dg, dr) = calcEstimation( m, sourceX, sourceY, grayPadding, checkFunctions[i] );
			sumR += dr;
			sumB += db;
			sumG += dg;
			nGoodGradients++;
		}
	}
	
	int currentIntensity = m[y][x];
	int B = currentIntensity;
	int R = currentIntensity;
	int G = currentIntensity;
	if( nGoodGradients > 0 ) {
		switch( currentChannel ) {
			case CN_Red:
				B += (sumB - sumR) / nGoodGradients;
				G += (sumG - sumR) / nGoodGradients;
				break;
			case CN_Green:
				R += (sumR - sumG) / nGoodGradients;
				B += (sumB - sumG) / nGoodGradients;
				break;
			case CN_Blue:
				R += (sumR - sumB) / nGoodGradients;
				G += (sumG - sumB) / nGoodGradients;
				break;
		}
	}
	return std::make_tuple( B, G, R );
}

void process( BitmapData& pData )
{
	const int w = pData.Width;
	const int h = pData.Height;
	const int bpr = pData.Stride;
	const int bpp = 3; // BGR24
	BYTE *pBuffer = (BYTE *)pData.Scan0;

	time_t start = clock();
	
	const int grayPadding = 3;
	CMatrix paddedGraySource = getPaddedGrayMatrix( pData, grayPadding, 0 );

	int baseAdr = 0;
	for( int y = 0; y < h; y++ )
	{
		int pixelAdr = baseAdr;
		for( int x = 0; x < w; x++ )
		{
			int B = 0;
			int G = 0;
			int R = 0;

			std::tie( B, G, R ) = computeVngInterpolation( paddedGraySource, x,
				y, grayPadding );

			pBuffer[pixelAdr] = B;
			pBuffer[pixelAdr + 1] = G;
			pBuffer[pixelAdr + 2] = R;

			pixelAdr += bpp;
		}
		baseAdr += bpr;
	}

	time_t end = clock();
	_tprintf( _T("Time: %.3f\n"), static_cast<double>( end - start ) / CLOCKS_PER_SEC );
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
		if( Ok != GDIBitmap.LockBits( &rc, ImageLockModeRead | ImageLockModeWrite, PixelFormat24bppRGB, &bmpData ) )
			_tprintf( _T("Failed to lock image: %s\n"), argv[1] );
		else
			_tprintf( _T("File: %s\n"), argv[1] );

		process( bmpData );

		GDIBitmap.UnlockBits( &bmpData );

		// Save result
		CLSID clsId;
		GetEncoderClsid( _T("image/bmp"), &clsId );
		GDIBitmap.Save( argv[2], &clsId, NULL );
	}

	GdiplusShutdown( gdiplusToken );

	return 0;
}
