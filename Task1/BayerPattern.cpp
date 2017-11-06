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
	CN_Red = 2,

	CN_Count
};

CMatrix getPaddedGrayMatrix( const BitmapData& pData, int padding, int fillValue )
{
	int matrixHeight = pData.Height + padding * 2;
	int matrixWidth = pData.Width + padding * 2;
	CMatrix matrix( matrixHeight, CRow( matrixWidth, fillValue ) );

	BYTE *pBuffer = (BYTE *)pData.Scan0;
	int baseAdr = 0;

	for( unsigned int y = 0; y < pData.Height; y++ ) {
		int pixelAdr = baseAdr;
		for( unsigned int x = 0; x < pData.Width; x++ ) {
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
	if( ( x + y ) % 2 != 0 ) {
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
	return dy < 0 && abs(dx) <= 1;
}

bool isSouth( int dx, int dy )
{
	return dy > 0 && abs( dx ) <= 1;
}

bool isWest( int dx, int dy )
{
	return dx < 0 && abs( dy ) <= 1;
}

bool isEast( int dx, int dy )
{
	return dx > 0 && abs( dy ) <= 1;
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
	int resultX = x + dx;
	int resultY = y + dy;
	while( getCurrentBayerChannel( x, y ) != getCurrentBayerChannel( resultX, resultY ) ) {
		resultX += dx;
		resultY += dy;
	}
	return std::make_pair( resultX, resultY );
}

#include <iostream>

int calcDirectionGradient( const CMatrix& m, int sourceX, int sourceY, int grayPadding, DeltaCheckFunction directionCheck, 
	int deltaX, int deltaY )
{
	const int radius = 2;
	int gradient = 0;
	TChannelName channel = getCurrentBayerChannel( sourceX, sourceY );

	for( int dx = -radius; dx <= radius; dx++ ) {
		for( int dy = -radius; dy <= radius; dy++ ) {
			if( !directionCheck( dx, dy ) ) {
				continue;
			}
			int newX, newY;
			std::tie(newX, newY) = findNearestSameColor( sourceX + dx, sourceY + dy, -deltaX, -deltaY );
			if( abs( newX - sourceX ) + abs( newY - sourceY ) >= 3 ) {
				continue;
			}
			int gradientValue = abs( m[sourceY + dy + grayPadding][sourceX + dx + grayPadding] - m[newY + grayPadding][newX + grayPadding] );
			if( ( channel != CN_Green || abs( deltaX ) != abs( deltaY ) ) 
				&& dx * deltaY - dy * deltaX != 0 ) 
			{
				gradientValue /= 2;
			}
			gradient += gradientValue;
		}
	}

	return gradient;
}

std::tuple<int, int, int> calcEstimation( const CMatrix& m, int sourceX, int sourceY,
	int pad, int dx, int dy )
{
	int tX = sourceX + dx;
	int tY = sourceY + dy;
	std::array<int, CN_Count> colorValues;
	colorValues.fill( 0 );

	for( TChannelName channel : { CN_Blue, CN_Green, CN_Red } ) {
		if( getCurrentBayerChannel( tX, tY ) == channel ) {
			colorValues[channel] = m[tY + pad][tX + pad];
		} else if( getCurrentBayerChannel( tX + dx, tY + dy ) == channel ) {
			colorValues[channel] = (m[sourceY + pad][sourceX + pad] + m[tY + dy + pad][tX + dx + pad]) / 2;
		} else {
			int sum = 0;
			int cnt = 0;
			for( int deltaX = -1; deltaX <= 1; deltaX++ ) {
				for( int deltaY = -1; deltaY <= 1; deltaY++ ) {
					if( getCurrentBayerChannel( tX + deltaX, tY + deltaY ) == channel ) {
						sum += m[tY + deltaY + pad][tX + deltaX + pad];
						cnt++;
					}
				}
			}
			colorValues[channel] = sum / cnt;
		}
	}
		
	return std::make_tuple( colorValues[CN_Blue], colorValues[CN_Green], colorValues[CN_Red] );
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
	const std::pair<int, int> directions[nGradients] = {
		{0, -1},
		{1, 0},
		{0, 1},
		{-1, 0},
		{1, -1},
		{1, 1},
		{-1, -1},
		{-1, 1}
	};
	std::array<int, nGradients> gradients;
	for( int i = 0; i < nGradients; i++ ) {
		int dx = directions[i].first;
		int dy = directions[i].second;
		if( currentChannel == CN_Green ) {
			dx *= 2;
			dy *= 2;
		}
		gradients[i] = calcDirectionGradient( m, sourceX, sourceY, grayPadding, 
			checkFunctions[i], dx, dy );
	}
	int min = *std::min_element( gradients.begin(), gradients.end() );
	int max = *std::max_element( gradients.begin(), gradients.end() );
	int treshold = static_cast<int>( k1 * min + k2 * ( max - min ) );

	int sumR = 0;
	int sumB = 0;
	int sumG = 0;
	int nGoodGradients = 0;

	for( int i = 0; i < nGradients; i++ ) {
		if( gradients[i] <= treshold ) {
			int db, dg, dr;
			std::tie(db, dg, dr) = calcEstimation( m, sourceX, sourceY, grayPadding,
				directions[i].first, directions[i].second );
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

			pBuffer[pixelAdr] = static_cast<BYTE>( max( 0, min( 255, B ) ) );
			pBuffer[pixelAdr + 1] = static_cast<BYTE>( max( 0, min( 255, G ) ) );
			pBuffer[pixelAdr + 2] = static_cast<BYTE>( max( 0, min( 255, R ) ) );

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
