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

using namespace Gdiplus;

int sqr( int x )
{
	return x * x;
}

void process( const BitmapData& originalData, const BitmapData& resultData )
{
	const int w = originalData.Width;
	const int h = originalData.Height;
	const int bpr = originalData.Stride;
	const int bpp = 3; // BGR24
	BYTE *originalBuffer = (BYTE *)originalData.Scan0;
	BYTE *resultBuffer = (BYTE *)resultData.Scan0;

	time_t start = clock();

	int maxValue = 0;

	int baseAdr = 0;
	int avg = 0;

	for( int y = 0; y < h; y++ ) {
		int pixelAdr = baseAdr;
		int avgRowDiff = 0;
		for( int x = 0; x < w; x++ ) {
			for( int z = 0; z < bpp; z++ ) {
				avgRowDiff += sqr( originalBuffer[pixelAdr + z] - resultBuffer[pixelAdr + z] );
				maxValue = max( originalBuffer[pixelAdr + z], maxValue );
			}

			pixelAdr += bpp;
		}
		avgRowDiff /= w;
		avg += avgRowDiff;
		baseAdr += bpr;
	}
	avg /= h;

	double PSNR = 10 * log( sqr( maxValue ) / avg ) / log( 10 );

	_tprintf( _T( "PSNR: %.3f\n" ), PSNR );

	time_t end = clock();
	_tprintf( _T( "Time: %.3f\n" ), static_cast<double>(end - start) / CLOCKS_PER_SEC );
}

int GetEncoderClsid( const WCHAR* format, CLSID* pClsid )
{
	UINT  num = 0;          // number of image encoders
	UINT  size = 0;         // size of the image encoder array in bytes

	ImageCodecInfo* pImageCodecInfo = NULL;

	GetImageEncodersSize( &num, &size );
	if( size == 0 )
		return -1;  // Failure

	pImageCodecInfo = (ImageCodecInfo*)(malloc( size ));
	if( pImageCodecInfo == NULL )
		return -1;  // Failure

	GetImageEncoders( num, size, pImageCodecInfo );

	for( UINT j = 0; j < num; j++ ) {
		if( wcscmp( pImageCodecInfo[j].MimeType, format ) == 0 ) {
			*pClsid = pImageCodecInfo[j].Clsid;
			free( pImageCodecInfo );
			return j;  // Success
		}
	}

	free( pImageCodecInfo );
	return -1;  // Failure
}

int _tmain( int argc, _TCHAR* argv[] )
{
	if( argc != 3 ) {
		_tprintf( _T( "Usage: PSNR <originalFile.bmp> <resultFile.bmp>\n" ) );
		return 0;
	}

	GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR			gdiplusToken;
	GdiplusStartup( &gdiplusToken, &gdiplusStartupInput, NULL );

	{
		Bitmap originalGDIBitmap( argv[1] );

		int w = originalGDIBitmap.GetWidth();
		int h = originalGDIBitmap.GetHeight();
		BitmapData originalBmpData;
		Rect originalRect( 0, 0, w, h ); // whole image
		if( Ok != originalGDIBitmap.LockBits( &originalRect, ImageLockModeRead, PixelFormat24bppRGB, &originalBmpData ) )
			_tprintf( _T( "Failed to lock image: %s\n" ), argv[1] );
		else
			_tprintf( _T( "File: %s\n" ), argv[1] );

		Bitmap resultGDIBitmap( argv[2] );

		assert( resultGDIBitmap.GetWidth() == w );
		assert( resultGDIBitmap.GetHeight() == h );

		BitmapData resultData;
		Rect resultRect( 0, 0, w, h ); // whole image
		if( Ok != resultGDIBitmap.LockBits( &resultRect, ImageLockModeRead, PixelFormat24bppRGB, &resultData ) )
			_tprintf( _T( "Failed to lock image: %s\n" ), argv[2] );
		else
			_tprintf( _T( "File: %s\n" ), argv[2] );

		process( originalBmpData, resultData );
	}

	GdiplusShutdown( gdiplusToken );

	return 0;
}
