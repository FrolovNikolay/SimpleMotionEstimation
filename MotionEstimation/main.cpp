// Автор: Николай Фролов.
// Описание: оценка межкадрового смещения изображений.

#include <PictureReader.h>
#include <MotionEstimator.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>

using namespace Gdiplus;
using namespace std;

int main( int argc, char* argv[] )
{
	if( argc != 4 ) {
		cout << "Usage: MotionEstimation <ImagePaths.txt> <ResultVectorsPath.txt> <WorkTimesPath.txt>" << endl;
		system( "pause" );
		return 0;
	}
	// Инициализация Gdiplus.
	GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR gdiplusToken;
	GdiplusStartup( &gdiplusToken, &gdiplusStartupInput, NULL );

	// Считывание картинок.
	wifstream imagePathsFile( argv[1] );
	vector<wstring> imagePaths;
	wstring lastPath;
	while( imagePathsFile >> lastPath ) {
		imagePaths.push_back( lastPath );
	}
	CPictureReader pictureReader;
	vector<BitmapData*> pictureData;
	for( size_t pictureIdx = 0; pictureIdx < imagePaths.size(); pictureIdx++ ) {
		BitmapData* data = pictureReader.LoadImage( imagePaths[pictureIdx], CF_Gray );
		if( data != 0 ) {
			pictureData.push_back( data );
		} else {
			wcerr << L"Failed to load image:" + imagePaths[pictureIdx] << endl;
		}
	}
	if( imagePaths.size() != pictureData.size() ) {
		wcerr << L"Some images was not load." << endl;
		system( "pause" );
		return 0;
	}

	// Рассчет вектора.
	// Понастраивать бы параметры, да не на чем :(
	static const int blockSize = 16;
	static const int searchRadius = 7;
	static const double dispersionThreshold = 80;
	CMotionEstimator motionEstimator( blockSize, searchRadius, dispersionThreshold );
	vector<TMotionEstimationResult> results;
	for( size_t pictureIdx = 1; pictureIdx < pictureData.size(); pictureIdx++ ) {
		results.push_back( motionEstimator.CalculateEstimation( *pictureData[pictureIdx - 1], *pictureData[pictureIdx] ) );
	}

	// Вывод результатов.
	wofstream motionVectors( argv[2] );
	wofstream workTimes( argv[3] );
	TMotionVector totalVectorToFirst( 0, 0 );
	for( auto result : results ) {
		totalVectorToFirst.first += result.first.first;
		totalVectorToFirst.second += result.first.second;
		motionVectors << L"( " << totalVectorToFirst.first << L", " << totalVectorToFirst.second << " ) " << endl;
		workTimes << result.second << endl;
	}

	system( "pause" );
	return 0;
}