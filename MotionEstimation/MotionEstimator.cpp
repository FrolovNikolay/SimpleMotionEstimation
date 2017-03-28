// Автор: Николай Фролов
// Описание: Оценка вектора межкадрового сдвига.

#include <MotionEstimator.h>
#include <ctime>
#include <iostream>
#include <limits>

using namespace Gdiplus;
using namespace std;

CMotionEstimator::CMotionEstimator( int _blockSize, int _searchRadius, double _dispersionThreshold ) :
	blockSize( _blockSize ),
	searchRadius( _searchRadius ),
	dispersionThreshold( _dispersionThreshold )
{}

TMotionEstimationResult CMotionEstimator::CalculateEstimation( const BitmapData& original, const BitmapData& moved ) const
{
	time_t start = clock();

	vector<CBlockInfo> blocksForProcess;
	createBlocksForProcess( original, blocksForProcess );
	calculateBlocksEstimation( original, moved, blocksForProcess );
	TMotionVector motionVector = consolidateMotionVectors( blocksForProcess );

	time_t end = clock();
	return TMotionEstimationResult( motionVector, static_cast<double>( end - start ) / CLOCKS_PER_SEC );
}

void CMotionEstimator::createBlocksForProcess( const BitmapData& image, vector<CBlockInfo>& blocksForProcess ) const
{
	blocksForProcess.clear();
	for( size_t startX = searchRadius; startX < ( image.Width - blockSize - searchRadius ); startX += blockSize ) {
		for( size_t startY = searchRadius; startY < ( image.Height - blockSize - searchRadius ); startY += blockSize ) {
			Rect blockCoordinates( startX, startY, blockSize, blockSize );
			// Если у блока малая дисперсия, то не рассматриваем.
			if( dispersionThreshold < calculateRectDispersion( image, blockCoordinates ) ) {
				blocksForProcess.emplace_back( CBlockInfo( blockCoordinates ) );
			}
		}
	}
}

double CMotionEstimator::calculateRectDispersion( const BitmapData& image, const Rect& rect ) const
{
	double mean = calculateRectMean( image, rect );
	double dispSum = 0.0;
	BYTE* imageBuffer = ( BYTE* )image.Scan0;
	for( int currentY = rect.GetTop(); currentY < rect.GetBottom(); currentY++ ) {
		for( int currentX = rect.GetLeft(); currentX < rect.GetRight(); currentX++ ) {
			double difference = mean - imageBuffer[currentY * image.Width + currentX];
			dispSum += difference * difference;
		}
	}
	return sqrt( dispSum / ( static_cast<double>( blockSize * blockSize ) ) );
}

double CMotionEstimator::calculateRectMean( const BitmapData& image, const Rect& rect ) const
{
	double sum = 0.0;
	BYTE* imageBuffer = ( BYTE* )image.Scan0;
	for( int currentY = rect.GetTop(); currentY < rect.GetBottom(); currentY++ ) {
		for( int currentX = rect.GetLeft(); currentX < rect.GetRight(); currentX++ ) {
			sum += imageBuffer[currentY * image.Width + currentX];
		}
	}
	return sum / static_cast<double>( blockSize * blockSize );
}

void CMotionEstimator::calculateBlocksEstimation( const BitmapData& original, const BitmapData& moved,
	vector<CBlockInfo>& blocksForProcess ) const
{
	for( size_t blockIdx = 0; blockIdx < blocksForProcess.size(); blockIdx++ ) {
		int minimumDistance = INT_MAX;
		for( int deltaX = -searchRadius; deltaX <= searchRadius; deltaX++ ) {
			for( int deltaY = -searchRadius; deltaY <= searchRadius; deltaY++ ) {
				double blocksDistance = calculateBlocksDistance( original, blocksForProcess[blockIdx].Coordinates,
					moved, deltaX, deltaY );
				if( minimumDistance > blocksDistance ) {
					minimumDistance = blocksDistance;
					blocksForProcess[blockIdx].CalculatedVector = TMotionVector( deltaX, deltaY );
				}
			}
		}
	}
}

// L2 метрика без нормировки.
double CMotionEstimator::calculateBlocksDistance( const BitmapData& original, const Rect& originalRect,
	const BitmapData& moved, int deltaX, int deltaY ) const
{
	BYTE* originalImageBuffer = ( BYTE* )original.Scan0;
	BYTE* movedImageBuffer = ( BYTE* )moved.Scan0;
	double difference = 0.0;
	for( int originalY = originalRect.GetTop(); originalY < originalRect.GetBottom(); originalY++ ) {
		for( int originalX = originalRect.GetLeft(); originalX < originalRect.GetRight(); originalX++ ) {
			int movedY = originalY + deltaY;
			int movedX = originalX + deltaX;
			int delta = originalImageBuffer[originalY * original.Width + originalX]
				- movedImageBuffer[movedY * moved.Width + movedX];
			difference += delta * delta;
		}
	}
	return difference;
}

TMotionVector CMotionEstimator::consolidateMotionVectors( const vector<CBlockInfo>& processedBlocks ) const
{
	// Здесь бы придумать что-то поумнее - ничего дельного в статьях не нашел. :(
	int sumDeltaX = 0;
	int sumDeltaY = 0;
	for( size_t blockIdx = 0; blockIdx < processedBlocks.size(); blockIdx++ ) {
		sumDeltaX += processedBlocks[blockIdx].CalculatedVector.first;
		sumDeltaY += processedBlocks[blockIdx].CalculatedVector.second;
	}
	return TMotionVector( sumDeltaX / static_cast<int>( processedBlocks.size() ), sumDeltaY / static_cast<int>( processedBlocks.size() ) );
}