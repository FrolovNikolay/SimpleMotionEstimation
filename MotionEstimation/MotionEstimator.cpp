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

// Общая сложность по сумме подопераций: ~2HW * ( 1 + 2 * searchRadius^2 ) + HW / ( blockSize^2 ).
TMotionEstimationResult CMotionEstimator::CalculateEstimation( const BitmapData& original, const BitmapData& moved ) const
{
	time_t start = clock();

	vector<CBlockInfo> blocksForProcess;
	// 2WH.
	createBlocksForProcess( original, blocksForProcess );
	// 4*WH*searchRadius^2.
	double meanDistance = 0.0;
	calculateBlocksEstimation( original, moved, blocksForProcess, meanDistance );
	// HW / ( blockSize^2 )
	TMotionVector motionVector = consolidateMotionVectors( blocksForProcess, meanDistance );

	time_t end = clock();
	return TMotionEstimationResult( motionVector, static_cast<double>( end - start ) / CLOCKS_PER_SEC );
}

// 2WH.
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

// В сумме по картинке ~2WH
double CMotionEstimator::calculateRectDispersion( const BitmapData& image, const Rect& rect ) const
{
	double mean = calculateRectMean( image, rect );
	double dispSum = 0.0;
	BYTE* imageBuffer = ( BYTE* )image.Scan0;
	for( int currentY = rect.GetTop(); currentY < rect.GetBottom(); currentY++ ) {
		for( int currentX = rect.GetLeft(); currentX < rect.GetRight(); currentX++ ) {
			double difference = mean - imageBuffer[currentY * image.Stride + currentX];
			dispSum += difference * difference;
		}
	}
	return sqrt( dispSum / ( static_cast<double>( blockSize * blockSize ) ) );
}

// В сумме по картинке сложность ~WH
double CMotionEstimator::calculateRectMean( const BitmapData& image, const Rect& rect ) const
{
	double sum = 0.0;
	BYTE* imageBuffer = ( BYTE* )image.Scan0;
	for( int currentY = rect.GetTop(); currentY < rect.GetBottom(); currentY++ ) {
		for( int currentX = rect.GetLeft(); currentX < rect.GetRight(); currentX++ ) {
			sum += imageBuffer[currentY * image.Stride + currentX];
		}
	}
	return sum / static_cast<double>( blockSize * blockSize );
}

// Каждый блок картинки матчим (2 * searchRadius )^2 раз. В худшем случае, если дисперсией ничего не откинули, получаем 4*WH*searchRadius^2.
void CMotionEstimator::calculateBlocksEstimation( const BitmapData& original, const BitmapData& moved,
	vector<CBlockInfo>& blocksForProcess, double& meanDistance ) const
{
	meanDistance = 0;
	for( size_t blockIdx = 0; blockIdx < blocksForProcess.size(); blockIdx++ ) {
		blocksForProcess[blockIdx].Distance = DBL_MAX;
		for( int deltaX = -searchRadius; deltaX <= searchRadius; deltaX++ ) {
			for( int deltaY = -searchRadius; deltaY <= searchRadius; deltaY++ ) {
				double blocksDistance = calculateBlocksDistance( original, blocksForProcess[blockIdx].Coordinates,
					moved, deltaX, deltaY );
				if( blocksForProcess[blockIdx].Distance > blocksDistance ) {
					blocksForProcess[blockIdx].Distance = blocksDistance;
					blocksForProcess[blockIdx].CalculatedVector = TMotionVector( deltaX, deltaY );
				}
			}
		}
		meanDistance += blocksForProcess[blockIdx].Distance;
	}
}

// L2 метрика.
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
			int delta = originalImageBuffer[originalY * original.Stride + originalX]
				- movedImageBuffer[movedY * moved.Stride + movedX];
			difference += delta * delta;
		}
	}
	return difference / ( blockSize * blockSize );
}
// Суммируем по ~HW / ( blockSize^2 ) блокам.
TMotionVector CMotionEstimator::consolidateMotionVectors( const vector<CBlockInfo>& processedBlocks, double meanDistance ) const
{
	// Здесь бы придумать что-то поумнее - ничего дельного в статьях не нашел. :(
	double sumDeltaX = 0;
	double sumDeltaY = 0;
	int countedBlocks = 0;
	for( size_t blockIdx = 0; blockIdx < processedBlocks.size(); blockIdx++ ) {
		// Подбирал на пальцах, чтобы совсем не усреднеть что попало.
		// Считаем, что если блок сматчился лушчим образом, но все еще не очень хорошо, то его данные лучше проигнорировать.
		if( processedBlocks[blockIdx].Distance <= meanDistance / 2 ) {
			sumDeltaX += processedBlocks[blockIdx].CalculatedVector.first;
			sumDeltaY += processedBlocks[blockIdx].CalculatedVector.second;
			countedBlocks++;
		}
	}
	return TMotionVector( sumDeltaX / static_cast<double> ( countedBlocks ), sumDeltaY / static_cast< double >( countedBlocks ) );
}