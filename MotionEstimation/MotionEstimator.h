// Автор: Николай Фролов
// Описание: Оценка вектора межкадрового сдвига.

#pragma once

#include <Windows.h>
#include <gdiplus.h>
#include <utility>
#include <vector>

// Вектор смещения.
typedef std::pair<int, int> TMotionVector;
// Вектор смещения + время работы.
typedef std::pair<TMotionVector, double> TMotionEstimationResult;

class CMotionEstimator {
public:
	CMotionEstimator( int blockSize, int searchRadius, double dispersionThreshold );

	TMotionEstimationResult CalculateEstimation( const Gdiplus::BitmapData& original, const Gdiplus::BitmapData& moved ) const;

private:
	struct CBlockInfo {
	public:
		explicit CBlockInfo( const Gdiplus::Rect& coordinates ) :
			Coordinates( coordinates )
		{}

		Gdiplus::Rect Coordinates;
		TMotionVector CalculatedVector;
	};

	int blockSize; // размеры блока.
	int searchRadius; // радиус поиска. Регион поиска имеет размеры blockSize + 2 * searchRadius.
	double dispersionThreshold; // минимальный порог дисперсии блока. Блоки с малой дисперсией рассматривать не будем.

	// Вычленяем блоки для обработки.
	void createBlocksForProcess( const Gdiplus::BitmapData& image, std::vector<CBlockInfo>& blocksForProcess ) const;
	// Рассчет дисперсии блока.
	double calculateRectDispersion( const Gdiplus::BitmapData& image, const Gdiplus::Rect& rect ) const;
	double calculateRectMean( const Gdiplus::BitmapData& image, const Gdiplus::Rect& rect ) const;
	// Рассчет вектора сдвига поблоково. 
	void calculateBlocksEstimation( const Gdiplus::BitmapData& original, const Gdiplus::BitmapData& moved,
		std::vector<CBlockInfo>& blocksForProcess ) const;
	// Межблоковое расстояние.
	double calculateBlocksDistance( const Gdiplus::BitmapData& original, const Gdiplus::Rect& originalRect,
		const Gdiplus::BitmapData& moved, int deltaX, int deltaY ) const;
	// Итоговый вектор сдвига изображения на основе обработнных блоков.
	TMotionVector consolidateMotionVectors( const std::vector<CBlockInfo>& processedBlocks ) const;
};