// Автор: Николай Фролов
// Описание: Считывание картинки.

#pragma once

#include <windows.h>
#include <gdiplus.h>
#include <vector>
#include <string>
#include <memory>

// В какой цветности считывать.
enum TColorFormat {
	CF_Gray,

	CF_Count
};

class CPictureReader {
public:
	Gdiplus::BitmapData* LoadImage( const std::wstring& path, TColorFormat format );

private:
	class CBitmapHolder {
	public:
		explicit CBitmapHolder( const std::wstring& path );
		~CBitmapHolder();

		Gdiplus::BitmapData* GetData( TColorFormat format );

	private:
		bool locked;
		Gdiplus::Bitmap bitmap;
		Gdiplus::BitmapData data;

		static Gdiplus::PixelFormat getPixelFormat( TColorFormat format );
	};

	std::vector< std::shared_ptr<CBitmapHolder> > lockedImages;
};