// Автор: Николай Фролов
// Описание: Считывание картинки.

#include <PictureReader.h>
#include <cassert>

using namespace Gdiplus;

CPictureReader::CBitmapHolder::CBitmapHolder( const std::wstring& path ) :
	locked( false ),
	bitmap( path._Myptr() )
{}

CPictureReader::CBitmapHolder::~CBitmapHolder()
{
	if( locked ) {
		bitmap.UnlockBits( &data );
	}
}

Gdiplus::BitmapData* CPictureReader::CBitmapHolder::GetData( TColorFormat format )
{
	if( locked ) {
		// Вообще так использовать не планируется, но лишним не будет.
		return &data;
	}

	int width = bitmap.GetWidth();
	int height = bitmap.GetHeight();
	Rect rect( 0, 0, width, height );
	if( bitmap.LockBits( &rect, ImageLockModeRead | ImageLockModeWrite, getPixelFormat( format ), &data ) != Ok ) {
		return 0;
	} else {
		locked = true;
		return &data;
	}
}

PixelFormat CPictureReader::CBitmapHolder::getPixelFormat( TColorFormat format )
{
	switch( format ) {
		case CF_Gray:
			return PixelFormat8bppIndexed;
		default:
			assert( false );
			return 0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Gdiplus::BitmapData* CPictureReader::LoadImage( const std::wstring& path, TColorFormat format )
{
	lockedImages.emplace_back( std::shared_ptr<CBitmapHolder>( new CBitmapHolder( path ) ) );
	return lockedImages.rbegin()->get()->GetData( format );
}