// #include <GL/gl3w.h>

#include "TracyTexture.hpp"

#ifndef COMPRESSED_RGB_S3TC_DXT1_EXT
#  define COMPRESSED_RGB_S3TC_DXT1_EXT 0x83F0
#endif

namespace tracy
{

void* MakeTexture()
{
    // GLuint tex;
    // glGenTextures( 1, &tex );
    // glBindTexture( GL_TEXTURE_2D, tex );
    // glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    // glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    // return (void*)(intptr_t)tex;
    return nullptr;
}

void FreeTexture( void* _tex )
{
    // auto tex = (GLuint)(intptr_t)_tex;
    // glDeleteTextures( 1, &tex );
}

void UpdateTexture( void* _tex, const char* data, int w, int h )
{
    // auto tex = (GLuint)(intptr_t)_tex;
    // glBindTexture( GL_TEXTURE_2D, tex );
    // glCompressedTexImage2D( GL_TEXTURE_2D, 0, etc ? GL_COMPRESSED_RGB8_ETC2 : COMPRESSED_RGB_S3TC_DXT1_EXT, w, h, 0, w * h / 2, data );
}

}
