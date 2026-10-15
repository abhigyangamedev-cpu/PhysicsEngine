#pragma once
#include <GL/glcorearb.h>

#define PE_GL_FUNCS(X)                                                                                                 \
    X(PFNGLCLEARPROC, glClear)                                                                                         \
    X(PFNGLCLEARCOLORPROC, glClearColor)                                                                               \
    X(PFNGLENABLEPROC, glEnable)                                                                                       \
    X(PFNGLDISABLEPROC, glDisable)                                                                                     \
    X(PFNGLVIEWPORTPROC, glViewport)                                                                                   \
    X(PFNGLCULLFACEPROC, glCullFace)                                                                                   \
    X(PFNGLPOLYGONOFFSETPROC, glPolygonOffset)                                                                         \
    X(PFNGLCREATESHADERPROC, glCreateShader)                                                                           \
    X(PFNGLSHADERSOURCEPROC, glShaderSource)                                                                           \
    X(PFNGLCOMPILESHADERPROC, glCompileShader)                                                                         \
    X(PFNGLGETSHADERIVPROC, glGetShaderiv)                                                                             \
    X(PFNGLGETSHADERINFOLOGPROC, glGetShaderInfoLog)                                                                   \
    X(PFNGLDELETESHADERPROC, glDeleteShader)                                                                           \
    X(PFNGLCREATEPROGRAMPROC, glCreateProgram)                                                                         \
    X(PFNGLATTACHSHADERPROC, glAttachShader)                                                                           \
    X(PFNGLLINKPROGRAMPROC, glLinkProgram)                                                                             \
    X(PFNGLGETPROGRAMIVPROC, glGetProgramiv)                                                                           \
    X(PFNGLGETPROGRAMINFOLOGPROC, glGetProgramInfoLog)                                                                 \
    X(PFNGLUSEPROGRAMPROC, glUseProgram)                                                                               \
    X(PFNGLGETUNIFORMLOCATIONPROC, glGetUniformLocation)                                                               \
    X(PFNGLUNIFORMMATRIX4FVPROC, glUniformMatrix4fv)                                                                   \
    X(PFNGLUNIFORM3FPROC, glUniform3f)                                                                                 \
    X(PFNGLUNIFORM1IPROC, glUniform1i)                                                                                 \
    X(PFNGLGENVERTEXARRAYSPROC, glGenVertexArrays)                                                                     \
    X(PFNGLBINDVERTEXARRAYPROC, glBindVertexArray)                                                                     \
    X(PFNGLGENBUFFERSPROC, glGenBuffers)                                                                               \
    X(PFNGLBINDBUFFERPROC, glBindBuffer)                                                                               \
    X(PFNGLBUFFERDATAPROC, glBufferData)                                                                               \
    X(PFNGLVERTEXATTRIBPOINTERPROC, glVertexAttribPointer)                                                             \
    X(PFNGLENABLEVERTEXATTRIBARRAYPROC, glEnableVertexAttribArray)                                                     \
    X(PFNGLVERTEXATTRIBDIVISORPROC, glVertexAttribDivisor)                                                             \
    X(PFNGLDRAWELEMENTSINSTANCEDPROC, glDrawElementsInstanced)                                                         \
    X(PFNGLDRAWARRAYSPROC, glDrawArrays)                                                                               \
    X(PFNGLGENTEXTURESPROC, glGenTextures)                                                                             \
    X(PFNGLBINDTEXTUREPROC, glBindTexture)                                                                             \
    X(PFNGLACTIVETEXTUREPROC, glActiveTexture)                                                                         \
    X(PFNGLTEXIMAGE2DPROC, glTexImage2D)                                                                               \
    X(PFNGLTEXSUBIMAGE2DPROC, glTexSubImage2D)                                                                         \
    X(PFNGLPIXELSTOREIPROC, glPixelStorei)                                                                             \
    X(PFNGLBLENDFUNCPROC, glBlendFunc)                                                                                 \
    X(PFNGLTEXPARAMETERIPROC, glTexParameteri)                                                                         \
    X(PFNGLTEXPARAMETERFVPROC, glTexParameterfv)                                                                       \
    X(PFNGLGENFRAMEBUFFERSPROC, glGenFramebuffers)                                                                     \
    X(PFNGLBINDFRAMEBUFFERPROC, glBindFramebuffer)                                                                     \
    X(PFNGLFRAMEBUFFERTEXTURE2DPROC, glFramebufferTexture2D)                                                           \
    X(PFNGLDRAWBUFFERPROC, glDrawBuffer)                                                                               \
    X(PFNGLREADBUFFERPROC, glReadBuffer)

namespace gl {
#define PE_GL_DECLARE(type, name) extern type name;
PE_GL_FUNCS(PE_GL_DECLARE)
#undef PE_GL_DECLARE

bool load();
} // namespace gl
