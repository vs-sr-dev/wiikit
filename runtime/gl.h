// wiikit runtime — the OpenGL 4.5 core functions the renderer uses, loaded
// through SDL (video.cpp defines them and gl_load fills them in).
#pragma once
#include <GL/glcorearb.h>

#define WIIKIT_GL_FUNCS(X)                                                                          \
    X(PFNGLGETERRORPROC, glGetError)                                                                \
    X(PFNGLGETSTRINGPROC, glGetString)                                                              \
    X(PFNGLENABLEPROC, glEnable)                                                                    \
    X(PFNGLDISABLEPROC, glDisable)                                                                  \
    X(PFNGLVIEWPORTPROC, glViewport)                                                                \
    X(PFNGLVIEWPORTINDEXEDFPROC, glViewportIndexedf)                                                \
    X(PFNGLSCISSORPROC, glScissor)                                                                  \
    X(PFNGLCLEARPROC, glClear)                                                                      \
    X(PFNGLCLEARCOLORPROC, glClearColor)                                                            \
    X(PFNGLCLEARDEPTHPROC, glClearDepth)                                                            \
    X(PFNGLCOLORMASKPROC, glColorMask)                                                              \
    X(PFNGLDEPTHMASKPROC, glDepthMask)                                                              \
    X(PFNGLDEPTHFUNCPROC, glDepthFunc)                                                              \
    X(PFNGLBLENDFUNCSEPARATEPROC, glBlendFuncSeparate)                                              \
    X(PFNGLBLENDEQUATIONPROC, glBlendEquation)                                                      \
    X(PFNGLLOGICOPPROC, glLogicOp)                                                                  \
    X(PFNGLCULLFACEPROC, glCullFace)                                                                \
    X(PFNGLFRONTFACEPROC, glFrontFace)                                                              \
    X(PFNGLPIXELSTOREIPROC, glPixelStorei)                                                          \
    X(PFNGLDRAWARRAYSPROC, glDrawArrays)                                                            \
    X(PFNGLDRAWELEMENTSBASEVERTEXPROC, glDrawElementsBaseVertex)                                    \
    X(PFNGLCLIPCONTROLPROC, glClipControl)                                                          \
    X(PFNGLDEBUGMESSAGECALLBACKPROC, glDebugMessageCallback)                                        \
    X(PFNGLCREATESHADERPROC, glCreateShader)                                                        \
    X(PFNGLSHADERSOURCEPROC, glShaderSource)                                                        \
    X(PFNGLCOMPILESHADERPROC, glCompileShader)                                                      \
    X(PFNGLGETSHADERIVPROC, glGetShaderiv)                                                          \
    X(PFNGLGETSHADERINFOLOGPROC, glGetShaderInfoLog)                                                \
    X(PFNGLDELETESHADERPROC, glDeleteShader)                                                        \
    X(PFNGLCREATEPROGRAMPROC, glCreateProgram)                                                      \
    X(PFNGLATTACHSHADERPROC, glAttachShader)                                                        \
    X(PFNGLLINKPROGRAMPROC, glLinkProgram)                                                          \
    X(PFNGLGETPROGRAMIVPROC, glGetProgramiv)                                                        \
    X(PFNGLGETPROGRAMINFOLOGPROC, glGetProgramInfoLog)                                              \
    X(PFNGLUSEPROGRAMPROC, glUseProgram)                                                            \
    X(PFNGLGETUNIFORMLOCATIONPROC, glGetUniformLocation)                                            \
    X(PFNGLPROGRAMUNIFORM1IPROC, glProgramUniform1i)                                                \
    X(PFNGLPROGRAMUNIFORM4IPROC, glProgramUniform4i)                                                \
    X(PFNGLCREATEBUFFERSPROC, glCreateBuffers)                                                      \
    X(PFNGLNAMEDBUFFERSTORAGEPROC, glNamedBufferStorage)                                            \
    X(PFNGLNAMEDBUFFERSUBDATAPROC, glNamedBufferSubData)                                            \
    X(PFNGLBINDBUFFERBASEPROC, glBindBufferBase)                                                    \
    X(PFNGLINVALIDATEBUFFERDATAPROC, glInvalidateBufferData)                                        \
    X(PFNGLCREATEVERTEXARRAYSPROC, glCreateVertexArrays)                                            \
    X(PFNGLBINDVERTEXARRAYPROC, glBindVertexArray)                                                  \
    X(PFNGLVERTEXARRAYVERTEXBUFFERPROC, glVertexArrayVertexBuffer)                                  \
    X(PFNGLVERTEXARRAYELEMENTBUFFERPROC, glVertexArrayElementBuffer)                                \
    X(PFNGLENABLEVERTEXARRAYATTRIBPROC, glEnableVertexArrayAttrib)                                  \
    X(PFNGLVERTEXARRAYATTRIBFORMATPROC, glVertexArrayAttribFormat)                                  \
    X(PFNGLVERTEXARRAYATTRIBIFORMATPROC, glVertexArrayAttribIFormat)                                \
    X(PFNGLVERTEXARRAYATTRIBBINDINGPROC, glVertexArrayAttribBinding)                                \
    X(PFNGLCREATETEXTURESPROC, glCreateTextures)                                                    \
    X(PFNGLTEXTURESTORAGE2DPROC, glTextureStorage2D)                                                \
    X(PFNGLTEXTURESUBIMAGE2DPROC, glTextureSubImage2D)                                              \
    X(PFNGLTEXTUREPARAMETERIPROC, glTextureParameteri)                                              \
    X(PFNGLBINDTEXTUREUNITPROC, glBindTextureUnit)                                                  \
    X(PFNGLDELETETEXTURESPROC, glDeleteTextures)                                                    \
    X(PFNGLGETTEXTUREIMAGEPROC, glGetTextureImage)                                                  \
    X(PFNGLREADPIXELSPROC, glReadPixels)                                                            \
    X(PFNGLCREATESAMPLERSPROC, glCreateSamplers)                                                    \
    X(PFNGLSAMPLERPARAMETERIPROC, glSamplerParameteri)                                              \
    X(PFNGLSAMPLERPARAMETERFPROC, glSamplerParameterf)                                              \
    X(PFNGLBINDSAMPLERPROC, glBindSampler)                                                          \
    X(PFNGLCREATEFRAMEBUFFERSPROC, glCreateFramebuffers)                                            \
    X(PFNGLNAMEDFRAMEBUFFERTEXTUREPROC, glNamedFramebufferTexture)                                  \
    X(PFNGLBINDFRAMEBUFFERPROC, glBindFramebuffer)                                                  \
    X(PFNGLBLITNAMEDFRAMEBUFFERPROC, glBlitNamedFramebuffer)                                        \
    X(PFNGLCHECKNAMEDFRAMEBUFFERSTATUSPROC, glCheckNamedFramebufferStatus)

#define WIIKIT_GL_DECLARE(T, n) extern T n;
WIIKIT_GL_FUNCS(WIIKIT_GL_DECLARE)
#undef WIIKIT_GL_DECLARE
