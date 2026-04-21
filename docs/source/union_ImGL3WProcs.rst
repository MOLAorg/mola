.. index:: pair: union; ImGL3WProcs
.. _doxid-union_im_g_l3_w_procs:

union ImGL3WProcs
=================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <imgui_impl_opengl3_loader.h>
	
	union ImGL3WProcs
	{
		// fields
	
		:ref:`GL3WglProc<doxid-imgui__impl__opengl3__loader_8h_1ab3b2dd5b9cba7f31de0317722d9375b7>` :target:`ptr<doxid-union_im_g_l3_w_procs_1a332efdc371d8ffb91e74105620208117>`[63];
		PFNGLACTIVETEXTUREPROC :target:`ActiveTexture<doxid-union_im_g_l3_w_procs_1a8de6d13ae8fc9381bd058eaeec875e46>`;
		PFNGLATTACHSHADERPROC :target:`AttachShader<doxid-union_im_g_l3_w_procs_1ab0f8a1c6d263bb17d757eece35a8e690>`;
		PFNGLBINDBUFFERPROC :target:`BindBuffer<doxid-union_im_g_l3_w_procs_1a9f109b39a0370d042a263b4a5bc622e6>`;
		PFNGLBINDSAMPLERPROC :target:`BindSampler<doxid-union_im_g_l3_w_procs_1ae1b4dabb1cb52cb3a127170a78fdce5e>`;
		PFNGLBINDTEXTUREPROC :target:`BindTexture<doxid-union_im_g_l3_w_procs_1a524cc7f2cf2a47a32cd0ffcb6a62e17d>`;
		PFNGLBINDVERTEXARRAYPROC :target:`BindVertexArray<doxid-union_im_g_l3_w_procs_1a36ec67db1be93e971140d9352cbcf641>`;
		PFNGLBLENDEQUATIONPROC :target:`BlendEquation<doxid-union_im_g_l3_w_procs_1aa67f8e7fa99adb51d4af9fdb120720f1>`;
		PFNGLBLENDEQUATIONSEPARATEPROC :target:`BlendEquationSeparate<doxid-union_im_g_l3_w_procs_1a987a46d8626b9598d5f9c169dc9ef4c0>`;
		PFNGLBLENDFUNCSEPARATEPROC :target:`BlendFuncSeparate<doxid-union_im_g_l3_w_procs_1a198e9efc80ccffb94f8b767374001142>`;
		PFNGLBUFFERDATAPROC :target:`BufferData<doxid-union_im_g_l3_w_procs_1a045e3870ea7fb49da845876f996e097f>`;
		PFNGLBUFFERSUBDATAPROC :target:`BufferSubData<doxid-union_im_g_l3_w_procs_1a6406796e5fd236945c3fc8ea3835c3b0>`;
		PFNGLCLEARPROC :target:`Clear<doxid-union_im_g_l3_w_procs_1ae1f4ac6bfb7550c40248974c2df8a602>`;
		PFNGLCLEARCOLORPROC :target:`ClearColor<doxid-union_im_g_l3_w_procs_1abefd40974c62c03eee4dcca327f5a878>`;
		PFNGLCOMPILESHADERPROC :target:`CompileShader<doxid-union_im_g_l3_w_procs_1a5f37447f46f6203124a4adba82aada51>`;
		PFNGLCREATEPROGRAMPROC :target:`CreateProgram<doxid-union_im_g_l3_w_procs_1a8257a19bfe45f2e16bc6278d8140732b>`;
		PFNGLCREATESHADERPROC :target:`CreateShader<doxid-union_im_g_l3_w_procs_1ab58e6354f62ad6a60bd91981adbb425d>`;
		PFNGLDELETEBUFFERSPROC :target:`DeleteBuffers<doxid-union_im_g_l3_w_procs_1a21f8b9b279f9f360c3fea73bd9b806df>`;
		PFNGLDELETEPROGRAMPROC :target:`DeleteProgram<doxid-union_im_g_l3_w_procs_1afc34a93712a985d2bd99e48c924cc9aa>`;
		PFNGLDELETESAMPLERSPROC :target:`DeleteSamplers<doxid-union_im_g_l3_w_procs_1a02fdc55f3f284b912f780848c468a7c6>`;
		PFNGLDELETESHADERPROC :target:`DeleteShader<doxid-union_im_g_l3_w_procs_1a4c661060128d1d0ff8c0ef36ec36d2d7>`;
		PFNGLDELETETEXTURESPROC :target:`DeleteTextures<doxid-union_im_g_l3_w_procs_1ab11376688525d1f30b3f3fdb951a4a33>`;
		PFNGLDELETEVERTEXARRAYSPROC :target:`DeleteVertexArrays<doxid-union_im_g_l3_w_procs_1aa36cc089a00829a19d2743a7352f4abe>`;
		PFNGLDETACHSHADERPROC :target:`DetachShader<doxid-union_im_g_l3_w_procs_1a83b8b2656e4b0dd5df33c68d1ad7ada2>`;
		PFNGLDISABLEPROC :target:`Disable<doxid-union_im_g_l3_w_procs_1aa10a09d5b6dc8fac252a6f1beb6806ea>`;
		PFNGLDISABLEVERTEXATTRIBARRAYPROC :target:`DisableVertexAttribArray<doxid-union_im_g_l3_w_procs_1ae3f7cf1a46d65f209adb34e395bab032>`;
		PFNGLDRAWELEMENTSPROC :target:`DrawElements<doxid-union_im_g_l3_w_procs_1a520d59c638470205c4b481098e55bc20>`;
		PFNGLDRAWELEMENTSBASEVERTEXPROC :target:`DrawElementsBaseVertex<doxid-union_im_g_l3_w_procs_1a0c495e775cdb6058aeacf56c89d3f193>`;
		PFNGLENABLEPROC :target:`Enable<doxid-union_im_g_l3_w_procs_1afbec2ef597b4d26a248b71fc402543b8>`;
		PFNGLENABLEVERTEXATTRIBARRAYPROC :target:`EnableVertexAttribArray<doxid-union_im_g_l3_w_procs_1a20e96b3d1517a58b6077f4651c4622d0>`;
		PFNGLFLUSHPROC :target:`Flush<doxid-union_im_g_l3_w_procs_1aaf816d086c23be22a7f8f61691d2d998>`;
		PFNGLGENBUFFERSPROC :target:`GenBuffers<doxid-union_im_g_l3_w_procs_1a5da63e6c23da9e8b97020ae4b2df5100>`;
		PFNGLGENSAMPLERSPROC :target:`GenSamplers<doxid-union_im_g_l3_w_procs_1adb5101ed97f8ffcce448d676eb14ba26>`;
		PFNGLGENTEXTURESPROC :target:`GenTextures<doxid-union_im_g_l3_w_procs_1a23c50e367e10e6746b76410e5dc0796c>`;
		PFNGLGENVERTEXARRAYSPROC :target:`GenVertexArrays<doxid-union_im_g_l3_w_procs_1a363f25bd1ab58cab9e2e0e9830b76884>`;
		PFNGLGETATTRIBLOCATIONPROC :target:`GetAttribLocation<doxid-union_im_g_l3_w_procs_1af9ae6b66091256c9330a813b0ec883c0>`;
		PFNGLGETERRORPROC :target:`GetError<doxid-union_im_g_l3_w_procs_1ad1aaec7dcae9e1670f2c365639957f5e>`;
		PFNGLGETINTEGERVPROC :target:`GetIntegerv<doxid-union_im_g_l3_w_procs_1a4889b2694b0e6a4a20c9e9e9351c7369>`;
		PFNGLGETPROGRAMINFOLOGPROC :target:`GetProgramInfoLog<doxid-union_im_g_l3_w_procs_1a816123c6fdd99c5ef1252659f47cd559>`;
		PFNGLGETPROGRAMIVPROC :target:`GetProgramiv<doxid-union_im_g_l3_w_procs_1a3163cd54049a9e8adc7a0c59007dc82f>`;
		PFNGLGETSHADERINFOLOGPROC :target:`GetShaderInfoLog<doxid-union_im_g_l3_w_procs_1a52e4e72602abf8489065773bc3c1653a>`;
		PFNGLGETSHADERIVPROC :target:`GetShaderiv<doxid-union_im_g_l3_w_procs_1a9a160093774d139e60be039445573c41>`;
		:ref:`PFNGLGETSTRINGPROC<doxid-imgui__impl__opengl3__loader_8h_1ada30dc3fccd01b4d8b4744092b7bc93f>` :target:`GetString<doxid-union_im_g_l3_w_procs_1a5d2756731243c11575d691c7dfa16e30>`;
		:ref:`PFNGLGETSTRINGIPROC<doxid-imgui__impl__opengl3__loader_8h_1a75bad8d467f959c6d5b724ed03c0d092>` :target:`GetStringi<doxid-union_im_g_l3_w_procs_1a34894929f415caa15c2d3261d4e4212c>`;
		PFNGLGETUNIFORMLOCATIONPROC :target:`GetUniformLocation<doxid-union_im_g_l3_w_procs_1a35f965ba92a706c98e891cf36a55d906>`;
		PFNGLGETVERTEXATTRIBPOINTERVPROC :target:`GetVertexAttribPointerv<doxid-union_im_g_l3_w_procs_1abeffb19d607a5ea47725379599a4a7e9>`;
		PFNGLGETVERTEXATTRIBIVPROC :target:`GetVertexAttribiv<doxid-union_im_g_l3_w_procs_1a3ad0430760e672ea731fc04b63c2c9d4>`;
		PFNGLISENABLEDPROC :target:`IsEnabled<doxid-union_im_g_l3_w_procs_1aac7ad0f2970978d316941601bddaa906>`;
		PFNGLISPROGRAMPROC :target:`IsProgram<doxid-union_im_g_l3_w_procs_1ad38a8b3eda897e9b93115a054f43989a>`;
		PFNGLLINKPROGRAMPROC :target:`LinkProgram<doxid-union_im_g_l3_w_procs_1af79251d573cca52e1977a304f8acb0f6>`;
		PFNGLPIXELSTOREIPROC :target:`PixelStorei<doxid-union_im_g_l3_w_procs_1a10a68da6e7217440e51121178edd1e10>`;
		PFNGLPOLYGONMODEPROC :target:`PolygonMode<doxid-union_im_g_l3_w_procs_1a9778aff8127e9dc4e50d7c963dd11822>`;
		PFNGLREADPIXELSPROC :target:`ReadPixels<doxid-union_im_g_l3_w_procs_1af062bdc89cd28e7ac6fff3f35c808da3>`;
		PFNGLSAMPLERPARAMETERIPROC :target:`SamplerParameteri<doxid-union_im_g_l3_w_procs_1afd122c7f9eb5d48367425e03eaf0c8ea>`;
		PFNGLSCISSORPROC :target:`Scissor<doxid-union_im_g_l3_w_procs_1a48d9715041a3f506e08c84ec244ace32>`;
		PFNGLSHADERSOURCEPROC :target:`ShaderSource<doxid-union_im_g_l3_w_procs_1a84cd318db8780f0cb566a57870b2ceb7>`;
		PFNGLTEXIMAGE2DPROC :target:`TexImage2D<doxid-union_im_g_l3_w_procs_1a6dc07e2f49b2bf873fbe0c84fe1e398b>`;
		PFNGLTEXPARAMETERIPROC :target:`TexParameteri<doxid-union_im_g_l3_w_procs_1a784966765c36446050494a0e35c8fa87>`;
		PFNGLTEXSUBIMAGE2DPROC :target:`TexSubImage2D<doxid-union_im_g_l3_w_procs_1a983556e8b26aac2754267ef526d78008>`;
		PFNGLUNIFORM1IPROC :target:`Uniform1i<doxid-union_im_g_l3_w_procs_1aa3269caceaec09b127985d786e5689ec>`;
		PFNGLUNIFORMMATRIX4FVPROC :target:`UniformMatrix4fv<doxid-union_im_g_l3_w_procs_1a4a4589063c11e88d151c3e63fe208c9f>`;
		PFNGLUSEPROGRAMPROC :target:`UseProgram<doxid-union_im_g_l3_w_procs_1a3ff636f93be8c25e0c8690a90a739e23>`;
		PFNGLVERTEXATTRIBPOINTERPROC :target:`VertexAttribPointer<doxid-union_im_g_l3_w_procs_1a229722ab5765b9c808a1c2bb6cff914c>`;
		PFNGLVIEWPORTPROC :target:`Viewport<doxid-union_im_g_l3_w_procs_1ae65c541ae10af47f8d00dfc7badbce10>`;
		struct ImGL3WProcs::@0 :target:`gl<doxid-union_im_g_l3_w_procs_1a71abd9a5f298e14a3408fa49bfffc754>`;
	};
