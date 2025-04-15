@echo off

if not exist ..\build mkdir ..\build
pushd ..\build

set SRC=..\src\main.cpp 
set IMGUI=imgui.obj imgui_draw.obj imgui_tables.obj imgui_widgets.obj imgui_impl_glfw.obj imgui_impl_opengl3.obj
call cl -std:c++17 -I..\src\vendor -arch:AVX2 -Od -Zi -W4 -wd4100 -wd4189 -wd4201 -wd4005 -wd4456 -wd4505 -nologo %SRC% -Fe:main.exe /link glad.obj glfw3dll.lib glew32.lib %IMGUI%

popd
