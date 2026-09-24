# The wiikit runtime, included by the generated CMakeLists.txt.
#   wiikit_core  guest memory, dispatch, hooks, the address space above 0xC0000000
#   wiikit_stub  no hardware: tests that only call the game's own code
#   wiikit_hw    the Wii replaced: OS threads and interrupts, the hardware
#                registers, IOS, the renderer (SDL3, OpenGL 4.5), the audio
#                mixer and output (SDL3); with
#                wiiboot, the game boots
# A target links wiikit_core, one of stub/hw, then recomp.
set(_rt ${CMAKE_CURRENT_LIST_DIR})
add_library(wiikit_core OBJECT ${_rt}/core.cpp ${_rt}/mem.cpp)
add_library(wiikit_stub OBJECT ${_rt}/services_stub.cpp)
add_library(wiikit_hw OBJECT ${_rt}/os.cpp ${_rt}/hw.cpp ${_rt}/gx.cpp ${_rt}/ios.cpp
            ${_rt}/disc.cpp ${_rt}/boot.cpp ${_rt}/wpad.cpp ${_rt}/ax.cpp ${_rt}/audio.cpp
            ${_rt}/video.cpp ${_rt}/gxshader.cpp ${_rt}/gxtex.cpp)
find_package(SDL3 REQUIRED CONFIG)
target_link_libraries(wiikit_hw PUBLIC SDL3::SDL3)
foreach(t wiikit_core wiikit_stub wiikit_hw)
  target_include_directories(${t} PUBLIC ${_rt})
endforeach()
add_executable(wiiboot ${_rt}/wiiboot.cpp)
target_link_libraries(wiiboot wiikit_core wiikit_hw recomp)
if(WIN32)
  target_link_libraries(wiiboot winmm)
endif()
