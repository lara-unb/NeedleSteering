cmake_minimum_required (VERSION 2.4)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)
project (Needle_Segmentation)

list (APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
find_package (OpenIGTLink REQUIRED)
include(${OpenIGTLink_USE_FILE})
set (LIBS ${LIBS} OpenIGTLink)

find_package (OpenCV REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS})
set (LIBS ${LIBS} ${OpenCV_LIBS})

#Bring the headers, such as Student.h into the project
#include_directories (include)
 
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable (ProcImagem src/ProcImagem.cxx)
target_link_libraries (ProcImagem ${LIBS})

add_executable (ProcVideo src/ProcVideo.cxx)
target_link_libraries (ProcVideo ${LIBS})

add_executable (Cliente src/Cliente.cxx)
target_link_libraries (Cliente ${LIBS})


