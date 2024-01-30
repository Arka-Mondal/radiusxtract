cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(radiusxtract)

find_package(VTK COMPONENTS
  CommonColor
  CommonCore
  FiltersSources
  InteractionStyle
  RenderingContextOpenGL2
  RenderingCore
  RenderingFreeType
  RenderingGL2PSOpenGL2
  RenderingOpenGL2
  GUISupportQt
)

find_package(Qt5 COMPONENTS Widgets REQUIRED)  # Add QT components

find_package(PCL COMPONENTS REQUIRED) # Add PCL components

if (NOT VTK_FOUND)
  message(FATAL_ERROR "radiusxtract: Unable to find the VTK build folder.")
endif()

if (NOT Qt5_FOUND)
  message(FATAL_ERROR "radiusxtract: Unable to find Qt5.")
endif()

add_executable(radiusxtract main.cpp)

# Include directories
target_include_directories(radiusxtract PRIVATE
  ${VTK_INCLUDE_DIRS}
)

target_include_directories(radiusxtract PRIVATE
  ${PCL_INCLUDE_DIRS}
)

# Link QT libraries
target_link_libraries(radiusxtract PRIVATE
  ${VTK_LIBRARIES}
  ${PCL_LIBRARIES}
  Qt5::Widgets
)

# vtk_module_autoinit is needed
vtk_module_autoinit(
  TARGETS radiusxtract
  MODULES ${VTK_LIBRARIES}
)

# Set C++ standard
set_target_properties(radiusxtract PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)

# Enable automoc for Qt
set_target_properties(radiusxtract PROPERTIES AUTOMOC ON)
