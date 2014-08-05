cmake_minimum_required(VERSION 2.6)

PROJECT(RigidCorrespondences)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

#FIND_PACKAGE(vtkDiscreteRemeshing REQUIRED)
#INCLUDE(${vtkDiscreteRemeshing_USE_FILE})

#FIND_PACKAGE(vtkSurface REQUIRED)
#INCLUDE(${vtkSurface_USE_FILE})

# Set your files and resources here  
SET(RIGID_CORRESPONDENCES_SRC RigidCorrespondences.cxx)
SET(ICP_SRC icp.cxx)

ADD_EXECUTABLE(rigidCorrespondences ${RIGID_CORRESPONDENCES_SRC})
ADD_EXECUTABLE(icp ${ICP_SRC})


TARGET_LINK_LIBRARIES(rigidCorrespondences ${VTK_LIBRARIES})
TARGET_LINK_LIBRARIES(icp ${VTK_LIBRARIES})
