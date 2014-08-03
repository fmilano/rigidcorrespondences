cmake_minimum_required(VERSION 2.6)

PROJECT(RigidCorrespondences)

FIND_PACKAGE(VTK REQUIRED)

# Set your files and resources here  
SET(RIGID_CORRESPONDENCES_SRC RigidCorrespondences.cxx)

ADD_EXECUTABLE(rigidCorrespondences ${RIGID_CORRESPONDENCES_SRC})

TARGET_LINK_LIBRARIES(rigidCorrespondences ${VTK_LIBRARIES})
