# MOLA:
# Instead of including the original DBoW2 cmake file, which "polutes" the
# global space of cmake variables, let's just create here the targets that
# we need, in the parent directory of the actual DBoW2 directory:
# =================================================================
# DBoW2: bag-of-words library for C++ with generic descriptors
# Copyright (c) 2015 Dorian Galvez-Lopez. http://doriangalvez.com
# All rights reserved.
# =================================================================
# License: 3-BSD
# =================================================================
project(dbow2)

set(HDRS
  DBoW2/include/DBoW2/BowVector.h
  DBoW2/include/DBoW2/FBrief.h
  DBoW2/include/DBoW2/QueryResults.h
  DBoW2/include/DBoW2/TemplatedDatabase.h
  DBoW2/include/DBoW2/FORB.h
  DBoW2/include/DBoW2/DBoW2.h
  DBoW2/include/DBoW2/FClass.h
  DBoW2/include/DBoW2/FeatureVector.h
  DBoW2/include/DBoW2/ScoringObject.h
  DBoW2/include/DBoW2/TemplatedVocabulary.h
)
set(SRCS
  DBoW2/src/BowVector.cpp
  DBoW2/src/FBrief.cpp
  DBoW2/src/FORB.cpp
  DBoW2/src/FeatureVector.cpp
  DBoW2/src/QueryResults.cpp
  DBoW2/src/ScoringObject.cpp
)

find_package(OpenCV REQUIRED)


mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${SRCS}
	PUBLIC_LINK_LIBRARIES
	  ${OpenCV_LIBS}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/DBoW2/include/>
  $<INSTALL_INTERFACE:include>
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES
  FOLDER "ExternalProjectTargets"
)

#TODO: export cmake target
