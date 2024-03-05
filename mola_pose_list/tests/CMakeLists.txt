# Unit tests:
mola_add_test(
  TARGET  test-searchable-pose-list
  SOURCES test-searchable-pose-list.cpp
  LINK_LIBRARIES
    mola::mola_pose_list
)
