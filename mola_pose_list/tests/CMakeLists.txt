# Unit tests:
mola_add_test(
  TARGET  test-searchable-pose-list
  SOURCES test-searchable-pose-list.cpp
  LINK_LIBRARIES
    mola::mola_pose_list
)

mola_add_test(
  TARGET  test-hashed-set-se3
  SOURCES test-hashed-set-se3.cpp
  LINK_LIBRARIES
    mola::mola_pose_list
)
