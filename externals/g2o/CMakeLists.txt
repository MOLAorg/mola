# MOLA:
# Instead of including the original g2o cmake file, which "polutes" the
# global space of cmake variables, let's just create here the targets that
# we need, in the parent directory of the actual g2o directory:
# =================================================================
# Rainer Kuemmerle, Giorgio Grisetti, Hauke Strasdat,
# Kurt Konolige, and Wolfram Burgard
# g2o: A General Framework for Graph Optimization
# =================================================================
# License: 3-BSD
# (We exclude here for MOLA the non 3-BSD parts)
# =================================================================
project(g2o)

configure_file(g2o/config.h.in ${g2o_BINARY_DIR}/g2o/config.h)


set(SRCS
  #types
  g2o/g2o/types/sba/types_sba.h
  g2o/g2o/types/sba/types_six_dof_expmap.h
  g2o/g2o/types/sba/types_sba.cpp
  g2o/g2o/types/sba/types_six_dof_expmap.cpp
  g2o/g2o/types/sim3/types_seven_dof_expmap.cpp
  g2o/g2o/types/sim3/types_seven_dof_expmap.h
  g2o/g2o/types/slam3d/se3quat.h
  g2o/g2o/types/slam3d/se3_ops.h
  g2o/g2o/types/slam3d/se3_ops.hpp
  #core
  g2o/g2o/core/base_edge.h
  g2o/g2o/core/base_binary_edge.h
  g2o/g2o/core/hyper_graph_action.cpp
  g2o/g2o/core/base_binary_edge.hpp
  g2o/g2o/core/hyper_graph_action.h
  g2o/g2o/core/base_multi_edge.h
  g2o/g2o/core/hyper_graph.cpp
  g2o/g2o/core/base_multi_edge.hpp
  g2o/g2o/core/hyper_graph.h
  g2o/g2o/core/base_unary_edge.h
  g2o/g2o/core/linear_solver.h
  g2o/g2o/core/base_unary_edge.hpp
  g2o/g2o/core/marginal_covariance_cholesky.cpp
  g2o/g2o/core/base_vertex.h
  g2o/g2o/core/marginal_covariance_cholesky.h
  g2o/g2o/core/base_vertex.hpp
  g2o/g2o/core/matrix_structure.cpp
  g2o/g2o/core/batch_stats.cpp
  g2o/g2o/core/matrix_structure.h
  g2o/g2o/core/batch_stats.h
  g2o/g2o/core/openmp_mutex.h
  g2o/g2o/core/block_solver.h
  g2o/g2o/core/block_solver.hpp
  g2o/g2o/core/parameter.cpp
  g2o/g2o/core/parameter.h
  g2o/g2o/core/cache.cpp
  g2o/g2o/core/cache.h
  g2o/g2o/core/optimizable_graph.cpp
  g2o/g2o/core/optimizable_graph.h
  g2o/g2o/core/solver.cpp
  g2o/g2o/core/solver.h
  g2o/g2o/core/creators.h
  g2o/g2o/core/optimization_algorithm_factory.cpp
  g2o/g2o/core/estimate_propagator.cpp
  g2o/g2o/core/optimization_algorithm_factory.h
  g2o/g2o/core/estimate_propagator.h
  g2o/g2o/core/factory.cpp
  g2o/g2o/core/optimization_algorithm_property.h
  g2o/g2o/core/factory.h
  g2o/g2o/core/sparse_block_matrix.h
  g2o/g2o/core/sparse_optimizer.cpp
  g2o/g2o/core/sparse_block_matrix.hpp
  g2o/g2o/core/sparse_optimizer.h
  g2o/g2o/core/hyper_dijkstra.cpp
  g2o/g2o/core/hyper_dijkstra.h
  g2o/g2o/core/parameter_container.cpp
  g2o/g2o/core/parameter_container.h
  g2o/g2o/core/optimization_algorithm.cpp
  g2o/g2o/core/optimization_algorithm.h
  g2o/g2o/core/optimization_algorithm_with_hessian.cpp
  g2o/g2o/core/optimization_algorithm_with_hessian.h
  g2o/g2o/core/optimization_algorithm_levenberg.cpp
  g2o/g2o/core/optimization_algorithm_levenberg.h
  g2o/g2o/core/jacobian_workspace.cpp
  g2o/g2o/core/jacobian_workspace.h
  g2o/g2o/core/robust_kernel.cpp
  g2o/g2o/core/robust_kernel.h
  g2o/g2o/core/robust_kernel_factory.cpp
  g2o/g2o/core/robust_kernel_factory.h
  g2o/g2o/core/robust_kernel_impl.cpp
  g2o/g2o/core/robust_kernel_impl.h
  #stuff
  g2o/g2o/stuff/string_tools.h
  g2o/g2o/stuff/color_macros.h
  g2o/g2o/stuff/macros.h
  g2o/g2o/stuff/timeutil.cpp
  g2o/g2o/stuff/misc.h
  g2o/g2o/stuff/timeutil.h
  g2o/g2o/stuff/os_specific.c
  g2o/g2o/stuff/os_specific.h
  g2o/g2o/stuff/string_tools.cpp
  g2o/g2o/stuff/property.cpp
  g2o/g2o/stuff/property.h
)

add_library(${PROJECT_NAME}
  ${SRCS}
)

# Was: FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)
find_package(mrpt-math REQUIRED) # to ensure we use the same Eigen version than MRPT
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  mrpt::math
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${g2o_BINARY_DIR}  # for config.h (JLBC for MOLA)
  ${g2o_SOURCE_DIR}/g2o/
)

# Set optimization & warning-level flags
mola_set_target_build_options(${PROJECT_NAME})

# Silent tons of warnings from the original code:
if (CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-unused-result" "-Wno-shadow" "-Wno-sign-compare")
endif()


set_target_properties(${PROJECT_NAME}
  PROPERTIES
  FOLDER "ExternalProjectTargets"
)
