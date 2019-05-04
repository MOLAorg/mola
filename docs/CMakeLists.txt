# Build Sphinx docs:
add_custom_target(build_docs ALL
    ${SPHINX_EXECUTABLE} -b html
    ${CMAKE_CURRENT_SOURCE_DIR}/source
    ${CMAKE_CURRENT_BINARY_DIR}/html
    COMMENT "Building documentation"
)

# TODO: Recursive build docs of modules?
