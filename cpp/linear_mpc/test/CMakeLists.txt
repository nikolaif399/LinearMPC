#macro for conveniently adding tests and linking against correct libs
macro(package_add_test TESTNAME)
    # Add gtest based cpp test target and link libraries
    add_executable(${TESTNAME} ${ARGN})
    target_link_libraries(${TESTNAME} gtest gtest_main linear_mpc)
    add_test(NAME ${TESTNAME} COMMAND ${TESTNAME})
    set_target_properties(${TESTNAME} PROPERTIES FOLDER test)
    list(APPEND UNIT_TEST_TARGETS ${TESTNAME})
endmacro()

add_subdirectory(unit)

## install tests
include(GNUInstallDirs)
install(
    TARGETS ${UNIT_TEST_TARGETS}
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}        
)