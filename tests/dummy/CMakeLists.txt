add_executable(dummy-test
    dummy_test.cpp
)

target_link_libraries(dummy-test
    gtest_main
)

include(GoogleTest)

if(NOT CMAKE_CROSSCOMPILING)
  gtest_discover_tests(dummy-test)
endif()

install(TARGETS dummy-test DESTINATION ${CMAKE_INSTALL_BINDIR})
