cmake_minimum_required(VERSION 3.16)
project(mission_executor)

find_package(ament_cmake REQUIRED)

set(RUST_TARGET ${CMAKE_CURRENT_SOURCE_DIR}/target/release/${PROJECT_NAME})

add_custom_command(
  OUTPUT ${RUST_TARGET}
  COMMAND cargo build --release --quiet
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/main.rs ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
  COMMENT "Building Rust executable"
  VERBATIM
)

if(WIN32)
  set(SUFFIX ".exe")
else()
  set(SUFFIX "")
endif()
add_custom_target(build_rust ALL DEPENDS ${RUST_TARGET})

install(
  PROGRAMS ${CMAKE_SOURCE_DIR}/../../target/release/${PROJECT_NAME}${SUFFIX}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
