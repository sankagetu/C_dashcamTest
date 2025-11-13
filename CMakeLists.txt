cmake_minimum_required(VERSION 3.16)
project(c_drvassist C)

# C標準を固定
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ---------- ライブラリ（テスト対象） ----------
add_library(drvassist src/drvassist.c)
target_include_directories(drvassist PUBLIC ${CMAKE_SOURCE_DIR}/include)

# Debugビルド時のみカバレッジ計装を付与（本体ライブラリ）
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(drvassist PRIVATE --coverage -O0 -g)
  target_link_options(drvassist PRIVATE --coverage)
endif()

# ---------- テスト ----------
enable_testing()

add_executable(test_drvassist tests/test_drvassist.c)
target_link_libraries(test_drvassist PRIVATE drvassist criterion)

# 実行ファイル側にも --coverage を付与（gcovランタイムを引き込むため）
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_link_options(test_drvassist PRIVATE --coverage)
endif()

add_test(NAME unit COMMAND test_drvassist)

# （任意）警告を少し厳しめにしたい場合は以下を有効化
# target_compile_options(drvassist PRIVATE -Wall -Wextra -Wpedantic)
# target_compile_options(test_drvassist PRIVATE -Wall -Wextra -Wpedantic)
