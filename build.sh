#!/bin/sh
# Copyright (c) 2019 Oscar Ostlund (oscar.ostlund@gmail.com)
#
# Distributed under the MIT License (MIT) (See accompanying file LICENSE.txt
# or copy at http://opensource.org/licenses/MIT)

# exit on firts error
set -e

mkdir -p _build
cd _build

# Generate a Makefile for GCC (or Clang, depanding on CC/CXX envvar)
cmake -DCMAKE_BUILD_TYPE=Release -DLCD_USE_GCOV=OFF -DLCD_BUILD_EXAMPLES=ON -DLCD_RUN_CPPLINT=OFF VERBOSE=ON ..

# Build (ie 'make')
cmake --build .

# Build and run unit-tests (ie 'make test')
ctest --output-on-failure
