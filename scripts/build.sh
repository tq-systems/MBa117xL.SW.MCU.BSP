#!/bin/bash

# ****************************************************************************
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
# D-82229 Seefeld, Germany.
# Author:    Isaac L. L. Yuki
#
# Description: Script for setting up the build system and building targets.
# ******************************************************************************

set -e
set -C # noclobber

trap 'error_abort $LINENO' ERR
function error_abort () {
	echo "error at $1"
}

# Build Configuration
# Load variables from config file
readonly SCRIPT="$(basename "${0}")"
readonly PROJECT_PATH="$(dirname "$(readlink -f "$0")")/.."
readonly CONFIG_FILE="$(dirname "$(readlink -f "$0")")/.config"

. ${CONFIG_FILE}

main() {
	# Set MCUXSDK_ROOT based on PROJECT_PATH if not explicitly defined
	if [ -z "${MCUXSDK_ROOT}" ]; then
	    MCUXSDK_ROOT="${PROJECT_PATH}/dependencies/${MCU_EXPRESSO_SDK_DIR}"
	    echo "MCUXSDK_ROOT is set to default."
	fi

	if [ -z "${CMAKE}" ]; then
	    CMAKE=$(which cmake)
	    echo "CMAKE is set to system default."
	fi

	# Step 0
	${CMAKE} --version > /dev/null

	export ARMGCC_DIR
	for type in ${BUILD_TYPES}; do
		for target in ${LINK_TARGETS}; do
			cd "${PROJECT_PATH}" || exit
			# Step 1: Generate Build System
			echo "Generating build system for ${target} (${type}) ..."
			build_dir="${PROJECT_PATH}/build-${target}-${type}"

			$CMAKE -DCMAKE_TOOLCHAIN_FILE="${MCUXSDK_ROOT}/core/tools/cmake_toolchain_files/armgcc.cmake" \
			      -DCMAKE_BUILD_TYPE="${type}" \
				  -DDETAILED_COMPILER_WARNINGS=OFF \
			      -DbootDisk="${target}" \
			      -B "${build_dir}" \
			      -G "${GENERATOR}" \
			      --fresh

			# Step 2: Build the Target
			echo "Building ..."
			$CMAKE --build "${build_dir}" --config ${type} --target all -j "$(nproc)"

			tar --create --verbose --file=${PROJECT_NAME}-${target}-${type}.tar.gz \
				--use-compress-program=gzip \
				--directory=${build_dir}/dist/${type}/ \
				 ${target}
		done
	done
	echo "Build completed ..."
}

main "$@"
