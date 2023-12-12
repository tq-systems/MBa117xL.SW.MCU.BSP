#!/bin/bash
# ****************************************************************************
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
# D-82229 Seefeld, Germany.
# Author:    Isaac L. L. Yuki
#
# Description: Script for setting up the mcuxsdk.
#******************************************************************************

set -e
set -C # noclobber

trap 'error_abort $LINENO' ERR
function error_abort () {
	echo "error at $1"
}

readonly SCRIPT="$(basename "${0}")"
readonly PROJECT_PATH="$(dirname "$(readlink -f "$0")")/.."
readonly CONFIG_FILE="$(dirname "$(readlink -f "$0")")/.config"

. ${CONFIG_FILE}

# Script for Automating the Build Process
main() {
	# Ensure this script is run from the project directory
	cd "${PROJECT_PATH}" || exit

	# Step 0
	WEST=$(which west)
	${WEST} --version > /dev/null
	PYTHON=$(which python3)

	# Step 1: Clone NXP's MCUXpresso SDK
	cd "${PROJECT_PATH}/dependencies" || exit
	${WEST} init --manifest-url ${MCU_EXPRESSO_SDK_URL} \
		--manifest-rev ${MCU_EXPRESSO_SDK_REV} \
		${MCU_EXPRESSO_SDK_DIR}
	cd ${MCU_EXPRESSO_SDK_DIR} || exit
	${WEST} update

	# Step 2: Run the replace_include_guards.py script
	cd ../
	echo "Running replace_include_guards.py..."
	${PYTHON} "${PROJECT_PATH}/dependencies/replace_include_guards.py"
}

main "$@"
