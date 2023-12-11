#!/bin/sh
# ****************************************************************************
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
# D-82229 Seefeld, Germany.
# Author:    Maximilian Kürth
# Description: Script to generate version header based on information 
# from VCS
#******************************************************************************

FILENAME=version.h
FILE=${FILENAME}

printf "generating version information header\r\n"

VERSION=$(git describe --match "*BSP.*" --abbrev=0 --tags HEAD)
BRANCH=$(git branch --show-current)

rm -f ${FILE}

(
  printf "//*****************************************************************************\r\n"
  printf "/*! \r\n"
  printf " * \\\file  ${FILENAME}\r\n"
  printf " * \\\brief Auto generated version information file.\r\n"
  printf " * \\\note  Do not edit this file.\r\n"
  printf " */\r\n"
  printf "//*****************************************************************************\r\n"
  printf "\r\n"
  printf "#ifndef VERSION_H\r\n"
  printf "#define VERSION_H\r\n"
  printf "\r\n"
  printf "///! Defines the plain version string.\r\n"
  printf "#define VERSION      \"${VERSION}\"\r\n"
  printf "///! Defines the current branch in VCS.\r\n"
  printf "#define BRANCH       \"${BRANCH}\"\r\n"
  printf "\r\n"
  printf "#endif\r\n"
) >> ${FILE}
