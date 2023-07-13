#******************************************************************************
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
# D-82229 Seefeld, Germany.
# Author: Isaac L. L. Yuki
#******************************************************************************

import os
import fnmatch

def find_files(directory, pattern, exclude_dir='examples'):
    for root, _, filenames in os.walk(directory):
        if exclude_dir in root.split(os.path.sep):
            continue 
        for filename in fnmatch.filter(filenames, pattern):
            yield os.path.join(root, filename)

def replace_include_guard(file_path):
    with open(file_path, 'r') as file:
        content = file.read()
    if 'include_guard(GLOBAL)' in content:
        new_content = content.replace('include_guard(GLOBAL)', 'include_guard(DIRECTORY)')
        with open(file_path, 'w') as file:
            file.write(new_content)
    elif 'include_guard()' in content:
        new_content = content.replace('include_guard()', 'include_guard(DIRECTORY)')
        with open(file_path, 'w') as file:
            file.write(new_content)
    print(f"Processed file: {file_path}")

def patch(directory, patchConfirmedFile):
    patchConfirmedFilePath = os.path.join(directory, patchConfirmedFile)
    if not os.path.isfile(patchConfirmedFilePath):
        for cmake_file in find_files(directory, '*.cmake'):
            replace_include_guard(cmake_file)
        # After processing, create patched.txt in the directory
        with open(patchConfirmedFilePath, "w") as file:
            file.write("true")
        print(f"SDK patch concluded for {directory}")
    else:
        print(f"{directory} already patched!")

if __name__ == '__main__':
    current_directory = os.getcwd()
    print(current_directory)
    mcuxaddonsdir = os.path.join(".", "mcuxaddons")
    mcuxsdkdir = os.path.join(".", "mcuxsdk")
    search_dirs = []
    atLeastOnePathWasFound = False
    if os.path.isdir(mcuxsdkdir):
        search_dirs.append(mcuxsdkdir)
        atLeastOnePathWasFound = True
    else: 
        print("The path to the SDK was not found.")
    if os.path.isdir(mcuxaddonsdir):
        search_dirs.append(mcuxaddonsdir)
        atLeastOnePathWasFound = True
    else:
        print("No addon was found.")
    if(atLeastOnePathWasFound):
        patchConfirmedFile = 'patched.txt'
        for base_directory in search_dirs:
            # Check patch in main-sdk
            if base_directory == mcuxsdkdir:
                patch(base_directory, patchConfirmedFile)
            else:
                # Loop through the immediate subdirectories
                for subdir in os.listdir(base_directory):
                    subdir_path = os.path.join(base_directory, subdir)
                    if os.path.isdir(subdir_path):
                        patch(subdir_path, patchConfirmedFile)
    else:
        print("No paths found! Please check installation.")
