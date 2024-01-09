# Readme

This guide contains instructions for configuring the build system, building targets, and executing the demonstration applications included in this repository, specifically designed for TQ-developed boards using the i.MX MCU family from NXP.

## Table of Contents

[[_TOC_]]

## Getting Started

### Requirements:

__NOTE__: The versions provided are the ones with which the build system and its artifacts were tested.

- Arm GNU Toolchain: arm-gnu-toolchain-12.2.rel1-[host system]-arm-none-eabi
- GCC: 12.2.1
- Visual Studio Code: v1.8.1^
- NXP's [MCUXpresso SDK](https://github.com/nxp-mcuxpresso/mcux-sdk): MCUX_2.14.0_UPDATE_ENHC1
- CMake: v3.27.4
- West: v1.0.0
- Python: v3.10.4
- For Windows:
  - MinGW: v13.1.0
- For Debug only:
  - [Segger J-link](https://www.segger.com/downloads/jlink/): v7.92l

#### VS-Code Extensions

- CMake tools (from Microsoft): v1.16.3^
  - ID: `ms-vscode.cmake-tools`
  - __Advice:__  Check if the 'Status-Bar-Visibility' option is set to 'visible'.
- For Debug only:
  - Cortex-Debug (from marus25): v1.12.0^
    - ID: `marus25.cortex-debug`

### Preparation

* Python3 installation (needed for the west repo tool and helper scripts).
* Install [Arm GNU Toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain).
  Use the .exe file for installation or unpack the archive to the desired path.
* Install [CMake](https://cmake.org/download/) and ensure that `CMake` is added to the system path.
* Windows only: Install [MinGW](https://github.com/nxp-mcuxpresso/mcux-sdk/blob/main/docs/run_a_project_using_armgcc.md)
  and add its bin directory to the system PATH variable. For example: `<Path to MinGW>/MinGW/bin`.
* Set the environmental variable ARMGCC_DIR pointing to the toolchain installation dir:
   - Use the "cmake.environment" option under [settings.json](https://github.com/microsoft/vscode-cmake-tools/blob/main/docs/cmake-settings.md).
   - Create a system variable.
* Debugging only: Install [Segger J-link](https://www.segger.com/downloads/jlink/) (Version used: `7.92`).
* Clone via 'west' [NXP's mcu-sdk](https://github.com/nxp-mcuxpresso/mcux-sdk) repository
  following the instructions under [Clone repository from NXP](#clone-repository-from-nxp).

#### Clone Repository from NXP

This repository uses [`NXP's MCUX SDK`](https://github.com/nxp-mcuxpresso/mcux-sdk) repository.
NXP's MCUX SDK uses the [`Zephyr West Tool`](https://github.com/zephyrproject-rtos/west)
to manage multiple Git repositories.

To clone the NXP's mcu-sdk repository using West, open a terminal and execute the following commands:

```
pip install west
cd  <project path>\dependencies
west init -m https://github.com/NXPmicro/mcux-sdk --mr MCUX_2.14.0_UPDATE_ENHC1 mcuxsdk
cd mcuxsdk
west update
```

__NOTE__: You can also install `mcuxsdk` in another path. If you did so, set the `MCUXSDK_ROOT` environment variable to the `mcuxsdk` path before [building](#building). Use the same procedure as for the variable [ARMGCC_DIR](#preparation).

Afterwards, copy `replace_include_guards.py` into the same directory as the `mcuxsdk`.
__ATTENTION__: This step is absolutely necessary! 

After downloading the SDK from NXP using West, the Python script `replace_include_guards.py`
must be run once. This is required because targets set to `global` cannot be built
in the same build folder due to the existing include guards in the CMake files 
in the NXP SDK.

```
cd <project path>
python3 <project path>/dependencies/replace_include_guards.py
```

This step will be executed automatically every time CMake generates the build system. CMake will check for the `patched.txt` file in
your `mcuxsdk` dir after every execution. If this file exists, the patch script
won't be executed again through a build command.

## Building 

### Building from the Command Line

When using from the command line, CMake has to know where to find the toolchain:

* Set the environment variable `ARMGCC_DIR` pointing to the toolchain installation dir.
* If using multiple versions of `cmake` in parallel, you can define a variable
  `CMAKE` pointing to the executable to use.
* Use the following flags to configure your build from the `cmake` command line.

|          Flag          |                                              Meaning                                              |
| :--------------------: | :-----------------------------------------------------------------------------------------------: |
| `CMAKE_TOOLCHAIN_FILE` | must point to `<project path>/dependencies/mcuxsdk/core/tools/cmake_toolchain_files/armgcc.cmake` |
|   `CMAKE_BUILD_TYPE`   |                  following different types are tested in SDK: `Debug`, `Release`                  |
|       `bootDisk`       |                            Linker file selection, e.g., `Ram`, `Flash`                            |

__NOTE__: Available `bootDisk` settings are listed under the specific board README.

* Set the build directory to the path `build` using `-B <project path>/build`.
* Set the build generator using `-G <your generator>`.

#### Generate Build System Command

```bash
cd <project path>
cmake -DCMAKE_TOOLCHAIN_FILE="dependencies\mcuxsdk\core\tools\cmake_toolchain_files\armgcc.cmake" -DCMAKE_BUILD_TYPE="Debug" -DbootDisk="Ram" -B ".\build" -G "MinGW Makefiles" --fresh
```

* Build the target.

#### Build Target Command

```bash
cd <project path>
cmake --build ".\build" --config Debug --target < name of .elf-file or "all" > -j 14 -- 
```

### Building with VS-Code

When using the VS-Code support, the following additional steps are needed:

* Install [VS-Code](https://code.visualstudio.com/).
* Follow instructions for [setting up VS-Code](#setting-up-vs-code).

Flags like `CMAKE_TOOLCHAIN_FILE`, `CMAKE_BUILD_TYPE`, `bootDisk` should be set
in the [.vscode](./.vscode) folder. Use the [examples](./templates/README.md) for guidance.

#### Setting Up VS-Code

- Open Visual Studio from the applications folder.
- Please refer to the [example templates](./templates/) when setting up VS-Code. You should have at least the same `.json-files` and `.yaml-files` in your [.vscode folder](./.vscode) by the end of this section.
- Install the [CMake tools extension by Microsoft](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools).
  - Ctrl+P: `ext install ms-vscode.cmake-tools` or search for the extension under the menu extensions on the left bar.
  - Adjust [settings.json](https://github.com/microsoft/vscode-cmake-tools/blob/main/docs/cmake-settings.md)
    in the [.vscode](./.vscode) folder accordingly to your environment:
    [Click here to open the example file for Windows](./templates/settings.json).
  - Ensure you have the [cmake-variants.yaml file](./templates/cmake-variants.yaml). 
- For Debug only: Install [cortex-debug](https://github.com/Marus/cortex-debug/wiki)
  extension from marus25.
  - either use `Ctrl+P` and `ext install marus25.cortex-debug`
  - or search for the extension under the menu extensions on the left bar
  - Set up the desired configurations for debug on VS-Code. You will need two files,
    one to configure the debug settings for VS-Code ([settings.json](./templates/settings.json)) 
    and a second to set up the debugger configuration ([launch.json](./templates/launch.json)).
    [Here is an entire guide for setting up VS-Code in order to use J-Link Segger as a debugger with VS-Code](https://wiki.segger.com/J-Link_Visual_Studio_Code).
- __ATTENTION:__ Please refer, if existing, to the README of your board for further details on setting up the debugger. Some boards may require special settings.
  - The README should be placed under: `examples/README.md`.

#### Building Target

After fulfilling the preparations, VS-Code should be displaying the CMake-Tools
option on the lower bar. In order to build a target, select the build variant by
clicking on the corresponding option. You can also select your target by clicking
on "set default build target".

Click after selection of the desired options on "Build" to build your target(s). 

Optionally you can choose the target within the CMake-menu-bar. 

## Loading Targets

Before running a target, make sure you've minded the [boot instructions of your board](./examples/README.md/#booting).

### Loading a Target via GDB Server 

If you prefer to boot without VS-Code, follow the instructions for debugging within the board's [README](./examples/README.md/#gdb-server).

### Loading a Target with VS-Code

  You can follow the instructions for [debugging with VS-Code](#debugging-with-vs-code). 

## Debugging

### Debugging with VS-Code

The repository utilizes `JLinkGDBServerCL` alongside the appropriate hardware for debugging in VS-Code. To initiate debugging, you need to configure the [debug tool](#setting-up-vs-code). Once configured, select the debug tool from the left menu bar and choose the desired debug configuration specified within the [launch.json](/templates/launch.json). To begin debugging, either click on `start debugging` or press `F5`.

- __ATTENTION:__ Please refer, if existing, to the README of your board for further details on setting up the debugger. Some boards may require special settings. 
  - The README should be placed under: `examples/README.md`.

## Applications

The list of all applications can be found in the [board](./examples/README.md#applications) README file.

## Build System

### Structure of Repository

The general structure is constituted in such a way that it was tried to create as
little maintenance effort for the demo repo as possible. Thus, everything that can
be summarized in higher-level folders is also deposited there.

| Folder/File                              | Description                                                                    |
| ---------------------------------------- | ------------------------------------------------------------------------------ |
| `templates`                              | Examples for the VS-Code setting files.                                        |
| `templates/cmake-variants.yaml`          | The Build-Type configurations.                                                 |
| `templates/settings.json`                | Configurations for  CMake and the cortex-debug tool.                           |
| `templates/launch.json`                  | All debug configurations.                                                      |
| `examples`                               | Path for the demo applications.                                                |
| `examples/board`                         | Generic files used for every app and board-specific information/configuration. |
| `examples/board/cmake`                   | CMake related files used for integrating the board into the build system.      |
| `examples/board/<file>.mex`              | Configuration file used in MCUX-Config Tools.                                  |
| `build`                                  | All build-related files.                                                       |
| `build/boards/<boardtype>/dist`          | Location for images.                                                           |
| `cmake`                                  | Location for CMake scripts.                                                    |
| `dependencies`                           | Dependencies and dependencies related files.                                   |
| `dependencies/TQDevices`                 | Custom programmed APIs used for the applications.                              |
| `dependencies/mcuxsdk`                   | The downloaded repository from NXP.                                            |
| `dependencies/replace_include_guards.py` | A patch script for the mcusdk.                                                 |
| `doxy`                                   | Doxy-file generator configurations.                                            |
| `scripts`                                | Scripts for automated build tests and version generation.                      |

The build system was created based on the specifications of NXP. Accordingly, for each app in NXP's repo,
all sources from the Driver-APIs and other libs are added to the target named `MCUX_SDK_PROJECT_NAME`
using the `target_sources()` command. This target is created in the `CMakeLists.txt` from the application.
This scheme is basically also used in this repo. However, all Include-Guards were changed from Global to
Directory using the patch script `replace_include_guards.py`. This has the consequence that
one build directory can now be used for all apps here. Using the `add_subdirectory()`
command, a directory is added per level, so that there are no conflicts per target.
However, this has the disadvantage that the order of all includes must be correct.

### Programming Apps

This guide provides instructions and hints on structuring the build system within the context of MCUXpresso SDKs and the build system structure within this repository when creating new apps or boards.

To create a new application, you must create a new folder for it and update the `CMakeLists.txt` file located in `../examples`. Place a new `CMakeLists.txt` file within the new application directory. You can use any of the `CMakeLists.txt` files from the demo applications as a template. Once a new directory with an application is created, it must be added to `examples/CMakeLists.txt`, and the variables in the `examples/CMakeList.txt` folder must be adjusted accordingly.

The `CMakeLists.txt` file in the `examples/<app_name>` directory ensures the correct inclusion of files. To use `flags_macros` for linking/compiling and configure NXP modules, create a `config.cmake` file. Templates for this can also be found in the demo applications.

You can add special files globally to all targets by adding them to the `BOARD_SRCS` variable, or specifically to one target by adding them to the list under `add_executable`. Note that you should also point to the directory where these files are located by adding it to `target_include_directories`, especially if the directory is newly created.

If you need your own board files instead of the default ones in this repository, reset the `BOARD_SRCS` variable with your own file list and the `BoardDirPath` variable with your desired directory.

Compiler and linker flags are set in the `flags.cmake` file in the `examples/board/cmake` directory. However, to configure specific flags, use the `config.cmake` file in the application folder. There is also a `flags_macros.cmake` file under [cmake](./cmake) in the top-level directory.

When programming, use the linker scripts provided by TQ. You can include your own linker files using the set commands with the variables:

```
set(LINKER_FILE_RAM "<your_path>")
set(LINKER_FILE_FLASH "<your_path>")
...
```

Place the command in the `CMakeLists.txt` of the application directory before including `flags.cmake`.

For learning about NXP's API, the examples under `mcuxsdk/examples` can be helpful.

## License

Except where otherwise noted, all files within this repository are licensed under the following terms, excluding build system files (such as .cmake) and configuration files for development environments (such as Visual Studio Code setup files):

SPDX-License-Identifier: [BSD-3-Clause](./license.bsd3.md)

All documentation is licensed under [CC-BY-4.0 (Creative Commons Attribution 4.0International Public License)](./COPYING.CC-BY-4.0).

Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,  
D-82229 Seefeld, Germany.  
Author: Isaac L. L. Yuki

## Support Wiki

For more information, please refer to our [support wiki](https://support.tq-group.com/).
