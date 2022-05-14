# AvionicsSoftware

## Table of Contents

1. [About Us](#about-us)
    1. [Code Base](#code-base)
    2. [Boards](#boards)
2. [Setting Up Development Environment](#setting-up-development)
    1. [Download Git](#download-git)
        1. [On Linux](#on-linux)
        2. [On Windows](#on-windows)
    2. [IDEs](#development)
    3. [Documentation](#documentation)
3. [TODO](#todo)

# About US

This is the code repository for the Student Organisation for Aerospace Research.

The original code in this repository was moved without history from the 2017-2018 repository. The old repository can be found [here](https://github.com/StudentOrganisationForAerospaceResearch/VanderAvionics).


## Code base

The majority of this codebase is written in C, with [additional scripting tools written in python](https://github.com/StudentOrganisationForAerospaceResearch/SoftwareTestingTools).

Standard POSIX and standard library features of the C language are not available for embedded devices.
As a result we use FreeRTOS (Free Real Time Operating System) for handling standard Operating System tasks like threading.

## Boards

We program for multiple boards. The two boards we are currently using are STM32 boards and ESP S3 boards.


# Setting Up Development

The following tools are required for making changes to the repo.

## Download Git

Git is our chosen version control system.
### On Linux

#### Debian/Ubuntu
```bash
sudo apt-get install git
```
#### Fedora 21
```bash
sudo yum install git
```
#### Fedora 22+
```bash
sudo dnf install git
```
#### Arch
```bash
sudo pacman -S git
```

### On Windows

[Download Git](https://git-scm.com/download/win) for Windows.
Select all the recommended and default options.

## IDEs

Development requires both the STM32CubeIDE for the STM32 boards and Espressif IDF for ESP boards.

* [Download STMCubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html#get-software)
* [Download the Espressif IDF](https://idf.espressif.com/)
  * Select the S3 model.

## Documentation

* [Getting started with STM32 ARM Cortex MCUs](https://deepbluembedded.com/getting-started-with-stm32-arm-cortex-mcus/)
* [Using FreeRTOS in small embedded systems](https://www.freertos.org/tutorial/index.html)
* [FreeRTOS documentation](https://www.freertos.org/fr-content-src/uploads/2018/07/161204_Mastering_the_FreeRTOS_Real_Time_Kernel-A_Hands-On_Tutorial_Guide.pdf)

# TODO

- [ ] Document this thing