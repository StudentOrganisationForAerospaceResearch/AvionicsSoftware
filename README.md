# AvionicsSoftware

The original code in this repository was moved without history from the 2017-2018 repository. The old repository can be found here: https://github.com/StudentOrganisationForAerospaceResearch/VanderAvionics.

Dockerfile Commands:

```bash
sudo docker build . -t avionics
```
```bash
sudo docker run --rm -it -v ~/path/to/AvionicsSoftware:/AvionicsSoftware/:rw --privileged -v /dev/bus/usb:/dev/bus/usb avionics 
```

^^^^ Deal with this later ^^^^

# About US

This is the code repository for the Student Organisation for Aerospace Research.

# Code base

The majority of this codebase is written in C, with additional scripting tools written in python.

Standard POSIX and standard library features of the C language are not available for embedded devices.
As a result we use FreeRTOS (Free Real Time Operating System) for handling standard Operating System tasks like threading.

Development requires both the STM32CubeIDE and Espressif IDF.

# Resources

## Download Git

### On linux

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

## Development

* [Download STMCubeIDE]()
* [Download the Espressif IDF](https://idf.espressif.com/)
  * Select the S3 model.

## Documentation

* [Getting started with STM32 ARM Cortex MCUs](https://deepbluembedded.com/getting-started-with-stm32-arm-cortex-mcus/)
* [Using FreeRTOS in small embedded systems](https://www.freertos.org/tutorial/index.html)
* [FreeRTOS documentation](https://www.freertos.org/fr-content-src/uploads/2018/07/161204_Mastering_the_FreeRTOS_Real_Time_Kernel-A_Hands-On_Tutorial_Guide.pdf)
