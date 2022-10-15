<div align="center">
<img alt="Avionics" src="https://user-images.githubusercontent.com/78698227/194971475-40b42a4f-bcb2-423e-96fe-d8b0756077d5.png" width="500"/>
</div>

<a href="https://en.cppreference.com/w/" rel="CppReference">![Foo](https://user-images.githubusercontent.com/78698227/185344746-ace6a502-a9a3-43d6-9379-daad6f4ffcd3.svg)</a>
<a href="https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf" rel="STM32F4 Reference Manual">![STMicro](https://user-images.githubusercontent.com/78698227/185344511-0296b5ed-15a3-4013-a98a-6dcd38222382.svg)</a>
<a href="https://www.freertos.org/features.html" rel="FreeRTOS">![FreeRTOS](https://camo.githubusercontent.com/da874464f191d72e47b3f6834b0ae26e8c3b5edbaeafcdacf526a844a972f87c/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f4672656552544f532d2532333234373133332e7376673f7374796c653d666f722d7468652d6261646765)</a>

![](https://img.shields.io/github/repo-size/StudentOrganisationForAerospaceResearch/AvionicsSoftware?label=Size)
![](https://img.shields.io/github/commit-activity/m/StudentOrganisationForAerospaceResearch/AvionicsSoftware)
![](https://img.shields.io/github/contributors/StudentOrganisationForAerospaceResearch/AvionicsSoftware)

## Table of Contents
1. [About This](#about-this)
    1. [IDE support](#ide-support)
    2. [Timeline](#timeline)
    3. [Links](#links)
2. [Languages](#languages)
3. [Motivation](#motivation)
4. [MVP Requirements](#mvp-requirements)
5. [Design Decisions](#design-decisions)
    1. [Diagrams](#diagrams)
        1. [Sensor Poll](#sensor-poll)
        2. [UART poll](#uart-poll)
        3. [Class Diagram](#class-diagram)
        4. [Module & Folder Structure](#module-and-folder-structure)
6. [Formatting Guidelines](#formatting-guidelines)
7. [Milestones](#milestones)
8. [Processes](#processes)
9. [Folder Structure](#folder-structure)

## About This

This is the code repository for staging the
Student Organisation for Aerospace Research (SOAR's) C++ rewrite.

The original code in this repository was written in C,
The old repository can be found [here](https://github.com/StudentOrganisationForAerospaceResearch/AvionicsSoftware/tree/Andromeda_V3.31_Legacy).

### IDE support

By default we are using the [STMCubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html#get-software).

Use the `_IDE` folder to add support for your IDE of choice if needed.\
Make sure to update the `.gitignore`.


### Timeline

- [ ] Layout desired requirements
- [ ] Design and document architecture
  - [ ] Diagram
  - [ ] Reference other teams (if possible)
- [ ] Scope MVP
  - [ ] Before September 6th (Classes begin)
- [ ] Build MVP
  - [ ] PREREQ: Merge A3.3, Clang Format
  - [ ] Integration test on hardware
  - [ ] Full SW team approval
- [ ] Create tasks and documentation to update each code section
- [ ] Create on-boarding presentation
  - [ ] What it is
  - [ ] How it works
  - [ ] Why decisions were made
- [ ] Merge MVP into master

1. Design Freeze
    - [ ] End of August
2. Design Review
    - [x] July 30th
3. Testing Schedule
    - [ ] Build MVP
4. Presentation
    - [ ] Mid September
5. Competition Data
    - June 2023

### Links

1. [Brainstorming Document](https://docs.google.com/document/d/19tSGNcbYLIuioOCkpJu3X-sNt-DKgJVUJyrzreMyuKA/edit)
2. [Formal Document](https://docs.google.com/document/d/1YyJIRSh0NKUMdpxNTZI0eOd3DQ4soUk3D-bQbSyjYyE/edit?usp=sharing )

The most relevant information from both documents is distributed in
this repo.

## Languages
Embedded software is written primarily in C/C++, using [CMSIS](https://www.keil.com/pack/doc/CMSIS/RTOS/html/index.html) with [FreeRTOS](https://www.freertos.org/) as the underlying operating system. </br>
Scripts and Tools are written in various languages, primarily [Python](https://docs.python.org/3/reference/). </br>
Diagrams are designed with a variety of software, code based diagrams are written in [PlantUML](https://plantuml.com/), other diagrams are designed in [Diagrams.net](https://app.diagrams.net/)

## Motivation

- Readable
- Robust
- Modular
- Consistent
- Easier for new members
- Documented from the start
- Test Driven Design

## MVP Requirements

- [ ] Compile flags/`#ifdef` flag for solid vs hybrid.
- [ ] Base Task Communication
- [ ] UART poll and debug logging
- [ ] Sensor log output over UART5
- [ ] Queue
- [ ] Abstractions for important systems
  - [ ] Class Diagrams
- [ ] Module
- [ ] Well documented
- [ ] Testing Framework

## Design Decisions

- Tasks are contained and communicate using queues.
  - Possibly look into a priority queue to handle queues getting too
      full.
- Abstract Base Classes for things like `Sensor` or `Task`.
- Pointer hierarchy
    1. References
    2. Smart Pointers
        - Beware RTOS support
    3. Pointers
- Reduce cognitive overhead by documenting your thought process
  for complicated tasks.
  - Especially in non-trivial ifs or big loops.

### Diagrams

#### Sensor Poll

#### UART Poll

#### Class Diagram

#### Module and Folder Structure

## Formatting Guidelines

- Functions are lowercase.
  - Class related functions are uppercase.
- Infinite Loops are written with `while(1)`
  as oppose to `for(;;)`.

## Milestones

- [ ] Architecture Design Complete
- [ ] MVP Complete
- [ ] Rest of Codebase complete

## Processes
### Changing .ioc file
- This should be improved in the future using a rename script, or by modifying CMake file if necessary. There may be a setting in CubeIDE
- Whenever you change the .ioc file the following must be done
1. Rename Core/main.cpp to Core/main.c
2. Generate code from .ioc change
3. Rename back to Core/main.cpp

## Folder Structure
- The `Core` directory contains code primarily generated by STM32CubeIDE, or for other Middlewares such as FreeRTOS.
- Most of the written code should go into `Components` under the most appropriate sub-directory.


