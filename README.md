<div align="center">
<img alt="Avionics" src="https://user-images.githubusercontent.com/78698227/185337251-e8da5b86-772f-4e64-9f8b-6669e7fdacce.png" width="400" height="80"/>
</div>


## Table of Contents

1. [About This](#about-us)
    1. [Timeline](#timeline)
    2. [Links](#links)
    3. [IDE support](#ide-support)
2. [Motivation](#motivation)
3. [MVP Requirements](#mvp-requirements)
4. [Design Decisions](#design-decisions)
    1. [Diagrams](#diagrams)
        1. [Sensor Poll](#sensor-poll)
        2. [UART poll](#uart-poll)
        3. [Class Diagram](#class-diagram)
        4. [Module & Folder Structure](#module-and-folder-structure)
5. [Formatting Guidelines](#formatting-guidelines)
6. [Milestones](#milestones)
7. [Processes](#processes)

## About This

This is the code repository for staging the
Student Organisation for Aerospace Research (SOAR's) C++ rewrite.

The original code in this repository was written in C,
The old repository can be found [here](https://github.com/StudentOrganisationForAerospaceResearch/AvionicsSoftware).

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

### IDE support

By default we are using the [STMCubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html#get-software).

Use the `_IDE` folder to add support for your IDE.\
Make sure to update the `.gitignore`.

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

