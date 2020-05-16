# AvionicsSoftware

The original code in this repository was moved without history from the 2017-2018 repository. The old repository can be found here: https://github.com/StudentOrganisationForAerospaceResearch/VanderAvionics.

Dockerfile Commands:
```
docker build . -t avionics

docker run --rm -it -v ~/path/to/AvionicsSoftware:/AvionicsSoftware/:rw --privileged -v /dev/bus/usb:/dev/bus/usb avionics
```

To compile code, use: ``make``

Docker Aliases:

``clean`` : Used to delete the build folder. It is used before doing a clean build.

``erase`` : Erases the existing code on the microcontroller.

``flash`` : Flashes the microcontroller with the new code.

``format`` : Runs the formatting script.
