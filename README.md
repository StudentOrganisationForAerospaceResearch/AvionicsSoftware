# AvionicsSoftware

Dockerfile Commands:
```
sudo docker build . -t avionics

sudo docker run --rm -it -v ~/path/to/AvionicsSoftware:/AvionicsSoftware/:rw --privileged -v /dev/bus/usb:/dev/bus/usb avionics
```

To compile code, use: ``make``

Aliases:

``clean`` : Used to delete the build folder. It is used before doing a clean build.

``erase`` : Erases the existing code on the microcontroller.

``flash`` : Flashes the microcontroller with the new code.

``format`` : Runs the formatting script.
