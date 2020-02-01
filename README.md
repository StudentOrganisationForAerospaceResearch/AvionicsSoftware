# AvionicsSoftware

The original code in this repository was moved without history from the 2017-2018 repository. The old repository can be found here: https://github.com/StudentOrganisationForAerospaceResearch/VanderAvionics.

Dockerfile Commands:

sudo docker build . -t avionics

sudo docker run --rm -it -v ~/path/to/AvionicsSoftware:/AvionicsSoftware/:rw --privileged -v /dev/bus/usb:/dev/bus/usb avionics 
