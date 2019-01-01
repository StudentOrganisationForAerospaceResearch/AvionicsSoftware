# AvionicsSoftware

Dockerfile Commands:

sudo docker build . -t avionics

sudo docker run --rm -it -v ~/path/to/AvionicsSoftware:/AvionicsSoftware/:rw --privileged -v /dev/bus/usb:/dev/bus/usb avionics