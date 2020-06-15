This directory stores important linux system configuration changes. 

This directory can be considered as linux root. Please place all files according to a real linux file system structure here (allows direct copy'n'paste of this directory).

# Udev rules

Load and enable new udev rules after copying with:

`sudo udevadm control --reload-rules && sudo udevadm trigger`
