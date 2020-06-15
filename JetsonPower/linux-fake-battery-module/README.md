# Fake Battery Module for the Linux kernel

This is a kernel module I wrote (based mainly on the `test_power` module
included in the Linux kernel source) for simulating multiple batteries on
Linux.

This is my first module, so don't hold me responsible if you use it and it
causes a kernel panic. =)


## Loading the module

You can build the module with a simple `make`, and load it with `insmod`:

    $ sudo insmod ./fake_battery.ko

## Changing battery values via jetson TX2 Power script

Just Run ./jetsonTX2Power to update the values once. 
Also `watch -n 1000 ./jetsonTX2Power` can be used to update per second.

