# Ground Rover Daemon

Ground Rover Daemon is service that simulates [MAVLink](https://mavlink.io/en/) communication with QGroundStation to let it send MANUAL_CONTROL messages from Joystic.

## Building and running

    $ git submodule update --init --recursive
    $ make
    $ ./ground_rover_daemon
