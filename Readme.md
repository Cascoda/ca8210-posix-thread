# ca821x-posix-thread

A modified version of openthread to work on posix based systems using a Cascoda CA-821x radio.

In the examples folder is an example program which presents a command line interface to test the thread interface. For more documentation on how to use this, and also information on the openthread API, see the main openthread repo:

<https://github.com/openthread/openthread>

## Building
This project must be configured with cmake before building. It is recommended to use the cmake gui, ccmake or cmake interactive mode, as there are user-configurable options. This will generate the required config files and setup whatever build system/projects you need. Cmake can also be used to configure project files for eclipse, codeblocks, xcode, etc. The system is designed for out-of-source builds, so the source directories do not get touched at all when building. This has many benefits:
- Works nicely with version control systems like git, as no clutter ever appears, and a .gitignore is uneccesary
- You can have several different 'configurations' of the same project open in your IDE at the same time
- You can keep your source on a different partition to your build dir, even across a network

For example, on a posix system:
```bash
# Make a directory to work in
mkdir ca821x-posix-thread && cd ca821x-posix-thread
# clone this repo
git clone https://github.com/Cascoda/ca8210-posix-thread.git
# make a build directory
mkdir build && cd build
# configure the build & download dependancies
ccmake ../ca8210-posix-thread
# build
make -j8
```

For more information, consult https://cmake.org/runningcmake/

### If using the usb exchange, then the hidusb shared library needs to be installed - look at the readme in the hidapi src, which is at _deps/hidapi-src by default - this is a WIP and the aim is to do this automatically eventually

## Using wpantund to enable as linux network interface

On a posix system, a thread node can act as a linux network interface using the wpantund tool available from https://github.com/openthread/wpantund/

In order to install, follow the guide here (use the latest master commit, not full/latest-release): https://github.com/openthread/wpantund/blob/master/INSTALL.md#wpantund-installation-guide

then start wpantund using a command of the form:
```bash
sudo /usr/local/sbin/wpantund -o Config:NCP:SocketPath "system:/home/pi/ca8210-posix-thread/example/ncpapp" -o SyslogMask " -info" -o Config:TUN:InterfaceName utun6
```

where /home/pi/ca8210-posix-thread/example/ncpapp is a path to a thread application using the ncp library (call otNcpInit and include the NCP library in the application - example in the ncp branch of this repo)

Then follow a similar process as used in this tutorial to start the control panel and connect: https://github.com/openthread/wpantund/wiki/OpenThread-Simulator-Tutorial

