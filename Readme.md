#ca8210-posix-thread

A modified version of openthread to work on posix based systems using the cascoda ca8210 radio.

in the examples folder is an example program which presents a command line interface to test the thread interface. For more documentation on how to use this, and also information on the openthread API, see the main openthread repo:

<https://github.com/openthread/openthread>

For usage with a custom program, see the example makefile provided. To summarise, the otposixca8210 library (which is built to the root repo folder) should be linked into the custom program. The directories for useful header files are:

```
	../openthread/include/\
	../openthread/src/core/\
	./include/\
	../platform/\
	../ca8210-kernel-exchange/cascoda-api/include/\
	../ca8210-kernel-exchange/
```

##Build instructions for debian-like systems:

Set the permissions for the cloned repo, you can use the included fix_permissions.sh script to do this, just change pi:pi in the first line to be your own username in the form:

```
chown <username>:<group> -R .
```

```bash
sudo ./fix_permissions.sh
```

Retreive all of the git submodules required
```bash
./update_submodules.sh
```

Now the tools required to build the tools required to build openthread must be installed
```bash
sudo apt-get install autoconf -y
sudo apt-get install m4 -y
```

and for testing, the following is useful:
```bash
sudo apt-get install python-pexpect -y
```

Now to build the required tools to build openthread (this takes a while)
```bash
sudo ./fix_permissions.sh
./build_buildtools.sh
```

and prepare openthread for compilation
```bash
./prepare_openthread.sh
```

And finally, build everything (specifying the exchange method desired - default is kernel)
```bash
make
#OR
make EXCHANGE=kernel
#OR
make EXCHANGE=usb
```
Or for the nuc970:
```bash
./build_nuc970.sh
```
##Using wpantund to enable as linux network interface

On a posix system, a thread node can act as a linux network interface using the wpantund tool available from https://github.com/openthread/wpantund/

In order to install, follow the guide here (use the latest master commit, not full/latest-release): https://github.com/openthread/wpantund/blob/master/INSTALL.md#wpantund-installation-guide

then start wpantund using a command of the form:
```bash
sudo /usr/local/sbin/wpantund -o Config:NCP:SocketPath "system:/home/pi/ca8210-posix-thread/example/ncpapp" -o SyslogMask " -info" -o Config:TUN:InterfaceName utun6
```

where /home/pi/ca8210-posix-thread/example/ncpapp is a path to a thread application using the ncp library (call otNcpInit and include the NCP library in the application - example in the ncp branch of this repo)

Then follow a similar process as used in this tutorial to start the control panel and connect: https://github.com/openthread/wpantund/wiki/OpenThread-Simulator-Tutorial

