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

And finally, build everything
```bash
make
```
