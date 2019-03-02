# Test framework

## Background


This is a frame work for writing unit tests for modules or libs.
It compiles only src/libs/... and a few select needed modules by default.
It runs on the target and the output is written to the serial UART, it runs well with MRI so you can also debug with GDB.

You can edit a file to specify which module you are going to test by default the example has tools/temperaturecontrol selected.

The unit tests go in a directory named src/testframework/unittests/*XXXX*/*YYYY* where *XXXX* is the module subdirectory (tools,utils) and *YYYY* is the modulename

The Kernel and main are replaced with special versions for testing.

The main way to test is to stimulate the modules by sending it commands via
on_gcode_received() (or other events it is registered for), and monitor its
actions my mocking the ON_PUBLIC_SET/GET calls it makes. A properly written
module does not access other modules any other way.

Many HAL calls are made via THEKERNEL->xxx (eg THEKERNEL->adc) and these can be intercepted by mocks to provide the module with its external data.

An example is provided here...

`src/testframework/unittests/tools/temperatureswitch/TEST_TemperatureSwitch.cpp`

## Usage

You compile a unit test and framework using [rake](http://rake.rubyforge.org/)...

```shell
> rake testing=1
> rake upload
```

The unit test will run, the results are printed to the serial uart port, and then it is left in DFU mode so `rake upload`  can be run again for the next test.

You select which tests and modules you want to compile by creating a file called rakefile.defaults in the same directory as the Rakefile is found.

```shell
TESTMODULES= %w(tools/temperatureswitch libs)
```

Here we enable unit tests for the tools/temperatureswitch module and any unit tests in src/testframework/unittests/tools/temperatureswitch/.
Also any unittests in the src/testframework/unittests/libs folder will be included

In this case all files in src/libs/... are compiled anyway so we only need to enable unit tests in the src/testframework/unittests/libs folder.

the tools/temperatureswitch both compiles all files in the src/modules/tools/temperatureswitch/... directory and enables the
unit tests in src/testframework/unittests/tools/temperatureswitch/...

by default no other files in the src/modules/... directory tree are compiled unless specified above.





