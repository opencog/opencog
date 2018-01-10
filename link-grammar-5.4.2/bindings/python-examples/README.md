Python bindings for Link Grammar
================================

Description
-----------
A Link Grammar library test is implemented in `tests.py`.
An example program `example.py` is provided.

Configuring (if needed)
-----------------------
### For Python2
   $ configure --enable-python-bindings
### For Python3
   $ configure --enable-python3-bindings


How to use
----------
(See below under **Testing the installation** for directions on how to set
`PYTHONPATH` in case it is needed.)

Parsing simple sentences:

```
$ python

>>> from linkgrammar import Sentence, ParseOptions, Dictionary
>>> sent = Sentence("This is a simple sentence.", Dictionary(), ParseOptions())
>>> linkages = sent.parse()
>>> len(linkages)
>>> for linkage in linkages:
...    print linkage.diagram()
...
```
```
      +-------------------Xp------------------+
      |              +--------Osm-------+     |
      +----->WV----->+  +-----Ds**x-----+     |
      +---Wd---+-Ss*b+  |     +----A----+     |
      |        |     |  |     |         |     |
  LEFT-WALL this.p is.v a simple.a sentence.n .
```
Additional examples can be found in `examples.py`.

Testing
-------
The test collection `tests.py` should run 56 tests, none of them should fail.
However, 3 tests will get skipped if the library is not configured with a
speller, and one test will get skipped if the library is not configured with
the SAT solver (this is the status for now on native Windows).

The following shows how to issue the tests on systems other then natives
Windows/MinGW (for testing on native Windows see msvc14/README under
"Running Python programs").

Note: For less verbosity of the `make` command output you can use the `-s`
flag of make.

### Testing the build directory
The following is assumed:
**$SRC_DIR** - Link Grammar source directory.
**$BUILD_DIR** - Link Grammar build directory.

#### By `make`
```
$ cd $BUILD_DIR/bindings/python-examples
$ make [-s] check
```
The results of tests.py are in the current directory under in the file
`tests.log`.

Note: To run also the tests in the **$SRC_DIR/tests/** directory, issue
`make check` directly from **$BUILD_DIR**.

#### Manually
To run tests.py manually, or to run `example.py`, you have to set the
`PYTHONPATH` environment variable as follows:
```
PYTHONPATH=$SRC_DIR/bindings/python:$BUILD_DIR/bindings/python:$BUILD_DIR/bindings/python/.libs
```
(Export it, or prepend it it the `make` command.)
```
$ cd $SRC_DIR
$ python tests.py [-v]
```

### Testing the installation
This can be done only after `make install`.

#### By `make`
```
$ cd $BUILD_DIR/bindings/python-examples
$ make [-s] installcheck
```
To run the whole package installcheck, issue `make installcheck` from
$BUILD_DIR.

#### Manually
Set the `PYTHONPATH` environment variable to the location of the installed
Python's **linkgrammar** module, e.g.:

```
PYTHONPATH=/usr/local/lib/python2.7/site-packages
```
(Export it, or prepend it to the `python` command.)
<br>
Note: This is not needed if the package has been configured to install to the
OS standard system locations.

**NOTE:** Make sure you invoke `tests.py` from a directory from which it cannot
find the `data` directory in **$SRCDIR/.** ! This will enforce it to use the
system-installed data directory. Two directory levels under **$SRCDIR**, as
shown below, is fine for that purpose.

```
$ cd $SRCDIR/binding/python-examples
$ python tests.py
```
