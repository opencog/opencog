Link Grammar Parser
===================
***Version 5.4.2***

The Link Grammar Parser implements the Sleator/Temperley/Lafferty
theory of natural language parsing. This version of the parser is
an extended, expanded version of the last official CMU release, and
includes many enhancements and fixes created by many different
developers.

This code is released under the LGPL license, making it freely
available for both private and commercial use, with few restrictions.
The terms of the license are given in the LICENSE file included with
this software.

Please see the
[main web page](http://www.abisource.com/projects/link-grammar/)
for more information.  This version is a continuation of the
[original CMU parser](http://www.link.cs.cmu.edu/link).

Quick Overview
---------------
The parser includes API's in various different programming languages,
as well as a handy command-line tool for playing with it.  Here's some
typical output:
```
linkparser> This is a test!
	Linkage 1, cost vector = (UNUSED=0 DIS= 0.00 LEN=6)

    +-------------Xp------------+
    +----->WV----->+---Ost--+   |
    +---Wd---+-Ss*b+  +Ds**c+   |
    |        |     |  |     |   |
LEFT-WALL this.p is.v a  test.n !

(S (NP this.p) (VP is.v (NP a test.n)) !)

            LEFT-WALL    0.000  Wd+ hWV+ Xp+
               this.p    0.000  Wd- Ss*b+
                 is.v    0.000  Ss- dWV- O*t+
                    a    0.000  Ds**c+
               test.n    0.000  Ds**c- Os-
                    !    0.000  Xp- RW+
           RIGHT-WALL    0.000  RW-

```
This rather busy display illustrates many interesting things. For
example, the `Ss*b` link connects the verb and the subject, and
indicates that the subject is singular.  Likewise, the `Ost` link
connects the verb and the object, and also indicates that the object
is singular. The `WV` (verb-wall) link points at the head-verb of
the sentence, while the `Wd` link points at the head-noun. The `Xp`
link connects to the trailing punctuation. The `Ds**c` link connects
the noun to the determiner: it again confirms that the noun is singular,
and also that the noun starts with a consonant. (The `PH` link, not
required here, is used to force phonetic agreement, distinguishing
'a' from 'an').  These link types are documented in the
[English Link Documentation](https://www.abisource.com/projects/link-grammar/dict/index.html).

The bottom of the display is a listing of the "disjuncts" used for
each word. The disjuncts are simply a list of the connectors that
were employed to form the links. They are particularly interesting
because they serve as an extremely fine-grained form of a "part of
speech".  This, for example: the disjunct `S- O+` indicates a
transitive verb: its a verb that takes both a subject and an object.
The additional markup above indicates that 'is' is not only being used
as a transitive verb, but it also indicates finer details: a transitive
verb that took a singular subject, and was used (is usable as) the head
verb of a sentence.  The floating-point value is the "cost" of the
disjunct; it very roughly captures the idea of the log-probability
of this particular grammatical usage.  Much as parts-of-speech correlate
with word-meanings, so also fine-grains parts-of-speech correlate with
much finer distinctions and gradations of meaning.

The link-grammar parser also supports morphological analysis. Here is
an example in Russian:
```
linkparser> это теста
	Linkage 1, cost vector = (UNUSED=0 DIS= 0.00 LEN=4)

             +-----MVAip-----+
    +---Wd---+       +-LLCAG-+
    |        |       |       |
LEFT-WALL это.msi тест.= =а.ndnpi
```

The `LL` link connects the stem 'тест' to the suffix 'а'. The `MVA`
link connects only to the suffix, because, in Russian, it is the
suffixes that carry all of the syntactic structure, and not the stems.
The Russian lexis is
[documented here](https://www.abisource.com/projects/link-grammar/russian/doc/).

Theory and Documentation
------------------------
An extended overview and summary can be found in the
[Link Grammar Wikipedia page](https://en.wikipedia.org/wiki/Link_grammar),
which touches on most of the import, primary aspects of the theory.
However, it is no substitute for the original papers published on the
topic:
* Daniel D. K. Sleator, Davy Temperley,
  ["Parsing English with a Link Grammar"](http://www.cs.cmu.edu/afs/cs.cmu.edu/project/link/pub/www/papers/ps/tr91-196.pdf)
  October 1991 *CMU-CS-91-196*.
* Daniel D. Sleator, Davy Temperley,
  ["Parsing English with a Link Grammar"](http://www.cs.cmu.edu/afs/cs.cmu.edu/project/link/pub/www/papers/ps/LG-IWPT93.pdf),
  *Third International Workshop on Parsing Technologies* (1993).
* Dennis Grinberg, John Lafferty, Daniel Sleator,
  ["A Robust Parsing Algorithm for Link Grammars"](http://www.cs.cmu.edu/afs/cs.cmu.edu/project/link/pub/www/papers/ps/tr95-125.pdf),
  August 1995 *CMU-CS-95-125*.
* John Lafferty, Daniel Sleator, Davy Temperley,
  ["Grammatical Trigrams: A Probabilistic Model of Link Grammar"](http://www.cs.cmu.edu/afs/cs.cmu.edu/project/link/pub/www/papers/ps/gram3gram.pdf),
  1992 *AAAI Symposium on Probabilistic Approaches to Natural Language*.

There are many more papers and references listed on the
[primary Link Grammar website](https://www.abisource.com/projects/link-grammar/).

See also the
[C/C++ API documentation](https://www.abisource.com/projects/link-grammar/api/index.html).
Bindings for other programming languages, including python, python3 and
java, can be found in the [bindings directory](bindings).


Contents
--------

| Content       | Description |
| ------------- |-------------|
| LICENSE     | The license describing terms of use |
| link-grammar/*.c | The program.  (Written in ANSI-C) |
| link-grammar/corpus/*.c | Optional corpus statistics database. |
| link-grammar/minisat/ | Optional SAT Solver. (Written in C++) |
| link-grammar/sat-solver/ | Optional SAT Solver. (Written in C++) |
| link-grammar/viterbi/ | Experimental Viterbi algorithm parser. |
|  |  |
| bindings/autoit/  | Optional AutoIt language bindings. |
| bindings/java/ | Optional Java language bindings. |
| bindings/lisp/ | Optional Common Lisp language bindings. |
| bindings/ocaml/ | Optional OCaML language bindings. |
| bindings/python/  | Optional Python2 language bindings. |
| bindings/python3/ | Optional Python3 language bindings. |
| bindings/python-examples/ | Link-grammar test suite and Python language binding usage example. |
| bindings/swig/ | SWIG interface file, for other FFI interfaces. |
|  |  |
| data/en/ | English language dictionaries. |
| data/en/4.0.dict | The file containing the dictionary definitions. |
| data/en/4.0.knowledge | The post-processing knowledge file. |
| data/en/4.0.constituents | The constituent knowledge file. |
| data/en/4.0.affix | The affix (prefix/suffix) file. |
| data/en/4.0.regex | Regular expression-based morphology guesser. |
| data/en/tiny.dict | A small example dictionary. |
| data/en/words/ | A directory full of word lists. |
| data/en/corpus*.batch | These files contain sentences (both grammatical and ungrammatical ones) that are used for testing the link-parser These can be run through the parser with the command `./link-parser < corpus.*.batch` |
|  |  |
| data/ru/ | A full-fledged Russian dictionary |
| data/ar/ | A fairly complete Arabic dictionary |
| data/fa/ | A Persian (Farsi) dictionary |
| data/de/ | A small prototype German dictionary |
| data/lt/ | A small prototype Lithuanian dictionary |
| data/id/ | A small prototype Indonesian dictionary |
| data/vn/ | A small prototype Vietnamese dictionary |
| data/he/ | An experimental Hebrew dictionary |
| data/kz/ | An experimental Kazakh dictionary |
| data/tr/ | An experimental Turkish dictionary |
|  |  |
| morphology/ar/ | An Arabic morphology analyzer |
| morphology/fa/ | An Persian morphology analyzer |
|  |  |
| COPYING | The license for this code and data |
| ChangeLog | A compendium of recent changes. |
| configure | The GNU configuration script |
| autogen.sh | Developer's configure maintenance tool |
| debug/ | Information for debugging the library |
| msvc14/ | Microsoft Visual-C project files |
| mingw/ | Information on using MinGW under MSYS or Cygwin |

UNPACKING and signature verification
------------------------------------
The system is distributed using the normal tar.gz format; it can be
extracted using the `tar -zxf link-grammar.tar.gz` command at the
command line.

A tarball of the latest version can be downloaded from:<br>
http://www.abisource.com/downloads/link-grammar

The files have been digitally signed to make sure that there was no
corruption of the dataset during download, and to help ensure that
no malicious changes were made to the code internals by third
parties. The signatures can be checked with the gpg command:

`gpg --verify link-grammar-5.4.2.tar.gz.asc`

which should generate output identical to (except for the date):
```
gpg: Signature made Thu 26 Apr 2012 12:45:31 PM CDT using RSA key ID E0C0651C
gpg: Good signature from "Linas Vepstas (Hexagon Architecture Patches) <linas@codeaurora.org>"
gpg:                 aka "Linas Vepstas (LKML) <linasvepstas@gmail.com>"
```
Alternately, the md5 check-sums can be verified. These do not provide
cryptographic security, but they can detect simple corruption. To
verify the check-sums, issue `md5sum -c MD5SUM` at the command line.


CREATING the system
-------------------
To compile the link-grammar shared library and demonstration program,
at the command line, type:
```
./configure
make
make check
```

To install, change user to "root" and say
```
make install
ldconfig
```

This will install the liblink-grammar.so library into /usr/local/lib,
the header files in /usr/local/include/link-grammar, and the
dictionaries into /usr/local/share/link-grammar.  Running 'ldconfig'
will rebuild the shared library cache.  To verify that the install was
successful, run (as a non-root user)
```
make installcheck
```

Editline
--------
If libedit-dev is installed, then the arrow keys can be used to edit
the input to the link-parser tool; the up and down arrow keys will
recall previous entries.  You want this; it makes testing and
editing much easier.  Note, however, most versions of editline are
not UTF8-capable, and so won't work, for example, with the Russian
dictionaries.  A UTF8-enabled version of libedit can be found here:

http://www.thrysoee.dk/editline/

If you use the above, be sure to say:
```
./configure --enable-widec
```
when building it, otherwise you won't actually get the UTF8 support!
Attention: the above configure is for libedit, not for link-grammar!
(In addition, you will need to uninstall the system default editline
in order to get the above. You may also need to set the environment
variable PKG_CONFIG_PATH to include /usr/local/lib/pkgconfig)

Use of editline in the link-parser can be disabled by saying:
```
./configure --disable-editline
```
**Note**: utf8 support for libedit is still missing in Ubuntu 1404 and
Mint 17 Qiana See https://bugs.launchpad.net/linuxmint/+bug/1389438
https://bugs.launchpad.net/ubuntu/+source/libedit/+bug/1375921

Java Bindings
-------------
By default, the Makefiles attempt to build the Java bindings.
The use of the Java bindings is *OPTIONAL*; you do not need these if
you do not plan to use link-grammar with Java.  You can skip building
the Java bindings by disabling as follows:
```
./configure --disable-java-bindings
```

If JAVA_HOME isn't set, if jni.h isn't found, or if ant isn't found,
then the java bindings will not be built.

Python2 and Python3 Bindings
----------------------------
The Python2 and Python3 bindings are built by default, providing that
the corresponding Python development packages are installed.

These packages are:
- Linux:
 * Systems using 'rpm' packages: Python2: python-devel; Python3: python3-devel
 * Systems using 'deb' packages: Python2: python-dev; Python3: python3-dev
- Windows:
 * Install Python2 and Python3 from https://www.python.org/downloads/windows/ .
   You also have to install SWIG from http://www.swig.org/download.html .
- MacOS:
 * Install the python and python3 packages using [HomeBrew](http://brew.sh/).

The use of the Python bindings is *OPTIONAL*; you do not need these if
you do not plan to use link-grammar with python.  If you like
to disable these bindings, use one of:

```
./configure --disable-python-bindings
./configure --enable-python-bindings=2
./configure --enable-python-bindings=3
```

The linkgrammar.py module provides a high-level interface in Python.
The example.py and sentence-check.py scripts provide a demo,
and tests.py runs unit tests.

Install location
----------------
The /usr/local install target can be over-ridden using the
standard GNU configure --prefix option, so for example:
```
./configure --prefix=/opt/link-grammar
```

By using pkg-config (see below), non-standard install locations
can be automatically detected.

Configure help
--------------
Additional config options are printed by
```
./configure --help
```

The system has been tested and works well on 32 and 64-bit Linux
systems, FreeBSD, MacOSX, as well as on many Microsoft Windows
systems, under various different Windows development environments.
Specific OS-dependent notes follow.

BUILDING from the [GitHub repository](https://github.com/opencog/link-grammar)
------------------------------------------------------------------------------

End users should download the tarball (see
[UNPACKING and signature verification](#unpacking-and-signature-verification)).

The current GitHub version is intended for developers (including anyone who
is willing to provide a fix, a new feature or an improvement). The tip of
the master branch is often unstable, and can sometimes have bad code in it
as it is under development. It also needs installing of development tools
that are not installed by default. Due to these reason the use of the GitHub
version is discouraged for regular end users.

### Installing from GitHub
Clone it:
`git clone https://github.com/opencog/link-grammar.git`<br>
Or download it as a ZIP:<br>
`https://github.com/opencog/link-grammar/archive/master.zip`

Tools that may need installation before you can compile the system:

make<br>
gcc<br>
gcc-c++ (for the SAT solver)<br>
autoconf<br>
autoconf-archive<br>
swig (for language bindings)<br>
graphviz (if you like to use the word-graph display feature)

The GitHub version doesn't include a `configure` script.
To generate it, use:
```
autogen.sh
```

If you get errors, make sure you have installed the above-listed
development packages, and that your system installation is up to date.

For more info about how to proceed, continue at the section
[CREATING the system](#creating-the-system) and the relevant sections after it.

### Additional notes for developers

To configure **debug** mode, use:
```
configure --enable-debug
```
It adds some verification debug code and functions that can
pretty-print several data structures.

A feature that may be useful for debugging is the word-graph
display.  Use the `configure` option `--enable-wordgraph-display` to enable
it. For more details on this feature, see
[Word-graph display](link-grammar/tokenize/README.md#word-graph-display).

BUILDING on MacOS
-----------------
Plain-vanilla Link Grammar should compile and run on Apple MacOSX
just fine, as described above.  At this time, there are no reported
issues.

The language bindings for python and java may require additional
packages to be installed.  A working editline is nice, since it
allows you to use the arrow keys in the command-line client.
See http://www.macports.org/ to find these.

You almost surely do not need a Mac portfile; but you can still
find one here:
http://trac.macports.org/browser/trunk/dports/textproc/link-grammar/Portfile .<br>
It does not currently specify any additional steps to perform.

If you do NOT need the java bindings, you should almost surely
configure with:
```
./configure --disable-java-bindings
```

By default, java requires a 64-bit binary, and not all MacOS systems
have a 64-bit devel environment installed.

If you do want Java bindings, be sure to set the JDK_HOME environment
variable to wherever `<Headers/jni.h>` is.   Set the JAVA_HOME variable
to the location of the java compiler.  Make sure you have ant
installed.


BUILDING on Windows
-------------------
There are three different ways in which link-grammar can be compiled
on Windows.  One way is to use Cygwin, which provides a Linux
compatibility layer for Windows.  Unfortunately, the Cygwin system
is not compatible with Java for Windows.  Another way is use the
MSVC system.  A third way is to use the MinGW system, which uses the
Gnu toolset to compile windows programs. The source code supports
Windows systems from Vista on.

Link-grammar requires a working version of POSIX-standard regex
libraries.  Since these are not provided by Microsoft, a copy must
be obtained elsewhere.  One popular choice is
[TRE](http://gnuwin32.sourceforge.net/packages/tre.htm).

Another popular choice is PCRE, 'Perl-Compatible Regular Expressions',
available at: http://www.pcre.org/ .<br>
For building on Windows: https://github.com/rivy/PCRE .<br>
Another popular choice is
[PCRE, 'Perl-Compatible Regular Expressions'](http://www.pcre.org/).<br>
Older 32-bit binaries are at:
http://gnuwin32.sourceforge.net/packages/regex.htm .<br>
See also:
http://ftp.gnome.org/pub/gnome/binaries/win32/dependencies/regex.README .

BUILDING on Windows (Cygwin)
----------------------------
The easiest way to have link-grammar working on MS Windows is to
use Cygwin, a Linux-like environment for Windows making it possible
to port software running on POSIX systems to Windows.  Download and
install [Cygwin](http://www.cygwin.com/).

Unfortunately, the Cygwin system is not compatible with Java, so if
you need the Java bindings, you must use MSVC or MinGW, below.

BUILDING on Windows (MinGW)
---------------------------
Another way to build link-grammar is to use MinGW, which uses the GNU
toolset to compile Windows programs for Windows. Using MinGW/MSYS is
probably the easiest way to obtain workable Java bindings for Windows.
Download and install [MinGW, MSYS and MSYS-DTK](http://mingw.org).

For more details see [mingw/README.MSYS](mingw/README.MSYS).
You can also build with MinGW under Cygwin.
See [mingw/README.Cygwin](mingw/README.Cygwin).


BUILDING and RUNNING on Windows (MSVC)
--------------------------------------
Microsoft Visual C/C++ project files can be found in the msvc14 directory.
For directions see the [README.md](msvc14/README.md) file there.

RUNNING the program
-------------------
To run the program issue the command (supposing it is in your PATH):
```
link-parser [arguments]
```

This starts the program.  The program has many user-settable variables
and options. These can be displayed by entering `!var` at the link-parser
prompt.  Entering `!help` will display some additional commands.

The dictionaries are arranged in directories whose name is the 2-letter
language code. The link-parser program searches for such a language
directory in that order, directly or under a directory names `data`:

1. Under your current directory.
2. Unless compiled with MSVC or run under the Windows console:
   At the installed location (typically in /usr/local/share/link-grammar).
3. If compiled on Windows: In the directory of the link-parser
   executable (may be in a different location than the link-parser
   command, which may be a script).

If link-parser cannot find the desired dictionary, use verbosity
level 3 to debug the problem; for example:
```
link-parser ru -verbosity=3
```

Other locations can be specified on the command line; for example:
```
link-parser ../path/to-my/modified/data/en
```

When accessing dictionaries in non-standard locations, the standard
file-names are still assumed (i.e. 4.0.dict, 4.0.affix, etc.).

The Russian dictionaries are in data/ru. Thus, the Russian parser
can be started as:
```
link-parser ru
```

If you don't supply an argument to link-parser, it searches for a
language according to your current locale setup. If it cannot find such
a language directory, it defaults to "en".

If you see errors similar to this:
```
Warning: The word "encyclop" found near line 252 of en/4.0.dict
matches the following words:
encyclop
This word will be ignored.
```

then your UTF-8 locales are either not installed or not configured.
The shell command `locale -a` should list en_US.utf8 as a locale.
If not, then you need to `dpkg-reconfigure locales` and/or run
`update-locale` or possibly `apt-get install locales`, or
combinations or variants of these, depending on your operating
system.


TESTING the program
-------------------
There are several ways to test the resulting build.  If the Python
bindings are built, then a test program can be found in the file
`./bindings/python-examples/tests.py` -- When run, it should pass.
For more details see [README.md](bindings/python-examples/README.md)
in the `bindings/python-examples` directory.

There are also multiple batches of test/example sentences in the
language data directories, generally having the names corpus-*.batch
The parser program can be run in batch mode, for testing the system
on a large number of sentences.  The following command runs the
parser on a file called corpus-basic.batch;
```
link-parser < corpus-basic.batch
```

The line `!batch` near the top of corpus-basic.batch turns on batch
mode.  In this mode, sentences labeled with an initial `*` should be
rejected and those not starting with a `*` should be accepted.  This
batch file does report some errors, as do the files `corpus-biolg.batch`
and `corpus-fixes.batch`.  Work is ongoing to fix these.

The `corpus-fixes.batch` file contains many thousands of sentences
that have been fixed since the original 4.1 release of link-grammar.
The `corpus-biolg.batch` contains biology/medical-text sentences from
the BioLG project. The `corpus-voa.batch` contains samples from Voice
of America; the `corpus-failures.batch` contains a large number of
failures.

The following numbers are subject to change, but, at this time, the
number of errors one can expect to observe in each of these files
are roughly as follows:
```
en/corpus-basic.batch:      72 errors
en/corpus-fixes.batch:     404 errors
lt/corpus-basic.batch:      15 errors
ru/corpus-basic.batch:      47 errors
```
The bindings/python directory contains a unit test for the python
bindings. It also performs several basic checks that stress the
link-grammar libraries.


USING the parser in your own applications
-----------------------------------------
There is an API (application program interface) to the parser.  This
makes it easy to incorporate it into your own applications.  The API
is documented on the web site.


USING CMake
-----------
The FindLinkGrammar.cmake file can be used to test for and set up
compilation in CMake-based build environments.


USING pkg-config
----------------
To make compiling and linking easier, the current release uses
the pkg-config system. To determine the location of the link-grammar
header files, say `pkg-config --cflags link-grammar`  To obtain
the location of the libraries, say `pkg-config --libs link-grammar`
Thus, for example, a typical makefile might include the targets:
```
.c.o:
   cc -O2 -g -Wall -c $< `pkg-config --cflags link-grammar`

$(EXE): $(OBJS)
   cc -g -o $@ $^ `pkg-config --libs link-grammar`
```

JAVA bindings
-------------
This release includes Java bindings.  Their use is optional.

The bindings will be built automatically if `jni.h` can be found.
Some common java JVM distributions (most notably, the ones from Sun)
place this file in unusual locations, where it cannot be
automatically found.  To remedy this, make sure that JAVA_HOME is
set. The configure script looks for jni.h in `$JAVA_HOME/Headers`
and in `$JAVA_HOME/include`; it also examines corresponding locations
for $JDK_HOME.  If `jni.h `still cannot be found, specify the location
with the CPPFLAGS variable: so, for example,
```
export CPPFLAGS="-I/opt/jdk1.5/include/:/opt/jdk1.5/include/linux"
```
or
```
export CPPFLAGS="-I/c/java/jdk1.6.0/include/ -I/c/java/jdk1.6.0/include/win32/"
```

Please note that the use of /opt is non-standard, and most system
tools will fail to find packages installed there.

The building of the Java bindings can be disabled by configuring as
below:
```
./configure --disable-java-bindings
```


Using JAVA
----------
This release provides java files that offer three ways of accessing
the parser.  The simplest way is to use the org.linkgrammar.LinkGrammar
class; this provides a very simple Java API to the parser.

The second possibility is to use the LGService class.  This implements
a TCP/IP network server, providing parse results as JSON messages.
Any JSON-capable client can connect to this server and obtain parsed
text.

The third possibility is to use the org.linkgrammar.LGRemoteClient
class, and in particular, the parse() method.  This class is a network
client that connects to the JSON server, and converts the response
back to results accessible via the ParseResult API.

The above-described code will be built if Apache `ant` is installed.


Using the Network Server
------------------------
The network server can be started by saying:
```
java -classpath linkgrammar.jar org.linkgrammar.LGService 9000
```

The above starts the server on port 9000. It the port is omitted,
help text is printed.  This server can be contacted directly via
TCP/IP; for example:
```
telnet localhost 9000
```

(Alternately, use netcat instead of telnet). After connecting, type
in:
```
text:  this is an example sentence to parse
```

The returned bytes will be a JSON message providing the parses of
the sentence.  By default, the ASCII-art parse of the text is not
transmitted. This can be obtained by sending messages of the form:
```
storeDiagramString:true, text: this is a test.
```

Spell Guessing
--------------
The parser will run a spell-checker at an early stage, if it
encounters a word that it does not know, and cannot guess, based on
morphology.  The configure script looks for the aspell or hunspell
spell-checkers; if the aspell devel environment is found, then
aspell is used, else hunspell is used.

Spell guessing may be disabled at runtime, in the link-parser client
with the `!spell=0` flag.  Enter `!help` for more details.


MULTI-THREADED USE
------------------
It is safe to use link-grammar for parsing in multiple threads, once
the dictionaries have been loaded.  The dictionary loading itself is
not thread-safe; it is not protected in any way.  Thus, link-grammar
should not be used from multiple threads until the dictionary has
been loaded.  Different threads may use different dictionaries.
Parse options can be set on a per-thread basis, with the exception
of verbosity, which is a global, shared by all threads.  It is the
only global, outside of the Java bindings.

For multi-threaded Java use, a per-thread variable is needed.  This
must be enabled during the configure stage:
```
./configure --enable-pthreads
```

The following exceptions and special notes apply:

*utilities.c* −
> has global "verbosity". Memory usage code (disabled
> by default) also has a global, and so requires
> pthreads for tracking memory usage.

*jni-client.c* −
> uses per-thread struct. This should somehow be
> attached to JNIEnv somehow.  A Java JNI expert is needed.

*malloc-dbg.c* −
> not thread safe, not normally used;
> only for debugging.

*pp_lexer.c*  −
> autogened code, original lex sources lost.
> This is only used when reading dictionaries,
> during initialization, and so doesn't need
> to be thread safe.


SAT solver
----------
The current parser uses an algorithm that runs in O(N^3) time, for
a sentence containing N words.

The SAT solver aims to replace this parser with an algorithm based
on Boolean Satisfiability Theory; specifically using the MiniSAT
solver. The SAT solver has a bit more overhead for shorter sentences,
but is faster for long sentences.  To work properly, it needs to be
attached to a parse ranking system.  This work is incomplete,
although the prototype works.  It is not yet well-integrated with
the system, and needs cleanup.
Still not handled (or handled incorrectly):
- Disjunct cost: Cost of null expressions is disregarded. Thus, it
  still cannot rank sentences by cost, which is the most basic parse
  ranking that we've got... In order not to show incorrect costs, the
  DIS= field in the status message is always 0.
- Connector order shown by the `!disjunct` link-parser command.
  Currently it is just a "random" order.
- Parsing with null count.
- No panic timeout.

The SAT solver is enabled by default. If the minisat2 library package
is installed in the system along with its header files (e.g. RPM
package minisat2-devel, deb package minisat2) then it is used. Else,
a bundled minisat2 library code is used.

The following forces using the bundled minisat library:
```
./configure --enable-sat-solver=bundled
```

The SAT solver can be disabled by specifying:
```
./configure --disable-sat-solver
```

(Both are to be done prior to compiling.)


Phonetics
---------
A/An phonetic determiners before consonants/vowels are handled by a
new PH link type, linking the determiner to the word immediately
following it.  Status: Introduced in version 5.1.0 (August 2014).
Mostly done, although many special-case nouns are unfinished.


Directional Links
-----------------
Directional links are needed for some languages, such as Lithuanian,
Turkish and other free word-order languages. The goal is to have
a link clearly indicate which word is the head word, and which is
the dependent. This is achieved by prefixing connectors with
a single *lower case* letter: h,d, indicating 'head' and 'dependent'.
The linkage rules are such that h matches either nothing or d, and
d matches h or nothing. This is a new feature in version 5.1.0
(August 2014). The website provides additional documentation.

Although the English-language link-grammar links are un-oriented,
it seems that a defacto direction can be given to them that is
completely consistent with standard conceptions of a dependency
grammar.

The dependency arrows have the following properties:

 * Anti-reflexive (a word cannot depend on itself; it cannot point
   at itself.)

 * Anti-symmetric (if Word1 depends on Word2, then Word2 cannot
   depend on Word1) (so, e.g. determiners depend on nouns, but
   never vice-versa)

 * The arrows are neither transitive, nor anti-transitive: a single
   word may be ruled by several heads.  For example:
```text
    +------>WV------->+
    +-->Wd-->+<--Ss<--+
    |        |        |
LEFT-WALL   she    thinks.v
```
That is, there is a path to the subject, "she", directly from the
left wall, via the Wd link, as well as indirectly, from the wall
to the root verb, and thence to the subject.  Similar loops form
with the B and R links.  Such loops are useful for constraining
the possible number of parses: the constraint occurs in
conjunction with the "no links cross" meta-rule.

 * The graphs are planar; that is, no two edges may cross. See,
   however, the "link-crossing" discussion below.


There are several related mathematical notions, but none quite
capture directional LG:

 * Directional LG graphs resemble DAGS, except that LG allows only
   one wall (one "top" element).

 * Directional LG graphs resemble strict partial orders, except that
   the LG arrows are usually not transitive.

 * Directional LG graphs resemble
   [catena](http://en.wikipedia.org/wiki/Catena_(linguistics))
   except that catena are strictly anti-transitive -- the path to
   any word is unique, in a catena.


Link Crossing
-------------
The foundational LG papers mandate the planarity of the parse graphs.
This is based on a very old observation that dependencies almost never
cross in natural languages: humans simply do not speak in sentences
where links cross.  Imposing planarity constraints then provides a
strong engineering and algorithmic constraint on the resulting parses:
the total number of parses to be considered is sharply reduced, and
thus the overall speed of parsing can be greatly increased.

However, there are occasional, relatively rare exceptions to this
planarity rule; such exceptions are observed in almost all languages.
A number of these exceptions are given for English, below.

Thus, it seems important to relax the planarity constraint, and find
something else that is almost as strict, but still allows infrequent
exceptions.  It would appear that the concept of "landmark transitivity"
as defined by Richard Hudson in his theory of "Word Grammar", and then
advocated by Ben Goertzel, just might be such a mechanism.

ftp://ftp.phon.ucl.ac.uk/pub/Word-Grammar/ell2-wg.pdf<br>
http://www.phon.ucl.ac.uk/home/dick/enc/syntax.htm<br>
http://goertzel.org/ProwlGrammar.pdf

This mechanism works as follows:

 * First, every link must be directional, with a head and a dependent.
That is, we are concerned with directional-LG links, which are
of the form x--A-->y or y<--A--x for words x,y and LG link type A.

 * Given either the directional-LG relation x--A-->y or y<--A--x,
define the dependency relation x-->y.  That is, ignore the link-type
label.

 * Heads are landmarks for dependents. If the dependency relation
x-->y holds, then x is said to be a landmark for y, and the
predicate land(x,y) is true, while the predicate land(y,x) is false.
Here, x and y are words, while --> is the landmark relation.

 * Although the basic directional-LG links form landmark relations,
the total set of landmark relations is extended by transitive closure.
That is, if land(x,y) and land(y,z) then land(x,z).  That is, the
basic directional-LG links are "generators" of landmarks; they
generate by means of transitivity.  Note that the transitive closure
is unique.

 * In addition to the above landmark relation, there are two additional
relations: the before and after landmark relations. (In English,
these correspond to left and right; in Hebrew, the opposite).
That is, since words come in chronological order in a sentence,
the dependency relation can point either left or right.  The
previously-defined landmark relation only described the dependency
order; we now introduce the word-sequence order. Thus, there are
are land-before() and land-after() relations that capture both
the dependency relation, and the word-order relation.

 * Notation: the before-landmark relation land-B(x,y) corresponds to
x-->y (in English, reversed in right-left languages such as Hebrew),
whereas the after-landmark relation land-A(x,y) corresponds to y<--x.
That is, land(x,y) == land-B(x,y) or land-A(x,y) holds as a statement
about the predicate form of the relations.

 * As before, the full set of directional landmarks are obtained by
transitive closure applied to the directional-LG links.  Two
different rules are used to perform this closure:
```
-- land-B(x,y) and land(y,z) ==> land-B(x,y)
-- land-A(x,y) and land(y,z) ==> land-A(x,y)
```
Parsing is then performed by joining LG connectors in the usual manner,
to form a directional link. The transitive closure of the directional
landmarks are then computed. Finally, any parse that does not conclude
with the "left wall" being the upper-most landmark is discarded.

Here is an example where landmark transitivity provides a natural
solution to a (currently) broken parse. The "to.r" has a disjunct
"I+ & MVi-" which allows "What is there to do?" to parse correctly.
However, it also allows the incorrect parse "He is going to do".
The fix would be to force "do" to take an object; however, a link
from "do" to "what" is not allowed, because link-crossing would
prevent it.

Examples where the no-links-cross constraint seems to be violated,
in English:
```text
  "He is either in the 105th or the 106th battalion."
  "He is in either the 105th or the 106th battalion."
```
Both seem to be acceptable in English, but the ambiguity of the
"in-either" temporal ordering requires two different parse trees, if
the no-links-cross rule is to be enforced. This seems un-natural.
Similarly:
```text
  "He is either here or he is there."
  "He either is here or he is there."
```
Other examples, per And Rosta:

The *allowed--by* link crosses *cake--that*:
```text
He had been allowed to eat a cake by Sophy that she had made him specially
```

*a--book*, *very--indeed*
```text
"a very much easier book indeed"
```

*an--book*, *easy--to*
```text
"an easy book to read"
```

*a--book*, *more--than*
```text
"a more difficult book than that one"
```

*that--have* crosses *remains--of*
```text
"It was announced that remains have been found of the ark of the covenant"
```

There is a natural crossing, driven by conjunctions:
```text
"I was in hell yesterday and heaven on Tuesday."
```

the "natural" linkage is to use MV links to connect "yesterday" and "on
Tuesday" to the verb. However, if this is done, then these must cross
the links from the conjunction "and" to "heaven" and "hell".  This can
be worked around partly as follows:
```text
              +-------->Ju--------->+
              |    +<------SJlp<----+
+<-SX<-+->Pp->+    +-->Mpn->+       +->SJru->+->Mp->+->Js->+
|      |      |    |        |       |        |      |      |
I     was    in  hell   yesterday  and    heaven    on  Tuesday
```
but the desired MV links from the verb to the time-prepositions
"yesterday" and "on Tuesday" are missing -- whereas they are present,
when the individual sentences "I was in hell yesterday" and
"I was in heaven on Tuesday" are parsed.  Using a conjunction should
not wreck the relations that get used; but this requires link-crossing.

Another, simpler example:

```text
    +---->WV---->+
    |            +--------IV---------->+
    |            |           +<-VJlpi--+
    |            |           |    +---xxx------------Js------->+
    +--Wd--+-Sp*i+--TO-+-I*t-+-MVp+    +--VJrpi>+--MVp-+---Js->+
    |      |     |     |     |    |    |        |      |       |
LEFT-WALL I.p want.v to.r look.v at and.j-v listen.v to.r everything
```
The above really wants to have a `Js` link from 'at' to 'everything',
but this `Js` link crosses (clashes with - marked by xxx) the link
to the conjunction.  These two cases suggest that one sould/should
allow most links to cross over the down-links to conjunctions.



Type Theory
-----------
Link Grammar can be understood in the context of type theory.
A simple introduction to type theory can be found in chapter 1
of the [HoTT book](https://homotopytypetheory.org/book/).<br>
This book is freely available online and strongly recommended if
you are interested in types.

Link types can be mapped to types that appear in categorial grammars.
The nice thing about link-grammar is that the link types form a type
system that is much easier to use and comprehend than that of categorial
grammar, and yet can be directly converted to that system!  That is,
link-grammar is completely compatible with categorial grammar, and is
easier-to-use.

The foundational LG papers make comments to this effect; however, see
also work by Bob Coecke on category theory and grammar.  Coecke's
diagramatic approach is essentially identical to the diagrams given in
the foundational LG papers; it becomes abundantly clear that the
category theoretic approach is equivalent to Link Grammar. See, for
example, this introductory sketch
http://www.cs.ox.ac.uk/people/bob.coecke/NewScientist.pdf
and observe how the diagrams are essentially identical to the LG
jigsaw-puzzle piece diagrams of the foundational LG publications.


ADDRESSES
---------
If you have any questions, please feel free to send a note to the
[mailing list](http://groups.google.com/group/link-grammar).

The source code of link-parser and the link-grammar library is located at
[GitHub](https://github.com/opencog/link-grammar).<br>
For bug reports, please open an **issue** there.

Although all messages should go to the mailing list, the current
maintainers can be contacted at:
```text
  Linas Vepstas - <linasvepstas@gmail.com>
  Amir Plivatsky - <amirpli@gmail.com>
  Dom Lachowicz - <domlachowicz@gmail.com>
```
A complete list of authors and copyright holders can be found in the
AUTHORS file.  The original authors of the Link Grammar parser are:
```text
  Daniel Sleator                    sleator@cs.cmu.edu
  Computer Science Department       412-268-7563
  Carnegie Mellon University        www.cs.cmu.edu/~sleator
  Pittsburgh, PA 15213

  Davy Temperley                    dtemp@theory.esm.rochester.edu
  Eastman School of Music           716-274-1557
  26 Gibbs St.                      www.link.cs.cmu.edu/temperley
  Rochester, NY 14604

  John Lafferty                     lafferty@cs.cmu.edu
  Computer Science Department       412-268-6791
  Carnegie Mellon University        www.cs.cmu.edu/~lafferty
  Pittsburgh, PA 15213
```


TODO -- Working Notes
---------------------
Some working notes.

Easy to fix: provide a more uniform API to the constituent tree.
i.e provide word index.   Also .. provide a clear word API,
showing word extent, suffix, etc.

Capitalized first words:
There are subtle technical issues for handling capitalized first
words. This needs to be fixed. In addition, for now these words are
shown uncapitalized in the result linkages. This can be fixed.

Maybe capitalization could be handled in the same way that a/an
could be handled!  After all, it's essentially a nearest-neighbor
phenomenon!

Capitalization-mark tokens:
The proximal issue is to add a cost, so that Bill gets a lower
cost than bill.n when parsing "Bill went on a walk".  The best
solution would be to add a 'capitalization-mark token' during
tokenization; this token precedes capitalized words. The
dictionary then explicitly links to this token, with rules similar
to the a/an phonetic distinction.  The point here is that this
moves capitalization out of ad-hoc C code and into the dictionary,
where it can be handled like any other language feature.
The tokenizer includes experimental code for that.

Corpus-statistics-based parse ranking:
The old for parse ranking via corpus statistics needs to be revived.
The issue can be illustrated with these example sentences:
```text
"Please the customer, bring in the money"
"Please, turn off the lights"
```
In the first sentence, the comma acts as a conjunction of two
directives (imperatives). In the second sentence, it is much too
easy to mistake "please" for a verb, the comma for a conjunction,
and come to the conclusion that one should please some unstated
object, and then turn off the lights. (Perhaps one is pleasing
by turning off the lights?)

Punctuation, zero-copula, zero-that:
Poorly punctuated sentences cause problems:  for example:
```text
"Mike was not first, nor was he last."
"Mike was not first nor was he last."
```
The one without the comma currently fails to parse.  How can we
deal with this in a simple, fast, elegant way?  Similar questions
for zero-copula and zero-that sentences.

Zero/phantom words:  Expressions such as "Looks good" have an implicit
"it" (also called a zero-it or phantom-it) in them; that is, the
sentence should really parse as "(it) looks good".  The dictionary
could be simplified by admitting such phantom words explicitly,
rather than modifying the grammar rules to allow such constructions.
Other examples, with the phantom word in parenthesis, include:
 * I ate all (of) the cookies.
 * I taught him (how) to swim.
 * I told him (that) it was gone.
 * (It) looks good.
 * (You) go home!
 * (Are) you all right?

See [this issue on GitHub](https://github.com/opencog/link-grammar/issues/224).

One possible solution is to perform a one-point compactification.
The dictionary contains the phantom words, and thier connectors.
Ordinary disjuncts can link to these, but should do so using
a special initial lower-case letter (say, 'z', in addition to
'h' and 't' as is currently implemented).  The parser, as it
works, examines the initial letter of each connector: if it is
'z', then the usual pruning rules no longer apply, and one or
more phantom words are selected out of the bucket of phantom words.
(This bucket is kept out-of-line, it is not yet placed into
sentence word sequence order, which is why the usual pruning rules
get modified.)  Otherwise, parsing continues as normal. At the end
of parsing, if there are any phantom words that are linked, then
all of the connectors on the disjunct must be satisfied (of course!)
else the linkage is invalid. After parsing, the phantom words can
be inserted into the sentence, with the location deduced from link
lengths.

Bad grammar: When a sentence fails to parse, look for:
 * confused words: its/it's, there/their/they're, to/too, your/you're ...
   These could be added at high cost to the dicts.
 * missing apostrophes in possessives: "the peoples desires"
 * determiner agreement errors: "a books"
 * aux verb agreement errors: "to be hooks up"

Poor agreement might be handled by giving a cost to mismatched
lower-case connector letters.

Poor linkage choices:
Compare "she will be happier than before" to "she will be more happy
than before." Current parser makes "happy" the head word, and "more"
a modifier w/EA link.  I believe the correct solution would be to
make "more" the head (link it as a comparative), and make "happy"
the dependent.  This would harmonize rules for comparatives... and
would eliminate/simplify rules for less,more.

However, this idea needs to be double-checked against, e.g. Hudson's
word grammar.  I'm confused on this issue ...

Stretchy links:
Currently, some links can act at "unlimited" length, while others
can only be finite-length.  e.g. determiners should be near the
noun that they apply to.  A better solution might be to employ
a 'stretchiness' cost to some connectors: the longer they are, the
higher the cost. (This eliminates the "unlimited_connector_set"
in the dictionary).

Repulsive parses: Sometimes, the existence of one parse should suggest
that another parse must surely be wrong: if one parse is possible,
then the other parses must surely be unlikely. For example: the
conjunction and.j-g allows the "The Great Southern and Western
Railroad" to be parsed as the single name of an entity. However,
it also provides a pattern match for "John and Mike" as a single
entity, which is almost certainly wrong. But "John and Mike" has
an alternative parse, as a conventional-and -- a list of two people,
and so the existence of this alternative (and correct) parse suggests
that perhaps the entity-and is really very much the wrong parse.
That is, the mere possibility of certain parses should strongly
disfavor other possible parses. (Exception: Ben & Jerry's ice
cream; however, in this case, we could recognize Ben & Jerry as the
name of a proper brand; but this is outside of the "normal"
dictionary (?) (but maybe should be in the dictionary!))

More examples: "high water" can have the connector A joining high.a
and AN joining high.n; these two should either be collapsed into
one, or one should be eliminated.


WordNet hinting:
Use WordNet to reduce the number for parses for sentences containing
compound verb phrases, such as "give up", "give off", etc.

Incremental parsing: to avoid a combinatorial explosion of parses,
it would be nice to have an incremental parsing, phrase by phrase,
using a Viterbi-like algorithm to obtain the parse. Thus, for example,
the parse of the last half of a long, run-on sentence should not be
sensitive to the parse of the beginning of the sentence.

Doing so would help with combinatorial explosion. So, for example,
if the first half of a sentence has 4 plausible parses, and the
last half has 4 more, then link-grammar reports 16 parses total.
It would be much, much more useful to instead be given the
factored results: i.e. the four plausible parses for the
first half, and the four plausible parses for the last half.
The lower combinatoric stress would ease the burden on
downstream users of link-grammar.

(This somewhat resembles the application of construction grammar
ideas to the link-grammar dictionary).

Caution: watch out for garden-path sentences:
```text
  The horse raced past the barn fell.
  The old man the boat.
  The cotton clothing is made of grows in Mississippi.
  The current parser parses these perfectly; a viterbi parser could
  trip on these.
```
Other benefits of a Viterbi decoder:
* Less sensitive to sentence boundaries: this would allow longer,
  run-on sentences to be parsed far more quickly.
* Could do better with slang, hip-speak.
* Would enable co-reference resolution across sentences (resolve
  pronouns, etc.)
* Would allow richer state to be passed up to higher layers:
  specifically, alternate parses for fractions of a sentence,
  alternate reference resolutions.
* Would allow plug-in architecture, so that plugins, employing
  some alternate, higher-level logic, could disambiguate (e.g.
  by making use of semantic content).
* Eliminate many of the hard-coded array sizes in the code.
* Fixes the word-count problem during spell-guessing. So, for
  example, if the mis-spelled word "dont" shows up in the input, it
  could be issued as one word ("done") or two ("do n't") and the
  current suffix-stripping/word-issuing algo cannot deal with this
  correctly. By contrast, this should not be an issue for the
  Viterbi algo, as it could explore both states at once.

One may argue that Viterbi is a more natural, biological way of
working with sequences.  Some experimental, psychological support
for this can be found at
http://www.sciencedaily.com/releases/2012/09/120925143555.htm
per Morten Christiansen, Cornell professor of psychology.


Registers, sociolects, dialects (cost vectors):
Consider the sentence "Thieves rob bank" -- a typical newspaper
headline. LG currently fails to parse this, because the determiner
is missing ("bank" is a count noun, not a mass noun, and thus
requires a determiner. By contrast, "thieves rob water" parses
just fine.) A fix for this would be to replace mandatory
determiner links by (D- or {[[()]] & headline-flag}) which allows
the D link to be omitted if the headline-flag bit is set.
Here, "headline-flag" could be a new link-type, but one that is
not subject to planarity constraints.

Note that this is easier said than done: if one simply adds a
high-cost null link, and no headline-flag, then all sorts of
ungrammatical sentences parse, with strange parses; while some
grammatical sentences, which should parse, but currently don't,
become parsable, but with crazy results.

More examples, from And Rosta:
```text
   "when boy meets girl"
   "when bat strikes ball"
   "both mother and baby are well"
```

A natural approach would be to replace fixed costs by formulas.
This would allow the dialect/sociolect to be dynamically
changeable.  That is, rather than having a binary headline-flag,
there would be a formula for the cost, which could be changed
outside of the parsing loop.  Such formulas could be used to
enable/disable parsing specific to different dialects/sociolects,
simply by altering the network of link costs.

Perhaps a simpler alternative would be to have labeled costs (a
cost vector), so that different dialects assign different costs to
various links.  A dialect would be specified during the parse,
thus causing the costs for that dialect to be employed during
parse ranking.

Imperatives:
```text
"Push button"
"Push button firmly"
```
The zero/phantom-word solution, described above, should help with this.

Hand-refining verb patterns:<br>
   A good reference for refining verb usage patterns is:<br>
   COBUILD GRAMMAR PATTERNS 1: VERBS<br>
   from THE COBUILD SERIES /from/ THE BANK OF ENGLISH<br>
   HARPER COLLINS<br>
   online at https://arts-ccr-002.bham.ac.uk/ccr/patgram/<br>
   http://www.corpus.bham.ac.uk/publications/index.shtml


*Quotations*: tokenize.c tokenizes Double-quotes and some UTF8 quotes
   (see the RPUNC/LPUNC class in en/4.0.affix - the QUOTES class is
   not used for that, but for capitalization support), with some very
   basic support in the English dictionary (see "% Quotation marks."
   there).  However, it does not do this for the various "curly" UTF8
   quotes, such as ‘these’ and “these”.  This results is some ugly
   parsing for sentences containing such quotes. (Note that these are
   in 4.0.affix).
   A mechanism is needed to disentangle the quoting from the quoted
   text, so that each can be parsed appropriately.  It's somewhat
   unclear how to handle this within link-grammar. This is somewhat
   related to the problem of morphology (parsing words as if they
   were "mini-sentences",) idioms (phrases that are treated as if
   they were singe words), set-phrase structures (if ... then ... not
   only... but also ...) which have a long-range structure similar to
   quoted text (he said ...).

  "to be fishing": Link grammar offers four parses of "I was fishing for
  evidence", two of which are given low scores, and two are given
  high scores. Of the two with high scores, one parse is clearly bad.
  Its links "to be fishing.noun" as opposed to the correct
  "to be fishing.gerund". That is, I can be happy, healthy and wise,
  but I certainly cannot be fishing.noun.  This is perhaps not
  just a bug in the structure of the dictionary, but is perhaps
  deeper: link-grammar has little or no concept of lexical units
  (i.e. collocations, idioms, institutional phrases), which thus
  allows parses with bad word-senses to sneak in.

  The goal is to introduce more knowledge of lexical units into LG.

  Different word senses can have different grammar rules (and thus,
  the links employed reveal the sense of the word): for example:
  "I tend to agree" vs. "I tend to the sheep" -- these employ two
  different meanings for the verb "tend", and the grammatical
  constructions allowed for one meaning are not the same as those
  allowed for the other. Yet, the link rules for "tend.v" have
  to accommodate both senses, thus making the rules rather complex.
  Worse, it potentially allows for non-sense constructions.
  If, instead, we allowed the dictionary to contain different
  rules for "tend.meaning1" and "tend.meaning2", the rules would
  simplify (at the cost of inflating the size of the dictionary).

  Another example: "I fear so" -- the word "so" is only allowed
  with some, but not all, lexical senses of "fear". So e.g.
  "I fear so" is in the same semantic class as "I think so" or
  "I hope so", although other meanings of these verbs are
  otherwise quite different.

  [Sin2004] "New evidence, new priorities, new attitudes" in J.
  Sinclair, (ed) (2004) How to use corpora in language teaching,
  Amsterdam: John Benjamins

  See also: Pattern Grammar: A Corpus-Driven Approach to the Lexical
  Grammar of English<br>
  Susan Hunston and Gill Francis (University of Birmingham)<br>
  Amsterdam: John Benjamins (Studies in corpus linguistics,
  edited by Elena Tognini-Bonelli, volume 4), 2000<br>
  [Book review](http://www.aclweb.org/anthology/J01-2013).

  "holes" in collocations (aka "set phrases" of "phrasemes"):
  The link-grammar provides several mechanisms to support
  circumpositions or even more complicated multi-word structures.
  One mechanism is by ordinary links; see the V, XJ and RJ links.
  The other mechanism is by means of post-processing rules.
  (For example, the "filler-it" SF rules use post-processing.)
  However, rules for many common forms have not yet been written.
  The general problem is of supporting structures that have "holes"
  in the middle, that require "lacing" to tie them together.

  For a general theory, see
  [catena](http://en.wikipedia.org/wiki/Catena_(linguistics)).

  For example, the adposition:
```text
... from [xxx] on.
    "He never said another word from then on."
    "I promise to be quiet from now on."
    "Keep going straight from that point on."
    "We went straight from here on."

... from there on.
    "We went straight, from the house on to the woods."
    "We drove straight, from the hill onwards."
```
Note that multiple words can fit in the slot [xxx].
Note the tangling of another prepositional phrase:
`"... from [xxx] on to [yyy]"`

More complicated collocations with holes include
```text
 "First.. next..."
 "If ... then ..."
```
'Then' is optional ('then' is a 'null word'), for example:
```text
"If it is raining, stay inside!"
"If it is raining, [then] stay inside!"


"if ... only ..." "If there were only more like you!"
"... not only, ... but also ..."


"As ..., so ..."  "As it was commanded, so it shall be done"


"Either ... or ..."
"Both ... and  ..."  "Both June and Tom are coming"
"ought ... if ..." "That ought to be the case, if John is not lying"


"Someone ... who ..."
"Someone is outside who wants to see you"


"... for ... to ..."
"I need for you to come to my party"
```
The above are not currently supported. An example that is supported
is the "non-referential it", e.g.
```
"It ... that ..."
"It seemed likely that John would go"
```
The above is supported by means of special disjuncts for 'it' and
'that', which must occur in the same post-processing domain.

See also:<br>
http://www.phon.ucl.ac.uk/home/dick/enc2010/articles/extraposition.htm<br>
http://www.phon.ucl.ac.uk/home/dick/enc2010/articles/relative-clause.htm

 "...from X and from Y"
 "By X, and by Y, ..."
 Here, X and Y might be rather long phrases, containing other
 prepositions. In this case, the usual link-grammar linkage rules
 will typically conjoin "and from Y" to some preposition in X,
 instead of the correct link to "from X". Although adding a cost to
 keep the lengths of X and Y approximately equal can help, it would
 be even better to recognize the "...from ... and from..." pattern.

 The correct solution for the "Either ... or ..." appears to be this:
```text
---------------------------+---SJrs--+
       +------???----------+         |
       |     +Ds**c+--SJls-+    +Ds**+
       |     |     |       |    |    |
   either.r the lorry.n or.j-n the van.n
```
 The wrong solution is
```text
--------------------------+
     +-----Dn-----+       +---SJrs---+
     |      +Ds**c+--SJn--+     +Ds**+
     |      |     |       |     |    |
 neither.j the lorry.n nor.j-n the van.n
```
 The problem with this is that "neither" must coordinate with "nor".
 That is, one cannot say "either.. nor..." "neither ... or ... "
 "neither ...and..." "but ... nor ..."  The way I originally solved
 the coordination problem was to invent a new link called Dn, and a
 link SJn and to make sure that Dn could only connect to SJn, and
 nothing else. Thus, the lower-case "n" was used to propagate the
 coordination across two links. This demonstrates how powerful the
 link-grammar theory is: with proper subscripts, constraints can be
 propagated along links over large distances. However, this also
 makes the dictionary more complex, and the rules harder to write:
 coordination requires a lot of different links to be hooked together.
 And so I think that creating a single, new link, called ???, will
 make the coordination easy and direct. That is why I like that idea.

 The ??? link should be the XJ link, which-see.


 More idiomatic than the above examples:
 "...the chip on X's shoulder"
 "to do X a favour"
 "to give X a look"

 The above are all examples of "set phrases" or "phrasemes", and are
 most commonly discussed in the context of MTT or Meaning-Text Theory
 of Igor Mel'cuk et al (search for "MTT Lexical Function" for more
 info). Mel'cuk treats set phrases as lexemes, and, for parsing, this
 is not directly relevant. However, insofar as phrasemes have a high
 mutual information content, they can dominate the syntactic
 structure of a sentence.

 MTT suggests that perhaps the correct way to understand the contents
 of the post-processing rules is as an implementation of 'lexical
 functions' projected onto syntax.  That is, the post-processing
 rules allow only certain syntactical constructions, and these are
 the kinds of constructions one typically sees in certain kinds
 of lexical functions.

 Alternately, link-grammar suffers from a combinatoric explosion
 of possible parses of a given sentence. It would seem that lexical
 functions could be used to rule out many of these parses.  On the
 other hand, the results are likely to be similar to that of
 statistical pare ranking (which presumably captures such
 quasi-idiomatic collocations at least weakly).

 Ref. I. Mel'cuk: "Collocations and Lexical Functions", in ''Phraseology:
 theory, analysis, and applications'' Ed. Anthony Paul Cowie (1998)
 Oxford University Press pp. 23-54.

 More generally, all of link-grammar could benefit from a MTT-izing
 of infrastructure.

 Compare the above problem to Hebrew morphological analysis. To quote
   Wikipedia:

   > This distinction between the word as a unit of speech and the
   > root as a unit of meaning is even more important in the case of
   > languages where roots have many different forms when used in
   > actual words, as is the case in Semitic languages. In these,
   > roots are formed by consonants alone, and different words
   > (belonging to different parts of speech) are derived from the
   > same root by inserting vowels. For example, in Hebrew, the root
   > gdl represents the idea of largeness, and from it we have gadol
   > and gdola (masculine and feminine forms of the adjective "big"),
   > gadal "he grew", higdil "he magnified" and magdelet "magnifier",
   > along with many other words such as godel "size" and migdal
   > "tower".

- Dealing with long, ambiguous sentences:

   These are busted up by humans into smaller sentences that "hang
   together" as phrase-structures, viz compounded sentences. The most
   likely parse is then when each of the quasi sub-sentences is
   parsed correctly.

- Alternatives:

   A partial solution to the morphology problem and the idiom problem
   in link-grammar is to elevate the use of "alternatives" in the
   Word struct.  Originally, these were morphological split alternatives
   for the Russian dicts, but really, they are a way of hierarchically
   arranging choices for words...

   Status: DONE! Implemented from version 5.3.0. See the section
   titled "Introduction of a word-graph for tokenizing" in
   [link-grammar/README](link-grammar/README).

- Morphology printing:

   Instead of hard-coding LL, declare which links are morpho links
   in the dict.

-  Word-order flexibility (For Lithuanian, the following are desperately needed):
   *  Connectors with * direction, i.e. either left or right.
   *  Symmetric (commuting) version of &.
   *  DONE! The new symbols are ^ for commuting-& and $ to meaneither + or -.

   This still needs to be documented.

- Incremental sentence parsing.
   There are multiple reasons to support incremental parsing:
   * Real-time dialog.
   * Parsing of multiple streams, e.g. from play/movie scripts.
   * Segmentation of exceptionally long sentences.

   This could be implemented by saving dangling right-going
   connectors into a parse context, and then, when another sentence
   fragment arrives, use that context in place of the left-wall.

- UTF-8 cleanup:
   Replace the mbrtowc code with proper language support; it seems
   that the correct solution is to use [ICU](http://site.icu-project.org/)
   *  ICU pros: runs on windows.
   *  ICU cons: big, complex.

   Another alternative is [libunistring](http://www.gnu.org/software/libunistring/)
   (which seems to be LGPL!?)
   *  Pros: smaller, simpler than ICU.
   *  Cons: might have problems with MS Windows.

- Assorted minor cleanup:
   * Should provide a query that returns compile-time consts,
      e.g. the max number of characters in a word, or max words
      in a sentence.
   * Should remove compile-time constants, e.g. max words, max
      length etc.

- Misc TODO:
   * Finish sqlite3 work.

Version 6.0 TODO list:
Version 6.0 will change `Sentence` to `Sentence*,` `Linkage` to `Linkage*` in the API.  Perhaps this is a bad idea...


A performance diary
-------------------
Time to parse some long sentences:
The original results below were for version 5.0.8 (April 2014)
The June 2014 results are for version 5.1.0
The Feb 2017 results are on version 5.3.15
Times are user-times, with dict loading subtracted.

These are very highly dependent on the aggressiveness of the token
splitter, on the short length, on the cost-max and the spell checker.
Suggest using flags: -spell=0 -short=10 -cost-max=2.1

25 words + 2 punct, 0.2 seconds  (0.7 seconds June 2014)
(0.2 secs SAT, June 2014):
```text
Hot runners usually make the mold more expensive to manufacture and run,
but allow savings by reducing plastic waste and by reducing the cycle time.
```

38 words + 4 punct: 2.4 seconds (2.6 secs, June 2014)
(0.32 secs, SAT, June 2014) (3.2 sec Feb 2017):
```text
The strongest rain ever recorded in India shut down the financial hub
of Mumbai, snapped communication lines, closed airports and forced
thousands of people to sleep in their offices or walk home during the
night, officials said today.
```

50 words + 9 punct: 14 seconds (3.9 secs June 2014)
(0.64 secs, SAT June 2014) (2.1 secs Feb 2017):
```text
In vivo studies of the activity of four of the kinases, KinA, KinC,
KinD (ykvD) and KinE (ykrQ), using abrB transcription as an indicator
of Spo0A~P level,revealed that KinC and KinD were responsible for
Spo0A~P production during the exponential phase of growth in the absence
of KinA and KinB.
```

56 words + 8 punct: 4.5 seconds (1.45 secs June 2014)
(0.38 secs, SAT June 2014) (broken, Feb 2017):
```text
New York Post: The new Mel Brooks/Susan Stroman musical extravaganza ...
is nearly very good indeed - but it is not the The Producers ...
this story ... does not lend itself to stage adaptation in the way of
the earlier movie ...  Now for the good news ... Brooks and Stroman
pull out every stop.
```

57 words + 10 punct: 7.5 seconds (6.8 seconds June 2014)
(0.68 secs, SAT June 2014) (4.5 seconds 4.3.15 Feb 2017):
```text
However, the few tracts, the poetry, and the novels that embodied the
social vision of Young England were directed to a New Generation of
educated, religious, and socially conscious conservatives, who, like
Young Englanders, were appalled at the despiritualizing effects of
industrialization and the perceived amorality of Benthamite philosophy,
which they blamed equally for Victorian social injustices.
```

73 words + 8 punct: 145 seconds:
```text
Cortes in his various letters again and again claims the Emperor's
patronage of his bold defiance of the Emperor's officers on the ground
that the latter in their action were moved solely by considerations of
their personal gain, whereas he, Cortes, was striving to endow his
sovereign with a rich new empire and boundless treasure whilst carrying
into the dark pagan land, at the sword's point, the gentle creed of the
Christian God.
```
