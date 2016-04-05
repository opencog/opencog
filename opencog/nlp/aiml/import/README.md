AIML to Atomese Conversion
--------------------------
This directory contains a perl script to convert AIML to OpenCog
Atomese, so that the AIML rules can be integrated into the rest of
the OpenCog natural language processing infrastructure.


### Usage
```
perl aiml2oc.pl --dir ./aimldir --final atomese.scm
```
or
```
perl aiml2oc.pl --overwrite --dir ./aimldir --final atomese.scm
```
where, `./aimldir` contains aiml files and `atomese.scm` is the output,
containing the atomese representation of aiml rules in scheme.

For details, run:
```
perl aiml2oc.pl --help
```
For aiml files, see https://github.com/jstnhuang/chatbot/tree/master/aiml

### Overview
Conversion is done in a two-pass process.  The first pass flattens
the AIML format into a simplified linear format.  A second pass
converts this flattened format into Atomese.

* Pass One : The AIML XML is converted into an intermediate neutral,
  word based format. The format is a linear sequence of tokens,
  annotated with their meaning, and optionally any performative code.
  The AIML 2.0 interpreter uses something similar to this, with
  its AIMLIF csv based format.

* Pass Two: The linearized format is converted into Atomese.

One issue with conversion is the AIML convention that "the last
definition loaded is definitive".  Typical AIML systems will sort and
load AIML files in alphabetical order, so that the later files "overlay"
or "overwrite" previous definitions. This AIML-to-opencog converter
makes all defintions visible to OpenCog, and so the OpenCog NLP system
will have to decide how it wants to handle duplicate definitions.

One reason for having an intermediate format is the handling of <set>
tags in patterns. These could either be passed on to OpenCog as either
a pointer to a collection concept, the collection list itself, a
collection concept with its definition, or various ways of unrolling
the defined set in intermediate format using duplicate definitions.

### Notes

1. To preserve the "last definition loaded is definitive" semantics of
   AIML file loading, one option is to alter the phase-two code to use
   the derived path to overwrite the final output on a last-in-only-out
   basis. Of course, doing this loses the one-to-one-to-one tracing,
   since it goes through a hash table to capture duplicates. But you can
   leave off the "--overwrite" flag for debugging, and turn it on for
   final output.

2. There is probably a better name for the "--overwrite" switch, since
   it's really not really overwriting any output, but instead the
   categories as they come in.
