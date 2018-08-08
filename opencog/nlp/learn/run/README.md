
Parse management scripts
========================

The scripts here are used to automate the ingestion of plain-text
UTF-8 files into the language learning pipeline.  These can be applied
to any flat text files from any origin of your choice.  Some tools
for downloading Wikipedia and Project Gutenberg texts can be found
in the `../download` directory.

You will typically want to make copies of these, and tailor them to
your specific needs and procedures. In particular, many of these
files require database credentials to be set; the exact credentials
to use will depend on which copy of which database you are using.
You WILL be copying around a lot of databases!

A quick overview:

* `run-shells.sh`: multi-tasking terminal server.  Opens multiple
  terminal sessions with tmux/byobu, and starts the cogserver in one
  of them.  Use F3 and F4 to switch to different terminals.

* `pair-submit-??.sh`: language-specific word-pair-counting scripts.
  These pull text files, one by one, from the data directory, and
  submit them for word-pair counting. Pick one, and run it manually
  the 'submit' byobu window.  Be sure to open the database, first.
  The directory containing the text files needs to be manually adjusted
  here; its `beta-pages` by default, but you can use any directory
  that you wish.

* `mst-submit-??.sh`: language-specific MST processing scripts.
  These pull text files, one by one, from the data directory, and
  submit them for MST processing. Pick one, and run it manually
  the 'submit' byobu window.  Be sure to have performed the mutual
  information step first. Be sure to make a copy of your database.
  Be sure to open the database, first.

  The directory containing the text files needs to be manually adjusted
  here; its `gamma-pages` by default, but you can use any directory
  that you wish.

* `pair-one.sh`: the actual sentence-splitting workhorse. It handles each
  text file, moving the file to a different directory when finished
  with it.  Note that there are hard-coded paths in here, pointing to
  the sentence splitter.

* `pair-nosplit-one.sh`: similar to above, but assumes that the
  text-file contains one sentence per line - i.e. has been pre-split.

* `submit-one.pl`: script to send sentences to the cogserver.

* `split-sentences.pl`: split text into sentences. Accepts free-form text,
  and looks for likely end-of sentence locations, so that there is one
  sentence per line.


Sentence Splitting
------------------
Raw text needs to be split up into sentences.  Some distant future day,
opencog will do this automatically. For now, we hack it.

Currently, splitting is done with the `split-sentences.pl` perl script
in the this directory.  It was stolen from the `moses-smt` package.
https://github.com/moses-smt/mosesdecoder/tree/master/scripts/share/nonbreaking_prefixes
It splits French, Polish, Lithuanian, and more.  Its LGPL.

You can verify that it works, like so:
```
   cat text-file | ./split-sentences.pl -l en > x
```
Replace `en` by the language of your choice.

Some typical sentence-splitting concerns that the above script seems
to mostly handle correctly:

A question mark or exclamation mark always ends a sentence.  A period
followed by an upper-case letter generally ends a sentence, but there
are a number of exceptions.  For example, if the period is part of an
abbreviated title ("Mr.", "Gen.", ...), it does not end a sentence.
A period following a single capitalized letter is assumed to be a
person's initial, and is not considered the end of a sentence.
