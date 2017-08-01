
Parse management scripts
========================

The scripts here are used to automate the ingestion of plain-text
UTF-8 files into the language learning pipleine.  These can be applied
to any flat text files from any origin of your choice.  Some tools
for downloading Wikipedia and Project Gutenberg texts can be found
in the `../download` directory.

You may want to tailor some of these scripts to fit your needs.

A quick overview:

* `run-all-servers.sh`: the top-level shell script; opens multiple
  terminal sessions with tmux/byobu, and starts both opencog and
  the relex servers. Use F3 and F4 to switch to different terminals.

* `wiki-ss-*.sh`: the top-level parser script. It pulls text files, one
  by one, from the data directory, and submits them for parsing and
  counting.  This script should be manually lanuched in the 'parse'
  byobu window.  Be sure to open the database, first. The data directory
  needs to be manually adjusted here, and also in the ss-one.sh script.

* `ss-one.sh`: the actual sentence-splitting workhorse. It handles each
  text file, moving the file to a different directory when finished
  with it.  Note that there are hard-coded paths in here, pointing to
  the sentence splitter.

* `ss-nosplit-one.sh`: similar to above, but assumes that the text-file
  contains one sentence per line - i.e. has been pre-split.

* `submit-one.pl`: script to actually send sentences to the REPL server.

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
