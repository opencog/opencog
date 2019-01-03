Here is a brief description of the relevant files for each of the parts
of the language learning pipeline. They don't need modification by default,
unless you want to do something unusual.

Connecting to Cogserver:
------------------------

 -- `utilities.scm` the functions in this files are used to specify the
                    command-line arguments for getting language and
                    database details.

 -- `det-port-num.sh` script used to determine the port number for the cogserver,
                      takes as language the processing mode and the language
                      (inside config folder).

 -- `opencog-pairs-??.conf` contains configuration settings for the cogserver,
                      including the language-specific port to use (inside
                      config folder).

 -- `opencog-cmi-??.conf` same as above but with particular configurations
                          for MI computing (inside config folder).

 -- `opencog-mst-??.conf` same as above but with particular configurations
                          for MST parsing (inside config folder).

 -- `launch-cogserver.scm` starts the cogserver and opens the database. You
                            need to pass it the processing mode, the language
                            and the database credentials.



Text Processing / Parsing:
--------------------------

 -- `params.txt`  contains the parameters for the counting and parsing methods to be
                  used (inside config folder).

 -- `run-multiple-terminals.sh` the top-level shell script; it opens multiple terminal
                                sessions with tmux/byobu where you can keep an eye on
                                the processes. One terminal one of the `launch-??.scm`
                                scripts to start the cogserver, and another one telnets
                                into it using a language-specific port. You need to pass
                                it the parsing mode (pair counting or mst), the language
                                of the text to process and the database credentials as
                                arguments. You can use spare terminals to process several
                                languages at the same time. Use F3 and F4 to switch between
                                the different terminals.

 -- `text-process.sh` the top-level parser script; it performs batch processing of
                      input text files in the specified mode and language you pass as
                      arguments. It pulls text files, one by one, from the `beta-pages`
                      or `gamma-pages` directory accordingly, and submits them to
                      `process-one.sh` for later parsing and/or counting. This script
                      should be manually launched in the 'parse' byobu window. Be
                      sure to open the database first.

 -- `process-one.sh` the actual sentence-splitting workhorse; it handles each text file,
                     moving the file to a different directory when finished with it. It
                     calls `split-sentences.pl` to split the text file into sentences,
                     and then calls `submit-one.pl` to send the sentences to the cogserver
                     for parsing. None of these should need any modification. Note that
                     there are hard-coded paths in here, pointing to the sentence splitter.

 -- `split-sentences.pl` splits text into sentences. Accepts free-form text, and looks
                         for likely end-of sentence locations, so that there is one
                         sentence per line (see Sentence Splitting section).

 -- `submit-one.pl` script to actually send a sentence to the REPL server. It calls one of
                    the "observe-??" options configured for processing in the server.



Mutual Information of Word Pairs:
---------------------------------

 -- `compute-mi.scm` this script defines the function used to compute the mutal entropy between
                     the word pairs registerd in the database. The function takes as input
                     the counting mode and it calls functions defined in `(opencog nlp ullparser)`.
                     The actual code for computing word-pair MI is in `batch-word-pair.scm`.
                     It uses the `(opencog matrix)` subsystem to perform the core work.

 -- `export-mi.scm` this script is used to export the word-pairs FMI for every possible pair
                    in the corpus to allow analysis of the MI-based MST-parser. For research
                    purposes only, it is not used in the pipeline.

 -- `process-word-pairs.sh` this script is used to pass the intructions to the cogserver for
                            computing the MI between word pairs or for fetching them if the
                            last has already been done. You have to pass it the processing
                            mode and the language.

-- `fetch-word-pairs.scm` this scripts defines the function used to loads the word-pairs and
                          its counts. The function takes as input the counting mode.

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

