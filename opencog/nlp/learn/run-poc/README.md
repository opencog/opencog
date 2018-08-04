
Proof-of-concept-style run  scripts
===================================
These work differently than the other `run` scripts. Not sure why.


```
  guile -l observe-launch.scm  -- --lang en --db learn-pairs --user your_user --password asdf
```
   The --user option is needed only if the database owner is different
   from the current user.
   --password is also optional, not needed if no password was setup for
   the database


* `run-server-parse.sh` starts the cogserver and sets a default prompt:
   set
   up by default to avoid conflicts and confusion, and to allow multiple
   languages to be processed at the same time.
   You need to pass the language and database credentials as arguments;
   see next step.

* `wiki-ss-en.sh` starts the process of observing the text files
   contained
   in the beta-pages folder, more on that later (see Bulk Text Parsing).
   The rest of the relevant files are explained later.

Here is a brief description of the relevant files for each of the parts
of the language learning pipeline. They don't need modification by default,
unless you want to do something unusual.

Connecting to Cogserver:
------------------------

 -- `utilities.scm` the functions in this files are used to specify the
                    command-line arguments for getting processing mode,
                    language and database details.

 -- `det-db-uri.sh` script used to determine the database name, postgres user
                    and password according to the language given as input
                      (inside config folder).

 -- `det-port-num.sh` script used to determine the port number for the cogserver,
                      takes as input the processing mode and the language
                      (inside config folder).

 -- `opencog-pairs-??.conf` contains configuration settings for the cogserver, 
                      including the language-specific port to use (inside
                      config folder).

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
                                the processes. One terminal calls `launch-cogserver.scm`
                                with guile to start the cogserver, and another one telnets 
                                into it using a language-specific port. You need to pass
                                it the parsing mode (pair, cmi or mst) and the language
                                of the texts to process. Optionally you can pass the database
                                credentials as arguments. You can use spare terminals to 
                                process several corpora at the same time. Use F3 and F4 to 
                                switch between the different terminals.

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

 -- `compute-mi.scm` this script is used to compute the mutal entropy between the word pairs
                     registerd in the database. It calls functions defined in `(opencog nlp learn)`.
                     The actual code for computing word-pair MI is in `batch-word-pair.scm`.
                     It uses the `(opencog matrix)` subsystem to perform the core work.

 -- `export-mi.scm` this script is used to export the word-pairs FMI for every possible pair
                    in the corpus to allow analysis of the MI-based MST-parser. For research
                    purposes only, it is not used in the pipeline.
