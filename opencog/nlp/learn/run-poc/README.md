
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

