Language Learning
=================
Linas Vepstas December 2013

Updated May 2018 by Claudia Castillo

Current project, under construction. See the [language learning wiki](http://wiki.opencog.org/w/Language_learning)
for an alternate overview.

Table of Contents
-----------------
1. [Summary](#summary)
2. [Setting up the AtomSpace](#setting-up-the-atomspace)
3. [Bulk Text Parsing](#bulk-text-parsing)
4. [Mutual Information of Word Pairs](#mutual-information-of-word-pairs)
5. [Minimum Spanning Tree Parsing](#minimum-spanning-tree-parsing)
6. [Exploring Connector Sets](#exploring-connector-sets)
7. [Setting up a Docker Container](#setting-up-a-docker-container)

Summary
-------
The goal of the project is to build a system that can learn parse
dictionaries for different languages, and possibly do some rudimentary
semantic extraction. The primary design point is that the learning is
to be done in an unsupervised fashion. A sketch of the theory that
enables this can be found in the paper "Language Learning", B. Goertzel
and L. Vepstas (2014) on [ArXiv abs/1401.3372](https://arxiv.org/abs/1401.3372).
A shorter sketch is given below. Most of this README concerns the
practical details of configuring and operating the system, and some
diary-like notes about system configuration and operation. A diary of
scientific notes and results is in the [`nlp/learn/learn-lang-diary`](https://github.com/opencog/opencog/tree/master/opencog/nlp/learn/learn-lang-diary) directory.

The basic algorithmic steps, as implemented so far, are as follows:

* **A)** Ingest a lot of raw text, such as novels and narrative literature,
   and count the occurrence of nearby word-pairs.

* **B)** Compute the mutual information (mutual entropy) between the word-pairs.

* **C)** Use a Minimum-Spanning-Tree algorithm to obtain provisional parses
   of sentences. This requires ingesting a lot of raw text, again.
   (Independently of step A)

* **D)** Extract linkage disjuncts from the parses, and count their frequency.

* **E)** Use maximum-entropy principles to merge similar linkage disjuncts.
   This will also result in sets of similar words. Presumably, the
   sets will roughly correspond to nouns, verbs, adjectives, and so-on.

Currently, the software implements steps A, B, C and D. Step E is
a topic of current research; its not entirely clear what the best
merging algorithm might be, or how it will work.
 
Steps A-C are "well-known" in the academic literature, with results
reported by many researchers over the last two decades. The results
from Steps D & E are new, and have never been published before.
(Results from Step D can be found in the `drafts/connector-sets.lyx` file,
inside the [`nlp/learn/learn-lang-diary`](https://github.com/opencog/opencog/tree/master/opencog/nlp/learn/learn-lang-diary) directory, the PDF of which was posted to the mailing lists)
 
All of the statistics gathering is done within the OpenCog AtomSpace,
where counts and other statistical quantities are associated with various
different hypergraphs. The contents of the atomspace are saved to an
SQL (Postgres) server for storage. The system is fed with raw text
using assorted ad-hoc scripts, which include link-grammar as a 
central components of the processing pipeline. Most of the
data analysis is performed with an assortment of scheme scripts.
 
Thus, operating the system requires three basic steps:
 * Setting up the atomspace with the SQL backing store,
 * Setting up the misc scripts to feed in raw text, and
 * Processing the data after it has been collected.
 
Each of these is described in greater detail in separate sections below.

 
Setting up the AtomSpace
------------------------
This section describes how to set up the atomspace to collect
statistics. Most of it revolves around setting up postgres, and for this
you have two options:

   1. You can choose to install everything directly on your machine,
   in which case you should just continue reading this section, or

  2. You can follow the instructions in the [Setting up a Docker Container](#setting-up-a-docker-container)
  section below to setup a docker container with all the environment
  needed to run the ULL ready for you to use.

If you choose the second option go to the section [Bulk Text Parsing](#bulk-text-parsing)
once you have your container working.

**Pre-installations:**
 
0.0) Optional. If you plan to run the pipeline on multiple
   different languages, it can be convenient, for various reasons,
   to run the processing in an LXC container. If you already know
   LXC, then do it. If not, or this is your first time, then don't
   bother.

0.1) Probably mandatory. Chances are good that you'll work with large
   datasets; in this case, you also need to do the below. Skipping this
   step will lead to the error `Too many heap sections: Increase
   MAXHINCR or MAX_HEAP_SECTS`. So:
```
   git clone https://github.com/ivmai/bdwgc
   cd bdwgc
   git checkout release-7_6
   ./autogen.sh
   ./configure --enable-large-config
   make; sudo make install
```

0.2) The atomspace MUST be built with guile version 2.2.2.1 or newer,
   which can only be obtained from git: that is, by
```
   git clone git://git.sv.gnu.org/guile.git
   git checkout stable-2.2
```
   Earlier versions have problems of various sorts. Version 2.0.11
   will quickly crash with the error message: 
   > guile: hashtab.c:137: vacuum_weak_hash_table: Assertion 'removed <= len' failed.
 
   Also par-for-each hangs:
   https://debbugs.gnu.org/cgi/bugreport.cgi?bug=26616
   (in guile-2.2, it doesn't hang, but still behaves very badly).
 
0.3) Opencog should be built with `link-grammar-5.4.3` or newer.
   You can check it by running `link-parser --version`
   If not, this version is available [here](https://www.abisource.com/projects/link-grammar/).

Now, let's set up the **text-ingestion pipeline:**

1) Set up and configure postgres, as described in
   [atomspace/opencog/persist/sql/README.md](https://github.com/opencog/atomspace/tree/master/opencog/persist/sql)

2) Test that your previous step was successful. Create and initialize
   a database. Pick any name you want; here it is `learn_pairs`.
```
   createdb learn_pairs
   cat $ATOMSPACE_SOURCE_DIR/opencog/persist/sql/multi-driver/atom.sql | psql learn_pairs
```

3) Create/edit the `~/.guile` file and add the following content:
```
   (use-modules (ice-9 readline))
   (activate-readline)
   (debug-enable 'backtrace)
   (read-enable 'positions)
   (add-to-load-path "/usr/local/share/opencog/scm")
   (add-to-load-path ".")
```

4) Start the REPL server. Eventually, you can use the
   `run-multiple-terminals.sh` script in the `run` directory to do this,
   which creates a `byobu` session with different processes in different
   terminals so you can keep an eye on them. However, the first 
   time through, it is better to do it by hand. So, for now in a terminal
   create your own working directory and copy all the files from the `run`
   directory into it. Now start the REPL server by writting:
   ```
     guile -l launch-cogserver  -- --mode pairs --lang en --db learn_pairs --user opencog_user --password cheese
   ```
   The *--user* option is needed only if the database owner is different from the
   current user.
   *--password* is also optional, not needed if no password was setup for the database

5) Verify that the language processing pipeline works. Try sending it input by running
   the following in a second terminal:
```
   rlwrap telnet localhost 17005
   opencog-en> (observe-text "this is a test" "any" 24)
```

   Or better yet (in a third terminal):
```
   echo -e "(observe-text \"this is a another test\" \"any\" 24)" |nc localhost 17005
   echo -e "(observe-text \"Bernstein () (1876\" \"any\" 24)" |nc localhost 17005
   echo -e "(observe-text \"Lietuvos žydų kilmės žurnalistas\" \"any\" 24)" |nc localhost 17005
```

   *Note:* 17005 is the default port for the REPL server in English. 
   This should result in activity on the cogserver and on the database:
   the "observe text" scheme code sends the text for parsing,
   counts the returned word-pairs, and stores them in the database.

6) Verify that the above resulted in data sent to the SQL database.
   For example, log into the database, and check:
```
   psql learn_pairs
   learn_pairs=# SELECT * FROM atoms;
   learn_pairs=# SELECT COUNT(*) FROM atoms;
   learn_pairs=# SELECT * FROM valuations;
```
   The above shows that the database now contains word-counts for
   pair-wise linkages for the input sentences. If the above worked without
   trouble you are ready to use the pipeline and continue to the next section,
   but if the above are empty, something is wrong: go back to step zero!

7) Finally, there are some parameters you can optionally adjust before starting
   to feed the pipeline with real input. Take some time to read and understand
   the scripts in the `run-ull` (current) and the `scm` directories, 
   in particular the use of the `params.txt` configuration file (more on it 
   will come in the next sections).
   
   For instance, you might want to tune the forced garbage collection parameters.
   Currently, garbage collection is forced whenever the guile heap exceeds 750 MBytes;
   this helps keep RAM usage down on small-RAM machines. However, it does cost CPU time.

   You can adjust the `count-reach` parameter passed to `observe-text` (e.g. in
   process-one.sh) to suit your whims in RAM usage and time spent in GC.


Bulk Text Parsing
-----------------

This section describes how to feed text into the pipeline. To do
that you first need to find some adequate text corpora that you can
feed into the pipeline. It would be best to get text that consists
of narrative literature, adventure and young-adult novels, newspaper
stories. These contain a good mix of common nouns and verbs, which
is needed for conversational natural language.
 
It turns out that Wikipedia is a poor choice for a dataset. That's
because the "encyclopedic style" means it contains few pronouns,
and few action-verbs (hit, jump, push, take, sing, love) because
its mostly describing objects and events (is, has, was). It also
contains large numbers of product names, model numbers, geographical
place names, and foreign language words, which do little or nothing
for learning grammar. Finally, it has large numbers of tables and
lists of dates, awards, ceremonies, locations, sports-league names,
battles, etc. that get mistaken for sentences, and leads to unusual
deductions of grammar.  Thus, Wikipedia is not a good choice for
learning text.
 
There are various scripts in the [nlp/learn/download](https://github.com/opencog/opencog/tree/master/opencog/nlp/learn/download)
directory for downloading and pre-processing texts from Project Gutenberg, Wikipedia, and the "Archive of Our Own"
fan-fiction website. Once you are sure you have the right material to start, follow the next steps:


1) Put all the training plain text files of the same language in one
   directory inside your working directory. The scripts used in this
   section use by default the name `beta-pages` for such a directory,
   so if you want to use a different name make sure you change the
   respective path inside the `text-process.sh` script. Also, keep in
   mind that the files will be removed from the folder after being
   processed, so make sure you keep a back-up of them somewhere else
   (you don't want to mess up the original files after all the work
   done to get them).

   If you used the provided example scripts, you should have a test
   file in the `alpha-pages` folder. Make a copy of this folder with
   the desired name.
   ```
      cp -pr alpha-pages beta-pages
   ```

2) Set up distinct databases, one for each language you will work with:
   ```
      createdb fr_pairs lt_pairs pl_pairs en_pairs
      cat $ATOMSPACE_SOURCE_DIR/opencog/persist/sql/multi-driver/atom.sql | psql ??_pairs
   ```

3) Copy all files from the `opencog/opencog/nlp/learn/run-ull`
   directory into your working directory (if you don't have them 
   already).

   Review the file [opencog/nlp/learn/run-ull/README-run.md](https://github.com/opencog/opencog/tree/master/opencog/nlp/learn/run-ull)
   if you want to have a general understanding of what each of these scripts/files do.

4) If you are familiar with the counting and parsing methods used in the

   pipeline, open `config/params.txt` and choose the counting mode you want to use.
   Otherwise, just leave the default values. 
   There are currently three observing modes, set by cnt_mode and taking
   another integer parameter:
    - any: counts pairs of words linked by the LG parser in 'any' language.
           In this case, 'count-reach' specifies how many linkages from LG-parser
           to use. If cnt_reach is set to 0, then it uses all returned parses.
    - clique: itearates over each word in the sentence and pairs it with
           every word located within distance 'count-reach' to its right.
           Distance is defined as the difference between words positions
           in the sentence, so neighboring words have distance of 1.
    - clique-dist: same word-pairs as 'clique', but the count in each word
           pair is incremented by 'cnt_reach / distance'

5) In your working directory run the following:
   ```
      ./run-multiple-terminals.sh pairs lang ??_pairs your_user your_password
   ```
   This starts the cogserver and sets a default prompt: set up by
   default to avoid conflicts and confusion, and to allow multiple 
   languages to be processed at the same time. 

   Replace the arguements above with the ones that apply to the language
   you are using and your database credentials. User and password are
   optional, as previously explained. For example, for english run:
   ```
      ./run-multiple-terminals.sh pairs en en_pairs opencog_user cheese
   ```
6) In the parse tab of the byobu (you can navigate with the F3 and F4 keys),
   run the following:
   ```
      ./text-process.sh pairs en
   ```
   Or can change 'en' for the respective language initials. 
   If this command shows the error
   > nc: invalid option -- 'N'
   
   open `process-one.sh` and remove the -N option from the nc commands
   (some old version of netcat still support this option).

7) Wait some time, possibly a few days. When finished, stop the cogserver.
 
8) Verify that the information was correctly saved in the database. 

Some handy SQL commands:
 ```
    SELECT count(uuid) FROM atoms;
    SELECT count(uuid) FROM atoms WHERE type =123;
 ```
 
 type 123 is `WordNode` for me; verify with
 ```
    SELECT * FROM Typecodes;
 ```
 
 The total count accumulated is
 ```
    SELECT sum(floatvalue[3]) FROM valuations WHERE type=7;
 ```
 where type 7 is `CountTruthValue`.

Some extra notes:

The `submit-one.pl` script here is called with the "observe-text"
instruction for word-pair counting when send to the cogserver.
Obtaining word-pair counts requires digesting a lot of text, and
counting the word-pairs that occur in the text. There are several
ways of doing this at the moment. The original way was to parse the
text with link-grammar using the "any" language.  This pseudo-language
will link any word to any other as long as the links do not cross each
other, thus extracting word pairs from the parsed text. An alternative
method (although not necessarily more efficient) is to find all possible
word-pair combinations (a clique) that lie within a specified distance.

You may want to reduce the amount of data that is collected. Currently,
the `observe-text` function in `scm/link-pipeline.scm` can collect counts
on four different kinds of structures:
 
   * Word counts -- how often a word is seen.
   * Clique pairs, and pair-lengths -- This counts pairs using the "clique
                   pair" counting method.  The max length between words
                   must be specified. Optionally, the lengths of the pairs
                   can be recorded. Caution: enabling length recording
                   will result in 6x or 20x more data to be collected,
                   if you've set the length to 6 or 20.  That's because
                   any given word pair will be observed at almost any
                   length apart, each of these is an atom with a count.
                   Watch out!
   * Lg "ANY" word pairs -- how often a word-pair is observed.
   * Disjunct counts -- how often the random ANY disjuncts are used.
                 You almost surely do not need this.  This is for my
                 own personal curiosity.

Edit the comments on that file if you want to vary the collected data.

This pipeline requires postgres 9.3 or newer, for multiple reasons.
One reason is that older postgres don't automatically VACUUM. The other is
that the list membership functions are needed.

Also, be sure to perform the postgres tuning recommendations found in
various online postgres performance wikis, or in the
[atomspace/opencog/persist/sql/README.md](https://github.com/opencog/atomspace/tree/master/opencog/persist/sql) file. See 'Performance' section below.


Mutual Information of Word Pairs
--------------------------------

After accumulating a few million word pairs, we're ready to compute the
mutual entropy between them. Follow the next steps to do so. Note that
if the parsing is interrupted, you can restart the various scripts; they
will automatically pick up where they left off.

1) Copy all files from the [opencog/opencog/nlp/learn/run-ull](https://github.com/opencog/opencog/tree/master/opencog/nlp/ull-parser/run)
   directory into your working directory (if you don't have them already).

2) In your working directory run the following:
   ```
      ./run-multiple-terminals.sh cmi lang ??_pairs your_user your_password
   ```
   This starts the cogserver and sets a default prompt: set up by
   default to avoid conflicts and confusion, and to allow multiple 
   languages to be processed at the same time. 

   Replace the arguements above with the ones that apply to the language
   you are using and your database credentials. User and password are
   optional, as previously explained. For example, for english run:
   ```
      ./run-multiple-terminals.sh cmi en en_pairs opencog_user cheese
   ```
3) In one of the unused byobu windows (you can navigate with the F3 and F4 keys),
   run the following:
   ```
      ./process-word-pairs.sh cmi en
   ```
   Or can change 'en' for the respective language initials. 
   If this command shows the error
   > nc: invalid option -- 'N'
   
   open `process-word-pairs.sh` and remove the -N option from the nc commands
   (some old version of netcat still support this option).

4) Wait some time, possibly a few days. When finished, stop the cogserver.
   
   These scripts use commands from the scripts in the `scm` directory.
   The code for computing word-pair MI is in `batch-word-pair.scm`.
   It uses the `(opencog matrix)` subsystem to perform the core work.


General remakrs:

* The system might not be robust enough at this stage yet, so if you
  find an error while executing this code, run each command from the
  function in compute-mi.scm directly on the cogserver and separately
  to trace the error.

* Batch-counting might take hours or longer, depending on your dataset
  size. The batching routine will print to stdout, giving a hint of
  the rate of progress.

Example stats and performance:

* current fr_pairs db has 16785 words and 177960 pairs.
  This takes 17K + 2x 178K = 370K total atoms loaded.
  These load up in 10-20 seconds-ish or so.

* New fr_pairs has 225K words, 5M pairs (10.3M atoms):
  Load 10.3M atoms, which takes about 10 minutes cpu time to load
  20-30 minutes wall-clock time (500K atoms per minute, 9K/second
  on an overloaded server).

* RSS for cogserver: 436MB, holding approx 370K atoms
  So this is about 1.2KB per atom, all included. Atoms are a bit fat...
  ... loading all pairs is very manageable even for modest-sized machines.

* RSS for cogserver: 10GB, holding 10.3M atoms
  So this is just under 1KB per atom.

 (By comparison, direct measurement of atom size i.e. class Atom:
 typical atom size: 4820384 / 35444 = 136 Bytes/atom
 this is NOT counting indexes, etc.)

* For dataset (fr_pairs) with 225K words, 5M pairs:
  Current rate is 150 words/sec or 9K words/min.

After the single-word counts complete, and all-pair count is done.
This is fast, takes a couple of minutes.

* Next: batch-logli takes 540 seconds for 225K words

* Finally, an MI compute stage.
  Current rate is 60 words/sec = 3.6K per minute.
  This rate is per-word, not per word-pair .

 Update Feb 2014: fr_pairs now contains 10.3M atoms
 SELECT count(uuid) FROM Atoms;  gives  10324863 (10.3M atoms)
 select count(uuid) from atoms where type = 77; gives  226030 (226K words)
 select count(uuid) from atoms where type = 8;  gives 5050835 (5M pairs ListLink)
 select count(uuid) from atoms where type = 27; gives 5050847 (5M pairs EvaluationLink)


Minimum Spanning Tree Parsing
-----------------------------

The MST parser discovers the minimum spanning tree that connects the
words together in a sentence.  The link-cost used is (minus) the mutual
information between word-pairs (so we are maximizing MI). Thus, MST
parsing cannot be started before the above steps to compute word-pair
MI have been accomplished.

The minimum spanning tree code is in `scm/mst-parser.scm`. The current
version works well. To run it follow the next steps:

1) Copy all files from the [opencog/opencog/nlp/learn/run-ull](https://github.com/opencog/opencog/tree/master/opencog/nlp/ull-parser/run)
   directory into your working directory (if you don't have them already):

   Review the file [opencog/nlp/learn/run-ull/README-run.md](https://github.com/opencog/opencog/tree/master/opencog/nlp/ull-parser/run)
   if you want to have a general understanding of what each of these scripts/files do.

2) Copy again all your text files, now to the `gamma-pages` directory
   (or edit `text-process.sh` and change the corresponding directory
   name). Once again, keep in mind that during processing, text files
   are removed from this directory.

3) (Optional but suggested) Make a copy of your word-pair database, 
   "just in case".  You can copy databases by saying:
   ```
      createdb -T existing_dbname backup_dbname
   ```

4) Tweak your parser parameters accordingly:

   * Open `config/params.txt` and make sure the cnt_mode matches the one used
   for pair-counting. Also, if you want to assign distance weight to the 
   word-pairs' MI values (adding a factor 1 / distance), assign mst_dist="#t". 
   Set the export
   parses variable to true (exp_parses="#t") if you want to export the
   sentence parses for each corpus file into the directory mst-parses. 
   The parameter cnt_reach does not
   have an effect at this stage, you can leave it as is.

5) In your working directory run the following:
   ```
      ./run-multiple-terminals.sh mst lang dbname your_user your_password
   ```
   Replace the arguements above with the ones that apply to the language
   you are using and your database credentials. User and password are
   optional, as previously explained. For example, for English run:
   ```
      ./run-multiple-terminals.sh mst en en_pairs opencog_user cheese
   ```

6) In an unused tab of the byobu (you can navigate with the F3 and F4 keys),
   run the following:
   ```
      ./process-word-pairs.sh mst en
   ```
   Or can change 'en' for the respective language initials. 
   Once again, if this command shows the error
   > nc: invalid option -- 'N'
   
   open `process-word-pairs.sh` and remove the -N option from the nc commands.

   Wait 10 to 60+ minutes for the guile prompt to appear. This script
   opens a connection to the database, and then loads all word-pairs
   into the atomspace. This can take a long time, depending on the
   size of the database. The word-pairs are needed to get the pair-costs
   that are used to perform the MST parse.

7) Once the above has finished loading, the parse script can be started.
   In the parse tab of the byobu run:
   ```
      ./text-process.sh mst en
   ```
   Remember to change 'en' to the respective language if it applies. Wait
   a few days for data to accumulate. Once again, if the above command
   shows the error
   > nc: invalid option -- 'N'
   
   open `process-one.sh` and remove the -N option from the nc commands.
   
   If the process is stopped for any reason, you can just re-run these
   scripts; they will pick up where they left off. When finished,
   remember to stop the cogserver.

Once this is done, you can move to the next step, which is explained in
the next section. If you activated the option, you can check out the
sentence parses in `mst-parses.txt`.


Exploring Connector-Sets
-------------------------
Once you have a database with some fair number of connector sets in it,
you can start exploring. For ideas checkout the original version of this
README in [opencog/nlp/learn](https://github.com/opencog/opencog/tree/master/opencog/nlp/learn).

I add here some links to other usefull resources for understanding:
* Structure of the atomstpace: [atoms](https://wiki.opencog.org/w/Atom) and its [types](https://wiki.opencog.org/w/Atom_types)
* The basic operations on atoms: [README](https://github.com/opencog/atomspace/tree/master/opencog/guile).
* Atom structure used for NLP: [sentence-representation-wiki](https://wiki.opencog.org/w/Sentence_representation).
* 

After this, clusterization and feedback steps should be performed,
but for now you are on your own.. Good luck!!


Setting up a Docker Container
-----------------------------

Before you follow the next steps make sure you have cloned the repositories from OpenCog (opencog, atomspace, cogutil) in your machine.

1) Download and set up docker from [here](https://docs.docker.com/install/).
   If you are using Linux, also install docker-compose from [here](https://docs.docker.com/compose/install/),
   or by:
   ```
   ~$ sudo pip install -U docker-compose
   ```

2) Clone the docker repository:
   ```
   ~$ git clone https://github.com/opencog/docker.git
   ```

3) Enter the opencog directory and build your images:
   ```
   ~$ cd docker/opencog/
   ~/docker/opencog$ ./docker-build.sh -a
   ```
   Make sure your user is in the docker group (`getent group docker`), otherwise you will get the error 
   > Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: ...

4) Create a directory in your machine to store code that will make building
   again the containers faster (change path to your own, ex: $HOME/.ccache):
   ```
   ~$ mkdir -p $HOME/path/to/where/you/want/to/save/ccache/output
   ```

   Optionally you can instead just comment out the `$CCACHE_DIR:/home/opencog/.ccache`
   line in *common.yml* file

5) Add these lines to *~/.bashrc* at *$HOME* of your host OS (change paths to your
   own) and run `source ~/.bashrc`.
   ```
   export OPENCOG_SOURCE_DIR=$HOME/path/to/opencog
   export ATOMSPACE_SOURCE_DIR=$HOME/path/to/atomspace
   export COGUTIL_SOURCE_DIR=$HOME/path/to/cogutil
   export CCACHE_DIR=$HOME/path/to/where/you/want/to/save/ccache/output`
   ```
   If you are trying to install the container in a shared server insert these lines in the file *~/.profile* instead, and restart your session before continuing.
   
6) Run the container:
   ```
   ~$ cd docker/opencog/
   ~/docker/opencog$ docker-compose run dev
   ```
   
7) Create a working directory inside your container and copy all the files from the `run` folder
   ```
   ~$ cp -pr /opencog/opencog/nlp/learn/run-ull $HOME/my_working_dir
   ```

8) Test that everything is working:

   a) Create and format a database (password is cheese):
   ```
    $ createdb learn_pairs
    $ cat /atomspace/opencog/persist/sql/multi-driver/atom.sql | psql learn_pairs
   ```
   Use tmux to create parallel sessions of the container. If you are not familiar with
   it you can use this cheatsheet in [here](https://gist.github.com/MohamedAlaa/2961058).

   b) In a separate session start the REPL server. Make sure you are inside
      your working directory (`my_working_dir`):
   ```
    $ guile -l launch-cogserver.scm  -- --mode pairs --lang en --db learn_pairs --user opencog_user --password cheese
   ```

   c) Send input to the pipeline:
   ```
    $ echo -e "(observe-text-mode \"This is a test\" \"any\" 24)" |nc localhost 17005
   ```

   d) Check that the input was registered in the database:
   ```
    $ psql learn_pairs
    learn_pairs=# SELECT * FROM atoms;
    ```
    The words from the sentence "*This is a test*" should appear inside the table below the *name* column.
    
IF EVERYTHING WORKED FINE YOU ARE READY TO WORK (go to [Bulk Text Parsing](#bulk-text-parsing)),
OTHERWISE GO BACK TO STEP 0 (or fix your bug if you happen to know what went wrong)!!

***Note 1***: If something went wrong when trying to connect to the cogserver consider making a clean build
and re-installing inside the container all three: cogutil, atomspace, opencog. For example for the first one:
  ```
   ~$ cd /cogutil/build
   /cogutil/build$ rm -rf *
   /cogutil/build$ cmake ..
   /cogutil/build$ make && sudo make install
   ```

***Note 2***: If you make changes to the code in your installed repos, you can update those in your
current container by `cd`-ing to the mount directory inside the container and running `/tmp/octool -bi`

***Note 3***: Steps 1-5 are only necessary the first time you install the docker container and images.
Afterwards, you just need to follow steps 6 and 7 every time you want to create a new opencog container,
or you might want to access directly your already existing container (see next note).

***Note 4***: Keep in mind that everytime you run `docker-compose run dev` it will create a new instance of *opencog*
but the same instances of *postgres* and *relex* will be running on the background. Use (Ctrl+D) to exit a container.
Some usefull commands for managing your containers on your local machine are listed below:
  - `docker ps`                         To see the list of all the active containers (it shows *container_ID*).
  - `docker ps -a`                      To see the list of all the existing containers.
  - `docker start container_ID`         To start an inactive existing container (for example an existing instance of *opencog*).
  - `docker attach container_ID`        To "log-in" to a running (existing & active) container in a terminal.
  - `docker stop container_ID`          To stop a running container.
  - `docker stop $(docker ps -q)`       To stop all running containers.
  - `docker kill container_ID`          To kill a running container (forces the stop).
  - `docker rm container_ID`            To delete an existing but inactive container.
  - `docker rm -f container_ID`         To delete a running container (it will kill it first).
  - `docker cp container_ID:/path file` To copy a file from host to the container.
  
**DO NOT** try to delete all running containers unless strictly necessary because it will delete the *postgres* instance as well, which means losing all your databases!!!

***Note 5***: Remember to always close any cogserver (Ctrl+D) sessions you have started before continuing,
otherwise you will have problems accessing your databases later.
