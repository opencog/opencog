
Unsupervised Language Learning
==============================
* Linas Vepstas December 2013
* Updated May 2017
* Updated June 2018

Current project, under construction.  See the
[language learning wiki](http://wiki.opencog.org/w/Language_learning)
for an alternate overview.

Project Summary
---------------
The goal of this project is to create a system that is capable of
learning the grammar and some of the semantics of natural language.
The fundamental goal is to do this in an unsupervised fashion, with
no training data beyond that of un-annotated raw text.

An early draft of how this can be done is presented in the paper
"Language Learning", B. Goertzel and L. Vepstas (2014) on ArXiv; see
[ArXiv abs/1401.3372](https://arxiv.org/abs/1401.3372). Some of this
is repeated on the
[language learning wiki](http://wiki.opencog.org/w/Language_learning).
A very important update, describing how to move past state-of-the-art
results, is presented in the
"[Sheaf Theory](https://github.com/opencog/atomspace/blob/master/opencog/sheaf/docs/sheaves.pdf)"
paper. This important step is simply summarized in a later section,
below.

The key process is a sequence of steps that can be used to extract the
structure of the "language graph", with each step being noisey and
error-prone, but the subsequent step averaging over this structure in
such a way as to cancel noise, and allow the actual graphical structure
to emerge. Central to this process is extracting the actual structure of
the graph, as opposed to the extraction of vector spaces, as is currently
done with neural-net techniques or other styles of clustering.

The core insight is that although vector spaces can be layered onto
language data (thus explaining the success of modern-day neural net
techniques), ther is also a way of "stitching together" these vector
spaces in such a way that they extract the underlying graphical
structure.  The appearance of vector spaces is not an accident: the
axioms of algebraic linguistics (such as those found in categorial
grammar, pregroup grammar, link grammar) are similar to the axioms that
define a vector space. The similarity accounts for why techniques such
as Word2Vec are successful. The dis-similarity is why these same
tecniques are incomplete.  The road-block can be overcome by viewing
the language graph (and graphs in general) as being constructed out
of connected parts, with explicitly-named connectors. This
connector-based approach to the definition of a graph just so happens
to obey the axioms of a sheaf (as commonly understood in sheaf theory).
This connector-based approach also indicates how vector spaces can be
assembled, connected together to form  the graph from which they are
being sampled.  The code here is an exploration and implementation of
this insight.

An very early and yet pretty presentation of algebraic linguistics,
as seen axiomatically, is given by Solomon Marcus,
“[Algebraic Linguistics; Analytical Models](https://monoskop.org/images/2/26/Marcus_Solomon_editor_Algebraic_Linguistics_Analytical_Models_1967.pdf)”,
(1967), Elsevier.

A feels-right explanation of how one extracts semantics from disjuncts is
given by EA Nida,
[“The Molecular Level of Lexical Semantics”](https://www.academia.edu/36534355/The_Molecular_Level_of_Lexical_Semantics_by_EA_Nida),
(1997) *International Journal of Lexicography*, **10**(4): 265–274.
What Nida is saying can, in fact, be measured, by correlating, for
example, disjuncts with WordNet word-senses. The correlation is real and
is measureable (and has been measured). The goal of this project is to
move beyond this.


Processing Overview
-------------------
Most of this README concerns the practical details of configuring and
operating the system, as it stands, today.  A diary of scientific notes
and results is in the [learn-lang-diary](learn-lang-diary) subdirectory.

The basic algorithmic steps, as implemented so far, are as follows:

A) Ingest a lot of raw text, such as novels and narrative literature,
   and count the occurrence of nearby word-pairs.

B) Compute the mutual information (mutual entropy) between the word-pairs.

C) Use a Minimum-Spanning-Tree algorithm to obtain provisional parses
   of sentences.  This requires ingesting a lot of raw text, again.
   (Independently of step A)

D) Extract linkage disjuncts from the parses, and count their frequency.

E) Use agglomerative clustering to merge similar linkage disjuncts,
   while simultaneously merging similar words.  This will result in
   a set of word-classes (grammatical classes) with disjuncts on them
   that use word-class connectors.

F) Place word-classes and word-class connectors into a link-grammar
   dictionary.

G) Parse a large quantity of raw text, using the parser constructed in
   the previous step. Using maximum-entropy-style techniques, attempt
   to extract higher-order relations (resolve anaphora and other
   referents, determine rheme+theme, learn "lexical functions")

Currently, the software implements steps A, B, C, D, F and much of
step E, although this step remains a topic of current research.
Its quite unclear how to determine the quality of lexis that falls out
of step F.  Some initial experiments on step G have been performed,
and look promising.

Steps A-C are "well-known" in the academic literature, with results
reported by many researchers over the last two decades. The results
from Steps D & E are new, and have never been published before.
Results from Step D can be found in the PDF file
[Connector Sets](learn-lang-diary/drafts/connector-sets.pdf)
in the diary subdirectory.

It is very important to understand that Step E is ***very different***
from commonly-reported algorithms used in the published literature,
such as using K-means clustering and dimensional reduction to obtain
word classes (grammatical categories). The reason for this is that the
disjuncts provide connectivity information, and that one must **not**
(merely) cluster over a vector space, but that one must instead cluster
over a "sheaf".  That is, the basis-elements of the vector space
themselves have non-trivial structure (which must not be ignored).
Sheaves resemble vector spaces, but are not the same.  They capture
and describe the connectivity information in the "basis elements".
This connectivity information is absent from ordinary approaches to
the machine-learning of grammatical classes.

As a result, ***none*** of the industry-standard classifiers and
clustering algorithms can be applied to step E: they all assume that
the input data can be structured as a vector space.  The axioms of
algebraic linguistics can resemble the axioms of a vector space, which
is why vector-space techniques can work pretty well in the machine-
learning of linguistic structure. However, ultimately, the axioms of
algebraic linguistics are ***not*** the axioms of a vector space.
In practical terms, this means that clustering/classification algorithm
used in step E is completely different than anything else available in
the industry.

The complete novelty of step E, coupled to the need to perform
experiments in earlier and later stages means that industry-standard,
generic big-data, machine-learning tools are wholly inadequate for the
task. It is for thus reason that all data processing is being done in
the AtomSpace, and not in some other machine learning or big-data
framework. Existing industry tools are not adequate for proper
linguistic analysis. This is a very important reason for developing
the AtomSpace, and other OpenCog infrastructure: to get past tool
limitations.

All of the statistics gathering is done within the OpenCog AtomSpace,
where counts and other statistical quantities are associated with various
different hypergraphs.  The contents of the AtomSpace are saved to an
SQL (Postgres) server for storage.  The system is fed with raw text
using assorted ad-hoc scripts, which include link-grammar as a central
component of the processing pipeline. Most of the data analysis is
performed with an assortment of scheme scripts.

Thus, operating the system requires three basic steps:
* Setting up the AtomSpace with the SQL backing store,
* Setting up the misc scripts to feed in raw text, and
* Processing the data after it has been collected.

Each of these is described in greater detail in separate sections below.

If you are lazy, you can just hop to the end, to the section titled
"Precomputed LXC containers", and grab one of those.  It will have
everything in it, up to the last step.  Of course, you won't know what
to do with it, if you don't read the below. Its not a magic "get smart"
pill. You've got a lot of RTFM in front of you.


Computational Pre-requisites
----------------------------
This section describes minimum mandatory hardware and software
requirements. Failing to take these seriously will result in an
unhappy, painful experience.

0.1) Optional, but strongly recommended! Consider investing in an
   uninterruptible power supply (UPS) to avoid data loss/data
   corruption in the event of a power outage. The scripts below can
   take weeks to run, and unexpected power outages can corrupt
   databases, resulting in weeks or months of lost work.

   *You have been warned! This is not a joke!*

0.2) Optional, but strongly recommended!  Consider investing in a
   pair of solid-state disk drives (SSD), configured as a RAID-1
   (mirroring) array, to hold the Postgres database.  Accumulating
   observation statistics, as well as other data processing operations,
   is database intensive, and the writes to disk are often the main
   bottleneck to performance. In practice, 1TB (one terabyte) devoted
   to just the Postgres server is just barely enough to perform the
   work described here. Database copies quickly chew up disk space.
   Individual databases are typically in the 10GB to 150GB in size,
   and the number of copies of a 150GB database that fit onto a
   1TB disk is not big.  2TB is a more comfortable size to work with.

0.3) Get a machine with 4 or more CPU cores, and a minimum of 64GB RAM.
   It will be less agonizing and infuriating if you have 128 GB RAM or
   more.  The datasets do get large, and although many of the scripts
   are designed to only fetch from disk "on demand", the in-RAM sizes
   can get large. As I write this, my grammatical-clustering process
   is using 53GB resident-in-RAM working-set size (57GB virtual size).
   You need additional RAM for the Postgres server -- 24GB is
   reasonable -- and so 53+24=78GB is the current bare-minimum.  More,
   if you want to e.g. run a web-browser or something.

   An alternative is to run Postgres on one box, and the processing
   on another. This is a bit more complicated to deal with -- you will
   want to have a high-speed Ethernet between the two.  Two machines
   are harder to monitor than one.

0.4) Optional but recommended. If you plan to run the pipeline on
   multiple different languages, it can be convenient, for various
   reasons, to run the processing in an LXC container. If you already
   know LXC, then do it. If not, or this is your first time, then don't
   bother right now. Come back to this later.

   LXC containers are nice because:
   * Each container can have a different config, different datasets,
     and be executing different steps of the pipeline.  This is great,
     for juggling multiple jobs.
   * The system install in one container won't corrupt the other
     containers; you can experiment, without impacting stable systems.
   * LXC containers can be stopped and move to other system with more
     RAM (or less RAM), more disk, or less disk. Handy for load-
     balancing, and/or easily moving to a bigger system.

0.5) Optional but strongly recommended. Install system shutdown scripts.
   This will help protect your data in case of an unexpected power loss.
   These scripts, and the instructions for them, are located in the
   `run/rc.local.shutdown`, `run/rc-local-shutdown.service` and the
   `run/rc.lxc.shutdown` files. You will need to alter these files, and
   use your own login credentials in place of the generic ones.

0.6) Mandatory.  You'll work with large datasets; default guile is
   not equipped to deal wit the sizes encountered here.  Skipping this
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

0.7) The AtomSpace MUST be built with guile version 2.2.2.1 or newer,
   which can be obtained from the Guile ftp repo
   https://ftp.gnu.org/gnu/guile/ or git, by doing
```
   git clone git://git.sv.gnu.org/guile.git
   git checkout stable-2.2
```
   Earlier versions have problems of various sorts. Version 2.0.11
   will quickly crash with the error message: `guile: hashtab.c:137:
   vacuum_weak_hash_table: Assertion 'removed <= len' failed.`

   Also par-for-each hangs:
   https://debbugs.gnu.org/cgi/bugreport.cgi?bug=26616
   (in guile-2.2, it doesn't hang, but still behaves very badly)

0.8) Create/edit the `~/.guile` file and add the content below.
   This makes the arrow keys work, and prints nicer stack traces.
```
   (use-modules (ice-9 readline))
   (activate-readline)
   (debug-enable 'backtrace)
   (read-enable 'positions)
   (add-to-load-path "/usr/local/share/opencog/scm")
   (add-to-load-path ".")
```

0.9) Opencog should be built with `link-grammar-5.5.0` or newer.
   The currently installed version can be displayed by running
```
   link-parser --version
```
   Newer versions are available at:
   https://www.abisource.com/downloads/link-grammar/

   The main link-grammar project page is here:
   https://www.abisource.com/projects/link-grammar/


Setting up the AtomSpace
------------------------
This section describes how to set up the AtomSpace to collect
statistics.  The most time-consuming, difficult and error-prone step
is the setup and configuration of Postgres.  Postgres is centrally
important for saving partial results.

1) Set up and configure Postgres, as described in
   `atomspace/opencog/persist/sql/README.md`

2) Create and initialize a database. Pick any name you want; here it
   is `learn-pairs`.  Later on, you will have to place this name into
   a config file (see below).
```
   createdb learn-pairs
   cat atomspace/opencog/persist/sql/multi-driver/atom.sql | psql learn_pairs
```

3) Copy all files from the `opencog/opencog/nlp/learn/run` directory to
   a new directory; suggest the directory `run-practice`.  Its best to
   try a few practice runs before committing to serious data processing.
   The next number of steps describe how to do a practice run; a later
   section focuses on batch processing.

   There are two types of files in this directory: generic processing
   scripts, and language-specific configuration files. Different
   languages require different processing pipelines; e.g. most languages
   will require a morphology processing step; English does not.
   Different languages will typically train on a different set of
   corpora organized in different directories in different ways. The
   multitude of config files allows each language to be configured in
   sharply different ways, running different kinds of experiments.

   For the practice run, suggest picking English, and using the `en`
   files. The other language files can be ignored (and can be deleted).

4) Start the OpenCog server.  Later on, the batch processing
   instructions (below) indicate how to automate this. However, for
   the practice run, it is better to do all this by hand.

   First, review the contents of `config/opencog-pairs-en.conf`. This
	simply declares the prompts that the cogserver will use; most
   importantly, it declares the port number for the cogserver. It's
   currently coded to be 17005.

   Edit the `pair-count-en.scm` and hard-code your database credentials
   into it. This saves you the trouble of having to remember them, and
   to type them in by hand.

   Finally, start the cogserver by
```
  guile -l pair-count-en.scm
```

5) Verify that the pair-counting pipeline works. In a second terminal,
   try this:
```
   rlwrap telnet localhost 17005
   opencog-en> (observe-text "this is a test")
```
   The port number 17005 was from the above-mentioned config file.

   Better yet:
```
   echo -e "(observe-text \"this is a another test\")" |nc localhost 17005
   echo -e "(observe-text \"Bernstein () (1876\")" |nc localhost 17005
   echo -e "(observe-text \"Lietuvos žydų kilmės žurnalistas\")" |nc localhost 17005
```

   This should result in activity in the cogserver and on the database:
   the "observe text" scheme code sends the text for parsing, counts
   the returned word-pairs, and stores them in the database.

   If you are truly curious, type in `(sql-stats)` in the guile shell.
   This will print some very technical stats about the Atomspace SQL
   database backend.

6) Verify that the above resulted in data sent to the SQL database.
   Log into the database, and check:
```
   psql learn-pairs
   learn-pairs=# SELECT * FROM atoms;
   learn-pairs=# SELECT COUNT(*) FROM atoms;
   learn-pairs=# SELECT * FROM valuations;
```
   The above shows that the database now contains word-counts for
   pair-wise linkages for the above sentences. If the above are empty,
   something is wrong. Go to back to step zero and try again.

7) Halt the cogserver by killing the guile process. In the next stage,
   it will be started in a different way, and having a running server
   here will interfere with things.  Unless, of course, you are careful
   to juggle the config files. There's nothing wrong with running
   multiple servers at the same time: you just have to be careful to
   avoid clashes, and for that, you can to be careful with config-file
   settings.

That's for the practice run. If stuff is showing up in the database,
then processing is proceeding as expected.

The next step is to set up bulk test processing. There are three general
stages: (I) collection of word-pair statistics (II) collection of
disjunct statistics (III) clustering.  These are described next.

Note Bene: Please do NOT spend any time at all trying to figure out
what is stored in the SQL tables.  There are "no user-serviceable parts
inside" and there is "risk of electrical shock". The SQL tables are a
very twisted, distorted and awkward representation of the AtomSpace
contents. Its kind-of like reading assembly code: you will not learn
how the AtomSpace works by reading the SQL code.  In fact, doing this
will probably make you anti-learn, by planting incorrect ideas in your
head.  Don't look in there. You have been warned!


Bulk Pair Counting
------------------
The first major stage of data processing is the generation of word-pair
statistics. Later stages depend on high-quality word-pair statistics,
and this is best obtained by processing a large number of text files;
typically thousands or tens of thousands of them, for run-time of a
few days to a week or two.  For a practice run, half-an-hour is
sufficient, but a half-hours worth of data will be quite low-quality.

The best text for "natural" English is narrative literature, adventure
and young-adult novels, and newspaper stories. These contain a good
mix of common nouns and verbs, which is needed for conversational
natural language.

It turns out that Wikipedia is a poor choice for a dataset. That's
because the "encyclopedic style" means it contains very few pronouns,
and very few action-verbs (hit, jump, push, take, sing, love). Most
Wikipedia articles state facts, describe objects, ideas and events
(primarily using the verbs is, has, was). It also contains large
numbers of product names, model numbers, geographical place names,
and foreign language words, which do little or nothing for learning
grammar. Finally, it has large numbers of tables and lists of dates,
awards, ceremonies, locations, sports-league names, battles, etc.
that get mistaken for valid sentences (they are not; they are lists),
and leads to unusual deductions of grammar.  Thus, it turns out
Wikipedia is a bad choice for learning text.

There are various scripts in the `download` directory for downloading
and pre-processing texts from Project Gutenberg, Wikipedia, and the
"Archive of Our Own" fan-fiction website.

8) Explore the scripts in the `download` directory. Download or
   otherwise obtain a collection of texts to process. Put them in
   some master directory, and also copy them to the `beta-pages`
   directory.  The name `beta-pages` is not mandatory, but some of
   the default configurations look for files there. During processing,
   files are moved to the `split-articles` directory and finally to the
   `submitted-articles` directory. Processing continues until the scripts
   are interrupted, or until the `beta-pages` directory is empty.

   Little or no pre-processing of the text files are needed. Mostly,
   one just needs to remove HTML markup or other binary crud. The
   pipeline expects all files to be in UTF-8 format.  The pipeline
   does not support any other format; if you have other formats,
   learn how to use the `iconv` command.

   There is no need to perform sentence splitting or other normalization
   (such as down-casing of initial words).  In fact, just about any
   pre-processing will lower the quality of the input data;
   pre-processing is strongly discouraged.

   It is best to split texts into files containing a few hundred to
   at most ten-thousand sentences each; larger files cause trouble
   if the counting needs to be restarted.

9) Review the `observe-text` function in `link-pipeline.scm`. The
   default, as it is, is fine, and this is almost surely what you want.
   (And so you can skip this step).  Just be aware that this function
   was written to collect a large amount of additional information,
   which you can explore, if you are adventurous. Caution, though:
   it creates a deluge of data.

   The `observe-text` function in 'link-pipeline.scm` collects
   observation counts on four different kinds of structures:

   * LG "ANY" word pairs -- how often a word-pair is observed.
   * Word counts -- how often a word is seen.
   * Clique pairs, and pair-lengths -- This counts pairs using the "clique
                  pair" counting method.  The max length between words
                  can be specified. Optionally, the lengths of the pairs
                  can be recorded. Caution: enabling length recording
                  will result in 6x or 20x more data to be collected,
                  if you've set the length to 6 or 20.  That's because
                  any given word pair will be observed at almost any
                  length apart, each of these is an atom with a count.
                  Watch out!
   * Disjunct counts -- how often the random ANY disjuncts are used.
                  You almost surely do not need this.  This is for my
                  own personal curiosity.

   Only the first bullet is needed: LG "ANY" word pairs. The other
   bullets are not needed. In particular, word counts are not needed;
   they are almost evil to collect, as they serve to confuse things.

   Knowing th length of the pairs might be important for "understanding"
   that adjectives, adverbs, determiners and possessives modify only
   nearby words. They might be important in analyzing morphology.
   Currently, no other code examines lengths; this is an open
   experiment.

10) (Optional) Review the `sometimes-gc` and `maybe-gc` settings in
   the file `link-pipeline.scm`.  These force garbage collection to
   occur more often than normal; they help keep the process size
   reasonable.  The current default is to force garbage collection
   whenever the guile heap exceeds 750 MBytes; this helps keep RAM
   usage down on small-RAM machines.  However, it does cost CPU time.
   Adjust the `max-size` parameter in `observe-text` in the
   `link-pipeline.scm` file to suit your whims in RAM usage and time
   spent in GC.  The default should be adequate for almost all users.

11) Its convenient to have lots of terminals open and ready for use;
    the `byobu+tmux` terminal server provides this, without chewing up
    a lot of screen real-estate.  The `run-shells.sh` script will run
    a `byobu/tmux` session, start the cogserver in one of the terminals,
    and leave open several other terminals for general use.

    Make a copy this file, and manually alter it to start the cogserver
    the way that you want it started.

    The `pair-submit-en.sh` script starts the process of observing text
    for pair-counting. It looks for files in the `beta-pages` directory,
    performs sentence-splitting (ss) on each, and then passes these to
    the pair-counting scripts.

    Make a copy of this file, and edit it to change the location of your
    text files (if not using the default `beta-pages`) and to change the
    port number (if not using the default 17005 for English). For
    convenience, there are other files `pair-submit-??.sh`, set up for other
    languages, using alternative default port numbers.

    Run `pair-submit-en.sh` in one of the byobu terminals.

12) Pair-counting can take days or weeks.  The `pair-submit-en.sh` will
    run until all of the text files have been processed, or until it is
    interrupted.  If it is interrupted, it can be restarted; it will
    resume with the previous unfinished text file. As text files are
    being processed, they are moved to the `split-articles` directory
    and then, to the `submitted-articles` directory.  The input directory
    will gradually empty out, the `submitted-articles` directory will
    gradually fill up.  Progress can be monitored by saying `find |wc`.

    Although high-quality pair-counts require a minimum of several days
    of processing, a half-hour or so is sufficient for a trial run.
    Feel free to ctrl-C at this point, and move to the next step
    (covered in the section **Mutual Information** below.)

    You may want to use the `renice.sh` script to lower the priority
    of the Postgres server.  This will help make your system more
    responsive, if you are also doing other work on it.  You would
    need to run this script as root (i.e. as `sudo renice.sh`).

    You may want to empty out the `submitted-articles` directory upon
    completion. The files there are no longer needed (assuming you kept
    a copy of the originals).

Notes:
* Segmentation for Chinese. Currently, only some preliminary work has
  been done for word segmentation in Chinese. It is incomplete. In the
  meanwhile, if you want to just skip to the next step, you can use an
  off-the-shelf segmenter.  Its got OK-but-not-great accuracy.

  This can be done using jieba
      https://github.com/fxsjy/jieba
  If you are working with Chinese texts, install:
  `pip install jieba` and then segment text:
  `run/jieba-segment.py PATH-IN PATH-OUT`. This python utility is in
  the `run` directory.  You will need to create modified versions of
  the `run/pair-one.sh` and `run/mst-one.sh` scripts to invoke jieba.

  To run this segmenter, you will also need to run the sentence-splitter
  differently.  Use the `zh-pre` or `yue-pre` languages with the
  sentence splitter -- these will split on sentence boundaries, only,
  and do no other processing.  By contrast, the `zh` or `yue` languages
  insert a blank space between Hanzi characters.

* Morphology. Some work on morphology has been done; but there are no
  pre-written scripts for the processing pipeline to handle this. You
  are on your own, for now.


Bulk Pair Counting - Quickstart/Cheat Sheet
-------------------------------------------
* Don't forget to perform the Postgres tuning recommendations
  found in various online Postgres performance wikis, or in the
  `atomspace/opencog/persist/sql/README.md` file.

* Use LXC containers, one for each language. Buy SSD disks. Buy an UPS.
  Install the system shutdown scripts.

* Set up distinct databases, one for each language:
```
    createdb fr_pairs lt_pairs pl_pairs simple_pairs
    cat atomspace/opencog/persist/sql/multi-driver/atom.sql | psql ??_pairs
```

* Copy input texts to the `beta-pages` directory (or another directory,
  if you wish).

* Select one of the cogserver config files in the config directory.
  Possibly alter the port number, if desired (after making a copy).

* Select one of the `pair-count-??.scm` files.  Copy it. Manually edit
  it to indicate the cogserver config from the above. Change the
  database credentials to match your database.

* Copy `run-shells.sh`, edit it to specify the `pair-count-??.scm` file
  from above. Run it.

* Select one of the `pair-submit-??.sh` files, copy it, and edit it to set
  the location of the input texts (if not `beta-pages`) and to use a
  port number that is consistent with the cogserver config file.
  Then run it in one of the byobu terminals.

If the above generates the error
```
   nc: invalid option -- 'N'
```
then edit `pair-one.sh` and remove the `-N` option from the `nc` commands.
Some versions of `nc` require the `-N` flag: the scripts will just hang
without it. Other versions of `nc` are unaware of the `-N` option, and
don't need it.  This is an `nc` design flaw.

Some of the other files used in processing:

-- `pair-one.sh` calls `split-sentences.pl` to split the text file into
   sentences, and then calls `submit-one.pl` to send the sentences,
   one at a time, to the cogserver for processing. None of these
   should need any modification.


Mutual Information of Word Pairs
--------------------------------
After observing word-pair counts, the mutual information (mutual
entropy) between them needs to be computed.  This is a required step
before disjunct observation can be started.

This is most-easily done manually, by cutting and pasting the scheme
code below into a guile shell.  Adjust the database URL for the specific
database that contains word-pairs.

```
   (use-modules (opencog) (opencog persist) (opencog persist-sql))
   (use-modules (opencog matrix))
   (use-modules (opencog nlp) (opencog nlp learn))
   (sql-open "postgres:///en_pairs?user=ubuntu&password=asdf")
   (define ala (make-any-link-api))
   (define asa (add-pair-stars ala))
   (batch-pairs asa)
   (print-matrix-summary-report asa)
```

Batch-counting might take hours or longer (maybe over a day), depending
on your dataset size. Small trial-run datasets should take no more than
5-10 minutes.  The `batch-pairs` routine will print to stdout, giving a
hint of the rate of progress.  The `print-matrix-summary-report` will
print some technical stats about the result of the run.

The above might double the size of the database, as it sits on disk --
adding frequencies and mutual-information values to word pairs will
increase the the storage size of each word-pair by a lot.  In addition,
the marginal stats are not insignificant in size.

Thus, you may want to:

* Check that you have enough storage allocated for the postgres
  database.

* Make a copy of the word-pair-only database, so that you can return
  to it, if you decide that you need to return to it for some reason.
  You can copy databases by saying:
```
  createdb -T existing_dbname new_dbname
```

Note that this means using a different set of database credentials
in the URL above!


The Vector Structure Encoded in Pairs
--------------------------------------
Note that any kind of pair `(x,y)` of things `x,y` that have a number
`N(x,y)` associated with the pair can be though of as a matrix from
linear algebra.  That is, `N` is a number, `x` is a row-index, and `y`
is a column-index, for the `(x,y)`'th entry of the matrix `N`.

In the current case, both `x` and `y` are `WordNode` Atoms, and `N(x,y)`
is an observation count, for how often the word-pair `x,y` was observed.
Note that this matrix is **extremely sparse**: if there are 10K words,
there are in principle 10K x 10K = 100M entries; of these, fewer than
a million will have been observed just once, and half of that twice.
(following Zipf's law - a quarter of those observed 4 times, etc.).

Given `N(x,y)`, one can compute the marginal sums `N(x,*)`, `N(*,y)`
and `N(*,*)` (with `*` denoting the wild-card).  The observed frequency
is then given by `p(x,y) = N(x,y)/N(*,*)` -- this is the so-called
"frequentist probability" -- a probability obtained from counting how
often something actually occurred.

Given the `p(x,y)`, one can go on to compute marginal entropies, such
as `H(x,*) = -sum_y p(x,y) log_2 p(x,y)` and so on. Mutual information
is defined similarly.  Please be aware that none of these quantities
are symmetric: in general, `N(x,y)` does NOT equal `N(y,x)`, and that
likewise, `N(x,y)` does NOT equal `N(y,x)`, and thus, in general, the
mutual information will also not be symmetric: `MI(x,y)` does NOT equal
`MI(y,x)`.  This is because the order of the words in a word-pair is
important: words do not come randomly distributed. This can lead to a
considerable amount of confusion, when comparing to the definition of
mutual information given in textbooks, references and Wikipedia:
usually, those references only provide a symmetric definition, suitable
only when events are not sequentially ordered. That definition is **not
the same** as that being used here, although it looks quite similar.
Failure to make note of this can lead to a lot of confusion!

Anyway ... the `batch-pairs` script computes all of these marginals.
The `print-matrix-summary-report` reports on the resulting matrix.
Note that both of these routines are generic: they will work for
any collection of pairs of atoms, and not just `WordNode`s. What's
more, the type of `x` and the type of `y` don't even have to be the
same; these can be two different types of Atoms.  Nor do the two atoms
have to be yoked by a single `Link`; they can exist as two distinct
atoms embedded in more complex AtomSpace patterns of any kind.  The
`matrix` subsystem slaps a two-D matrix API onto arbitrary patterns
in the AtomSpace, allowing essentially any complex graphical shape
to be viewed as a collection of vectors.

Again, this is a key insight: the axioms of graph-theory are somewhat
similar to the axioms of a vector space (but are not the same). This
similarity can be used and abused: the `matrix` subsystem allows
portions of a graph to be viewed as a collection of vectors.  The
tools then allow probabilities, entropies and mutual information to
be computed for these vectors.


Maximum Spanning Trees
----------------------
The next step of the processing pipeline is to determine some
provisional, plausible disjuncts that can be associated with words.
A disjunct can be thought of as "just like an N-gram" or "just like a
skip-gram", except that (unlike N-grams/skip-grams) the disjunct
contains additional connectivity information.  The disjunct indicates
which words are allowed to "connect" to which other words, and in
what order those connections can be performed.  It is the connectivity
information between words that captures the grammar of a language.

To be clear: if one has explicit and accurate information about the
connectivity between words, then it is straight-forward to convert
that information into a natural-language grammar, in any one of a
number of competing systems, such as Head-Phrase Structure Grammar
(HPSG), or Categorial Grammar (CG), or Dependency Grammar (DG). The
primary focus here is on Link Grammar (LG), which can be thought of
as a kind of dependency grammar. All of these different systems are
roughly equivalent, and grammars expressed in one of the systems can
be transformed into grammars expressed in one of the others, in a
fairly straight-forward, purely mechanical way.

Thus, the goal is to obtain reasonably-accurate connectivity information
between words, accumulating observation counts, so that, eventually,
the most likely word-usages and word-connectivity emerge from the noise.
This again requires crunching a lot of raw text.

Semi-accurate connectivity information can be obtained by MST parsing.
The result of such parsing is OK, but falls short of being great; this
has been demonstrated a number of times, most clearly and notably in
Deniz Yuret's PhD thesis.  The goal of this processing step is to
accumulate connectivity information from a very large number of MST
parses, with the hope that the incorrect/conflicting parses will average
each-other out (destructive interference) while the correct parses will
reinforce and become statistically clear (constructive interference).
(And this is indeed what actually happens!)

This is done by constructing a Maximum Spanning Tree (MST) between the
words in a sentence.  The tree contains just enough edges to connect
every word in the sentence.  The quantity being maximized is the sum of
the mutual information between word-pairs.  There is exactly one such
tree that maximizes the total MI.

After obtaining such a tree, each edge is "cut in half", leaving behind
a word, with dangling, disconnected half-edges attached to it. The
collection of dangling half-edges is called a "disjunct"; each half-
edge is called a "connector" (some parts of the code use the words
"connector set", "connector sequence" and "section" as synonyms for
"disjunct". Sometimes, they are called "pseudo-disjuncts" (and "pseudo-
connectors"), to distinguish from "real" connectors and disjuncts,
defined later.  This is a historical artifact.)

The specific disjunct that remains behind depends very strongly on the
word itself (of course) and on the MST tree for the sentence that the
word was in. The code in this part of the pipeline accumulates
observation counts on the large variety of possible disjuncts (that is,
word-disjunct pairs) that can be observed.

Running MST Disjunct Counting
-----------------------------
The overall processing is very similar to batch pair-counting. The steps
are as follows:

* Copy text files to the directory `gamma-pages` directory. This can
  be a different set of text files from before, or the same. It does
  not seem to matter. You can pick a different directory name, if you
  wish.

* Make a copy of the database containing word-pair mutual information.
  The disjunct observation counts will be accumulated into the database,
  further expanding it's size.  You probably want to keep a copy of the
  original, "just in case". You can copy databases by saying:
```
  createdb -T existing_dbname new_dbname
```
* Make a copy of `config/opencog-mst-en.conf` (if you wish) and edit
  it to suit your tastes. Or use the default.  Make note of the port
  number: this one uses port 19005.

* Make a copy of `mst-count-en.scm` (or another `mst-count-??.scm` file)
  Edit it, and insert the database credentials for the database that
  contains the word-pair MI data.  Indicate the cogserver config file to
  use (previous step).

* Make a copy of `run-shells.sh` and edit it to specify the copy of the
  `mst-count-en.scm` file, created above.  Also adjust the port number
  to match (19005 instead of 17005). Then run it, to get the byobu/tmux
  session started.

* Wait. And then wait a little bit longer. If you set up `run-shells.sh`
  to automatically start `guile -l mst-count-en.scm`, then it will
  stall. If you look at that file, you will notice the line
  `(pair-obj 'fetch-pairs)` -- this loads all word-pairs into the
  AtomSpace. This can take anywhere from 10 minutes to several hours,
  depending on the size of your word-pair database. If you start this
  in byobu, you will need to wait until it prints, and the guile prompt
  re-appears.  After it finishes loading, it will print some SQL
  performance stats, and a matrix summary report. You're good to
  go once these appear.

* Make a copy of the `mst-submit-en.sh` file, and adjust the location
  of the  text files, and the port number, to match the above.  If
  you accept the defaults, no changes are needed.  In one of the tmux
  terminals, run the `mst-submit-en.sh` file.  This starts the MST
  processing stage.  As before, the text files will be moved to the
  `split-articles` directory while they are being processed, and then
  finally to the `mst-articles` directory, when processing is complete.

  The processing will stop automatically, when the `gamma-pages`
  directory empties out.  You can monitor progress by saying `find|wc`.
  If you interrupt processing, you can safely resume it -- it will pick
  up again, with the last unprocessed file.  You may want to empty out
  the `mst-articles` directory -- those files are no longer needed
  (assuming you still have the originals).

  As before, it is reasonable to run this for at least several days,
  if not a week or two, to accumulate a sufficient number of observation
  counts for the disjuncts.  For a trial-run, a half-hour should be
  sufficient.

Disjunct Marginal Statistics
----------------------------
The next steps require marginal statistics to be available for the
disjuncts. The previous step gathered a large number of observation
counts for disjuncts. These need to be summarized, so that one obtains
per-word statistics.  Marginals will be needed both form the
pseudo-csets and for the cross-connector sets.  First, compute the
marginals for the pseudo-csets:
```
  (sql-open "postgres:///en_disjuncts?user=linas")
  (define pca (make-pseudo-cset-api))
  (define psa (add-pair-stars pca))
  (define btr (batch-transpose psa))
  (psa 'fetch-pairs)
  (btr 'mmt-marginals)
```
To obtain the marginals for the cross-connectors, repeat the above
steps, but this time with `(make-shape-vec-api)` as the base class.
At this time, the need for the cross-connectors (shapes) is optional...

The above omputations may take hours or days, depending on the size of
the disjunct set.  Be sure to make a backup copy of the resulting
database before proceeding to the next step.


Determining Grammatical Classes
-------------------------------
The point of counting disjuncts is to obtain a good statistical sampling
of what words connect to what words: a big collection of these "dangling
cut-in-half edges" that make up a disjunct.  With this dataset, you now
have a pretty good idea of which words typically attach to other words,
and how many connectors they need to do this.  This dataset has captured
the syntactic structure of the language, on a word-by-word basis,
complete with statistical information about how often each disjunct
occurs, relative to all the others.

Unfortunately, its a huge dataset, and it still contains some fair
amount of noise (although it contains less noise than a single MST
tree -- all of those bad MST parses are beginning to average out).
Further averaging and noise reduction (and thus, higher accuracy)
can be achieved by classifying individual words into grammatical
categories. Doing so, however, also requires that the disjuncts also
be classified: disjuncts consist of connectors, each connector
specifying just a single word. The words in the connectors **also**
have to be merged into grammatical categories. What's more, these
categories have to be consistent with the categories obtained from
clustering the words.

This requirement makes the algorithm for discovering grammatical
classes fundamentally unique, and different than any other clustering
algorithm employed before. This is not an exaggeration; however,
many/most people struggle with this concept, so its sketched below.
The paper on "Sheaf Theory", located at
https://github.com/opencog/atomspace/blob/master/opencog/sheaf/docs/sheaves.pdf
explains this in greater detail.  There will also be an upcoming blog
post on this.

The collection of (word,disjunct) pairs can be thought of as a
collection of vectors, one per word.  The observed frequency `p(w,d)`
for word `w` and disjunct `d` can be thought of as the vector magnitude
for the basis element `d`. That is, a word `w` can be represented as
the vector
```
   v = p(w,d_1) e_1 + p(w,d_2) e_2 + ... p(w, d_n) e_n
```
The `e_k` are the basis elements of the vector space.  There is exactly
one such basis element for each disjunct `d_k`, and so one can safely
think of the `d_k` and the `e_k` as being the same thing. We use the
notation `e_k` here because this is the textbook-standard way of writing
the basis vector of a vector space.  Its the same thing. The `e_k` are
basis vectors.

There is one such vector per word. The probabilities `p(w,d)` are
experimentally determined.  Since there is a vector per word, all of the
industry-standard concepts can be applied: one can compute the cosine
angle between two words. One can compute the jaccard distance.  One
can compute the (symmetric!) mutual information between two words (the
Kullbeck-Liebler divergence) because now, the word-order no longer
matters; the two words are no longer embedded in a sentence.  All of
these different metrics give a hint at how similar or different two
words might be.

It is now super-duper tempting to apply industry-standard big-data
tools to obtain clusters. For example, one can do K-means clustering,
if one wished.  One can take the famous, well-known Adagram or Word2Vec
algorithms, and replace the n-grams/skip-grams in these algos by the
disjuncts. That is, the disjunct-vector above is really very, very much
like a traditional n-gram/skip-gram.  If one applies these, or other
neural-net techniques to the disjunct vectors, one could get results
that are very similar to what the current n-gram/skip-gram techniques
are producing.  The results would be similar because the "statistical
power" in the word-disjunct vectors is very similar to the statistical
power in a skip-gram. They really are not all that different.

***HOWEVER...***
There is one hugely-important, critical difference.

The basis elements `e_k` are not indivisible, opaque, meaningless
boxes.  The have a structure!  They consist of sequences of connectors!
Those connectors are words! When the clustering/classification step
is performed, one must **not only** classify according to
word-similarity, as revealed by the vector, **but also** to classify
in such a fashion that the basis elements are renumbered, restructured
as well.  The vector space itself is not static: it is mutating and
changing shape, as classification proceeds, because the `e_k`
themselves **are not constants**.

This is the reason that applying K-means, PCA/SVM kernel methods on
the word-disjunct vectors is both boring and also counter-productive.
Disjuncts behave a whole lot like skip-grams (well, they might be,
maybe, a teeny-weeny bit more accurate. Maybe). Applying these
traditional algos to the word-disjunct dataset will give traditional
results, with traditional accuracies. Word-disjunct pairs, used in this
way, will not provide break-out performance or results.

The whole point of computing disjuncts is to get past these vector-based
algorithms, and to enable graph-based, connectivity-based classification
and clustering.  The point is to start with a network, and to extract
the structure of the network, accurately, by making explicit use of the
network connectivity.  Disjuncts do this.  The correct algorithm
requires that both the words be clustered, and also, at the same time,
for the connectors to be clustered, with both steps being done
simultaneously, in a coherent fashion.  During classification, portions
of the dataset resemble vector spaces, and that's OK.  This can be
leveraged and used. However, the vector spaces stitch together in a
very non-linear kind of way.  The disjuncts/connectors show you
exactly how they stitch together.

The correct determination of grammatical classes requires that the
stitching of this fabric be accounted for. The paper on 'sheaf theory'
tries to explain this in greater detail. The code here tries to actually
implement these ideas.

Creating Grammatical Classes
----------------------------
The clustering code, for isolating grammatical classes, is in active
development. The instructions here are provisional and subject to
change.

* Make a copy of the table holding the disjunct statistics. This is
  critical! The clustering algorithm(s) actively alter the per-word
  statistics as they proceed. The database contents will be scrambled
  in such a way that the original word-disjunct stats will be lost.

* At the guile prompt:
```
  (use-modules (opencog) (opencog persist) (opencog persist-sql))
  (use-modules (opencog nlp) (opencog nlp learn))
  (use-modules (opencog matrix) (opencog sheaf))
  (sql-open "postgres:///en_pairs_sim?user=linas")
```
  and the, try either
```
  (gram-classify-greedy-discrim)
```
  or
```
  (gram-classify-greedy-fuzz)
```
  Both will take days to run. You can start poking at the results
  earlier, though. the file `learn-lang-diary/word-classes/word-classes.scm`
  contains an ad-hoc assortment of tools that can be used to examine
  the word-classes discovered so far.  Read it for details.

  Note that, because the above alters word-vectors on the fly, the
  cosine-angles between words will change over time, and thus might
  not actually be what you expect. If you want to play with cosine
  distances or other metrics, you need to do so on a separate, clean
  database load. The above database **will** be altered.

Here's some games you can play with a clean dataset (before being
altered by the clustering code):
```
  (fetch-all-words)
  (length (get-all-words))

  (define pca (make-pseudo-cset-api))
  (define psa (add-pair-stars pca))
  (psa 'fetch-pairs)
  (define all-cset-words (get-all-cset-words))
  (length all-cset-words)
  ; This reports 37413 in for my `en_pairs_sim`.
  (define all-disjuncts (get-all-disjuncts))
  (length all-disjuncts)
  ; This reports 291637 in for my `en_pairs_sim`.

```
  You can now play games:
```
 (cset-vec-cosine (Word "this") (Word "that"))
 (cset-vec-cosine (Word "he") (Word "she"))

```
  The `pseudo-csets.scm` file contains code for working with word-
  disjunct vectors.  Any routine that is `define-public` can be
  invoked at the guile prompt. Most are safe to use.  If its not
  `define-public`, you should not call it by hand.

  The `lang-learn-diary/disjunct-stats.scm` file contains ad-hoc code
  used to prepare the summary report.  To use it, just cut-n-paste to
  the guile prompt, to get it to do things.

  The marginal entropies and the mutual information between words and
  disjuncts can be computed in the same way that it's done for word-
  pairs, by using the `(batch-pairs ...)` function. However, that
  function does more than is strictly needed, and one can save some
  disk space and CPU time by computing only the word-similarities:
```
  (define pca (make-pseudo-cset-api))
  (define psa (add-pair-stars pca))
  (define btp (batch-transpose psa))
  (btp 'mmt-marginals)
```
  Again, `(w,d)` is just a pair. Like other pairs, its a matrix, and
  has marginal probabilities associated with it.


TODO
----
Some things in the pipeline, but unfinished:

* Replace cosine distance in the clustering algos by the information
  divergence, as explained in the
  [Graph Models vs. Gradient Descent](https://github.com/opencog/opencog/raw/master/opencog/nlp/learn/learn-lang-diary/skippy.pdf)
  document.  There's alreaddy code for computing the divergence;
  to get it, just say
```
  (define smi (add-symmetric-mi-compute pca))
  (smi 'mmt-fmi (Word "foo") (Word "bar"))
```

* Replace centroid means, as explained above.

* The clustering algos above already perform word-sense factoring.
  Explicit word-senses can be identified by looking at multi-cluster
  membership.  These can now start to be used for re-parsing, to
  obtain word-sense pair-correlation statistics.


Exporting a Lexis
-----------------
The collection of grammatical classes, the words that belong to them,
and the attached disjuncts comprise a lexis that is compatible with the
Link Grammar parser. It can be exported with the `export-csets`
function. When copied to a location where Link Grammar can find it,
it can be used to parse text by the link parser.


Next Steps
----------
The clustering code is in development, and the best/fastest algorithms
are not yet known.  The best metrics are not known; currently, the code
uses cosine distance, but the (symmetric) mutual information is surely
better.

It is not clear how to judge the quality of the results. Manual
inspection looks pretty good. Manual inspection does not reveal that
one or another variant with different parameters is qualitatively
better.

Given a set of grammatical classes derived in this way, together with
disjuncts that are formed from the classes, these can be loaded into
link-grammar dictionaries. Use the `export-disjuncts.scm` file to do
this. The exported lexis is in `sqlite3` format, which the link-grammar
parser understands.

A reasonable next step is to run raw text through this parser,
accumulate statistics on the individual disjuncts, and see how they
stack up against the original stats. Are some disjuncts being used far
more often? Far less often? If so, what does that mean?

Can two different dictionaries be compared? Surely, some will give
different parses than others; where do they differ? Why?

Given a stable dictionary, an obvious next step is to attempt to
perform reference resolution.


Precomputed LXC containers
--------------------------
For your computing pleasure, there are some LXC containers that you
can download that have a fully-configured, functioning system on them.
The set of available containers will change over time.  The first one
actually has a bunch of bugs and other problems with it, but it's
enough to do some basic work. The steps are:

* You'll still need to satisfy at least some of the hardware
  requirements listed previously.

* Install lxc, Like so:
```
  sudo apt-get install lxc
```

* Download a container from
  https://linas.org/lxc-nlp-containers/

* The following describe root-owned containers. You can also have
  user-owned containers, if you know how to do that.

* Go to the directory `/var/lib/lxc/lxc`.  You may need to create
  this directory.  Unpack the file you downloaded above, in this
  directory. Do this as root. Do NOT change the ownership of the
  files!  Avoid changing the date-stamps on the files.

* Type in: `sudo lxc-ls -f` You should see a display similar to this:
```
NAME           STATE   AUTOSTART GROUPS IPV4      IPV6
simil-en       STOPPED 0         -      -         -
```

* Start the container by saying `lxc-start -n simil-en`. Give it
  5 or 10 or 30 seconds to boot up. Then `lxc-ls -f` again. You
  should see the below. It may take a minute for the IPV4 address to
  show up. You will probably get a different IP than the one shown.
```
NAME           STATE   AUTOSTART GROUPS IPV4      IPV6
simil-en       RUNNING 0         -      10.0.3.89 -
```

* You can now `ssh` into this container, just as if it were some
  other machine on your network.  It more-or-less is another machine
  on your network.
```
  ssh ubuntu@10.0.3.89
```
  The password is `asdfasdf`.

* The run-scripts are in the `run` directory.  The opencog sources
  are in the `src` directory. You can `git pull; cmake..; make` these
  if you wish. Or not. Its all set up already.  Pull only if you need
  to get the latest.

  (For example, this README file, that you are reading, is in
  `/home/ubuntu/src/opencog/opencog/nlp/learn/README`. Right now.)

* You can now hop directly to the section "Exploring Connector-Sets"
  above, and just go for it.  Everything should be set and done.
  Well, you do need to start guile, of course, etc.


That's all for now!
-------------------
THE END.
