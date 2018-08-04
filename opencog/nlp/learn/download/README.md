
Download tools
==============
Assorted scripts to download text files/corpora from the net.  This
includes tools to download and pre-process Wikipedia articles in
multiple langauges, and likewise for Project Gutenberg texts, and
also for "Archive of Our Own" fan-fiction titles.

Pre-processing for wikipedia means stripping out wiki markup, as
well as discarding Image, Category and List files.

Pre-processing for book texts means splitting up large texts into a
number of smaller "bite-size" files, containing some dozens of
paragraphs, hundreds of sentences each.  Smaller files are easier to
manage, and have more predicatable processing times.

This includes taking E-PUB format files, and dumping just the plain,
raw UTF-8 text in them.

The tranche-1,2,3 files indicate specific texts at specific URLS that
were actually used to obtain actual, specific texts for the tranche-1,2,3
database dumps. The goal here is reproducibility of processing.

Directories
===========
* en-tranche-* -- download five batches of books, mostly Gutenberg,
  but also some fanfic.  Prep for processing, including spiltting
  into smaller files. the tranche-5 scripts are the most mature.

* wikipedia -- download and prep wikipedia

* zh -- download Mandarin Chinese novels. TODO -- this downloads only;
  doesn't yet do the prep stage needed for processing.


Shell scripts
=============

chapters.sh, chapters-epub.sh
-----------------------------
Split Project Gutenberg files (and other text files) into pieces.

down-guten.sh
-------------
Download and prep for processing a bunch of Project gutenberg
books. These form the bulk of the English "tranche-1" series.

down-fanfic.sh
--------------
Download approx 150 fanfic titles from "Archive of Our Own", and
split them up into digestible chapters.  This was used to create
the "tranche-2" of the langauge-learning experiment for English
