This directory contains those files which were neccessary to extract
information about the data. The tools to plot graphs are in the folder
<i>graphing tools</i>. These files were used to plot graphs when I had
around 1.2million word pairs. As the number of word pairs grow, they
will probably need to be plotted again. A description of the files
follows.

chapters.sh, chapters-epub.sh
-----------------------------
Split Project Gutenberg files (and other text files) into pieces.


down-guten.sh
-------------
Download and prep for processing a bunch of Project gutenberg
books. These form teh bulk of the English "tranche-1" series.

down-fanfic.sh
--------------
Download approx 150 fanfic titles from "Archive of Our Own", and
split them up into digestible chapters.  This was used to create
the "tranche-2" of the langauge-learning experiment for English
