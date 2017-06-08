
Wikipedia pre-processing utilities
==================================
This directory contains assorted scripts to pre-process Wikipedia dumps.
This includes removing wiki markup, as well as removing Image, Category
and List files.  The result are text files suitable for feeding into
the language learning pipeline.


File overview
-------------
The wiki-scrub.pl utility will take a Wikipedia XML database dump,
marked up in the standard Wikipedia XML format, and split it into
multiple articles, one per file. In the process, it will remove all
markup, leaving only plain text. The goal is to have clean text that
can be input into Relex.  Runtime is roughly 5 minutes per 100K
articles, or one hour per million articles.

The wiki-clean-en.sh shell script will remove certain unwanted wikipedia
pages (templates, images, categories, etc.) for the English-language
wikipedia. These files contain almost no parsable text.

* `wiki-clean-fr.sh` As above for frwiki (French)
* `wiki-clean-gu.sh` As above for guwiki (Gujarati)
* `wiki-clean-hi.sh` As above for hiwiki (Hindi)
* `wiki-clean-lt.sh` As above for ltwiki (Lithuantian)
* `wiki-clean-pl.sh` As above for plwiki (Polish)
* `wiki-clean-tl.sh` As above for tlwiki (Tagalog)
* `wiki-clean-zh.sh` As above for zhwiki (Chinese)

The `wiki-alpha.sh` shell script will move (Latin-font) wikipedia
articles to alphabetical directories.  This can simplify and speed up
article processing.

* `wiki-alpha.sh` - Same, but for Chinese

Wikipedia processing
--------------------
Notes below are for four languages: French, Lithuanian, Polish, and
"Simple English".  Adjust as desired.

Download wikipedia database dumps:
```
https://dumps.wikimedia.org/enwiki/
https://dumps.wikimedia.org/ltwiki/
https://dumps.wikimedia.org/frwiki/
https://dumps.wikimedia.org/zhwiki/ (Mandarin)
https://dumps.wikimedia.org/zh_yuewiki/ (Cantonese)
```

The wikipedia articles need to be scrubbed of stray wiki markup, so
that we send (mostly) plain text into the system. Do this more-or-less
as follows:
```
    cd /storage/wikipedia
    cp shell scripts and perl scripts.

    mkdir wiki-stripped
    time cat blahwiki.xml.bz2 |bunzip2 | ../wiki-scrub.pl
    cd wiki-stripped
    find |wc
    ../wiki-clean.sh
    find |wc
    cd ..
    mkdir alpha-pages
    cd alpha-pages
    ../wiki-alpha.sh
```

simple:
```
find |wc     gives 131434 total files
find |wc     gives 98825 files after cat/template removal.
```

lt:
```
7 mins to unpack
find |wc gives 190364 total files
find |wc gives 161463 after cat/template removal
```

pl:
```
1 hour to unpack (15 minutes each part)
find | wc gives 1097612 articles
52K are categories
35K are templates
find |wc gives 1007670 files after cat/template removal
```

fr:
```
3 hours to unpack (25-60 minutes per glob)
find |wc gives 1764813 articles
214K are categories
55K are templates
find |wc gives 1452739 files after cat/template removal
```

en:
```
10 hours to unpack
find|wc gives 7495026 articles
936K are categories
866K are Files
259K are Templates
find| wc gives 5423537 files after cat/template removal
```
