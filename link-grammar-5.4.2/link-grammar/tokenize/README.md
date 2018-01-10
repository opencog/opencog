
Version 5.3.0 - Introduction of a word-graph for tokenizing
===========================================================

In this version the tokenizer code has been heavily modified once again.
Instead of tokenizing all the words in one pass directly into the
word-array of the parser, it now tokenizes them using a graph which its
nodes are "subwords".  Initially the original sentence words are tokenized
to subwords only by whitespace. After that step, each subword is handled in
its turn, and gets split to further subwords if needed. A special care is
taken if one of the alternatives of a subword is the subword itself
(alone, which is usual, or with a dict-cap token - a new experimental
mechanism which is described below).

The new way of tokenizing is much more flexible that the old one, and it
serves as an infrastructure on which new tokenizing and display features
can be implemented. One of them, that is implemented in this release, is
spelling for languages other then English. This is now possible because the
spell result can now pass further tokenization if needed. This also
enhances the spelling for English in case the spell result is a
contraction. In addition, the multi-level of tokenization, inherent to the
word-graph, allows multiple run-on and spell-correction fixes on the same
word at once.

The tokenizing code is still based much on the old code and further
work is needed to clean it up (or to replace it, e.g. by a
regex-tokenizer). It still doesn't use the full power of the word-graph,
and there are constructs that need to be tokenized but they are not (they
are also not in the sentence test batches). E.g. `--` between words without
whitespace.

There is still no API to get information from the word-graph. In particular,
it is not possible to find out the sentence words after punctuation
tokenization, as in previous releases.

Since the parser cannot currently use the word-graph directly, there is a
need to convert it to the 2D-word-array that it uses. This is implemented
by the function `flatten_wordgraph()`, which uses a complex algorithm: It
scans all the word-graph paths in parallel, trying to advance to the next
words and to issue them into the 2D-word-array.

It advances to the next word of a given word in rounds, using two passes, one
word per word-graph path on each round:
Pass 1. Next words which are in the same alternative of the given word.
Pass 2. Next words which are in a different alternative (of the common
ancestor word) of words that has already been advanced to in pass 1
and this pass.

The words that got advanced to are issued into the 2D-word-array.  It is
possible that the second pass above cannot get advance in particular
word-graph path, because the next word is in the same alternative of one of
the next words in other paths. In that case an empty word is issued. This
constraint arises because all the next words in each word-graph advance
round, are issued into the same 2D-word-array "slot", which cannot hold
words from the same alternative.

As in the previous versions, due to the way alternatives are implemented,
morphemes from different word-tokenization alternatives can be mixed in a
linkage. Such linkages are of course useless and need to be detected and
discarded. This is done by the `sane_linkage_morphism()` function. In the
previous versions it validated that the chosen words (the words in the
linkage) that are subwords (e.g. morphemes) of a sentence word, all belong
to a single tokenization alternative of that word.
It now works in another way - it validates that the chosen words create a
path in the word-graph. In case of "null-words" - words with no linkage -
the first path which is encountered is used. It means that a word in the
word-graph path corresponding to a null-word, may be only one of the potential
possibilities.

Word-graph display
------------------
Another feature that has been implemented, mainly for debug (but it can
also be useful for inspiration and fun), is displaying a graphical
representation of the word graph. The graphs can be displayed in several
ways, controlled by one-letter flags. The command `!test=wg` enables the
display of a graphs which includes no "prev" links for clarity, but
includes "unsplit word" links.  Other graphical representation modes can be
enabled by `!test=wg:FLAGS`, when FLAGS are lowercase letters as defined in
wordgraph.h.  For example, `!test=wg:sl` displays unsplit words as subgraphs,
with a legend, and adding the `p` flag (i.e. `!test=wg:slp`) adds "prev" links.
The graphical display still needs improvements in order to be able to
display complex word-graph situations in a simple manner.  The graphical
display code is not compiled by default because it involves invocation of
an external program (`dot`) and in addition, files are created, both things
may not be desired by some users. Use `--enable-wordgraph-display` to enable
this feature.

On Windows this feature is enabled when compiled with `USE_WORDGRAPH_DISPLAY`.
See "../msvcNN/RDADME" (NN is the MSVC version) for further details.

Quote handling
--------------
Quotes now are not discarded, but are considered to be regular dict tokens.
In this version they have no significant linkage and always attach to the word
before them (or to the LEFT-WALL). In order to compare detailed batch runs with
previous versions of the library, a `!test=removeZZZ` can be used to remove the
quote display.

Handling capitalized words
--------------------------
Not as in previous releases, capital letters which got downcased are not
restored for display if the affected words have a linkage.

A new experimental handling of capital words using the dictionary has been
introduced. It inserts the token `1stCAP` before the uc version, and `nonCAP`
before the lc one, as discussed in:
https://groups.google.com/forum/?hl=en#!topic/link-grammar/hmK5gjXYWbk
It is enabled by `!test=dictcap`. The special "dictcap" tokens are not yet
discarded, so in order to compare results to previous library versions, the
following can be used: `!test=dictcap,removeZZZ`.


HOWTO use the new regex tokenizer/splitter
==========================================
It's new, experimental code.

To compile: `../configure --enable-regex-tokenizer`



- At the linkparser> prompt, enter:
`!/REGEX/,tokentosplit`

Currently, if tokentosplit contains white space, command-line.c discards
it.
Also, case is currently observed.

The regex syntax is designed so the regex is a valid one (although
meaningless) as written, so compiling it would reveal syntax errors in
it (the result of this initial compilation is not used).

- All the /regexes/ are anchored at their start and end, as if `^` and `$`
  were used.
- Mentioning a word class (x is an optional constraint, defaults to
  `.*`):

`(?<CLASS>x)`

CLASS can be:
 * DICTWORD, to match a word from `4.0.dict`.
 * An affix class name (takes priority if there is a regex with the same
   name).
 * A regex name from `4.0.regex` (prefix it with `r` if there is such an
   affix class).

For regexes from `4.0.regex`, the code combine the ones with the same
name, taking care to omit the `^` and `$` from each, if exist (constraints
are said to be supported  (not tested) and can be added if needed, but I
could not find an example of anything useful).

DICTWORD can be optionally followed by a word mark, which is taken from
the affix file:

 * DICTWORDaM append M to DICTWORD before looking it up.
 * DICTWORDpM prepend  M to DICTWORD before looking it up.

If M contains more than one word (in the affix file), only the first one
is currently used.


Examples:
 * `(?<SUF>)` match a suffix from the affix file
 * `(?<NUMBER>)` match the regex `NUMBER`.
 * `(?<UNITS>)` match `UNITS` from the affix file.
 * `(?<rUNITS>)` match `UNITS` from the regex file.
 * `(?<DICTWORD>)` match a dictionary word.
 * `<?<DICTWORDaSTEMSUBSCR>)` match word.= (if `STEMSUBSCR` is ^C=).
 * `<?<DICTWORDpINFIXMARK)` match =word (if...)

- Using word constrains (_x_ in `(?<CLASS>x)` ):
Matching single letters by DISTWORD (because they are in the dict) may
note be desired.
In such a case _x_ can be constrained to include 2 letters at least, plus
the desired 1-letter words.
E.g.: `(?<DICTWORD>.{2,}|a)` , which matches words of 2 letters and more,
plus the word `a`.

- Currently the outer part of the regex should not contain alternations.
  This is because I was too lazy to add code for adding `(?:...)` over it
in such cases. So in order to contain alternations the `(?:...)` should
currently be added by hand, as in:

`/(?:(?<UNITS>)|(?<RPUNC>))*/,dfs,dsfadsdsa,.?!sfads`

- Holes are not supported. For example, this is not fine (and not
  tested):

`/(?<DICTWORD>)-(?<DICTWORD>)/,khasdkflhdsfa`

because the `-` character would create a hole in the result.
But this is fine (and also not tested...):

`/(?<DICTWORD>)(-)(?<DICTWORD>)/,asdfkjajfahlad`

Currently, since named capturing groups are used for classes, if the same
class name is used more than once, there may be a need to start the regex
by `(?J)`. This will be fixed later.

- The regex cannot include nested capture groups, so inner groups, if
  needed, should be non-capturing ones.

This is because currently the matching groups create a linear string,
without holes.
If you will find a use for internal capture groups, I can use them.
Because of that, backreferences in regexes from the regex file are not
supported (but there are currently none...).

So this is not valid (a DICTWORD which matches a `NUMBER`):

`/(?<DICTWORD(?<NUMBER>))/,qazwsx`

and this too (a nonsense constraint for demo):

`/(?<DICTWORD>([A-Z][0-9])*)/,qazwsx`

but this should be fine:

`/(?<DICTWORD>(?:[A-Z][0-9])*)/,qazwsx`


Some fun examples:

```
!/(.*)*/,test
Modified pattern: (?:(.*)(?C))*$(?C1)
Alternative 1:
 0 (1):  test (0,4)
 1 (1):   (4,4)
Alternative 2:
 0 (1):  test (0,4)
Alternative 3:
 0 (1):  tes (0,3)
 1 (1):  t (3,4)
 2 (1):   (4,4)
Alternative 4:
 0 (1):  tes (0,3)
 1 (1):  t (3,4)
Alternative 5:
 0 (1):  te (0,2)
 1 (1):  st (2,4)
 2 (1):   (4,4)
[...]
Alternative 14:
 0 (1):  t (0,1)
 1 (1):  e (1,2)
 2 (1):  st (2,4)
Alternative 15:
 0 (1):  t (0,1)
 1 (1):  e (1,2)
 2 (1):  s (2,3)
 3 (1):  t (3,4)
 4 (1):   (4,4)
Alternative 16:
 0 (1):  t (0,1)
 1 (1):  e (1,2)
 2 (1):  s (2,3)
 3 (1):  t (3,4)
```

(Some appear "twice" due to the terminating null match. I think I will
discard such matches.).

With splits to 2 parts only:
```
linkparser> !/(.*){2}/,test
Modified pattern: (?:(.*)(?C)){2}$(?C1)
Alternative 1:
 0 (1):  test (0,4)
 1 (1):   (4,4)
Alternative 2:
 0 (1):  tes (0,3)
 1 (1):  t (3,4)
Alternative 3:
 0 (1):  te (0,2)
 1 (1):  st (2,4)
Alternative 4:
 0 (1):  t (0,1)
 1 (1):  est (1,4)
Alternative 5:
 0 (1):  test (0,4)
linkparser>
```

```
!/(?:(?<DICTWORD>.{2,}|a)(?<RPUNC>)?)+/,theriver,dangeroustonavigatebutimportantforcommerce,hasmanyshoals.
```
(This is one long line, just test it...)


`!/(?<NUMBERS>)(?<rUNITS>)*(?<RPUNC>)*/,123.2milligram/terag/?!`<br>
(test it...)

```
!/(?<DICTWORD>)(?<SUF>)/,there's
Modified pattern: (?:(?<DICTWORD>.*)(?C))(?:(?<SUF>.*)(?C))$(?C1)
Alternative 1:
 0 (1):  there (0,5) [DICTWORD]
 1 (2):  's (5,7) [SUF]
linkparser>
```


In the next example, we get only whole word and double-dash because
it can only match wpwp (when w is DICTWORD and p is `--`).

```
!/(?:(?<DICTWORD>)(?<LPUNC>))+/,this--is--
Modified pattern: (?:(?:(?<DICTWORD>.*)(?C))(?:(?<LPUNC>.*)(?C)))+$(?C1)
Alternative 1:
 0 (1):  this (0,4) [DICTWORD]
 1 (2):  -- (4,6) [LPUNC]
 2 (1):  is (6,8) [DICTWORD]
 3 (2):  -- (8,10) [LPUNC]
 4 (1):   (10,10) [DICTWORD]
linkparser>
```

However, this breaks to single characters, as expected:
```
!/(?:(?<DICTWORD>)(?:(?<LPUNC>))*)+/,this--is--
...
Alternative 360:
 0 (1):  t (0,1) [DICTWORD]
 1 (1):  h (1,2) [DICTWORD]
 2 (1):  i (2,3) [DICTWORD]
 3 (1):  s (3,4) [DICTWORD]
 4 (1):  - (4,5) [DICTWORD]
 5 (1):  - (5,6) [DICTWORD]
 6 (1):  i (6,7) [DICTWORD]
 7 (1):  s (7,8) [DICTWORD]
 8 (1):  - (8,9) [DICTWORD]
 9 (1):  - (9,10) [DICTWORD]
10 (1):   (10,10) [DICTWORD]
linkparser>
```

But this stops after the first match:
```
!/(?:(?<DICTWORD>)(?:(?<LPUNC>)(*COMMIT))*)+/,this--is--
Alternative 1:
 0 (1):  this (0,4) [DICTWORD]
 1 (2):  -- (4,6) [LPUNC]
 2 (1):  is (6,8) [DICTWORD]
 3 (2):  -- (8,10) [LPUNC]
 4 (1):   (10,10) [DICTWORD]
linkparser>
````

And this is even more interesting:
```
!/(?:(?<DICTWORD>)(*COMMIT)(?:(?<LPUNC>))*)+/,this--is--
Alternative 1:
 0 (1):  this (0,4) [DICTWORD]
 1 (2):  -- (4,6) [LPUNC]
 2 (1):  is (6,8) [DICTWORD]
 3 (2):  -- (8,10) [LPUNC]
 4 (1):   (10,10) [DICTWORD]
Alternative 2:
 0 (1):  this (0,4) [DICTWORD]
 1 (2):  -- (4,6) [LPUNC]
 2 (1):  is (6,8) [DICTWORD]
 3 (2):  - (8,9) [LPUNC]
 4 (2):  - (9,10) [LPUNC]
 5 (1):   (10,10) [DICTWORD]
Alternative 3:
 0 (1):  this (0,4) [DICTWORD]
 1 (2):  -- (4,6) [LPUNC]
 2 (1):  is (6,8) [DICTWORD]
 3 (2):  - (8,9) [LPUNC]
 4 (1):  - (9,10) [DICTWORD]
 5 (1):   (10,10) [DICTWORD]
linkparser>
```

It seems as if conditional matching using (?(condition)yes-pattern|no-pattern)
or `(*THEN)` can do some fun things, but I don't have useful examples yet.

The question is how to use this code for tokenization. I have some
ideas, more on that later.
