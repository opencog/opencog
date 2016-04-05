
AIML in the AtomSpace
---------------------

# Status
Under development, very incomplete.

# Goal
Map AIML expressions to equivalent atomspace graphs, in such a
way that the OpenCog pattern matcher can perform all of the same
funcitions that AIML currently performs.

# Lightning reivew of AIML
Some example sentences.

```
        Input               That Topic     Result
(R1)  Hello                  *     *    Hi there
(R2)  What are you           *     *    I am Hanson Bot
(R3)  What are you thinking  *     *    How much I like you
(R4)  Hi                     *     *    <srai>Hello</srai>
(R5)  I * you                *     *    I <star/> you too.
(R6)  What time is it        *     *    The time is <syscall>
(R7)  *                      *     *    I don't get it.
(R8)  ...                    *     *    <set topic = "animals dog"/>
(R9)  ...                    *     *    <set name="topic">animals dog</set>
(R10) ...                    *     *    <set name="topic"><star/></set>
(R11) ...                    *   "* dog"   ...
(R12) ...                    *     *    I like <get name="topic"/>
(R13) ...                    *     *    I like <bot name="some_key"/>
(R14) Is it a <set>setname</set> ?
(R15) Are you <bot name="key"/> ?
(R16) Are you <set><bot name="species"/></set> ?
```

Future, not a current part of AIML:
```
(F1)  Do you like <topic/> ?
(F2)  Do you like <get name="topic"/> ?
```

## Notes
* Notice that the pattern match of R3 takes precendence over R2.
* srai == sent the result back through.
* that == what the robot said last time. (just a string)
  A full history of the dialog is kept, it can be refered to ...
  (TBD XXX How ???)
* topic == setable in the result. (just a string)
  topic is just one key in a general key-value store of per-conversation
  key-value pairs.

* star and underscore (`*` `_`) are essentially the same thing; it's an AIML
  implementation detail.
* star means "match one or more (space-separated) words"
  stars are greedy.
* carat and hash (`^` `#`) mean "match zero or more words"
* There is an implicit star at the end of all sentences it is less
  greedy).
* (R11) the wild-card match matches the second of two words in a
  2-word topic.  (TBD XXX what about N words ??)
* (R14) the wild-card match limited to one of a set.

# Pattern Recognition

Implementing AIML efficiently in the atomspace requires that the pattern
matcher be run "in reverse": rather than applying one query to a
database of thousands of facts, to find a small handful of facts that
match the query; we instead apply thousands of queries to a single
sentence, looking for the handful of queries that match the sentence.
That is, AIML is an example of pattern recognition, rather than
querying.  This means that the traditional link types BindLink and
SatisfactionLink are not appropriate; instead, we use the PatternLink
to specify the AIML patterns.

## Globbing

The pattern matcher uses GlobNode to perform globbing.
See `http://wiki.opencog.org/w/GlobNode` for details.
See `glob.scm` for a simple working example.


# OpenCog equivalents
* R1 example.

```
PatternLink
   AndLink
      WordSequenceLink
         WordNode "Hello"
         GlobNode "$eol"     # rest of the input line
      ListLink
         AnchorNode "#that"
         GlobNode "$that"
      ListLink
         AnchorNode "#topic"
         GlobNode "$topic"
      PutLink                    # If the above conditions are satisfied
         AnchorNode "#reply"     # then this PutLink is triggered.
         WordSequenceLink        # This is the reply.
            WordNode "Hi"
            WordNode "there"
```

There is another approach:

I really really * you, darling.
I really really love you, darling.

WordInstanceLink
   WordNode "I"
   Word
