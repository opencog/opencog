# RelEx2Logic

RelEx2Logic (R2L) produces a certain form of predicate-argument
structure for an English-language sentence.  That structure roughly
resembles classical propositional or predicate logic, thus the name.

The processing pipeline for a sentence is

1. sentence => link-grammar
2. link-grammar => relex
3. relex => relex2logic

Step 1 performs a syntactic parse of a sentence, using the
[Link Grammar](http://www.abisource.com/projects/link-grammar/) (LG)
parser.

Step 2 converts the LG output into a dependency grammar (DG) format
that is more closely aligned with de-facto DG standards. This includes
the unpacking of some of the LG output into standard linguistic feature
classifications, e.g. for person, number, tense, aspect, mood, voice.
This converter is called [RelEx](http://wiki.opencog.org/w/RelEx).

Step 3 extracts the predicate-argument logical structure.  This is done
by applying a set of rules to the RelEx format.  These are applied by
using the forward chainer provided by the
[unified rule engine](https://github.com/opencog/atomspace/tree/master/opencog/ure).

After the quick-start, below, there is a quick sketch of how the R2L
rules are applied to sentences in the AtomSpace. This is followed by
a long report describing the status of the actual rules.


###Quick-start
To use R2L:

1. Start the relex server using the `opencog-server.sh` script.
2. Start scheme as `guile -l run-r2l.scm`
3. At the scheme repl, enter `(nlp-parse "some complete sentence")`
4. :smile: Smile at the wondrous results.

## Algorithmic review
This section sketches out the mechanics of how the rules are applied.

The primary (sole) interface to the Relex2Logic module is the
`r2l-parse` function. It is given a single `SentenceNode`, and it
returns a `SetLink` holding all of the results from applying the R2L
rules.

The steps of the quick-start above unfold into the below:

1. The `nlp-parse` function, from the `chatbot` guile module, sends the
   plain-text to the RelEx server.

2. RelEx returns the parsed sentence attached to an `AnchorNode`, so that
   it can be easily found, and identified as being brand-new.

3. The `nlp-parse` function unties the newly parse sentence from the
   `AnchorNode`, and passes it to the `r2l-parse` function in this
   directory (in this scheme module).

4. The parsed sentence, here, is actually a `SentenceNode`.  By
   following the various links attached to it, all of the various
   words in the sentence, and the assorted depedency grammar markup
   can be found. See the
   [RelEx OpenCog format](http://wiki.opencog.org/w/RelEx_OpenCog_format)
   for documentation of this format.

5. The `r2l-parse` function invokes the Unified Rule Engine (URE) to
   apply the varous R2L rules to the parse. It does this by explicitly
   calling the `cog-fc` function that the URE provides, together with
   the name of the rulebase, and one giant `SetLink` holding all the
   various bits and pieces of the parse.

5.a. The `cog-fc` interface is such that the rules are applied only to
     the contents of the giant SetLink, and to no other part of the
     AtomSpace. The good news is that the application of the rules are
     limited in scope. The bad news is that a giant SetLink needs to be
     created.

5.b. Other good news with this interface is that individual R2L rules
     do not need to be told which sentence they are meant to be applied
     to. That is, the individual rules can omit having to link back to
     a specific `SentenceNode`. That makes the rules smaller.

6. The name of the rulebase is `R2L-en-RuleBase` and it is handed over
   to the forward chainer as `(ConceptNode "R2L-en-RuleBase")`.

7. There is little or no sophistication in the application of the rules.
   All of the rules are applied to all of the contents given to the
   chainer.  The chainer does not attempt to trim down the set of rules,
   before running them.  This is a potential performance bottleneck,
   but does not appear to be an issue at this time.

8. The result of calling `r2l-parse` is a list of `SetLink`s containing
   everything that the rules generated. There is one `SetLink` for each
   parse of the sentence.

9. This is handed back to the caller, `nlp-parse`, that does whatever
   it needs to do.


# Relex2Logic: Review of the Rules & Status Report
[A. Nitzkin](https://github.com/anitzkin), June 2015


### Overview:
Currently, the linguistic structures which R2L translates into logic
atoms can be divided into roughly three classes,

1. Predicate-argument structures (the main structure of any clause)
   such as SVO etc.,
2. A variety of types of constituents that can take the place of a
   predicate or its arguments, e.g. different parts of speech,
   variables, or clauses.
3. Features -- attributions of some property or modifying clause
   to an argument or predicate, such as an adjective, or a grammatical
   property, such as gender, or definiteness, or an adjectival or
   adverbial clause, such as “When I get to Memphis . . . .”

There are also, more-or-less, three kinds of rule, or aspects of
rule-structure (since some rules combine these approaches), which
roughly correspond to the three types of linguistic phenomena described
above, although this was never thought out ahead of time in that way;
it just sort-of coalesced.  The three kinds of rule are:

1. Rules which assign an entire predicate-argument structure at one
   time.
2. Rules which contain conditional branching in order to assign
   different types of arguments or predicates to a particular
   structural “slot”.
3. Rules which simply assign some property to a constituent, pretty
   much always using an InheritanceLink or ImplicationLink.

The currently evident 'issues' with Relex2Logic are also threefold
(why not, it's a magic number, right?):

1. A lot of redundancy (to be discussed in excruciating detail below).
2. RelEx output which is inadequate for the creation of 'good' R2L
   rules.
3. The question of whether R2L should be more or less specific in terms
   of its semantic classification of argument-roles and relationships.

I probably won't discuss (2) and (3) much in this document, except in
terms of why I didn't port certain rules from the old R2L, or have
replaced them with something else.  In general I agree with Ben's idea
of keeping R2L as general as possible, to leave room for accurate
semantic interpretation later; I just question it sometimes, because it
ends up meaning that R2L just becomes a translation of the RelEx output
into a slightly different language; in fact, it does more merging of
RelEx categories than differentiating; however, it also does some
important work in terms of hooking up certain relationships that are
merely listed in the RelEx output, that weren't explicitly marked
as being inter-related in RelEx.

### The Redundancy Problem
First I'm going to go over the sets of linguistic relationships that
play into the redundancy problem, with some comments on the issues with
each of them; then, by the time we get through that, the problems and
possibilities with the current rule set will be much clearer.

#### The eight main predicate-argument structures:

    1. Be-inheritance     – “Bill is the teacher.”
    2. Copula             – “When is the meeting?”

These two seem confused from the point of linguistics, but perhaps that
is just a matter of labeling, so OpenCog may not care.  Pretty much
anything that gets the be-inheritance rule in R2L is what most
grammarians would call a “copula”, as in the example above (“copula” is
said to be equivalent to equality: the “=” sign).  The stuff which does
get the “copula” relation instead instead of be-inheritance is pretty
arbitrary, which I am given to understand is simply because it was never
fully installed in RelEx.  I used the when-example above because I am
unable to find any declarative examples right now; you have to fish
around for them.  The R2L copula rule is triggered by anything that
gets a `_%copula()` relation from RelEx, and that doesn't seem to
happen all that much anymore, if it ever did. Anyway, at the moment,
both of these R2L rules call the same scheme-helper – which inherits
the complement to the subject, producing:
```
    (InheritanceLink (ConceptNode “Subject”) (ConceptNode “complement”))
```
As another issue, one might question whether copula-relations such as
that shown in (1) above are properly classified as cases of
“inheritance”, but the idea, as explained to me, is that “being the
teacher is a property of Bill.”  I think there may be an ambiguity being
abused here between the linguistic usage of “being a property” and the
logical meaning of being a property.

The other structures which seem closely related to these are:

    3. SP or “predadj” (predicative adjective) – “Bill is smart.”
    4. PREP or predicate preposition(al phrase) – “The book is on the table.”
    5. SV – subject-verb – “Bill sleeps.”

Linguistically, these make pretty good sense.  But when you go to the
scheme-helpers, you realize that not all of these distinctions are
necessary.  If you represent prepositions and adjectives as
PredicateNodes (which I did) then all three can use the same scheme
helper, producing:
```
    (EvaluationLink (PredicateNode “X”) (ListLink (ConceptNode “subject”))
```
where X could be a verb, a predicative adjective, or a preposition.

Another possibility is that SP and PREP could use the same rule as
be-inheritance, producing:
```
    (InheritanceLink (ConceptNode “subject”) (ConceptNode “Y”))
```
where `Y` could be a nominal complement ('be-inheritance'), a
predicative-adjective, or a preposition.

I didn't fully realize this at first, so right now, SV and SP both call
the SV scheme helper, and PREP goes to the SVO scheme helper, because
the existence of the `_pobj` (preposition object relation) made this seem
natural at the time.  But since `_pobj` also gets assigned by its own
rule, regardless of these rules, PREP structures could be handled by SV
or be-inheritance.

**This discussion is important, because if you continued with the old
design of R2L, then you need to write five different rules when you
combine these patterns with each of the question-types or other
sentence-types that require separate rules; so reducing the need for
five scheme helpers to two could save a lot of work and processing
time.**

Two more structures:

    6. SVO  –    “Bill ate a peach.”
    7. SVIO –    “Bill sent a bomb to the White House.”
                 “Bill sent the White House a bomb.”

The only thing to note here is that properly catching SVIO sentences is
tricky and always will be, because there are other sentences that look
exactly like them: for example, “Bill flew a plane to the White House” –
which is technically an “adverbial” phrase, not an Indirect Object. You
can tell the difference because you can say “Bill sent the White House a
bomb.” but you can't say “Bill flew the White House a plane.”

The difference may be important, because the “adverbial” modifies the
predicate, while the IO is a nuclear argument of the predicate — the
structure is fundamentally different (taking a lot of short-cuts with
the presentation of the code here ...):

Adverbial modification:
```
    (EvaluationLink
        (PredicateNode “to the white house”)
        (ListLink
            (EvaluationLink
                (PredicateNode “verb”)
                (ListLink (Subject) (Object)))))
```

Indirect Object:
```
    (EvaluationLink
        (PredicateNode “verb”)
        (ListLink (Subject) (Object) (“to the white house”)))
```

On the other hand, it's hard to think of how it would do the logic any
harm if you just classified all the to-indirect-objects as adverbial
modifiers . . . ?  Speaking form the POV of linguistic semantics, the
indirect object of a verb gets the thematic role “goal” or “recipient”
and, since locations are otherwise usually indicated by adverbial
phrases, it probably wouldn't hurt to forget entirely about the idea of
indirect object in the R2L output, and just treat them exactly like
adverbial modifiers (a predicate of which the modified verb is an
argument).

But anyway, at the moment there are two SVIO rules, one for “Bill sent
the White House a bomb” and another for “Bill sent a bomb to the White
House.” And they rely on relex not to mis-classify a “to” adverbial as
an indirect object.  I think it gets it right most of the time.  I used
to have a third rule to catch IO's which were mis-classified as
`_advmod`, but it doesn't seem necessary anymore. (Note:
mis-classifications should always be treated as bugs in LG or RelEx,
and should never be patched over in R2L. Just open a bug report.)

    8. TOBE – “Bill seems to be happy.”

This seems like an appropriate structure to have its own rule, although
there are a couple of things about it worth being aware of.  “to be
happy” here is known as an intensional complement, same as “Bill is
happy.”  The difference of course is that “seems” or similar verbs such
as “appears” or “looks” imply that Bill's being happy is not reality; it
“is” within the world of appearances.

My favorite theory for modeling the logic of this kind of thing is
Fauconnier and Turner's “mental spaces” but I am sure there are other
ways; see the discussion further below about `_rep()`.  Also note that
the “to be” can be elided in most cases: “Bill seems happy.”

To summarize the above . . . these are the eight main sentence types
that were recognized by R2L when I started, and there still are,
except that be-inheritance and copula now go to the same scheme
helper, SVO and PREP go to the same scheme helper, and SP and SV go to
the same scheme helper, and actually, as far as I can tell, PREP could
just as well go with the same scheme helper as SP, and both of them
could just as well use the be-inheritance rule, instead of SV . . .
Anyway, the current set-up works alright, questions about
inheritance-logic aside, but the reason to merge rules is that each of
these basic structures multiplies the number of rules you need to have
for each of the other categories of sentence-encompassing structure,
which are described below . . .

#### The “to-do” statements

Consider these sentences:

    - She wants to sing.
    - She wants you to sing.

The RelEx to-do rules seem to describe a sub-set of what are known as
“sentential complements” or “propositional attitudes.” In other words,
verbs which accept sentences, or verb-phrases with implicit subjects,
as their complements.  There were five “to-do” rules in the old R2L
rule-set. I ported three of them.  I didn't port the “be able to” rule,
because “able” is only one example of many predicates that could fit
in the same pattern (e.g. “ready”, “possible”, “glad”), which is also
the same pattern that appears in the other rule I didn't port, which
erroneously enshrines “must” as a key element of the pattern . . .
“must” is a modal verb, and all the modals could be handled by one rule
of their own.  Anyway, to summarize, the two “to-do”rules that I didn't
port are too specific, and there need to be two more rules to handle
those patterns instead — a rule for modals (can, must, will), and a
rule for this:

**A predadj-to-do rule**

For example,

    - She seems to be able to sing.
    - He intends to be ready to leave.
    - It must be possible to fix this.

It is also worth noting that these three sentences make a good example
of the redundancy issue that needs to be solved, in that one would
prefer not to have to write three rules for the above examples – a
“TOBE-predadj-todo” rule for the first one, a “SVO-predadj-to-do” rule
for the second one, and a “modal-predadj-todo” rule for the third one
(plus all the other combinations with other rules)!  There seems no
difficulty in doing it by assigning the two two-predicate relations
separately; I only didn't do it yet, because it needs to be done for
the whole rule-base or not at all, and because there are other more
tricky issues that might impact on how this is done (read on!)

To continue with the to-do rules . . . .The current set of “to-do”
rules seems arbitrary. They don't cover most of the eight basic
structures described above, nor many other two-predicate patterns that
we have no rules for, such as:

    - “I made her happy.”
    - “I consider him (to be) a fool.”
    - “I saw him running away.”

(see further on for a more complete list)

I didn't create all the necessary rules yet, because of the redundancy
problem. If I started writing rules to cover all combinations of the
eight basic structures with every 2-predicate pattern, plus
question-patterns, it would definitely extend into 100's of rules,
whereas there are probably no more than a dozen of these two-predicate
templates.  So, I started this discussion about doing the rules in a
different way.  Probably all of the two-predicate (“to-do”) rules can
be done simply – just assigning the second predicate as an argument of
the first, and leaving the other argument assignments for other rules
to fill in.  I didn't see this immediately because I was porting the
old R2L “to-do” rules which were all done as whole-sentence structures.

#### The “Which” Problem

However, another problem arises, which is that the whole-sentence
pattern combinations still multiply endlessly when combined with certain
question types in the current implementation, as follows:

**which-rules**

    - Which bomb did you send to the White House?
    - Which guy is smarter?
    - Which table is the book on?

**question-determiner-rules**

    - At what time will you arrive?
    - For what reason, would you do that?
    - In what way can we solve this problem?

Really, the “which” rules are also “question-determiners” but I needed
different names for the two sets.  The “question-determiners” are not
the problem, because they refer to constituents answered by adverbial
phrases (i.e.  how, when, where, and why).  There are many “which” rules
already, and there would need to be many many more, to hit all the
combinations possible with the eight basic patterns, multiplied by the
two-predicate patterns, multiplied by the choice of which argument is
getting the “which”.  Just a few examples:

    - Which girl told you to send a bomb to the White House?
    (which-subject + “to-do2” + “SVIO” pattern)

    - Which one do you want to make happy?
    (which-object + “to-do5” + a pattern not in R2L yet – make X Y)

    - Which color do you want the carpeting to be?
    (which-object + “be-inheritance” + “to-do2” pattern)

    etc. etc.

Under the current implementation, no “which”-type can be implemented
without the entire predicate-argument structure of the sentence, because
the whole sentence defines the set satisfying the variable referred to
by the question-word. That is, the SatisfyingSetLink requires the entire
definition to define the set.  An alternative would be to compose the
SatisfyingSetLink out of parts, using set intersection and set union.

### One Solution of Limited Value: Branching Scheme Helpers

This is probably a good place to bring the “what” / “who” questions into
the discussion, since “which” questions are very similar conceptually,
but treated very differently by R2L.  How, why, when, and where can wait
because they are very different.  I avoided making separate rules for
who and what questions, by making the scheme-helpers for the eight main
sentence-types branch conditionally, upon detection of variables as their
main arguments, e.g.:

```
(define (SV-rule subj_concept subj_instance verb verb_instance)
   (cond ((string=? subj_concept "_$qVar")
      (let ((var_name (choose-var-name)))
         (list
            (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
            (EvaluationLink
               (PredicateNode verb_instance)
               (ListLink (VariableNode var_name)))
         )
      ))
      ((string=? verb "_$qVar")
         (let ((var_name (choose-var-name)))
            (list
               (InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
               (EvaluationLink
                  (PredicateNode var_name)
                  (ListLink (ConceptNode subj_instance)))
            )
         )
      )
      (else (list
         (ImplicationLink
            (PredicateNode verb_instance) (PredicateNode verb))
         (InheritanceLink
            (ConceptNode subj_instance) (ConceptNode subj_concept))
         (EvaluationLink
            (PredicateNode verb_instance)
            (ListLink (ConceptNode subj_instance)))
      )
   ))
)
```

I haven't done this for “to-do” sentences yet, so they can't handle
“who” or “what” questions yet.

I'm not sure why “which”-questions require satisfying-set logic but
“what” and “who” questions don't.  This difference was defined before I
got here. It seems to me as if either both types require satisfying-set
logic, or none of them do.  This would seem to depend on what PLN and
the fuzzy pattern matcher do with those sentences.  If PLN and the fuzzy
pattern matcher can search for the “who” that “ate the pizza” without a
SatisfyingSetLink to tell them that, why can't they also search for the
“which guy” who “ate the pizza” in “which guy ate the pizza?”  I assume
there is a good reason for this, that I don't understand . . .
(There might not be a good reason. It might be an accident of history.)

Secondly, the branching rules really aren't much more efficient,
code-wise, than just having separate rules for each case. In fact,
th branching rules make it difficult for automating rule application.

### The “plug-n-play” solution

Much more efficient would be to insert the right constituents into
templates.  One rule (or perhaps, in some cases, 2-3 rules) for each
argument type (subject, object, and various phrasal or clausal
complements, and “who” and “what”),  and one rule for each
predicate-argument template, including two-predicate patterns.  I'm
pretty sure there are no more than two-dozen common templates in total.

A model for this is the way modifiers of all kinds are currently
processed — adverbials and adjectivals.   Adverbial phrases, for
example, can have many superficially different forms, including
complementized sentences, like “When I get there. . .”, “However
difficult it is . . .”, and “As much as I like you . . . ,”;
prepositional phrases, such as “In the best case scenario . . . “; and
simple adverbs, such as “quickly.”  In the current implementation, all
of these “adverbials”, and more, are just plugged into one “advmod
rule,” and their internal structures are handled by other rules.

To conclude about the “redundancy issue” . . .  the above discussion
pretty much is the whole story.  What I want to do now, before moving on
to talk about other aspects of R2L, is give you a list of all of the
patterns that seem to play into this issue.  In other words, you will be
completing some version of Relex2Logic that should probably cover all of
the patterns below, and preferably all of their combinations, which, if
you don't re-factor, would definitely require 100's of rules.


#### List of Main Predicate-Argument Patterns

I put in bold the patterns for which there are no rules at all yet in R2L

1. Mutually exclusive single-predicate patterns
    1. SV --- “He runs.”
    2. SVO --- “She eats pizza.”
    3. SVIO --- “She sent me a message.” / “She sent a message to me.”
    4. SP --- “Laura is happy.”
    5. PREP --- “Laura is on drugs.”
    6. copula --- “Laura is a teacher.” (in linguistics, not in Opencog)
    7. be-inheritance --- “Laura is a teacher.” (in Opencog)
    8. TOBE --- “Laura seems to be asleep.”
    9. ??? --- ** “Bob became a teacher.”**

2. Mutually exclusive multiple-predicate patterns
    1. SV+that-clause --- “Mary thinks that you are cute.”
    2. SV+infinitive clause --- “Mary wants to sing.”
    3. SVO+infinitive clause --- “Mary wants you to sing.”
    4. ** SVO+bare infinitive clause --- “Mary saw you come in.” **
    5. ** SVO+predicate-noun --- “Mary called you a fool.” **
    6. ** SVO+predicate-adjective --- “Mary called you stupid.” **
    7. ** SV+predadj --- “Mary wants to be happy.” **
    8. ** SVO+predadj --- “Mary wants you to be happy.” **
    9. ** SV+predprep --- “Mary got into the program.” **
    10. ** SVO+predprep --- “Mary got you into the program.” **
    11. ** S+predadj+other --- “Mary is happy to help her friends.” **
    12. ** SVO+-ing phrase --- “Mary saw him juggling cats.” **

3. Other predicate-argument pattern-types (not necessarily mutually exclusive)
    1. ** Sentential-clause subject + any predicate pattern
     --- “That you got in doesn't mean that you will get an A.” **
    2. ** S+modal+V --- “Bill can/may/will dance.” **
    3. ** Expletive subjects --- “There is a book on the table.” **
    4. Passives --- “The announcement was made (by Bob).” (not yet ported)
    5. Imperatives --- “Don't look now!” (not yet ported)

4. Questions (grouped by semantic patterns)
    1. Question word questions, except “which” (handled in branching scheme
       helpers so far)
        1. “who” / “what”
            - Who ate the pizza?
            - What did he do?
            - What did he eat?
        2. “when” / “where” / “how”-manner
            - When will you arrive?
            - Where is the meeting?
            - How does a computer work?
        3. “why”
            - Why can't you come?
        4. “how”-degree
            - How fast does it go?
            - How much does it cost?
            - How many books have you read?
    2. “which” questions
        1. “which/what” + noun = who / what
            - Which book is better?
            - Which man did you see?
    3. “question determiner” questions
        1. prep + “which/what” + noun = when / where / how-manner
            - At what/which time . . .
            - At what/which location . . .
            - In what/which way . . .
        2. prep + “which/what” + noun = why
            - For what reason . . .
        3. prep + “what/which” + noun = how-degree
            - to what/which extent . . .
    4. yes/no questions
        - Is this a question?

### Final Super Important Note about the “to-do” or “two-predicate” patterns

Most of these patterns represent some variation of a kind of logical
relationship mostly not yet recognized by R2L (except see discussion).
This is essential for reasoning correctly about these sentences.  Most
of these patterns might be called a “sentential complements”, among other
things.

Most sentential complements are either wholly or completely “opaque” to
logical inference, relative to the context “reality”; i.e., if “Mary
thinks that she is a wombat”, it does not imply that “Mary is a wombat.”
I have already incorporated logical opacity into R2L for “that-clauses”,
since the “that” becomes part of the R2L output; it is signaled in
RelEx by the `_rep()` relation (for 'representation').

This also applies to nearly all of the “to-do” patterns; for example,
“Mary wants to be a wombat” does not imply that “Mary is a wombat.”
I haven't incorporated logical-opacity into “to-do” patterns yet,
because, as I said, I stopped working on them when I realized that we
needed a discussion about how to re-factor the whole rule-base.

As far as logical-opacity goes, be aware that eventually, the system
needs to be able to infer that if “Mary thinks she is a wombat”, then
Mary is a wombat in Mary's mind.  The resolution of logical-opacity is
that the truth of the situations in the opaque clauses must be evaluated
relative to some context indicated by earlier words in the sentence, or
what Fauconnier would have called a “mental-space builder”.  Often the
context is the mind of the subject of the main verb, but also it could
be someone's speech, or some other “representation” (and perhaps other
things?):

    - “Mary called Bob a wombat.”
    (Bob is a wombat in Mary's verbal representation)

    - “Ben painted Mary as a wombat.”
    (Mary is a wombat in Ben's painting)

    - “Mary saw Bob as a wombat.”
    (Bob is a wombat in Mary's perception)

Note that the logic of opacity is probably most critical for dialogue,
as I discussed in my Dialogue Management design.  The application of
these contexts seems best left for a higher level of processing than
R2L, but as usual, I urge you all to consider what R2L needs to do, to
make this later processing possible and efficient, rather than waiting
for the problem to come up and then revising everything again . . . E.g.
is it sufficient that the “be” in “Mary is a wombat” be placed in the
out-going set of “Mary thinks”, or would it be wise to tag “Mary is a
wombat” in some other way, to ensure that PLN doesn't work on it in the
same way as predications which are part of “reality”?
