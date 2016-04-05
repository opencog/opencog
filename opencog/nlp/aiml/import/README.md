### Usage

`perl aiml2oc.pl --dir ./aimldir --final aiml-in-atomese.scm`

or

`perl aiml2oc.pl --overwrite --dir ./aimldir --final aiml-in-atomese.scm`

where, `./aimldir` contains aiml files & `aiml-in-atomese.scm` is the output
containing the atomese representation of aiml rules in scheme.

For details run:

`perl aiml2oc.pl --help`

For aiml files check https://github.com/jstnhuang/chatbot/tree/master/aiml

### Notes
1. One option is during pass 2 to use the derived path to over write the final
   output on a last-in-only-out basis. Of course to do so it loses the
   one-to-one-to-one tracing since it goes through a hash table to capture
   duplicates. But you can leave it off "--overwrite" for debugging, and turn it
   on for final output.

2. There is probably a better name for the "--overwrite" switch since it's
   really not really overwriting any output but the categories as they come in.
   I had it as "--merge" but its not merging the categories but overwriting
   them. You could have a merge but that's kind of what it does by default,
   just write out duplicates and let OpenCog sort it out. Maybe "--uselast" ?
   Then you could have a "--usefirst" and "--userandom" as a conflicting
   definition strategies. Or "--useLM" to prefer the one category that best
   matches some language model ( naughty, nice, polite, ...)


### Overview of script
It is a simple two-pass script.

* Pass One : XML to an intermediate neutral, word based format.
  Linear sequence of tokens annotated with their meaning, and optionally any
  performative code. The AIML 2.0 interpreter almost has something similar with
  its AIMLIF csv based format. I would just normalize that format. Of course
  the format could handled totally in RAM.

* Pass Two: From the linear format into Atomese.
  The only issue of not using an interpreter in generating the intermediate file is one AIML convention allows "last definition loaded is definitive" semantics. Most systems will sort and load the files in alphabetical order, allowing one to "overlay/overwrite" previous definitions. The OpenCog interpreter would have to decide how it wants to handle duplicate definitions. Associated with this is the concept of a "graph path" which would normally be the list of nodes from the root to a leaf in the graph (just like a path in a file system). Each path defines a unique condition for which one or more responses were specified. You could turn each path into its own atom and use that for indexing and editing purposes. You also would define the conflict resolution policy for duplicate paths.

  One possible reason for having an intermediate format might be handling of
  <set> tags in patterns. These could either be passed on to OpenCog as either
  a pointer to a collection concept, the collection list itself, a collection
  concept with its definition, or various ways of unrolling the defined set in
  intermediate format using duplicate definitions.

  Working on Pass One can be independent of Pass Two since translating from the
  intermediate format to Atomese should be straightforward once the Atomese
  format is defined. You could edit the intermediate format but that would be
  like editing assembly; definitely doable but probably not what most would
  consider fun. You could also post the fragments to a database which would
  allow for search and filter based editing, but that might be energy better
  spent on a general Atomspace editor/IDE .
