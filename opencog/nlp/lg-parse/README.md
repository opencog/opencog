
Link Grammar parsing
--------------------

Perform a Link Grammar parse of a sentence, and insert the results into
the AtomSpace.  This is compatible with the LG subset of the RelEx
parse.

==LgParseLink
Parsing is performed by calling the `execute()` method on the C++ class
LgParseLink, or equivalently, calling the scheme function `cog-execute!`
on it.  The LgParseLink is a kind of FunctionLink, and can thus be used
in any expression that FunctionLinks can be used with. For example, it
can be used (with variables) within BindLinks and PutLinks.

The expected format of an LgParseLink is:

    LgParseLink
        PhraseNode "this is a test."
        LgDictNode "en"
        NumberNode  6   -- optional, number of parses.

When executed, the result of parsing the phrase text, using the
specified dictionary, is placed in the atomspace.  Execution
returns a Sentencenode pointing at the parse results.  If the third,
optional NumberNode is present, then that will be the number of
parses that are captured. If the NumberNode is not present, it
defaults to four.

==Notes
This is a minimalist API to the Link Grammar parser, attempting to
live off the default options that Link Grammar provides.  At this time,
it is not envisioned that there will ever be any need to expand beyond
this minimalist API.

==Pros and Cons
Since this provides a format compatible with the RelEx parse server,
this means that there are two ways of getting parsed text into the
atomspace: using this link, or using the RelEx server.  There are
competing pros and cons of doing it each way:

* The RelEx server is a network server, and can be run on any
  network-connected machine.

* The RelEx server generates scheme strings, which must be parsed by
  the scheme interpreter in OpenCog. This adds a lot of overhead, and
  can be slow.

* The RelEx server supports a much larger set of LG options, and can
  thus be made to do unusual things, whatever that might be.
