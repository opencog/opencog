Guile usage examples
--------------------

See also opencog/guile/README or http://wiki.opencog.org/w/Scheme
for additional documentation.

If you have installed opencog, then start guile simply by saying
"guile" at the bash prompt $.  Otherwise, you will need to do this:
```
$ export LTDL_LIBRARY_PATH=build/opencog/guile:build/opencog/query
$ guile -L opencog/scm -L build
```
where "build" is where-ever you built opencog. Alternately, just
start plain guile, without the -L flags, and then say:

```
(add-to-load-path "/home/yourname/opencog/build")
(add-to-load-path "/home/yourname/opencog/opencog/scm")
```
Another possibility: add the above to your `~/.guile` file.  That way,
they'll be run every time you start.

Then say:
```
(use-modules (opencog))
```

After this, you can create atoms "as usual" e.g.
```
(ConceptNode "asdf")
```

You can load other scm files (for example, "utilities.scm") by saying:

```
(load-from-path "utilities.scm")
```

Some, but not all functions have documentation, which can be viewed by
saying `,describe` and the name of the function.  Note the comma, and no
parents.  For example:
```
,describe cog-chase-link
```
Additional help by saying 
```
,apropos cog
```
and more gnerally
```
,help
```
