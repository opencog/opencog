Guile usage examples
--------------------

See also opencog/guile/README or http://wiki.opencog.org/w/Scheme
for additional documentation.

Before starting guile, you have to set up your runtime library search
paths. You can do this in one of three ways:
1) install opencog
2) enter the below by hand
3) copy the below to your `~/.bashrc` file.

Option 3 is recommended.
```
OC_PATH=/home/yourname/opencog
export LTDL_LIBRARY_PATH=$OC_PATH/build/opencog/guile:$OC_PATH/build/opencog/query:$OC_PATH/build/opencog/nlp/lg-dict:$OC_PATH/build/opencog/nlp/sureal
```

Simlarly, you need to tell guile where to find the opencog modules.
The best way to do this is to add the below to your `~/.guile` file.
These will then run every time you start guile.
```
(add-to-load-path "/home/yourname/opencog/build")
(add-to-load-path "/home/yourname/opencog/opencog/scm")
(add-to-load-path ".")
```
Finally, start guile:
```
$ guile
```

You may want to copy the below to your `~/.guile` file as well, or you
can copy them manually to the guile interpreter prompt:
```
(use-modules (ice-9 readline))
(activate-readline)
(use-modules (opencog))
```

After the opencog module is loaded, you can create atoms "as usual" e.g.
```
(ConceptNode "asdf")
```

You can load other scm files (for example, "utilities.scm") by saying:

```
(load-from-path "utilities.scm")
```

Some, but not all functions have documentation, which can be viewed by
saying `,describe` and the name of the function.  Note the comma, and no
parentheses.  For example:
```
,describe cog-chase-link
```
Additional help can be gotten by saying
```
,apropos cog
```
and more generally
```
,help
```

#List of modules
The current list of modules that wrap C++ code includes:
```
(use-modules (opencog))
(use-modules (opencog query))
(use-modules (opencog nlp lg-dict))
(use-modules (opencog nlp sureal))
(use-modules (opencog persist))
(use-modules (opencog persist-sql))
```
