
SQLite-based dictionary
-----------------------

The SQLite-based dictionary storage is meant to provide a simple
machine-readable interface to the Link Grammar dictionaries.

Traditionally, the Link Grammar dictionaries are stored in text files
that are hand-edited. This is fine for manual maintenance of the
dictionary entries, but is a stumbling block when it is desired that
the dictionary be maintained in an automated fashion, e.g. by
language-learning tools.  The SQLite-based dictionary provides this
automation interface.

Database format
---------------
The current interface remains a hybrid: the affix, regex and
post-processing data will remain in text files, at least until the
system is more fully developed. (Or possibly forever, depending on
future plans and outcomes).

The current design for the entries (rows) of the SQL table is that
they should hold one disjunct only, each.  Thus, the `or` keyword
is not supported in the database entries. You should use multiple
rows if you want to `or` them together.

Also not supported: the multi-connector at-sign `@` and the optional
connector braces `{}`.  Again, these should be expanded into multi-row
entries.  It would be fairly easy to add this support, but this misses
the point: the SQL database is NOT supposed to be "just like the text
files, but different". Its really meant to support a different way of
doing data management. The at-sign and the braces can be thought of as
a form of compression, a way of making the dictionary entries more
human-readable, more human-friendly. By contrast, the SQL backend is
meant to be machine-readable, and machine-friendly, so that it can be
more easily updated by machine-learning algorithms.

Existing demo:
--------------
Run the existing demo:
```
    $ link-parser demo-sql
```
This can parse the following sentences: "this is a test", "this is
another test", "this is a dog", "this is a cat". All other sentences
will fail to parse (intentionally so).

Creating a demo dictionary:
---------------------------
Use the following commands, modified as desired:
```
mkdir data/foo
cp data/demo-sql/4.0.* data/foo
cat dict.sql |sqlite3 data/foo/dict.db
cat demo.sql |sqlite3 data/foo/dict.db
link-parser foo
```
The above should result in a dictionary that can parse the same sentences
as the demo database.
