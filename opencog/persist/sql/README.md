                                Persist
                                -------
                   Linas Vepstas <linasvepstas@gmail.com>
                    Basic implementation Feb-June 2008
                         Status update May 2009
                         Status update Dec 2013

A simple implementation of atom persistence into SQL.  This allows not
only saving and restoring of the atomspace, but it also allows multiple
cogservers to share a common set of data.  That is, it implements a
basic form of a distributed atomspace.

Status
======
It works and has been used with databases containing millions of atoms,
accessed by cogservers that ran for months to perform computations. It
has scaled trouble-free, without any slowdown, up to four cogserrvers.
No one has tried anything larger than that, yet.

Features
--------
 * Save and restore of atoms, and several kinds of truth values.
 * Bulk save-and-restore of entire AtomSpace contents.
 * Incremental save/restore (i.e. updates the SQL contents as AtomSpace
   changes).
 * Generic API, useful for inter-server communications.

Missing features/ToDo items
---------------------------
 * Add support for multiple atom spaces.
 * Add full support for attention values. (??)
 * Provide optimized table layout for EvaluationLinks.
 * Add support for Space/TimeServer data.

Performance status
------------------
In March 2014, 10.3M atoms were loaded in about 20 minutes wall-clock
time, 10 minutes of opencog-server CPU time.  This works out to about
500K atoms/minute, or 9K atoms/second.  The resulting cogserver required
about 10GBytes of RAM, which works out to about 1KByte/atom average.
The loaded hypergraphs were all EvaluationLinks, viz:
```
   EvaluationLink  (w/non-trivial TV)
      SomeNode
      ListLink
         WordNode  (w/ non-trivial TV)
         WordNode  (w/ non-trivial TV)
```

The above measurements were made on a busy server that was doing many
other CPU & RAM intensive things; there was no performance tuning of
the postgress server.  A section below explains how to performance tune
the postgres server for better results.  The above was also done through
the scheme interface; since then, garbage collection has been tuned a
little bit, and so RAM usage should drop a bit.


Design Goals
============
The goal of this implementation is to:

1) Provide OpenCog with a working memory, so that the Cogsever could
   be stopped and restarted without requiring a data dump.  That is,
   checkpointing should be possible: if the cogserver crashes, data is
   not lost.  By using a database, a file format does not need to be
   invented. By using a database, data integrity is assured.
   By using incremental access, only those atoms that are needed get
   loaded into the atomspace; one does NOT have to do a bulk restore
   if one doesn't want to.

2) Provide an API for inter-server communications and atom exchange.
   Multiple cogservers can share data simply by sending atoms to,
   and retreiving atoms from the database.  Although this may not be
   the fastest way to send just single atoms, most algorithms do not
   need to send just single atoms: they just need to share some atoms,
   but its not clear which ones need to be shared.  Since all atoms are
   in the database, only the ones that are needed can be retreived.

3) Provide a baseline/reference implementation by which other
   persistence designs can be measured. It is hoped that other systems
   would be at least as fast and as scalable as this one is: this is
   meant to provide a minimal function and performance level. The
   strength of the current design is supposed to be simplicity, not
   scalability or raw performance.

4) A non-design-goal (at this time) is to build a system that can scale
   to more than 100 cogserver instances.  The current design might be
   able to scale to this many, but probably not much more.  Scaling
   larger than this would probably require a fundamental redesign of
   all of opencog, starting with the atomspace.

5) A non-design-goal is fully automatic save-restore of atoms.  Both
   the save and restore of atoms must be triggered by calls to the
   atomspace API.  The reason for this design point is that the
   atomspace is at too low a level to be fully automatic.  It cannot
   guess what the user really wants to do.  If it did guess, it would
   probably guess wrong: saving atoms before the user is done with them,
   saving atoms that get deleted microseconds later, fetching atoms
   that the user is completely disinterested in, clogging up RAM and
   wasting CPU time.  Some other layer, a higher level layer, needs to
   implement a policy for save/restore.  This layer only provides a
   mechanism.  It would be very very wrong to implement an automatic
   policy at this layer.


Current Design
==============
The core design defines only a few very simple SQL tables, and some
simple readers and writers to save and restore atoms from an SQL
database.

Note that the core design does *not* make use of object reflection,
nor can it store arbitrary kinds of objects. It is very definitely
hard-wired. Yes, this can be considered to be a short-coming.
A more general, persistent object framework (for C) can be found
at http://estron.alioth.debian.org/  However, simplicity, at the
cost of missing flexibility, seems more important.

The current design can save/restore individual atoms, and it can
bulk-save/bulk-restore the entire contents of an AtomTable.
A semi-realized goal of the prototype is to implement incremental save
and restore -- that is, to fetch atoms in a "just in time" fashion, and
to save away atoms that are not needed in RAM (e.g. atoms with
low/non-existent attention values). The AtomSpace BackingStore provides
the current, minimalistic, low-function API for this.

Features
--------
 * The AtomStorage class should be completely thread-safe (I think ...
only lightly tested).

 * This implementation avoids UUID collisions, and automatically thunks
UUID's as needed if an accidental collision with the database occurs.
In order to avoid excess thunking, it is strongly recommended that
the SQL database should be opened immediately on cogserver start, before
any atoms are created.  This will automatically reserve the range of
UUID's that are stored in the DB, and thus avoid collisions as new atoms
are created.

 * UUID's are reseved in blocks, or ranges. That is, UUID's are issued
in blocks of a million each.  This way, multiple different cogservers
can have some reasonable chance of using UUID's that do not collide with
one-another.  That is, UUID's can be "malloced" in ranges.  XXX Caution:
this mechanism may be broken or incomplete.  Ask Linas for the current
status.

 * This implementation automatically handles clashing atom types.  That
is, if the database is written with one set of atom types, and then the
cogserver is stopped, the atomtypes are all changed (with some added,
some deleted), then pulling from the database will automatically
translate and use the new atom types. (The deleted atom types will not
be removed from the database.  Restoring atoms with deleted atomtypes
will cause an exception to be thrown.)

 * Non-blocking atom store requests are implemented.  Four asynchronous
write-back threads are used, with hi/lo watermarks for queue management.
That is, if a user asks that an atom be stored, then the atom will be
queued for storage, and one of these threads will perform the actual
store at a later time. Meanwhile, the store request returns to the user
immediately, so that the user can continue without blocking.  The only
time that an atom store request blocks is if the queue is completely
full; in that case, the user will be blocked until the queue drains
below the low watermark. The number of connections can be raised by
editing the AtomStorage constructor, and recompiling.

The fire-n-forget algo is implemented in the C++
`AtomStorage::storeAtom()` method.  If the backlog of unwritten atoms
gets too large, the storeAtom() method may stall. Its currently designed
to stall if there's a backlog of 100 or more unwritten atoms.  This can
be changed by searching for `HIGH_WATER_MARK`, changing it and recompiling.

 * Reading always blocks: if the user asks for an atom, the call will
not return to the user until the atom is available.  At this time,
pre-fetch has not been implemented.  But that's because pre-fetch is
easy: the user can do it in thier own thread :-)


Issues
------
 * The TV merge issue. Right now, the AtomSpace/AtomTable is designed to
merge truth values between an atom that is added to the table, and any
atom that is already present in the table.  This can lead to unexpected
truth-value changes when atoms are being fetched from the backend.  The
correct solution is probably to change the table to not auto-merge. This
still leaves the question of what to do with the two TV's, since
different users are likely to want different behaviors.

 * The AV issue.  Two issues, here.  First, if the AV changes to a non
zero VLTI, the atom should be auto-saved to the backend.  This has not
been implemented.  Second, if the VLTI is positive, and the TV changes,
should this trigger an auto-save?  Conceptually, it probably should;
practically, it may hurt performance. At any rate, this can be handled
in a "policy" thread (a thread that implements some sort of policy) that
is independent of the storage mechanism here.  Again: we implement
mechanism, here, not policy.


Install, Setup and Usage HOWTO
==============================
There are many steps needed to install and use this. Sorry!

Compiling
---------
Download and install UnixODBC devel packages. Run cmake; make.
Do NOT use IODBC, it fails to support UTF-8.


Database Setup
--------------
There are four basic steps needed to setup the database: installation,
device driver setup, user and password setup, and table initialization.
Each step discussed below.


Database Install
----------------
Download and install Postgres version 8.3 or newer.  The current design
simply won't work with MySQL, because of a lack of array support.
Same holds true for SQLite.  Sorry. There is some work in the
code-base to support these other databases, but the work-arounds for
the missing features are kind-of complicated, and likely to be slow.

Be sure to install the postgres server, the postgres client, and the
odbc-postgresql device driver.


Device Driver Setup
-------------------
Configure the ODBC driver. After install, verify that /etc/odbcinst.ini
contains the stanza below (or something similar).  If it is missing,
then edit this file (as root) and add the stanza.  Notice that it uses
the Unicode drivers, and NOT the ANSI (ASCII) drivers.  Opencog uses
unicode!
```
    [PostgreSQL]
    Description = PostgreSQL ODBC driver (Unicode version)
    Driver      = psqlodbcw.so
    Setup       = libodbcpsqlS.so
    Debug       = 0
    CommLog     = 0
```
The above stanza associates the name `PostgreSQL ODBC driver (Unicode
version)`  with a particular driver. This name is needed for later
steps.  Notice that this is a really long name, with spaces!  You can
change the name, (e.g. to shorten it) if you wish, however, it **MUST**
be consistent with the name given in the `.odbc.ini` file (explained
below).

MySQL users need the stanza below; the `/etc/odbcinst.ini` file can
safely contain multiple stanzas defining other drivers.

WARNING: MySQL is currently not supported. Anyway, if you wanted to
mess with it, then add the below:
```
    [MySQL]
    Description = MySQL driver
    Driver      = libmyodbc.so
    Setup       = libodbcmyS.so
    CPTimeout   =
    CPReuse     =
```

Performance tweaks
------------------
The Postgres default configuration can be/should be tweaked for
performance.  Newer version of Postgress seem to be OK (??) but in
some cases, performance will be a disaster if the database is not tuned.

Edit `postgresql.conf` (a typical location is
`/etc/postgresql/8.4/main/postgresql.conf`) and make the changes below.
The first two changes are recommended by
http://wiki.postgresql.org/wiki/Tuning_Your_PostgreSQL_Server
```
   shared_buffers = default was 32MB, change to 25% of install RAM
   work_mem = default was 1MB change to 32MB
   effective_cache_size = default was 128MB, change to 50%-75% of installed RAM
   fsync = default on  change to off
   synchronous_commit = default on change to off
   wal_buffers = default 64kB change to 2MB or even 32MB
   commit_delay = default 0 change to 10000 (10K) microseconds
   checkpoint_segments = 32 (each one takes up 16MB disk space)
   ssl = off
   autovacuum = on
   track_counts = on
```
A large value for `wal_buffers` is needed because much of the database
traffic consists of updates.  Enabling vacuum is very important, for
the same reason; performance degrades substantially (by factors of
3x-10x) without regular vacuuming. (Newer versions of postrgres vacuum
automatically. YMMV.)

Restarting the server might lead to errors stating that max shared mem
usage has been exceeded. This can be fixed by telling the kernel to use
6.4 gigabytes (for example):
```
   vi /etc/sysctl.conf
   kernel.shmmax = 6440100100
```
save file contents, then:
```
   sysctl -p /etc/sysctl.conf
```
If you continue to get errors after that, read this for help fixing them:

   http://stackoverflow.com/questions/12616935/postgresql-shared-memory-settings


User and Database Setup
-----------------------
A database user needs to be created; the database tables need to be
created. The database user is NOT the same thing as a unix user:
the user login is for the database, not the OS. Do NOT use the same
login and password!

Multiple databases can be created.  In this example, the daatabase
name will be "mycogdata".  Change this as desired.

So, at the Unix command line:
```
   $ createdb mycogdata
```
If you get an error message `FATAL:  role "<user>" does not exist`, then
try doing this:
```
   $ su - postgres; createuser <your-unix-username>
```
Answer the question (yes, you want to be superuser) and exit. Under
rare circumstances, you may need to edit `pg_hba.conf`. Google for
additional help.

Next, create a database user named `opencog_user` with password `cheese`.
You can pick a different username and password, but it must be consistent
with the `~/.odbc.ini` file. Do NOT use your unix login password!  Pick
something else! Create the user at the shell prompt:
```
   $ psql -c "CREATE USER opencog_user WITH PASSWORD 'cheese'" -d mycogdata
```
Check that the above worked, by manually logging in:
```
   $  psql mycogdata -U opencog_user -W -h localhost
```
If you can't login, something up above failed.

Next, create the database tables:
```
   $ cat opencog/persist/sql/atom.sql | psql mycogdata -U opencog_user -W -h localhost
```
Verify that the tables were created. Login as before, then enter `\d`
at the postgres prompt.  You should see this:
```
    mycogdata=> \d
                  List of relations
     Schema |   Name    | Type  |     Owner
    --------+-----------+-------+----------------
     public | atoms     | table | opencog_user
     public | global    | table | opencog_user
     public | typecodes | table | opencog_user
    (3 rows)
```
If the above doesn't work, go back, and try again.

Verify that `opencog_user` has write permissions to the tables. Do this
entering the below.
```
    mycogdata=> INSERT INTO TypeCodes (type, typename) VALUES (97, 'SemanticRelationNode');
```
You should see the appropriate respone:
```
    INSERT 0 1
```
If the above doesn't work, go back, and try again.


ODBC Setup, Part the Second
---------------------------
Edit `~/.odbc.ini` in your home directory to add a stanza of the form
below. You MUST create one of these for EACH repository you plan to
use! The name of the stanza, and of the database, can be whatever you
wish. The name given for `Driver`, here `PostgreSQL`, **must** match
a stanza name in `/etc/odbcinst.ini`.  Failure to have it match will
cause an error message:

```
Can't perform SQLConnect rc=-1(0) [unixODBC][Driver Manager]Data source name not found, and no default driver specified
```

Note that below specifies a username and a password. These should NOT
be your regular unix username or password!  Make up something else,
something completely different! These two will be the username and the
password that you use when connecting from the cogserver, with the
`sql-open` command (below).

Pay special attention to the name given for the `Database`.  This should
correspond to a database created with postgres. In the examples above,
it was `mycogdata`.

```
   [mycogdata]
   Description       = My Favorite Database
   Driver            = PostgreSQL
   Trace             = No
   TraceFile         =
   Database          = mycogdata
   Servername        = localhost
   Port              = 5432
   Username          = opencog_user
   Password          = cheese
   ReadOnly          = No
   RowVersioning     = No
   ShowSystemTables  = Yes
   ShowOidColumn     = Yes
   FakeOidIndex      = Yes
   ConnSettings      =
```

Opencog Setup
-------------
Edit `lib/opencog.conf` and set the `STORAGE`, `STORAGE_USERNAME` and
`STORAGE_PASSWD` there to the same values as in `~/.odbc.ini`.  Better
yet, copy lib/opencog.conf to your build directory, edit the copy, and
start the opencog server as:

```
   $ ./opencog/server/cogserver -c my.conf
```

Verify that everything works. Start the cogserver, and bulk-save:

```
   $ telnet localhost 17001
   opencog> sql-store
```

Some output should be printed in the cogserver window:

```
   Max UUID is 278
   Finished storing 277 atoms total.
```

The typical nuber of atoms stored will differ from this.

You don't have to put the database credentials into the `opencog.conf`
file.  In this case, the database would need to be opened manually,
using the `sql-open` command:

```
   $ telnet localhost 17001
   `opencog> sql-open mycogdata opencog_user cheese
```

Notice that the user-name and the password are the same as that given
in the `~/.odbc.ini` file.  These are NOT (and should not be) your unix
username and password!


How To Pass the Unit Test
=========================
There are two unit tests for this API, called BasicSaveUTest and
PersistUTest.  The CMakefile will not run compile or run these tests
unless postgres is configured just-so.  To run and past these tests,
you should:

 * Read tests/persist/sql/README and follow the instructions there.
   They are almost identical to the instructions above, except that
   they call for a test user to be set up. You can do that, or you
   can use your current DB user.
 * To compile and run:
```
    $ make test
```

So here's a super-short recap:

 * Create a user 'opencog_tester' with password 'cheese'.
 * Create a new database: e.g.  `$ createdb test-persist`
 * Create the tables: `$ cat atom.sql | psql test-persist`
 * Create an entry in `~/.odbc.ini`, as explained above.  The entry
   should be called `opencot_test`, and use `opencog_tester` as the user.
 * The file `lib/opencog-test.conf` already has the above as the default
   username and database names.  Stick to these.


After the above steps, `BasicSaveUTest` and `PersistUTest` should run
and pass.


Unit Test Status
----------------
* As of 2011-04-29 bzr revision 5314, BasicSaveUTest (still) passes.
* As of 2013-11-26 BasicSaveUTest works and passes, again.
* As of 2014-06-19 both unit tests work and pass.


Using the System
================
Some example walkthroughs of some typical usage scenarios.

Bulk Save and Restore
---------------------
At last! bulk save of atoms that were previous created is done by
getting to the opencog prompt (`telnet localhost 17001`) and issuing the
commands:
```
    opencog> ?
    Available commands:
      exit help list scm shutdown sql-open sql-close sql-store sql-load
    opencog> sql-open
    sql-open: invalid command syntax
    Usage: sql-open <dbname> <username> <auth>

    opencog> sql-open mycogdata opencog_user cheese
    Opened "mycogdata" as user "opencog_user"

    opencog> sql-store
    SQL data store thread started
```
At this point, a progres indicator will be printed by the opencog
server, on the OpenCog server's stdout. It  will say something like:
Stored 236000 atoms. When finished, its nice to say:
```
    opencog> sql-close
```
At this point, the cogserver can be stopped and restarted.  After a
restart, load the data with:
```
    opencog> sql-open mycogdata opencog_user cheese
    opencog> sql-load
    SQL loader thread started
```
The completion message will be on the server output, for example:
```
    Finished loading 973300 atoms in total
```

Individual-atom save and restore
--------------------------------
There is an interface to save and restore individual atoms. It may be
used as follows:

Start the server:
```
    $ opencog/server/cogserver -c ../lib/opencog.conf
```
Open a shell:
```
    $ telnet localhost 17001
    Trying 127.0.0.1...
    opencog> sql-open mycogdata opencog_user cheese
    Database opened
    opencog> scm
    Entering scheme shell; use ^D or a single . on a line by itself to exit.
    guile> (define x (ConceptNode "asdfasdf" (stv 0.123 0.789)))
    guile> (store-atom x)
    (ConceptNode "asdfasdf" (stv 0.123 0.78899997))
```
The above will have caused a single atom to be stored in the database.
It's presence can be verified by examining the database directly:
```
    $ psql mycogdata
    Welcome to psql 8.3.6, the PostgreSQL interactive terminal.
    mycogdata=# select * from atoms;
     uuid | type | tv_type | stv_mean | stv_confidence | stv_count | height |   name   | outgoing
    ------+------+---------+----------+----------------+-----------+--------+----------+----------
        2 |    3 |       1 |    0.123 |     0.78899997 | 2991.4688 |      0 | asdfasdf |
    (1 row)
```
The backing-store mechanism can now automatically retrieve this atom at
a later time.  Thus, for example, shut down the server, restart it,
re-open the database, and enter the scheme shell again. Then,
```
    guile> (define y (ConceptNode "asdfasdf"))
    guile> y
    (ConceptNode "asdfasdf" (stv 0.123 0.78899997))
```
Note that, this time, when the node was created, the database was
searched for a ConceptNode with the same string name. If it was found
in the database, then it is automatically loaded into the AtomSpace,
with the appropriate truth value taken from the database.

If the truth value is modified, the atom is *not* automatically saved
to the database.  The modified atom would need to be explicitly saved
again. If you need this to happen, you probably should design a thread
whose only job it is is to handle truth-value changes. Note: this could
be slow.

Once stored, the atom may be deleted from the AtomSpace; it will
remain in storage, and can be recreated at will:
```
    guile> (cog-purge y)
    #t
    guile> y
    #<Invalid handle>
    guile> (define y (ConceptNode "asdfasdf"))
    guile> y
    (ConceptNode "asdfasdf" (stv 0.123 0.78899997))
```
Notice, in the above, that the truth value is the same as it was before.
That is because the truth value was fetched from the database when the
atom is recreated.

The UUID associated with the atom will NOT change between purges
and server restarts. This can be verified with the cog-handle command,
which returns the UUID:
```
    guile> (cog-handle y)
    2
```

Purging vs. Deletion
--------------------
There are four related but distinct concepts of atom deletion: purge and
delete, each of which may also be done recursively. "Purge" with remove
the atom from the atomspace; it will NOT remove it from the database!
"Delete" will remove the atom from both the atomspace, and from the
database; thus, deletion is permanent!  (Well, if there are two
cogservers attached to one database, and one cogserver deletes the atom
from the database, it will still not be deleted from the second
cogserver, and so that second cogserver could still save that atom.
There is currently no way to broadcast a distributed system-wide
delete message. Its not even obvious that such a broadcast is evn a good
idea.)

Atoms can also be deleted or purged recusrively. Thus, normally, a
purge/delete will succeed only if the atom has no incoming set.
The recursive forms, `cog-purge-recursive` and `cog-delete-recursive`
purge/delete the atom and avery link that contains that atom, and so
on.


Using SQL Persistence from Scheme
---------------------------------
SQL fetch from store is not automatic!  That is because this layer 
cannot guess the user's intentions. There is no way for this layer
to guess which atoms might be deleted in a few milliseconds from now,
or to guess which atoms need to loaded into RAM (do you really want ALL
of them to be loaded ??)  Some higher level management and policy thread
will need to make the fetch and store decisions. This layer only
implements the raw capability: it does not implement the policy. 

There is one thing that is automatic: when a new atom is added to the
atomspace, then the database is automatically searched, to see if this
atom already exists.  If it does, then its truth value is restored.
However, **AND THIS IS IMPRTANT**, the incoming set of the atom is not
automatically fecthed from storage.  This must be done manually, using
either C++:
```
   AtomSpace::getImpl()->fetchIncomingSet(h, true);
```
or the scheme call
```
  (fetch-incoming-set atom)
```

Since UUID's are assigned uniquely, they can be used for out-of-band
communication of atoms between cogservers.  Thus, if by some magic, the
numeric uuid for an atom is known, but the atom itself is not, then the
scheme call
```
   (fetch-atom (cog-atom uuid))
```
can be used to pull an atom into the AtomSpace from the database server.
(You can also do this from C++, by creating a `Handle` with that UUID in
it, and then forcing resolution of the handle).

To force an atom to be saved, call:
```
   (store-atom atom)
```

Copying Databases
-----------------
Sooner or later you will want to make a copy of your database, for
backup purposes, or sharing. Here's the current 'best practices' for
that:
```
   $ pg_dump mycogdata -f filename.sql
```
That's all!



Experimental Diary & Results
============================
Diary entries from June 2008

Store performance
-----------------
This section reviews the performance for storage of data from opencog
to the SQL server (and thence to disk).  Performed in 2008, on a typical
Intel desktop that was new in 2004. Viz. under two GhZ, and 4GB RAM.

First run with a large data set (save of 1564K atoms to the database)
was a disaster.  Huge CPU usage, with 75% of CPU usage occurring in the
kernel block i/o layer, and 12% each for the opencog and postgres times:
```
   112:00 [md4_raid1] or 4.3 millisecs per atom
   17 minutes for postgres, and opencog, each. or 0.66 millisecs per atom
   1937576 - 1088032 kB = 850MBytes disk use
```
Experiment: is this due to the bad design for trying to figure whether
"INSERT" or "UPDATE" should be used? A local client-side cache of the
keys in the DB seems to change little:
```
   CPU usage for postgres 11:04  and opencog 10:40 and 112:30 for md
```
So this change drops postgres server and opencog CPU usage
significantly, but the insane kernel CPU usage remains.

The above disaster can be attributed to bad defaults for the postgres
server. In particular, sync to disk, while critical for commercial
database use, is pointless for current use. Also, buffer sizes are much
too small. Edit postgresql.conf and make the following changes:
```
   shared_buffers = default was 24MB, change to 384MB
   work_mem = default was 1MB change to 32MB
   fsync = default on  change to off
   synchronous_commit = default on change to off
   wal_buffers = default 64kB change to 512kB
   commit_delay = default 0 change to 10000 (10K) microseconds
   ssl = default true change to false
```
Restarting the server might lead to errors stating that max shared mem
usage has been exceeded. This can be fixed by:
```
   vi /etc/sysctl.conf
   kernel.shmmax = 440100100
(save file contents, then)
   sysctl -p /etc/sysctl.conf
```
After tuning, save of data to empty DB gives result:
```
   cogserver = 10:45 mins = 0.41  millisecs/atom (2.42K atoms/sec)
   postgres  =  7:32 mins = 0.29  millisecs/atom (2.65K atoms/sec)
   md        =  0:42 mins = 0.026 millisecs/atom (37K atoms/sec)
```
Try again, dropping the indexes on the atom and edge tables. Then,
after loading all atoms, rebuild the index. This time, get
```
   cogserver = 5:49 mins = 0.227 millisecs/atom (4.40K atoms/sec)
   postgres  = 4:50 mins = 0.189 millisecs/atom (5.30K atoms/sec)
```
Try again, this time with in-line outgoing sets. This improves
performance even further:
```
   cogserver = 2:54 mm:ss = 0.113 millisecs/atom (8.83K atoms/sec)
   postgres  = 2:22 mm:ss = 0.092 millisecs/atom (10.82K atoms/sec)
```
Try again, compiled with -O3, storing to an empty table, with
no indexes on it (and with in-line outgoing sets):
```
   cogserver = 2:40 mm:ss
   postgres  = 2:16 mm:ss
```
Try again, compiled with -O3, storing to empty table, while holding
the index on tables (and with in-line outgoing sets).
```
   cogserver = 2:51 mm:ss
   postgres  = 2:06 mm:ss
```
Apparently, the problem with the indexes has to do with holding them
for the edge table; when there's no edge table, then there's no index
issue!?

Try again, compiled with -O3, saving to (updating) a *full* database.
(i.e. database already has the data, so we are doing UPDATE not INSERT)
```
   cogserver = 2:19 mm:ss
   postgres  = 4:35 mm:ss
```
Try again, using UnixODBC instead of iODBC, to empty table, withOUT
index tables (and -O3 and in-lined outgoing):
```
   cogserver = 2:36 mm:ss
   postgres  = 2:13 mm:ss
```
It appears that UnixODBC is essentially identical to iODBC performance
```
begin; commit;
use analyze;
use prepare;
```

XML loading performance
-----------------------
Loading the dataset from XML files takes:
```
   cogserver 2:34 mm:ss when compiled without optimization
   cogserver 1:19 mm:ss when compiled with -O3 optimization
```

Loading performance
-------------------
Loading performance. Database contains 1564K Atoms, and 2413K edges.
CPU usage:
2:08 postgres = 82 microsecs/atom (12.2K atoms/sec)
similar to for opencog, but then AtomTable needs to reconcile, which
takes an additional 8:30 minutes to attach incoming handles!!

Conclude: database loading would be much faster if we loaded all of
the atoms first, then all of the lowest-height links, etc.  This assumes
that opencog is strictly hierarchically structured. (no "crazy loops")

After implementing height-structured restore, get following, loading
from a "hot" postgres instance (had not been stopped since previous
use, i.e. data should have been hot in RAM):
```
  cogserver = 2:36 mm:ss = 0.101 millisecs/atom (9.85K atoms/sec)
  postgres  = 1:59 mm:ss = 0.077 millisecs/atom (12.91K atoms/sec)
```
The dataset had 357162 Nodes, 1206544 Links at height 1

After a cold start, have
```
  cogserver = 2:32 mm:ss
  postgres  = 1:55 mm:ss
```
Appears that there is no performance degradation for cold-starts.

Note also: cogServer CPU usage is *identical* to its CPU usage when
loading XML! Hurrah! Also, see below: RAM usage is significantly
reduced; apparently, the reading of XML results in very bad memory
fragmentation.

Implement in-line edges, instead of storing edges in an outboard table.
```
  cogserver = 41 seconds = 26.7 microsecs/atom (37.5K atoms/sec)
  postgres  =  7 seconds =  4.56 microsecs/atom (219K atoms/sec)
```
Turn on -O3 optimization during compile ... all previous figures
were without *any* optimization. Now get
```
  cogserver = 24 seconds = 15.6 microsecs/atom (64.0K atoms/sec)
  postgres  = 11 seconds
```
Much much better!
```
10.78
23.15
```


TODO
====
 * See also to-do list way up top.

 * Store attention values. (??) maybe ??

 * Create custom table, tailored for EvaluationLink triples.
   Since majority of nodes/links in the DB will be in the form of
   EvaluationLink triples, a significant performance gain can be gotten
   by creating a custom table, and shunting queries to that.  This would
   decrease the SQL table sizes significantly, and decrease server I/O
   by factors of 2x-3x.  Another table, designed just for simple pairs,
   might help a lot, too.

 * Add support for multiple atomspaces.
