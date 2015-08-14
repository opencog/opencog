
Design overview
===============
This is a pre-prototype.  But still:

* The `face_track` directory contains code for visual servoing: it
  receives ROS messages about human face locations from the webcam
  + pi_vision subsystem.  It calls methods in `face_atomic.py` to
  poke face-ids (ID numbers) into the atomspace.


Running
=======

* At the bash prompt:
```
export LD_LIBRARY_PATH=/usr/local/lib/opencog/modules
```
Failure to do this will result in the message:
```
ERROR: In procedure dynamic-link: file: "libguile-cogserver", message: "file not found"
```

* Start guile, then from th guile prompt:
```
(use-modules (opencog cogserver))
(start-cogserver "../scripts/opencog.conf")
```
Then load the python code:
```
echo -e "py\n" | cat - atomic.py |netcat localhost 17001
```
Then back at the guile prompt:
```
(load-from-path "eva-fsm.scm")
```


Debugging notes
===============

Cython modules are installed here:
```
/usr/local/share/opencog/python/opencog
```

Get to the cogserver prompt like this:
```
rlwrap telnet localhost 17001
```
and then enter the python interpreter by saying `py`, or the scheme
interpreter by saying `scm`.

From the python prompt, the following should list the python
opencog modules:
```
help('opencog')
```

Face tracking debug
===================
Print all visible faces in the atomspace:

```
rlwrap telnet localhost 17001
(cog-incoming-set (PredicateNode "visible face"))
```
