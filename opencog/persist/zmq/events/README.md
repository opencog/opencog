Overview
========

The AtomSpacePublisherModule class publishes AtomSpace change events
across the network using [ZeroMQ sockets](http://zeromq.org) to allow for external
clients to receive updates from the AtomSpace via a publish/subscribe pattern.

Clients can subscribe to the events by subscribing to a ZeroMQ socket.

##### Supported events:

*   **add**        (Atom added)
*   **remove**     (Atom removed)
*   **tvChanged**  (Atom TruthValue changed)
*   **avChanged**  (Atom AttentionValue changed)
*   **addAF**      (Atom was added to the AttentionalFocus)
*   **removeAF**   (Atom was removed from the AttentionalFocus)

The message is a JSON-formatted string.

##### Potential usage examples:

- Debugging
- Monitoring the dynamics of the AttentionAllocation system as it propagates STI and LTI throughout the network and the AttentionalFocus changes
- Monitoring PLN
- Allowing external modules such as the AtomSpace Visualizer to subscribe to real-time updates

Configuration
=============

Prerequisites
-------------

#### ZeroMQ

http://zeromq.org/intro:get-the-software

Requires ZeroMQ version 3.2.4 (libzmq3-dev) or higher.

#### Threaded Building Blocks (Intel TBB)

https://www.threadingbuildingblocks.org/download

These will be installed automatically if you are using [ocpkg](https://github.com/opencog/ocpkg).

Module
------

This is implemented as *opencog/persist/zmq/events/libatomspacepublishermodule.so* which must be enabled in the *opencog.conf* file.

When loaded, it will be enabled by default. The module supports the following CogServer commands:

- **publisher-disable-signals** Disconnects the publisher from AtomSpace signals
- **publisher-enable-signals** Connects the publisher to AtomSpace signals

Parameters
----------

The default configuration parameters in opencog.conf are:

`  ZMQ_EVENT_USE_PUBLIC_IP = TRUE`

`  ZMQ_EVENT_PORT = 5563`

### ZMQ\_EVENT\_USE\_PUBLIC\_IP

If set to TRUE, this parameter will allow ZeroMQ to publish AtomSpace
events to the public internet. If set to FALSE, ZeroMQ will publish
events to localhost.

### ZMQ\_EVENT\_PORT

This is the port that ZeroMQ will use to publish AtomSpace events.

Message format
==============

Atom object
-----------

##### Format

    {
            "atom": {
	        "handle": UUID,
	        "type": TYPENAME,
	        "name": NAME,
                "attentionvalue": ATTENTIONVALUETYPE,
	        "truthvalue": {
	            "type": TRUTHVALUETYPE,
	            "details": TRUTHVALUEDETAILS 
	        },
	        "outgoing": "[ UUID1, UUID2 ... ]",
	        "incoming": "[ UUID1, UUID2 ... ]"
	    }
	}


#### UUID

integer

#### TYPENAME

*Atom type, see [OpenCog Atom types](http://wiki.opencog.org/w/OpenCog_Atom_types)*

string (choose from the list of available atom types)

#### NAME

string

#### ATTENTIONVALUETYPE

##### STI

integer

##### LTI

integer

##### VLTI

boolean

#### TRUTHVALUETYPE

Valid types are:

-   simple
-   count
-   indefinite

*see [TruthValue](http://wiki.opencog.org/w/TruthValue)*

#### TRUTHVALUEDETAILS

The format of the TRUTHVALUEDETAILS object depends on the value of the
TRUTHVALUETYPE parameter.

#### simple

    {
	    'count': TRUTHVALUECOUNT,
	    'confidence': TRUTHVALUECONFIDENCE,
	    'strength': TRUTHVALUESTRENGTH
	}

###### TRUTHVALUECOUNT
float

###### TRUTHVALUECONFIDENCE
float

###### TRUTHVALUESTRENGTH
float

#### count

    {
	    'count': TRUTHVALUECOUNT,
	    'confidence': TRUTHVALUECONFIDENCE,
	    'strength': TRUTHVALUESTRENGTH
	}

###### TRUTHVALUECOUNT
*(see above)*

###### TRUTHVALUECONFIDENCE
*(see above)*

###### TRUTHVALUESTRENGTH
*(see above)*

#### indefinite

	{
	    'strength': TRUTHVALUESTRENGTH,
	    'L': TRUTHVALUEL,
	    'U': TRUTHVALUEU,
	    'confidence': TRUTHVALUECONFIDENCE,
	    'diff': TRUTHVALUEDIFF,
	    'symmetric': TRUTHVALUESYMMETRIC
	}

###### TRUTHVALUECONFIDENCE
*(see above)*

###### TRUTHVALUESTRENGTH
*(see above)*

###### TRUTHVALUEL

###### TRUTHVALUEU

###### TRUTHVALUEDIFF

###### TRUTHVALUESYMMETRIC

Event types
===========

The AtomSpace change event publisher binds to Boost Signals2 signals generated 
by the AtomSpace class. The signals are processed and serialized using a 
multithreaded task scheduler implemented using Intel TBB.

##### Timestamp
Each of the following event types contains a **timestamp** field, which provides 
a timestamp of the atomspace event expressed as a UTC UNIX timestamp of seconds 
since epoch.

**The following event types are available:**

add
---

Triggered whenever an atom is added.

ZeroMQ subscription channel name: **add**

##### Format

    {
        "atom": ATOM,
        "timestamp": TIMESTAMP
    }

remove
------

Triggered whenever an atom is removed.

ZeroMQ subscription channel name: **remove**

##### Format

    {
        "atom": ATOM,
        "timestamp": TIMESTAMP
    }

avChanged
---------

Triggered whenever the AttentionValue of an atom is changed.

ZeroMQ subscription channel name: **avChanged**

##### Format

    {
        "handle": HANDLE,
        "avOld": ATTENTIONVALUETYPE,
        "avNew": ATTENTIONVALUETYPE,
        "atom": ATOM,
        "timestamp": TIMESTAMP
    }

tvChanged
---------

Triggered whenever the TruthValue of an atom is changed.

ZeroMQ subscription channel name: **tvChanged**

##### Format

    {
        "handle": HANDLE,
        "tvOld": TRUTHVALUETYPE,
        "tvNew": TRUTHVALUETYPE,
        "atom": ATOM,
        "timestamp": TIMESTAMP
    }

addAF
---------

Triggered whenever the AttentionValue of an atom changes, if the new STI value
is within the AttentionalFocus boundary and the old STI value is not.

ZeroMQ subscription channel name: **addAF**

##### Format

    {
        "handle": HANDLE,
        "avOld": ATTENTIONVALUETYPE,
        "avNew": ATTENTIONVALUETYPE,
        "atom": ATOM,
        "timestamp": TIMESTAMP
    }

removeAF
---------

Triggered whenever the AttentionValue of an atom changes, if the old STI value
is within the AttentionalFocus boundary and the new STI value is not.

ZeroMQ subscription channel name: **removeAF**

##### Format

    {
        "handle": HANDLE,
        "avOld": ATTENTIONVALUETYPE,
        "avNew": ATTENTIONVALUETYPE,
        "atom": ATOM,
        "timestamp": TIMESTAMP
    }

Example clients
===============

Python
------

There is an example client, as well as clients for monitoring and logging, available here:

<https://github.com/opencog/external-tools/tree/master/AtomSpaceSubscriber>

C++
---

C++ examples of how to subscribe to events and parse the JSON can be found in the unit tests, located here:

<https://github.com/opencog/opencog/blob/master/tests/persist/zmq/events/AtomSpacePublisherModuleUTest.cxxtest#L146>

JavaScript
----------

A server exists to forward ZeroMQ messages to socket.io sockets to allow
JavaScript web applications to subscribe to AtomSpace events, even
across domains.

To enable this server, follow the instructions in this file:

<https://github.com/opencog/opencog/blob/master/opencog/python/web/socketio/atomspace_publisher.py>

An example client application is available in this file:

<https://github.com/opencog/opencog/blob/master/opencog/python/web/socketio/index.html>

***

##### Todo

- There is a need in the AtomSpace for a new Boost Signals2 signal type,
AttentionalFocusBoundaryChanged. It should contain the old boundary and the new
boundary. Without that, there can be a case where the AttentionalFocusBoundary
changes, and as a result the publisher does not publish all of the addAF and
removeAF events that should have occured, because those events are currently
only triggered by the AVChangedSignal.

##### Benchmarks

You can perform benchmarking on this module by loading the libbenchmark.so module and libattention.so modules.
That will allow you to compare throughput with the publisher enabled vs. disabled.

March 18, 2014:

Test system: i7-4930K 3.4GHz (12 core) w/ 32GB RAM

Number of threads allocated to AtomSpace add operations: 2

###### Wall-clock time
Time taken to build a graph of 2000 nodes fully connected by 3,998,000 bidirectional directed edges
- Publisher disabled:  61 seconds (aggregate links/sec: 65541)
- Publisher enabled:   47 seconds (aggregate links/sec: 85064)
- Previous deprecated version: 175 seconds (aggregate links/sec: 22846)

Time taken to load this Scheme file: https://github.com/opencog/test-datasets/blob/master/conceptnet/conceptnet4.scm
- Publisher disabled:  23 seconds
- Publisher enabled:   35 seconds
- Previous deprecated version: 286 seconds

For historical purposes, the performance of the previous version was also tested, and is labeled 'Previous deprecated version' in the results.

Benchmark procedure:
```
./opencog/server/cogserver
> benchmark-fully-connected reset
> benchmark-fully-connected concurrent 2000 2
> shutdown
./opencog/server/cogserver
> publisher-disable-signals
> benchmark-fully-connected reset
> benchmark-fully-connected concurrent 2000 2
> shutdown
```
