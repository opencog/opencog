
ECAN/PLN experimentation
-----------------------
The purposes of this experiment are:

- Test HebbianLink creation between co-occuring atoms

- STI propagation works properly both along HebbianLinks and atoms inside the Attentional Focus

- Implementing Forgetting of atoms based on a threshold of memory for the atomspace

Once we are satisfied with the behaviour of the ECAN system, we might extend the experiment
to see how interaction with other opencog agents works out.


Code organization
-----------------

ExperimentSetupModule- A module that registers commands used for this particular experiments.The commands are *ecan-load,
                       ecan-start, ecan-pause, start-word-stimulator, pause-word-stimulator, stimulate-atoms, dump-tseries,
                       and load-word-dict*. Please type help COMMAND to get usage of these commands.

ArtificialStimulatorAgent - Used for fake stimulation of atoms. Since stimulation can only be provided via agents, this Agent
                            used to give artificial stimulation to atoms when the command stimulate-atoms is isssued.

SentenceGenStimulateAgent - Used for the experiment described [here](http://wiki.opencog.org/wikihome/index.php/Attention_Allocation#Ideas_for_simple_ECAN_tests)

visualization/ - Assorted scripts for plotting and processing dumped data.                            

Running the experiment
----------------------

**Step 1** - Get the ecan-experiment branch and build

 1. cd to opencog directory
 
 2. git remote add  misgana  https://github.com/misgeatgit/opencog
 
 3. git fetch misgana
 
 4. git checkout --track  misgana/ecan-wbench     
      
 5. go to you build directory  ( mkdir build && cd build)
 
 6. make 


**Step 2** -  Start cogserver and load and start surprisingness mind agent

 1. start cogserver by typing (**./opencog/server/cogserver**) without leaving the build directory
 
 2. open another terminal  and type  **telnet localhost 17001** OR preferably **rlwrap telnet localhost 17001** if you have rlwrap installed
 
 
**Step 3** - Running the Simple experiment
 
 1. type ecan-load followed by ecan-start to start all ECAN agents
 
 2. Load the word-dict.conf file found in data dir by typing the command load-word-dict ABS_PATH_TO_word-dict.conf 
 
 3. Run start-word-stimulator command to start WordNode and WordNodeInstances creation and stimulation

 4. After letting it run for a while, type dump-tseries ABS_PATH_TO/visualization/dump
    will create files named dump-sw.data and dump-nsw.data (sw for special words and nsw for non-special words).

 5. Plot the graphs for both speical and non spcial words by using the following commands *./plot.sh [sw|nsw] [sti|lti|vlti]*
  



 

TODO
----

 
    
