# Debug python agents
* A hacky sctipt to attach python agents to remote python debug server by gdb.
* It can be used in any of python code!

## Debug if you run agent in PyCharm
* Just debug program in PyCharm menu.

## Debug if you run agent in cogserver
* Prerequisites
  * gdb
  * PyCharm(My version is 4.5.1)
  * python2.7-dbg?, python2.7-dev?
  * expect (shell program)

* First, run python debug server in PyCharm.
* Config and run 10-debug-python-run-cogserver.sh in your shell.
  * Below code is only for mine, directly copy-n-paste works for me.
```bash
gdb

# Make debug environment for PyCharm.
set scheduler-locking off
set architecture i386:x86-64
handle SIGPWR nostop noprint
handle SIGXCPU nostop noprint

# Change the PyCharm's library file(attach_linux_amd64.so) if you using another OS.
set auto-solib-add on
set environment LD_PRELOAD /usr/lib/x86_64-linux-gnu/libdl.so:/usr/lib/x86_64-linux-gnu/libpython2.7.so.1.0:/home/kdm/pycharm/helpers/pydev/pydevd_attach_to_process/attach_linux_amd64.so

# Load cogserver program.
file cogserver
start -c ./opencog-kdm.conf

# PyCharm's debug attacher needs to python initialize before program load. So initialize in cogserver code should be ignored.
call Py_Initialize()

# Change the PyCharm's library file(attach_linux_amd64.so) if you using another OS.
call dlopen("/home/kdm/pycharm/helpers/pydev/pydevd_attach_to_process/attach_linux_amd64.so", 2)

# Try to attach remote debug server. DoAttach is wrap function to check environment.
# Parameters
# First: 0 gis normal, 1 is debug mode. (not work)
# Second: command. Wrap with GilStateEnsure & GilStateRelease functions.
# Third: 0 is normal, 1 is show debug info.
call DoAttach(0, "import sys;sys.path.append(\"/home/kdm/pycharm/helpers/pydev\");sys.path.append(\"/home/kdm/pycharm/helpers/pydev/pydevd_attach_to_process\");import attach_script;attach_script.attach(port=5678, host=\"127.0.0.1\");", 0)

# Wait few seconds to finish attach in python code...
shell sleep 2

# Continue cogserver process.
continue

# End or script. :D
```
* If you want to attach python debug server again after restart debug server, input below code.
```bash
call PyRun_SimpleString("attach_script.attach(port=5678, host=\"127.0.0.1\");")
```

## Dummy
* Why I can start debug normaly without below codes? (PyCharm load this code)
```bash
source /home/kdm/pycharm/helpers/pydev/pydevd_attach_to_process/linux/gdb_threads_settrace.py
call SetSysTraceFunc(1, 1)
```


