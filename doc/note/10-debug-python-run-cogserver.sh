#!/usr/bin/expect -f

### Settings

# Set your server's path, config's path
set custom_server_path cogserver
set custom_server_config ./opencog-kdm.conf

# Set your pycharm's path.
set pycharm_install_path /home/kdm/pycharm

# Set python remote debug server's address.
set pycharm_debug_address 127.0.0.1
set pycharm_debug_port 5678

# Change the PyCharm's library file if you using another OS.
# Currently OpenCog is only fun with linux 64-bit, right?
# attach_linux_amd64.so, attach_linux_x86.so,
# attach_amd64.dll, attach_x86.dll,
# attach_x86.dylib, attach_x86_64.dylib is available.
set pycharm_debug_attach_lib_name attach_linux_amd64.so

### End of settings

### Start script
set pycharm_debug_lib_path ${pycharm_install_path}/helpers/pydev
set pycharm_debug_attach_lib_path ${pycharm_debug_lib_path}/pydevd_attach_to_process
set pycharm_debug_attach_lib ${pycharm_debug_attach_lib_path}/${pycharm_debug_attach_lib_name}

set timeout 60

spawn gdb

# Make debug environment for PyCharm.
send "set scheduler-locking off\r"
send "set architecture i386:x86-64\r"
send "handle SIGPWR nostop noprint\r"
send "handle SIGXCPU nostop noprint\r"


send "set auto-solib-add on\r"
send "set environment LD_PRELOAD /usr/lib/x86_64-linux-gnu/libdl.so:/usr/lib/x86_64-linux-gnu/libpython2.7.so.1.0:$pycharm_debug_attach_lib\r"

# Load cogserver program.
send "file $custom_server_path\r"
send "start -c $custom_server_config\r"

# PyCharm's debug attacher needs to python initialize before program load. So initialize in cogserver code should be ignored.
send "call Py_Initialize()\r"

# Change the PyCharm's library file(attach_linux_amd64.so) if you using another OS.
send "call dlopen(\"$pycharm_debug_attach_lib_path\", 2)\r"

# Try to attach remote debug server. DoAttach is wrap function to check environment.
# Parameters
# First: 0 is normal, 1 is debug mode. (not work)
# Second: command. Wrap with GilStateEnsure & GilStateRelease functions.
# Third: 0 is normal, 1 is show debug info.
send "call DoAttach(0, \"import sys;sys.path.append(\\\"$pycharm_debug_lib_path\\\");sys.path.append(\\\"$pycharm_debug_attach_lib_path\\\");import attach_script;attach_script.attach(port=$pycharm_debug_port, host=\\\"$pycharm_debug_address\\\");\", 0)\r"

# Wait few seconds to finish attach in python code...
send "shell sleep 2\r"

# Continue cogserver process.
send "continue\r"

interact
### End or script. :D
