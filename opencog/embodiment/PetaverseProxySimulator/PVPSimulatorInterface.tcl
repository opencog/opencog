
#!/usr/bin/wish -f
# Program: PVPSimulatorInterface
# Tcl version: 8.4 (Tcl/Tk/XF)
# Tk version: 8.4
# XF version: 4.0
#

# module inclusion
global env
global xfLoadPath
global xfLoadInfo
set xfLoadInfo 0
if {[info exists env(XF_LOAD_PATH)]} {
  if {[string first $env(XF_LOAD_PATH) /home/sven/xf] == -1} {
    set xfLoadPath $env(XF_LOAD_PATH):/home/sven/xf
  } {
    set xfLoadPath /home/sven/xf
  }
} {
  set xfLoadPath /home/sven/xf
}

global argc
global argv
set tmpArgv ""
for {set counter 0} {$counter < $argc} {incr counter 1} {
  case [string tolower [lindex $argv $counter]] in {
    {-xfloadpath} {
      incr counter 1
      set xfLoadPath "[lindex $argv $counter]:$xfLoadPath"
    }
    {-xfstartup} {
      incr counter 1
      source [lindex $argv $counter]
    }
    {-xfbindfile} {
      incr counter 1
      set env(XF_BIND_FILE) "[lindex $argv $counter]"
    }
    {-xfcolorfile} {
      incr counter 1
      set env(XF_COLOR_FILE) "[lindex $argv $counter]"
    }
    {-xfcursorfile} {
      incr counter 1
      set env(XF_CURSOR_FILE) "[lindex $argv $counter]"
    }
    {-xffontfile} {
      incr counter 1
      set env(XF_FONT_FILE) "[lindex $argv $counter]"
    }
    {-xfmodelmono} {
      tk colormodel . monochrome
    }
    {-xfmodelcolor} {
      tk colormodel . color
    }
    {-xfloading} {
      set xfLoadInfo 1
    }
    {-xfnoloading} {
      set xfLoadInfo 0
    }
    {default} {
      lappend tmpArgv [lindex $argv $counter]
    }
  }
}
set argv $tmpArgv
set argc [llength $tmpArgv]
unset counter
unset tmpArgv

# procedure to show window .topGetAvatarId
proc ShowWindow.topGetAvatarId { args} {# xf ignore me 7

  # build widget .topGetAvatarId
  if {"[info procs XFEdit]" != ""} {
    catch "XFDestroy .topGetAvatarId"
  } {
    catch "destroy .topGetAvatarId"
  }
  toplevel .topGetAvatarId  \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # Window manager configurations
  wm positionfrom .topGetAvatarId "" 
  wm sizefrom .topGetAvatarId ""
  wm maxsize .topGetAvatarId 1265 770
  wm minsize .topGetAvatarId 1 1
  wm protocol .topGetAvatarId WM_DELETE_WINDOW {XFProcError {Application windows can not be destroyed.
Please use the "Current widget path:" to show/hide windows.}}
  wm title .topGetAvatarId {Dialog Box}


  # build widget .topGetAvatarId.frame1
  frame .topGetAvatarId.frame1 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .topGetAvatarId.frame1.frame3
  frame .topGetAvatarId.frame1.frame3 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .topGetAvatarId.frame1.frame3.label4
  label .topGetAvatarId.frame1.frame3.label4 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -borderwidth {0} \
    -highlightbackground {#eeeff2} \
    -text {(AvatarID posX posY)} \
    -textvariable {avatarGetIdInputLabel}

 # build widget .topGetAvatarId.frame1.frame3.entry6
  entry .topGetAvatarId.frame1.frame3.entry6 \
    -background {#eeeff2} \
    -highlightbackground {#ffffff} \
    -selectbackground {#7579ba} \
    -selectforeground {#ffffff} \
    -textvariable {newAvatarId} \
    -width {38}
  # bindings
  bind .topGetAvatarId.frame1.frame3.entry6 <Key-Return> {.topGetAvatarId.frame2.frame8.button9 invoke}


  # build widget .topGetAvatarId.frame2
  frame .topGetAvatarId.frame2 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .topGetAvatarId.frame2.frame8
  frame .topGetAvatarId.frame2.frame8 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .topGetAvatarId.frame2.frame8.button9
  button .topGetAvatarId.frame2.frame8.button9 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {OK} \
    -command {createAvatarOrPet} \
    -width {9}

  # build widget .topGetAvatarId.frame2.frame8.button10
  button .topGetAvatarId.frame2.frame8.button10 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {Cancel} \
    -command {DestroyWindow.topGetAvatarId} \
    -width {9}

  # pack master .topGetAvatarId.frame1
  pack configure .topGetAvatarId.frame1.frame3

  # pack master .topGetAvatarId.frame1.frame3
  pack configure .topGetAvatarId.frame1.frame3.label4 \
    -side left
  pack configure .topGetAvatarId.frame1.frame3.entry6 \
    -side left

  # pack master .topGetAvatarId.frame2
  pack configure .topGetAvatarId.frame2.frame8 \
    -anchor n

  # pack master .topGetAvatarId.frame2.frame8
  pack configure .topGetAvatarId.frame2.frame8.button9 \
    -side left
  pack configure .topGetAvatarId.frame2.frame8.button10 \
    -padx 20 \
    -side left

  # pack master .topGetAvatarId
  pack configure .topGetAvatarId.frame1 \
    -padx 57 \
    -pady 26
  pack configure .topGetAvatarId.frame2

  if {"[info procs XFEdit]" != ""} {
    catch "XFMiscBindWidgetTree .topGetAvatarId"
    after 2 "catch {XFEditSetShowWindows}"
  }
}

proc DestroyWindow.topGetAvatarId {} {# xf ignore me 7
  if {"[info procs XFEdit]" != ""} {
    if {"[info commands .topGetAvatarId]" != ""} {
      global xfShowWindow.topGetAvatarId
      set xfShowWindow.topGetAvatarId 0
      XFEditSetPath .
      after 2 "XFSaveAsProc .topGetAvatarId; XFEditSetShowWindows"
    }
  } {
    catch "destroy .topGetAvatarId"
    update
  }
}



# procedure to show window .topActionParameters
proc ShowWindow.topActionParameters {args} {# xf ignore me 7

  # build widget .topActionParameters
  if {"[info procs XFEdit]" != ""} {
    catch "XFDestroy .topActionParameters"
  } {
    catch "destroy .topActionParameters"
  }
  toplevel .topActionParameters  \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # Window manager configurations
  wm positionfrom .topActionParameters ""
  wm sizefrom .topActionParameters ""
  wm maxsize .topActionParameters 1265 770
  wm minsize .topActionParameters 1 1
  wm protocol .topActionParameters WM_DELETE_WINDOW {XFProcError {Application windows can not be destroyed.
Please use the "Current widget path:" to show/hide windows.}}
  wm title .topActionParameters {Avatar's action parameters}


  # build widget .topActionParameters.frame1
  frame .topActionParameters.frame1 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .topActionParameters.frame1.frame3
  frame .topActionParameters.frame1.frame3 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .topActionParameters.frame1.frame3.label4
  label .topActionParameters.frame1.frame3.label4 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -borderwidth {0} \
    -highlightbackground {#eeeff2} \
    -text {ANTICIPATE_PLAY} \
    -textvariable {avatarActionName}

  # build widget .topActionParameters.frame1.frame3.label5
  label .topActionParameters.frame1.frame3.label5 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -borderwidth {0} \
    -highlightbackground {#eeeff2} \
    -text {(}

  # build widget .topActionParameters.frame1.frame3.entry6
  entry .topActionParameters.frame1.frame3.entry6 \
    -background {#eeeff2} \
    -highlightbackground {#ffffff} \
    -selectbackground {#7579ba} \
    -selectforeground {#ffffff} \
    -textvariable {avatarActionParameters} \
    -width {38}
  # bindings
  bind .topActionParameters.frame1.frame3.entry6 <Key-Return> {.topActionParameters.frame2.frame8.button9 invoke}


  # build widget .topActionParameters.frame1.frame3.label7
  label .topActionParameters.frame1.frame3.label7 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {)}

  # build widget .topActionParameters.frame1.frame11
  frame .topActionParameters.frame1.frame11 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .topActionParameters.frame1.frame11.label12
  label .topActionParameters.frame1.frame11.label12 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -foreground {#999999} \
    -highlightbackground {#eeeff2} \
    -text {ANTICIPATE_PLAY()} \
    -textvariable {avatarActionSyntaxExample}

  # build widget .topActionParameters.frame2
  frame .topActionParameters.frame2 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .topActionParameters.frame2.frame8
  frame .topActionParameters.frame2.frame8 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .topActionParameters.frame2.frame8.button9
  button .topActionParameters.frame2.frame8.button9 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {OK} \
    -command {sendAvatarAction} \
    -width {9}

  # build widget .topActionParameters.frame2.frame8.button10
  button .topActionParameters.frame2.frame8.button10 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {Cancel} \
    -command {DestroyWindow.topActionParameters} \
    -width {9}

  # pack master .topActionParameters.frame1
  pack configure .topActionParameters.frame1.frame3
  pack configure .topActionParameters.frame1.frame11 \
    -anchor w

  # pack master .topActionParameters.frame1.frame3
  pack configure .topActionParameters.frame1.frame3.label4 \
    -side left
  pack configure .topActionParameters.frame1.frame3.label5 \
    -side left
  pack configure .topActionParameters.frame1.frame3.entry6 \
    -side left
  pack configure .topActionParameters.frame1.frame3.label7 \
    -side left

  # pack master .topActionParameters.frame1.frame11
  pack configure .topActionParameters.frame1.frame11.label12

  # pack master .topActionParameters.frame2
  pack configure .topActionParameters.frame2.frame8 \
    -anchor n

  # pack master .topActionParameters.frame2.frame8
  pack configure .topActionParameters.frame2.frame8.button9 \
    -side left
  pack configure .topActionParameters.frame2.frame8.button10 \
    -padx 20 \
    -side left

  # pack master .topActionParameters
  pack configure .topActionParameters.frame1 \
    -padx 57 \
    -pady 26
  pack configure .topActionParameters.frame2

  if {"[info procs XFEdit]" != ""} {
    catch "XFMiscBindWidgetTree .topActionParameters"
    after 2 "catch {XFEditSetShowWindows}"
  }
}

proc DestroyWindow.topActionParameters {} {# xf ignore me 7
  if {"[info procs XFEdit]" != ""} {
    if {"[info commands .topActionParameters]" != ""} {
      global xfShowWindow.topActionParameters
      set xfShowWindow.topActionParameters 0
      XFEditSetPath .
      after 2 "XFSaveAsProc .topActionParameters; XFEditSetShowWindows"
    }
  } {
    catch "destroy .topActionParameters"
    update
  }
}


# procedure to show window .
proc ShowWindow. {args} {# xf ignore me 7

  # Window manager configurations
  wm positionfrom . user
  wm sizefrom . program
  wm geometry . 1094x706
  wm maxsize . 1265 770
  wm minsize . 1 1
  wm protocol . WM_DELETE_WINDOW {XFProcError {Application windows can not be destroyed.
Please use the "Current widget path:" to show/hide windows.}}
  wm title . {PVPSimulatorInterface.tcl}


  # build widget .frame4
  frame .frame4 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame4.frame2
  frame .frame4.frame2 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0
  frame .frame4.frame2.frame0 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame9
  frame .frame4.frame2.frame0.frame9 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame9.frame0
  frame .frame4.frame2.frame0.frame9.frame0 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame9.frame0.button1
  button .frame4.frame2.frame0.frame9.frame0.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton0} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {ANTICIPATE_PLAY} \
    -textvariable {avatarButton0}

  # build widget .frame4.frame2.frame0.frame9.frame1
  frame .frame4.frame2.frame0.frame9.frame1 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame9.frame1.button1
  button .frame4.frame2.frame0.frame9.frame1.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton1} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {BARE_TEETH} \
    -textvariable {avatarButton1}

  # build widget .frame4.frame2.frame0.frame9.frame2
  frame .frame4.frame2.frame0.frame9.frame2 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame9.frame2.button1
  button .frame4.frame2.frame0.frame9.frame2.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton2} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {BARK} \
    -textvariable {avatarButton2}

  # build widget .frame4.frame2.frame0.frame9.frame3
  frame .frame4.frame2.frame0.frame9.frame3 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame9.frame3.button1
  button .frame4.frame2.frame0.frame9.frame3.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton3} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {BEG} \
    -textvariable {avatarButton3}

  # build widget .frame4.frame2.frame0.frame9.frame4
  frame .frame4.frame2.frame0.frame9.frame4 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame9.frame4.button1
  button .frame4.frame2.frame0.frame9.frame4.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton4} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {BLINK} \
    -textvariable {avatarButton4}

  # build widget .frame4.frame2.frame0.frame9.frame5
  frame .frame4.frame2.frame0.frame9.frame5 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame9.frame5.button1
  button .frame4.frame2.frame0.frame9.frame5.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton5} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {CHEW} \
    -textvariable {avatarButton5}

  # build widget .frame4.frame2.frame0.frame9.frame6
  frame .frame4.frame2.frame0.frame9.frame6 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame9.frame6.button1
  button .frame4.frame2.frame0.frame9.frame6.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton6} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {DIG} \
    -textvariable {avatarButton6}

  # build widget .frame4.frame2.frame0.frame9.frame7
  frame .frame4.frame2.frame0.frame9.frame7 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame9.frame7.button1
  button .frame4.frame2.frame0.frame9.frame7.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton7} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {DREAM} \
    -textvariable {avatarButton7}

  # build widget .frame4.frame2.frame0.frame9.frame8
  frame .frame4.frame2.frame0.frame9.frame8 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame9.frame8.button1
  button .frame4.frame2.frame0.frame9.frame8.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton8} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {DRINK} \
    -textvariable {avatarButton8}

  # build widget .frame4.frame2.frame0.frame9.frame9
  frame .frame4.frame2.frame0.frame9.frame9 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame9.frame9.button1
  button .frame4.frame2.frame0.frame9.frame9.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton9} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {DROP} \
    -textvariable {avatarButton9}

  # build widget .frame4.frame2.frame0.frame9.frame10
  frame .frame4.frame2.frame0.frame9.frame10 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame9.frame10.button1
  button .frame4.frame2.frame0.frame9.frame10.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton10} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {EARS_BACK} \
    -textvariable {avatarButton10}

  # build widget .frame4.frame2.frame0.frame11
  frame .frame4.frame2.frame0.frame11 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame11.frame0
  frame .frame4.frame2.frame0.frame11.frame0 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame11.frame0.button1
  button .frame4.frame2.frame0.frame11.frame0.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton11} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {EARS_PERK} \
    -textvariable {avatarButton11}

  # build widget .frame4.frame2.frame0.frame11.frame1
  frame .frame4.frame2.frame0.frame11.frame1 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame11.frame1.button1
  button .frame4.frame2.frame0.frame11.frame1.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton12} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {EARS_TWITCH} \
    -textvariable {avatarButton12}

  # build widget .frame4.frame2.frame0.frame11.frame2
  frame .frame4.frame2.frame0.frame11.frame2 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame11.frame2.button1
  button .frame4.frame2.frame0.frame11.frame2.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton13} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {EAT} \
    -textvariable {avatarButton13}

  # build widget .frame4.frame2.frame0.frame11.frame3
  frame .frame4.frame2.frame0.frame11.frame3 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame11.frame3.button1
  button .frame4.frame2.frame0.frame11.frame3.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton14} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {FLY} \
    -textvariable {avatarButton14}

  # build widget .frame4.frame2.frame0.frame11.frame4
  frame .frame4.frame2.frame0.frame11.frame4 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame11.frame4.button1
  button .frame4.frame2.frame0.frame11.frame4.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton15} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {FOLLOW} \
    -textvariable {avatarButton15}

  # build widget .frame4.frame2.frame0.frame11.frame5
  frame .frame4.frame2.frame0.frame11.frame5 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame11.frame5.button1
  button .frame4.frame2.frame0.frame11.frame5.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton16} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {GRAB} \
    -textvariable {avatarButton16}

  # build widget .frame4.frame2.frame0.frame11.frame6
  frame .frame4.frame2.frame0.frame11.frame6 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame11.frame6.button1
  button .frame4.frame2.frame0.frame11.frame6.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton17} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {GROWL} \
    -textvariable {avatarButton17}

  # build widget .frame4.frame2.frame0.frame11.frame7
  frame .frame4.frame2.frame0.frame11.frame7 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame11.frame7.button1
  button .frame4.frame2.frame0.frame11.frame7.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton18} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {HEEL} \
    -textvariable {avatarButton18}

  # build widget .frame4.frame2.frame0.frame11.frame8
  frame .frame4.frame2.frame0.frame11.frame8 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame11.frame8.button1
  button .frame4.frame2.frame0.frame11.frame8.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton19} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {HOWL} \
    -textvariable {avatarButton19}

  # build widget .frame4.frame2.frame0.frame11.frame9
  frame .frame4.frame2.frame0.frame11.frame9 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame11.frame9.button1
  button .frame4.frame2.frame0.frame11.frame9.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton20} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {JUMP_TOWARD} \
    -textvariable {avatarButton20}

  # build widget .frame4.frame2.frame0.frame11.frame10
  frame .frame4.frame2.frame0.frame11.frame10 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame11.frame10.button1
  button .frame4.frame2.frame0.frame11.frame10.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton21} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {JUMP_UP} \
    -textvariable {avatarButton21}

  # build widget .frame4.frame2.frame0.frame12
  frame .frame4.frame2.frame0.frame12 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame12.frame0
  frame .frame4.frame2.frame0.frame12.frame0 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame12.frame0.button1
  button .frame4.frame2.frame0.frame12.frame0.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton22} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {LICK} \
    -textvariable {avatarButton22}

  # build widget .frame4.frame2.frame0.frame12.frame1
  frame .frame4.frame2.frame0.frame12.frame1 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame12.frame1.button1
  button .frame4.frame2.frame0.frame12.frame1.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton23} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {LIE_DOWN} \
    -textvariable {avatarButton23}

  # build widget .frame4.frame2.frame0.frame12.frame2
  frame .frame4.frame2.frame0.frame12.frame2 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame12.frame2.button1
  button .frame4.frame2.frame0.frame12.frame2.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton24} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {MOVE_HEAD} \
    -textvariable {avatarButton24}

  # build widget .frame4.frame2.frame0.frame12.frame3
  frame .frame4.frame2.frame0.frame12.frame3 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame12.frame3.button1
  button .frame4.frame2.frame0.frame12.frame3.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton25} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {TAIL_FLEX} \
    -textvariable {avatarButton25}

  # build widget .frame4.frame2.frame0.frame12.frame4
  frame .frame4.frame2.frame0.frame12.frame4 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame12.frame4.button1
  button .frame4.frame2.frame0.frame12.frame4.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton26} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {NUDGE_TO} \
    -textvariable {avatarButton26}

  # build widget .frame4.frame2.frame0.frame12.frame5
  frame .frame4.frame2.frame0.frame12.frame5 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame12.frame5.button1
  button .frame4.frame2.frame0.frame12.frame5.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton27} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {PANT} \
    -textvariable {avatarButton27}

  # build widget .frame4.frame2.frame0.frame12.frame6
  frame .frame4.frame2.frame0.frame12.frame6 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame12.frame6.button1
  button .frame4.frame2.frame0.frame12.frame6.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton28} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {PEE} \
    -textvariable {avatarButton28}

  # build widget .frame4.frame2.frame0.frame12.frame7
  frame .frame4.frame2.frame0.frame12.frame7 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame12.frame7.button1
  button .frame4.frame2.frame0.frame12.frame7.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton29} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {PLAY_DEAD} \
    -textvariable {avatarButton29}

  # build widget .frame4.frame2.frame0.frame12.frame8
  frame .frame4.frame2.frame0.frame12.frame8 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame12.frame8.button1
  button .frame4.frame2.frame0.frame12.frame8.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton30} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {POO} \
    -textvariable {avatarButton30}

  # build widget .frame4.frame2.frame0.frame12.frame9
  frame .frame4.frame2.frame0.frame12.frame9 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame12.frame9.button1
  button .frame4.frame2.frame0.frame12.frame9.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton31} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {ROLL_OVER} \
    -textvariable {avatarButton31}

  # build widget .frame4.frame2.frame0.frame12.frame10
  frame .frame4.frame2.frame0.frame12.frame10 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame12.frame10.button1
  button .frame4.frame2.frame0.frame12.frame10.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton32} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {SCRATCH_SELF_NOSE} \
    -textvariable {avatarButton32}

  # build widget .frame4.frame2.frame0.frame13
  frame .frame4.frame2.frame0.frame13 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame13.frame0
  frame .frame4.frame2.frame0.frame13.frame0 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame13.frame0.button1
  button .frame4.frame2.frame0.frame13.frame0.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton33} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {SCRATCH_OTHER} \
    -textvariable {avatarButton33}

  # build widget .frame4.frame2.frame0.frame13.frame1
  frame .frame4.frame2.frame0.frame13.frame1 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame13.frame1.button1
  button .frame4.frame2.frame0.frame13.frame1.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton34} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {SIT} \
    -textvariable {avatarButton34}

  # build widget .frame4.frame2.frame0.frame13.frame2
  frame .frame4.frame2.frame0.frame13.frame2 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame13.frame2.button1
  button .frame4.frame2.frame0.frame13.frame2.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton35} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {SLEEP} \
    -textvariable {avatarButton35}

  # build widget .frame4.frame2.frame0.frame13.frame3
  frame .frame4.frame2.frame0.frame13.frame3 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame13.frame3.button1
  button .frame4.frame2.frame0.frame13.frame3.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton36} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {SNIFF} \
    -textvariable {avatarButton36}

  # build widget .frame4.frame2.frame0.frame13.frame4
  frame .frame4.frame2.frame0.frame13.frame4 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame13.frame4.button1
  button .frame4.frame2.frame0.frame13.frame4.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton37} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {STAY} \
    -textvariable {avatarButton37}

  # build widget .frame4.frame2.frame0.frame13.frame5
  frame .frame4.frame2.frame0.frame13.frame5 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame13.frame5.button1
  button .frame4.frame2.frame0.frame13.frame5.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton38} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {STRETCH} \
    -textvariable {avatarButton38}

  # build widget .frame4.frame2.frame0.frame13.frame6
  frame .frame4.frame2.frame0.frame13.frame6 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame13.frame6.button1
  button .frame4.frame2.frame0.frame13.frame6.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton39} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {TURN} \
    -textvariable {avatarButton39}

  # build widget .frame4.frame2.frame0.frame13.frame7
  frame .frame4.frame2.frame0.frame13.frame7 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame13.frame7.button1
  button .frame4.frame2.frame0.frame13.frame7.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton40} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {WAG} \
    -textvariable {avatarButton40}

  # build widget .frame4.frame2.frame0.frame13.frame8
  frame .frame4.frame2.frame0.frame13.frame8 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame13.frame8.button1
  button .frame4.frame2.frame0.frame13.frame8.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton41} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {WAKE} \
    -textvariable {avatarButton41}

  # build widget .frame4.frame2.frame0.frame13.frame9
  frame .frame4.frame2.frame0.frame13.frame9 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame13.frame9.button1
  button .frame4.frame2.frame0.frame13.frame9.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton42} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {WALK} \
    -textvariable {avatarButton42}

  # build widget .frame4.frame2.frame0.frame13.frame10
  frame .frame4.frame2.frame0.frame13.frame10 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame13.frame10.button1
  button .frame4.frame2.frame0.frame13.frame10.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton43} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {WHINE} \
    -textvariable {avatarButton43}

  # build widget .frame4.frame2.frame0.frame13.frame11
  frame .frame4.frame2.frame0.frame13.frame11 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame13.frame11.button1
  button .frame4.frame2.frame0.frame13.frame11.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton44} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {GREET} \
    -textvariable {avatarButton44}


  # build widget .frame4.frame2.frame0.frame14
  frame .frame4.frame2.frame0.frame14 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame14.frame0
  frame .frame4.frame2.frame0.frame14.frame0 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame14.frame0.button1
  button .frame4.frame2.frame0.frame14.frame0.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton45} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {KICK} \
    -textvariable {avatarButton45}

  # build widget .frame4.frame2.frame0.frame14.frame1
  frame .frame4.frame2.frame0.frame14.frame1 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame0.frame14.frame1.button1
  button .frame4.frame2.frame0.frame14.frame1.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {executeAvatarCommand $avatarButton46} \
    -font {Helvetica -10 bold} \
    -highlightbackground {#eeeff2} \
    -text {PET} \
    -textvariable {avatarButton46}




  # build widget .frame4.frame2.frame7
  frame .frame4.frame2.frame7 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame4.frame2.frame7.frame12
  frame .frame4.frame2.frame7.frame12 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame7.frame12.label13
  label .frame4.frame2.frame7.frame12.label13 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -font {-*-helvetica-bold-r-normal-*-12-*-*-*-p-*-iso8859-1} \
    -highlightbackground {#eeeff2} \
    -text {Predavese:}

  # build widget .frame4.frame2.frame7.frame12.entry14
  entry .frame4.frame2.frame7.frame12.entry14 \
    -background {#ffffff} \
    -font {-*-helvetica-medium-r-normal-*-12-*-*-*-p-*-iso8859-1} \
    -highlightbackground {#ffffff} \
    -selectbackground {#7579ba} \
    -selectforeground {#ffffff} \
    -textvariable {predaveseText} \
    -width {57}
  # bindings
  bind .frame4.frame2.frame7.frame12.entry14 <Key-Return> {.frame4.frame2.frame7.frame12.button0 invoke}

  # build widget .frame4.frame2.frame7.frame12.button0
  button .frame4.frame2.frame7.frame12.button0 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {sendPredavese $predaveseText; set predaveseText ""} \
    -highlightbackground {#bbbcc0} \
    -text {Send}

  # build widget .frame4.frame2.frame7.frame12.frame11
  frame .frame4.frame2.frame7.frame12.frame11 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame7.frame12.frame11.frame12
  frame .frame4.frame2.frame7.frame12.frame11.frame12 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame7.frame12.frame11.frame12.button13
  button .frame4.frame2.frame7.frame12.frame11.frame12.button13 \
    -activebackground {#eeeff2} \
    -background {#bbbcc0} \
    -command {set statusLine "Please wait while simulation starts..."
startPVPSimulator
.frame4.frame2.frame7.frame12.frame11.frame12.button13 configure -state disable} \
    -highlightbackground {#eeeff2} \
    -text {Start simulation} \
    -width {19}

  # build widget .frame4.frame2.frame7.frame12.frame11.frame12.button14
  button .frame4.frame2.frame7.frame12.frame11.frame12.button14 \
    -activebackground {#eeeff2} \
    -background {#bbbcc0} \
    -command {catch {exec ./killpb.csh}; exit} \
    -highlightbackground {#eeeff2} \
    -text {Shutdown PB} \
    -width {19}

  # build widget .frame4.frame2.frame7.frame12.frame11.label0
  label .frame4.frame2.frame7.frame12.frame11.label0 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -foreground {#ff00ff} \
    -highlightbackground {#eeeff2} \
    -textvariable {statusLine}

  # build widget .frame4.frame2.frame7.frame6
  frame .frame4.frame2.frame7.frame6 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame4.frame2.frame7.frame6.frame0
  frame .frame4.frame2.frame7.frame6.frame0 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame7.frame6.frame0.label8
  label .frame4.frame2.frame7.frame6.frame0.label8 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -font {-*-helvetica-bold-r-normal-*-12-*-*-*-p-*-iso8859-1} \
    -highlightbackground {#eeeff2} \
    -text {Shortcut to avatar feedback:}

  # build widget .frame4.frame2.frame7.frame6.frame0.frame9
  frame .frame4.frame2.frame7.frame6.frame0.frame9 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame4.frame2.frame7.frame6.frame0.frame9.button10
  button .frame4.frame2.frame7.frame6.frame0.frame9.button10 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {sendPredavese "Good boy!"} \
    -highlightbackground {#bbbcc0} \
    -text {Good boy!}

  # build widget .frame4.frame2.frame7.frame6.frame0.frame9.button11
  button .frame4.frame2.frame7.frame6.frame0.frame9.button11 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {sendPredavese "Bad boy!"} \
    -highlightbackground {#bbbcc0} \
    -text {Bad boy!}

  # build widget .frame4.frame2.frame7.frame6.frame3
  frame .frame4.frame2.frame7.frame6.frame3 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}
  button .frame4.frame2.frame7.frame6.frame3.button1  \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {getNewPetId} \
    -highlightbackground {#eeeff2} \
    -text {Create new pet}
  button .frame4.frame2.frame7.frame6.frame3.button2  \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {resetPhysiologicalModel} \
    -highlightbackground {#eeeff2} \
    -text {Reset Physiological Model}

# ---- current pet
  # build widget .frame5.frame0.frame1.frame0.label1
  label .frame4.frame2.frame7.frame6.frame3.label1 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -borderwidth {0} \
    -highlightbackground {#eeeff2} \
    -text {Current pet:}

  # build widget .frame5.frame0.frame1.frame0.value
  label .frame4.frame2.frame7.frame6.frame3.value \
    -activebackground {#eeeff2} \
    -anchor {w} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -relief {sunken} \
	 -text {Fido}
#    -text {Wynx}

  # build widget .frame5.frame0.frame1.frame0.menubutton2
  menubutton .frame4.frame2.frame7.frame6.frame3.menubutton2 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -menu {.frame4.frame2.frame7.frame6.frame3.menubutton2.m} \
    -padx {4} \
    -pady {3} \
    -text {v}

  # build widget .frame5.frame0.frame1.frame0.menubutton2.m
  menu .frame4.frame2.frame7.frame6.frame3.menubutton2.m \
    -activebackground {#eeeff2} \
    -background {#eeeff2}
#  .frame4.frame2.frame7.frame6.frame3.menubutton2.m add command \
 #   -command {OptionButtonSet .frame5.frame0.frame1.frame0} \
  #  -label {Fido}


# ----


  # build widget .frame4.frame2.frame7.frame6.frame3.frame7
  frame .frame4.frame2.frame7.frame6.frame3.frame7 \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame7.frame6
  frame .frame4.frame2.frame7.frame6.frame3.frame7.frame6 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame7.frame6.label7
  label .frame4.frame2.frame7.frame6.frame3.frame7.frame6.label7 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -font {-*-helvetica-bold-r-normal-*-12-*-*-*-p-*-iso8859-1} \
    -highlightbackground {#eeeff2} \
    -text {Energy:}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame7.frame6.canvas9
  canvas .frame4.frame2.frame7.frame6.frame3.frame7.frame6.canvas9 \
    -background {#ffffff} \
    -height {16} \
    -highlightbackground {#ffffff} \
    -relief {raised} \
    -width {100}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame7.frame5
  frame .frame4.frame2.frame7.frame6.frame3.frame7.frame5 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame7.frame5.label7
  label .frame4.frame2.frame7.frame6.frame3.frame7.frame5.label7 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -font {-*-helvetica-bold-r-normal-*-12-*-*-*-p-*-iso8859-1} \
    -highlightbackground {#eeeff2} \
    -text {Fitness:}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame7.frame5.canvas9
  canvas .frame4.frame2.frame7.frame6.frame3.frame7.frame5.canvas9 \
    -background {#ffffff} \
    -height {16} \
    -highlightbackground {#ffffff} \
    -relief {raised} \
    -width {100}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame9
  frame .frame4.frame2.frame7.frame6.frame3.frame9 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame9.frame1
  frame .frame4.frame2.frame7.frame6.frame3.frame9.frame1 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame9.frame1.label7
  label .frame4.frame2.frame7.frame6.frame3.frame9.frame1.label7 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -font {-*-helvetica-bold-r-normal-*-12-*-*-*-p-*-iso8859-1} \
    -highlightbackground {#eeeff2} \
    -text {Hunger:}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame9.frame1.canvas9
  canvas .frame4.frame2.frame7.frame6.frame3.frame9.frame1.canvas9 \
    -background {#ffffff} \
    -height {16} \
    -highlightbackground {#ffffff} \
    -relief {raised} \
    -width {100}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame9.frame2
  frame .frame4.frame2.frame7.frame6.frame3.frame9.frame2 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame9.frame2.label7
  label .frame4.frame2.frame7.frame6.frame3.frame9.frame2.label7 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -font {-*-helvetica-bold-r-normal-*-12-*-*-*-p-*-iso8859-1} \
    -highlightbackground {#eeeff2} \
    -text {Thirst:}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame9.frame2.canvas9
  canvas .frame4.frame2.frame7.frame6.frame3.frame9.frame2.canvas9 \
    -background {#ffffff} \
    -height {16} \
    -highlightbackground {#ffffff} \
    -relief {raised} \
    -width {100}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame10
  frame .frame4.frame2.frame7.frame6.frame3.frame10 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame10.frame3
  frame .frame4.frame2.frame7.frame6.frame3.frame10.frame3 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame10.frame3.label7
  label .frame4.frame2.frame7.frame6.frame3.frame10.frame3.label7 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -font {-*-helvetica-bold-r-normal-*-12-*-*-*-p-*-iso8859-1} \
    -highlightbackground {#eeeff2} \
    -text {Pee urgency:}


  # build widget .frame4.frame2.frame7.frame6.frame3.frame10.frame3.canvas9
  canvas .frame4.frame2.frame7.frame6.frame3.frame10.frame3.canvas9 \
    -background {#ffffff} \
    -height {16} \
    -highlightbackground {#ffffff} \
    -relief {raised} \
    -width {100}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame10.frame4
  frame .frame4.frame2.frame7.frame6.frame3.frame10.frame4 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame10.frame4.label7
  label .frame4.frame2.frame7.frame6.frame3.frame10.frame4.label7 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -font {-*-helvetica-bold-r-normal-*-12-*-*-*-p-*-iso8859-1} \
    -highlightbackground {#eeeff2} \
    -text {Poo urgency:}

  # build widget .frame4.frame2.frame7.frame6.frame3.frame10.frame4.canvas9
  canvas .frame4.frame2.frame7.frame6.frame3.frame10.frame4.canvas9 \
    -background {#ffffff} \
    -height {16} \
    -highlightbackground {#ffffff} \
    -relief {raised} \
    -width {100}

  # build widget .frame5
  frame .frame5 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame5.frame0
  frame .frame5.frame0 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame5.frame0.frame1
  frame .frame5.frame0.frame1 \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame5.frame0.frame1.button1
  button .frame5.frame0.frame1.button1 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {getNewAvatarId} \
    -highlightbackground {#eeeff2} \
    -text {Create new avatar}

  # build widget .frame5.frame0.frame1.frame0
  frame .frame5.frame0.frame1.frame0 \
    -borderwidth {2} \
    -relief {raised} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame5.frame0.frame1.frame0.label1
  label .frame5.frame0.frame1.frame0.label1 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -borderwidth {0} \
    -highlightbackground {#eeeff2} \
    -text {Current avatar:}

  # build widget .frame5.frame0.frame1.frame0.value
  label .frame5.frame0.frame1.frame0.value \
    -activebackground {#eeeff2} \
    -anchor {w} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -relief {sunken} \
    -text {Wynx}

  # build widget .frame5.frame0.frame1.frame0.menubutton2
  menubutton .frame5.frame0.frame1.frame0.menubutton2 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -menu {.frame5.frame0.frame1.frame0.menubutton2.m} \
    -padx {4} \
    -pady {3} \
    -text {v}

  # build widget .frame5.frame0.frame1.frame0.menubutton2.m
  menu .frame5.frame0.frame1.frame0.menubutton2.m \
    -activebackground {#eeeff2} \
    -background {#eeeff2}
#  .frame5.frame0.frame1.frame0.menubutton2.m add command \
 #   -command {OptionButtonSet .frame5.frame0.frame1.frame0} \
    -label {Wynx}

  # build widget .frame5.frame0.frame2
  frame .frame5.frame0.frame2 \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame5.frame0.frame2.label2
  label .frame5.frame0.frame2.label2 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {Pet actions:}

  # build widget .frame5.frame0.frame2.scrollbar3
  scrollbar .frame5.frame0.frame2.scrollbar3 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -command {.frame5.frame0.frame2.listbox1 xview} \
    -cursor {left_ptr} \
    -highlightbackground {#eeeff2} \
    -orient {horizontal} \
    -relief {raised} \
    -troughcolor {#eeeff2}

  # build widget .frame5.frame0.frame2.scrollbar2
  scrollbar .frame5.frame0.frame2.scrollbar2 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -command {.frame5.frame0.frame2.listbox1 yview} \
    -cursor {left_ptr} \
    -highlightbackground {#eeeff2} \
    -relief {raised} \
    -troughcolor {#eeeff2}

  # build widget .frame5.frame0.frame2.listbox1
  listbox .frame5.frame0.frame2.listbox1 \
    -background {#eeeff2} \
    -height {20} \
    -highlightbackground {#ffffff} \
    -selectbackground {#7579ba} \
    -selectforeground {#ffffff} \
    -width {37} \
    -xscrollcommand {.frame5.frame0.frame2.scrollbar3 set} \
    -yscrollcommand {.frame5.frame0.frame2.scrollbar2 set}

  # build widget .frame5.frame1
  frame .frame5.frame1 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame5.frame1.frame3
  frame .frame5.frame1.frame3 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame5.frame1.frame3.button4
  button .frame5.frame1.frame3.button4 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {showLog PROXY} \
    -highlightbackground {#bbbcc0} \
    -text {Snapshot Proxy's log}

  # build widget .frame5.frame1.frame3.button5
  button .frame5.frame1.frame3.button5 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {showLog Fido} \
    -highlightbackground {#bbbcc0} \
    -text {Snapshot OPC's log}

  # build widget .frame5.frame1.frame3.button6
  button .frame5.frame1.frame3.button6 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {showLog LS} \
    -highlightbackground {#bbbcc0} \
    -text {Snapshot LS's log}

  # build widget .frame5.frame1.frame3.button7
  button .frame5.frame1.frame3.button7 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {showLog ROUTER} \
    -highlightbackground {#bbbcc0} \
    -text {Snapshot Router's log}

  # build widget .frame5.frame1.frame3.button8
  button .frame5.frame1.frame3.button8 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {showLog SPAWNER} \
    -highlightbackground {#bbbcc0} \
    -text {Snapshot Spawner's log}

  # build widget .frame5.frame1.frame
  frame .frame5.frame1.frame \
    -relief {sunken} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame5.frame1.frame.scrollbar1
  scrollbar .frame5.frame1.frame.scrollbar1 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -command {.frame5.frame1.frame.text2 yview} \
    -cursor {left_ptr} \
    -highlightbackground {#eeeff2} \
    -relief {raised} \
    -troughcolor {#eeeff2}

  # build widget .frame5.frame1.frame.text2
  text .frame5.frame1.frame.text2 \
    -background {#eeeff2} \
    -height {56} \
    -highlightbackground {#ffffff} \
    -selectbackground {#7579ba} \
    -selectforeground {#ffffff} \
    -width {93} \
    -wrap {word} \
    -yscrollcommand {.frame5.frame1.frame.scrollbar1 set}

  # pack master .frame4
  pack configure .frame4.frame2 \
    -side left

  # pack master .frame4.frame2
  pack configure .frame4.frame2.frame0 \
    -padx 85
  pack configure .frame4.frame2.frame7 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0
  pack configure .frame4.frame2.frame0.frame9
  pack configure .frame4.frame2.frame0.frame11
  pack configure .frame4.frame2.frame0.frame12
  pack configure .frame4.frame2.frame0.frame13
  pack configure .frame4.frame2.frame0.frame14

  # pack master .frame4.frame2.frame0.frame9
  pack configure .frame4.frame2.frame0.frame9.frame0 \
    -side left
  pack configure .frame4.frame2.frame0.frame9.frame1 \
    -side left
  pack configure .frame4.frame2.frame0.frame9.frame2 \
    -side left
  pack configure .frame4.frame2.frame0.frame9.frame3 \
    -side left
  pack configure .frame4.frame2.frame0.frame9.frame4 \
    -side left
  pack configure .frame4.frame2.frame0.frame9.frame5 \
    -side left
  pack configure .frame4.frame2.frame0.frame9.frame6 \
    -side left
  pack configure .frame4.frame2.frame0.frame9.frame7 \
    -side left
  pack configure .frame4.frame2.frame0.frame9.frame8 \
    -side left
  pack configure .frame4.frame2.frame0.frame9.frame9 \
    -side left
  pack configure .frame4.frame2.frame0.frame9.frame10 \
    -side left

  # pack master .frame4.frame2.frame0.frame9.frame0
  pack configure .frame4.frame2.frame0.frame9.frame0.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame9.frame1
  pack configure .frame4.frame2.frame0.frame9.frame1.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame9.frame2
  pack configure .frame4.frame2.frame0.frame9.frame2.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame9.frame3
  pack configure .frame4.frame2.frame0.frame9.frame3.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame9.frame4
  pack configure .frame4.frame2.frame0.frame9.frame4.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame9.frame5
  pack configure .frame4.frame2.frame0.frame9.frame5.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame9.frame6
  pack configure .frame4.frame2.frame0.frame9.frame6.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame9.frame7
  pack configure .frame4.frame2.frame0.frame9.frame7.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame9.frame8
  pack configure .frame4.frame2.frame0.frame9.frame8.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame9.frame9
  pack configure .frame4.frame2.frame0.frame9.frame9.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame9.frame10
  pack configure .frame4.frame2.frame0.frame9.frame10.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame11
  pack configure .frame4.frame2.frame0.frame11.frame0 \
    -side left
  pack configure .frame4.frame2.frame0.frame11.frame1 \
    -side left
  pack configure .frame4.frame2.frame0.frame11.frame2 \
    -side left
  pack configure .frame4.frame2.frame0.frame11.frame3 \
    -side left
  pack configure .frame4.frame2.frame0.frame11.frame4 \
    -side left
  pack configure .frame4.frame2.frame0.frame11.frame5 \
    -side left
  pack configure .frame4.frame2.frame0.frame11.frame6 \
    -side left
  pack configure .frame4.frame2.frame0.frame11.frame7 \
    -side left
  pack configure .frame4.frame2.frame0.frame11.frame8 \
    -side left
  pack configure .frame4.frame2.frame0.frame11.frame9 \
    -side left
  pack configure .frame4.frame2.frame0.frame11.frame10 \
    -side left

  # pack master .frame4.frame2.frame0.frame11.frame0
  pack configure .frame4.frame2.frame0.frame11.frame0.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame11.frame1
  pack configure .frame4.frame2.frame0.frame11.frame1.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame11.frame2
  pack configure .frame4.frame2.frame0.frame11.frame2.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame11.frame3
  pack configure .frame4.frame2.frame0.frame11.frame3.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame11.frame4
  pack configure .frame4.frame2.frame0.frame11.frame4.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame11.frame5
  pack configure .frame4.frame2.frame0.frame11.frame5.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame11.frame6
  pack configure .frame4.frame2.frame0.frame11.frame6.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame11.frame7
  pack configure .frame4.frame2.frame0.frame11.frame7.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame11.frame8
  pack configure .frame4.frame2.frame0.frame11.frame8.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame11.frame9
  pack configure .frame4.frame2.frame0.frame11.frame9.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame11.frame10
  pack configure .frame4.frame2.frame0.frame11.frame10.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame12
  pack configure .frame4.frame2.frame0.frame12.frame0 \
    -side left
  pack configure .frame4.frame2.frame0.frame12.frame1 \
    -side left
  pack configure .frame4.frame2.frame0.frame12.frame2 \
    -side left
  pack configure .frame4.frame2.frame0.frame12.frame3 \
    -side left
  pack configure .frame4.frame2.frame0.frame12.frame4 \
    -side left
  pack configure .frame4.frame2.frame0.frame12.frame5 \
    -side left
  pack configure .frame4.frame2.frame0.frame12.frame6 \
    -side left
  pack configure .frame4.frame2.frame0.frame12.frame7 \
    -side left
  pack configure .frame4.frame2.frame0.frame12.frame8 \
    -side left
  pack configure .frame4.frame2.frame0.frame12.frame9 \
    -side left
  pack configure .frame4.frame2.frame0.frame12.frame10 \
    -side left

  # pack master .frame4.frame2.frame0.frame12.frame0
  pack configure .frame4.frame2.frame0.frame12.frame0.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame12.frame1
  pack configure .frame4.frame2.frame0.frame12.frame1.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame12.frame2
  pack configure .frame4.frame2.frame0.frame12.frame2.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame12.frame3
  pack configure .frame4.frame2.frame0.frame12.frame3.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame12.frame4
  pack configure .frame4.frame2.frame0.frame12.frame4.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame12.frame5
  pack configure .frame4.frame2.frame0.frame12.frame5.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame12.frame6
  pack configure .frame4.frame2.frame0.frame12.frame6.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame12.frame7
  pack configure .frame4.frame2.frame0.frame12.frame7.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame12.frame8
  pack configure .frame4.frame2.frame0.frame12.frame8.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame12.frame9
  pack configure .frame4.frame2.frame0.frame12.frame9.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame12.frame10
  pack configure .frame4.frame2.frame0.frame12.frame10.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame13
  pack configure .frame4.frame2.frame0.frame13.frame0 \
    -side left
  pack configure .frame4.frame2.frame0.frame13.frame1 \
    -side left
  pack configure .frame4.frame2.frame0.frame13.frame2 \
    -side left
  pack configure .frame4.frame2.frame0.frame13.frame3 \
    -side left
  pack configure .frame4.frame2.frame0.frame13.frame4 \
    -side left
  pack configure .frame4.frame2.frame0.frame13.frame5 \
    -side left
  pack configure .frame4.frame2.frame0.frame13.frame6 \
    -side left
  pack configure .frame4.frame2.frame0.frame13.frame7 \
    -side left
  pack configure .frame4.frame2.frame0.frame13.frame8 \
    -side left
  pack configure .frame4.frame2.frame0.frame13.frame9 \
    -side left
  pack configure .frame4.frame2.frame0.frame13.frame10 \
    -side left
  pack configure .frame4.frame2.frame0.frame13.frame11 \
    -side left


  # pack master .frame4.frame2.frame0.frame13.frame0
  pack configure .frame4.frame2.frame0.frame13.frame0.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame13.frame1
  pack configure .frame4.frame2.frame0.frame13.frame1.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame13.frame2
  pack configure .frame4.frame2.frame0.frame13.frame2.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame13.frame3
  pack configure .frame4.frame2.frame0.frame13.frame3.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame13.frame4
  pack configure .frame4.frame2.frame0.frame13.frame4.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame13.frame5
  pack configure .frame4.frame2.frame0.frame13.frame5.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame13.frame6
  pack configure .frame4.frame2.frame0.frame13.frame6.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame13.frame7
  pack configure .frame4.frame2.frame0.frame13.frame7.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame13.frame8
  pack configure .frame4.frame2.frame0.frame13.frame8.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame13.frame9
  pack configure .frame4.frame2.frame0.frame13.frame9.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame13.frame10
  pack configure .frame4.frame2.frame0.frame13.frame10.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame13.frame11
  pack configure .frame4.frame2.frame0.frame13.frame11.button1 \
    -expand 1 \
    -fill x


  # pack master .frame4.frame2.frame0.frame14
  pack configure .frame4.frame2.frame0.frame14.frame0 \
    -side left
  pack configure .frame4.frame2.frame0.frame14.frame1 \
    -side left


  # pack master .frame4.frame2.frame0.frame14.frame0
  pack configure .frame4.frame2.frame0.frame14.frame0.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame0.frame14.frame1
  pack configure .frame4.frame2.frame0.frame14.frame1.button1 \
    -expand 1 \
    -fill x

  # pack master .frame4.frame2.frame7
  pack configure .frame4.frame2.frame7.frame12 \
    -anchor w
  pack configure .frame4.frame2.frame7.frame6 \
    -anchor w

  # pack master .frame4.frame2.frame7.frame12
  pack configure .frame4.frame2.frame7.frame12.label13 \
    -anchor w \
    -side left
  pack configure .frame4.frame2.frame7.frame12.entry14 \
    -anchor w \
    -side left
  pack configure .frame4.frame2.frame7.frame12.button0 \
    -anchor w \
    -side left
  pack configure .frame4.frame2.frame7.frame12.frame11 \
    -anchor se \
    -padx 56 \
    -side left

  # pack master .frame4.frame2.frame7.frame12.frame11
  pack configure .frame4.frame2.frame7.frame12.frame11.frame12 \
    -anchor w
  pack configure .frame4.frame2.frame7.frame12.frame11.label0 \
    -anchor w

  # pack master .frame4.frame2.frame7.frame12.frame11.frame12
  pack configure .frame4.frame2.frame7.frame12.frame11.frame12.button13 \
    -side left
  pack configure .frame4.frame2.frame7.frame12.frame11.frame12.button14 \
    -side left

  # pack master .frame4.frame2.frame7.frame6
  pack configure .frame4.frame2.frame7.frame6.frame0 \
    -side left
  pack configure .frame4.frame2.frame7.frame6.frame3 \
    -padx 25 \
    -side left

  # pack master .frame4.frame2.frame7.frame6.frame0
  pack configure .frame4.frame2.frame7.frame6.frame0.label8 \
    -anchor w \
    -side left
  pack configure .frame4.frame2.frame7.frame6.frame0.frame9 \
    -anchor w

  # pack master .frame4.frame2.frame7.frame6.frame0.frame9
  pack configure .frame4.frame2.frame7.frame6.frame0.frame9.button10 \
    -side left
  pack configure .frame4.frame2.frame7.frame6.frame0.frame9.button11

  # pack master .frame4.frame2.frame7.frame6.frame3
  pack configure .frame4.frame2.frame7.frame6.frame3.button1 \
    -fill x 
  pack configure .frame4.frame2.frame7.frame6.frame3.button2 \
    -fill x \
	-side bottom

# --- current pet
  pack configure .frame4.frame2.frame7.frame6.frame3.label1 \
    -fill both \
    -side left
  pack configure .frame4.frame2.frame7.frame6.frame3.value \
    -expand 1 \
    -fill both \
    -side left
  pack configure .frame4.frame2.frame7.frame6.frame3.menubutton2 \
    -fill both \
    -side left 


# ---
 
  pack configure .frame4.frame2.frame7.frame6.frame3.frame7 \
    -side left
  pack configure .frame4.frame2.frame7.frame6.frame3.frame9 \
    -side left
  pack configure .frame4.frame2.frame7.frame6.frame3.frame10 \
    -side left

  # pack master .frame4.frame2.frame7.frame6.frame3.frame7
  pack configure .frame4.frame2.frame7.frame6.frame3.frame7.frame6 \
    -fill x
  pack configure .frame4.frame2.frame7.frame6.frame3.frame7.frame5 \
    -fill x

  # pack master .frame4.frame2.frame7.frame6.frame3.frame7.frame6
  pack configure .frame4.frame2.frame7.frame6.frame3.frame7.frame6.label7 \
    -anchor e \
    -expand 1 \
    -side left
  pack configure .frame4.frame2.frame7.frame6.frame3.frame7.frame6.canvas9

  # pack master .frame4.frame2.frame7.frame6.frame3.frame7.frame5
  pack configure .frame4.frame2.frame7.frame6.frame3.frame7.frame5.label7 \
    -anchor e \
    -expand 1 \
    -side left
  pack configure .frame4.frame2.frame7.frame6.frame3.frame7.frame5.canvas9

  # pack master .frame4.frame2.frame7.frame6.frame3.frame9
  pack configure .frame4.frame2.frame7.frame6.frame3.frame9.frame1 \
    -fill x
  pack configure .frame4.frame2.frame7.frame6.frame3.frame9.frame2 \
    -fill x

  # pack master .frame4.frame2.frame7.frame6.frame3.frame9.frame1
  pack configure .frame4.frame2.frame7.frame6.frame3.frame9.frame1.label7 \
    -anchor e \
    -expand 1 \
    -side left
  pack configure .frame4.frame2.frame7.frame6.frame3.frame9.frame1.canvas9

  # pack master .frame4.frame2.frame7.frame6.frame3.frame9.frame2
  pack configure .frame4.frame2.frame7.frame6.frame3.frame9.frame2.label7 \
    -anchor e \
    -expand 1 \
    -side left
  pack configure .frame4.frame2.frame7.frame6.frame3.frame9.frame2.canvas9

  # pack master .frame4.frame2.frame7.frame6.frame3.frame10
  pack configure .frame4.frame2.frame7.frame6.frame3.frame10.frame3 \
    -fill x
  pack configure .frame4.frame2.frame7.frame6.frame3.frame10.frame4 \
    -fill x

  # pack master .frame4.frame2.frame7.frame6.frame3.frame10.frame3
  pack configure .frame4.frame2.frame7.frame6.frame3.frame10.frame3.label7 \
    -anchor e \
    -expand 1 \
    -side left
  pack configure .frame4.frame2.frame7.frame6.frame3.frame10.frame3.canvas9

 # pack master .frame4.frame2.frame7.frame6.frame3.frame10.frame4
  pack configure .frame4.frame2.frame7.frame6.frame3.frame10.frame4.label7 \
    -anchor e \
    -expand 1 \
    -side left
  pack configure .frame4.frame2.frame7.frame6.frame3.frame10.frame4.canvas9

  # pack master .frame5
  pack configure .frame5.frame0 \
    -anchor nw \
    -fill y \
    -side left
  pack configure .frame5.frame1 \
    -side left

  # pack master .frame5.frame0
  pack configure .frame5.frame0.frame1 \
    -anchor w \
    -fill both
  pack configure .frame5.frame0.frame2 \
    -anchor w \
    -fill both

  # pack master .frame5.frame0.frame1
  pack configure .frame5.frame0.frame1.button1 \
    -fill x 

  pack configure .frame5.frame0.frame1.frame0 \
    -fill x

  # pack master .frame5.frame0.frame1.frame0
  pack configure .frame5.frame0.frame1.frame0.label1 \
    -fill both \
    -side left
  pack configure .frame5.frame0.frame1.frame0.value \
    -expand 1 \
    -fill both \
    -side left
  pack configure .frame5.frame0.frame1.frame0.menubutton2 \
    -fill both \
    -side right

  # pack master .frame5.frame0.frame2
  pack configure .frame5.frame0.frame2.label2 \
    -anchor w
  pack configure .frame5.frame0.frame2.scrollbar2 \
    -fill y \
    -side right
  pack configure .frame5.frame0.frame2.scrollbar3 \
    -fill x \
    -side bottom
  pack configure .frame5.frame0.frame2.listbox1 \
    -expand 1 \
    -fill both

  # pack master .frame5.frame1
  pack configure .frame5.frame1.frame3
  pack configure .frame5.frame1.frame \
    -anchor nw \
    -fill x

  # pack master .frame5.frame1.frame3
  pack configure .frame5.frame1.frame3.button4 \
    -side left
  pack configure .frame5.frame1.frame3.button5 \
    -side left
  pack configure .frame5.frame1.frame3.button6 \
    -side left
  pack configure .frame5.frame1.frame3.button7 \
    -side left
  pack configure .frame5.frame1.frame3.button8 \
    -side left

  # pack master .frame5.frame1.frame
  pack configure .frame5.frame1.frame.scrollbar1 \
    -fill both \
    -side right
  pack configure .frame5.frame1.frame.text2 \
    -expand 1 \
    -fill both

  # pack master .
  pack configure .frame4 \
    -expand 1 \
    -fill x
  pack configure .frame5 \
    -fill x

  # build canvas items .frame4.frame2.frame7.frame6.frame3.frame7.frame6.canvas9
  # build canvas items .frame4.frame2.frame7.frame6.frame3.frame7.frame5.canvas9
  # build canvas items .frame4.frame2.frame7.frame6.frame3.frame9.frame1.canvas9
  # build canvas items .frame4.frame2.frame7.frame6.frame3.frame9.frame2.canvas9
  # build canvas items .frame4.frame2.frame7.frame6.frame3.frame10.frame3.canvas9
  # build canvas items .frame4.frame2.frame7.frame6.frame3.frame10.frame4.canvas9
  .frame5.frame1.frame.text2 insert end {}



  if {"[info procs XFEdit]" != ""} {
    catch "XFMiscBindWidgetTree ."
    after 2 "catch {XFEditSetShowWindows}"
  }
}


# User defined procedures


# Procedure: OptionButtonGet
proc OptionButtonGet { widget} {

  if {"[winfo class $widget.value]" == "Label"} {
    return [lindex [$widget.value config -text] 4]
  } {
    if {"[winfo class $widget.value]" == "Entry"} {
      return [$widget.value get]
    }
  }
}


# Procedure: OptionButtonSet
proc OptionButtonSet { widget} {

  if {"[winfo class $widget.value]" == "Label"} {
    $widget.value config -text [lindex [$widget.menubutton2.m entryconfig [$widget.menubutton2.m index active] -label] 4]
  } {
    if {"[winfo class $widget.value]" == "Entry"} {
      $widget.value delete 0 end
      $widget.value insert 0 [lindex [$widget.menubutton2.m entryconfig [$widget.menubutton2.m index active] -label] 4]
    }
  }
}



# Procedure: drawBar
proc drawBar { cv level reverseFlag} {

    if {$reverseFlag} {
        if {$level > 0.75} {
            set color {#0000ff}
        } else {
            if {$level > 0.50} {
                set color {#2200dd}
            } else {
                if {$level > 0.25} {
                    set color {#5500aa}
                } else {
                    if {$level > 0.10} {
                        set color {#aa0055}
                    } else {
                        set color {#ff0000}
                    }
                }
            }
        }
    } else {
        if {$level < 0.25} {
            set color {#0000ff}
        } else {
            if {$level < 0.50} {
                set color {#2200dd}
            } else {
                if {$level < 0.75} {
                    set color {#5500aa}
                } else {
                    if {$level < 0.90} {
                        set color {#aa0055}
                    } else {
                        set color {#ff0000}
                    }
                }
            }
        }
    }

    $cv addtag toDelete all
    $cv delete toDelete
    set xfTmpTag [$cv create rectangle 0.0 0.0 [expr $level * 100] 16.0]
    $cv itemconfigure $xfTmpTag  -fill $color -outline {}  -width {2.0}
}

# Procedure: getNewAvatarId
proc getNewAvatarId {} {
	global newAvatarId
    global avatarGetIdInputLabel
    global createAvatarOrPetFlag

	set newAvatarId ""
    set avatarGetIdInputLabel "(AvatarId posX posY)"
    set createAvatarOrPetFlag "avatar"
	
 	global xfShowWindow.topGetAvatarId
   set xfShowWindow.topGetAvatarId 1
   ShowWindow.topGetAvatarId
}

proc getNewPetId {} {
	global newAvatarId
    global avatarGetIdInputLabel
    global createAvatarOrPetFlag

	set newAvatarId ""
    set avatarGetIdInputLabel "(PetId posX posY OwnerId)"
    set createAvatarOrPetFlag "pet"
	
 	global xfShowWindow.topGetAvatarId
    set xfShowWindow.topGetAvatarId 1
    ShowWindow.topGetAvatarId
}

proc resetPhysiologicalModel {} {
	global pipeChannel

	set petIdSelected [.frame4.frame2.frame7.frame6.frame3.value cget -text]
	puts $pipeChannel "RESETPHYSIOLOGICALMODEL $petIdSelected"
    flush $pipeChannel
}

proc createAvatarOrPet {} {
    global createAvatarOrPetFlag 

    if { $createAvatarOrPetFlag == "pet" } {
        createNewPet
    } else {
        if { $createAvatarOrPetFlag == "avatar" } {
            createNewAvatar
        }
    }
}

# Procedure: createNewAvatar
proc createNewAvatar {} {
    global pipeChannel
    global newAvatarId

    if {$newAvatarId == ""} {
        error "Provide the avatar id and X and Y avatar position"
        return 
    }
    if {([lindex $newAvatarId 1] == "") || ([lindex $newAvatarId 2] == "")} {
        error "Provide the X and Y avatar position"
        return 
    }
    #exec echo "CREATEAVATAR $newAvatarId" >> log
    puts $pipeChannel "CREATEAVATAR $newAvatarId"

    flush $pipeChannel

    DestroyWindow.topGetAvatarId
}

proc createNewPet {} {
    global pipeChannel
    global newAvatarId 
    

    if {$newAvatarId == ""} {
        error "Provide the pet id, X and Y pet position and owner id"
        return 
    }
    if {([lindex $newAvatarId 1] == "") || ([lindex $newAvatarId 2] == "")} {
        error "Provide the X and Y pet position"
        return 
    }
    if {([lindex $newAvatarId 3] == "")} {
        error "Provide the owner id"
        return 
    }
    #exec echo "CREATEAVATAR $newAvatarId" >> log
    puts $pipeChannel "CREATEPET $newAvatarId"

    flush $pipeChannel

    DestroyWindow.topGetAvatarId
}

# Procedure: executeAvatarCommand
proc executeAvatarCommand { cmdName} {

    global actionPrototype
    global avatarActionName
    global avatarActionParameters
    global avatarActionSyntaxExample

    set avatarActionName $cmdName
    set avatarActionParameters ""

    set required [lindex $actionPrototype($cmdName) 0]
    set optional [lindex $actionPrototype($cmdName) 1]
    
    if {($required == "") && ($optional == "")} {
        sendAvatarAction
    } else {
        set avatarActionSyntaxExample [format "%s(" $cmdName]
        if {$required != ""} {
            set avatarActionSyntaxExample [format "%s%s" $avatarActionSyntaxExample [join $required ", "]]
        }
        if {$optional != ""} {
            if {$required != ""} {
                set avatarActionSyntaxExample [format "%s \[, " $avatarActionSyntaxExample]
            } else {
                set avatarActionSyntaxExample [format "%s\[" $avatarActionSyntaxExample]
            }
            set avatarActionSyntaxExample [format "%s%s\]" $avatarActionSyntaxExample [join $optional ", "]]
        }
        set avatarActionSyntaxExample [format "%s)" $avatarActionSyntaxExample]
        global xfShowWindow.topActionParameters
        set xfShowWindow.topActionParameters 1
        ShowWindow.topActionParameters
    }
}


# Procedure: getNextAvatarButtonName
proc getNextAvatarButtonName {} {

    global getNextAvatarButtonNameFirstTimeFlag
    global avatarActionButtonsList

    if {$getNextAvatarButtonNameFirstTimeFlag} {
        set getNextAvatarButtonNameFirstTimeFlag 0
        set avatarActionButtonsList {}
        for {set i 0} {$i <= 46} {incr i} {
            lappend avatarActionButtonsList "avatarButton$i"
        }
    }

    set answer [lindex $avatarActionButtonsList 0]
    set avatarActionButtonsList [lreplace $avatarActionButtonsList 0 0]

    if {$avatarActionButtonsList == ""} {
        set getNextAvatarButtonNameFirstTimeFlag 1
    }

    return $answer
}


# Procedure: nameAvatarActionButtons
proc nameAvatarActionButtons {} {
    set names [list ANTICIPATE_PLAY BARE_TEETH BARK BEG BLINK CHEW DIG DREAM DRINK DROP EARS_BACK PERK_EAR EARS_TWITCH EAT FLY FOLLOW GRAB GROWL HEEL HOWL JUMP_TOWARD JUMP_UP LICK LIE_DOWN MOVE_HEAD MOVE_TAIL NUDGE PANT PEE PLAY_DEAD POO ROLL_OVER SCRATCH SCRATCH_OTHER SIT SLEEP SNIFF STAY STRETCH TURN WAG_TAIL WAKE WALK WHINE KICK PET]
    for {set i 0} {$i <= 46} {incr i} {
        global avatarButton$i
        set avatarButton$i [lindex $names $i]
    }

    global actionPrototype

    set EMPTY ""

    set BOOLEAN "boolean"
    set INT "int"
    set FLOAT "float"
    set STRING "string"
    set VECTOR "($FLOAT X, $FLOAT Y, $FLOAT Z)"
    set ROTATION "($FLOAT Pitch, $FLOAT Roll, $FLOAT Yaw)"
    set ENTITY "($STRING ID, $STRING Type)"

    set FLOAT_and_ROTATION [list $FLOAT $ROTATION]
    set VECTOR_and_FLOAT [list $VECTOR $FLOAT]
    set ENTITY_and_FLOAT [list $ENTITY $FLOAT]
    set ENTITY_and_VECTOR [list $ENTITY $VECTOR]

    set VECTOR_ROTATION_and_FLOAT [list $VECTOR $ROTATION $FLOAT]

    set actionPrototype(BARK) [list $EMPTY [list $ENTITY_and_FLOAT]]
    set actionPrototype(ROLL_OVER) [list $EMPTY $EMPTY]
    set actionPrototype(EAT) [list [list $ENTITY] [list $FLOAT]]
    set actionPrototype(WALK) [list $VECTOR_and_FLOAT [list $ROTATION]]
    set actionPrototype(GRAB) [list [list $ENTITY] [list $FLOAT]]
    set actionPrototype(DROP) [list $EMPTY $EMPTY]
    set actionPrototype(SIT) [list $EMPTY [list $FLOAT]]
    set actionPrototype(LIE_DOWN) [list $EMPTY [list $FLOAT]]
    set actionPrototype(FLY) [list $VECTOR [list $FLOAT_and_ROTATION]]
    set actionPrototype(STRETCH) [list $EMPTY $EMPTY]
    set actionPrototype(SCRATCH_SELF_NOSE) [list $EMPTY [list $FLOAT]]
    set actionPrototype(DIG) [list $EMPTY [list $FLOAT]]
    set actionPrototype(ANTICIPATE_PLAY) [list $EMPTY $EMPTY]
    set actionPrototype(BEG) [list $EMPTY $EMPTY]
    set actionPrototype(HEEL) [list $EMPTY $FLOAT]
    set actionPrototype(STAY) [list $EMPTY $FLOAT]
    set actionPrototype(PLAY_DEAD) [list $EMPTY $EMPTY]
    set actionPrototype(FOLLOW) [list [list $ENTITY] $FLOAT]
    set actionPrototype(LICK) [list $EMPTY [list $ENTITY]]
    set actionPrototype(NUDGE_TO) [list $ENTITY_and_VECTOR $EMPTY]
    set actionPrototype(PANT) [list $EMPTY [list $FLOAT]]
    set actionPrototype(BARE_TEETH) [list $EMPTY [list $ENTITY_and_FLOAT]]
    set actionPrototype(GROWL) [list $EMPTY [list $ENTITY_and_FLOAT]]
    set actionPrototype(HOWL) [list $EMPTY $FLOAT]
    set actionPrototype(WHINE) [list $EMPTY [list $ENTITY_and_FLOAT]]
    set actionPrototype(SNIFF) [list $EMPTY $ENTITY]
    set actionPrototype(BLINK) [list $EMPTY $FLOAT]
    set actionPrototype(EARS_BACK) [list $EMPTY $FLOAT]
    set actionPrototype(EARS_TWITCH) [list $EMPTY $FLOAT]
    set actionPrototype(MOVE_HEAD) [list $VECTOR_ROTATION_and_FLOAT $EMPTY]
    set actionPrototype(WAKE) [list $EMPTY $EMPTY]
    set actionPrototype(SLEEP) [list $EMPTY [list $FLOAT]]
    set actionPrototype(DRINK) [list [list $ENTITY] [list $FLOAT]]
    set actionPrototype(PEE) [list $EMPTY $EMPTY]
    set actionPrototype(POO) [list $EMPTY $EMPTY]
    set actionPrototype(WAG) [list $EMPTY $FLOAT]
    set actionPrototype(TAIL_FLEX) [list $VECTOR [list $FLOAT]]
    set actionPrototype(CHEW) [list $EMPTY [list $ENTITY]]
    set actionPrototype(DREAM) [list $EMPTY [list $ENTITY]]
    set actionPrototype(TURN) [list $EMPTY [list $ROTATION]]
    set actionPrototype(SCRATCH_OTHER) [list $ENTITY [list $FLOAT]]
    set actionPrototype(EARS_PERK) [list $EMPTY $FLOAT]
    set actionPrototype(JUMP_UP) [list $EMPTY $EMPTY]
    set actionPrototype(JUMP_TOWARD) [list [list $VECTOR] $EMPTY]
    set actionPrototype(PAY_ATTENTION) [list $EMPTY $EMPTY]
    set actionPrototype(KICK) [list [list $ENTITY] [list $FLOAT]]
    set actionPrototype(PET) [list [list $ENTITY] [list $FLOAT]]
}


# Procedure: openParametersDialog
proc openParametersDialog {} {

}


# Procedure: processInput
proc processInput {} {
    global pipeChannel
    #exec echo processInput >> log
    fileevent $pipeChannel readable "processOneLine"
    #processOneLine
}


# Procedure: processOneLine
proc processOneLine {} {

    global statusLine

    #exec echo processOneLine >> log 
    global pipeChannel

    global hungerLevel
    global thirstLevel
    global peeUrgencyLevel
    global pooUrgencyLevel
    global fitnessLevel
    global energyLevel
    global listboxIndex

    fileevent $pipeChannel readable ""
    if {![eof $pipeChannel]} {
        if {[gets $pipeChannel line] != -1} {
            exec echo "line = $line" >> log
            switch [lindex [split $line] 0] {
                {PETSTATUS} {
                    set statusLine ""
                    set petId [lindex $line 1]
                    set petIdSelected [.frame4.frame2.frame7.frame6.frame3.value cget -text]

                    if { $petId == $petIdSelected } {
                        set [lindex $line 2] [lindex $line 3]
                        updatePetStatus
                    }
                }
                {PETACTION} {
                    set statusLine ""
                    set cmd [lindex [split $line ":"] end]
                    set ticket [lindex [split $line " "] 1]
                    set petId [lindex [split $line " "] 2]
                    set listboxIndex($ticket) [.frame5.frame0.frame2.listbox1 size]
                    .frame5.frame0.frame2.listbox1 insert end "ISSUED : $petId.$cmd"
                    .frame5.frame0.frame2.listbox1 see end
                    .frame5.frame0.frame2.listbox1 selection clear 0 end
                    .frame5.frame0.frame2.listbox1 selection set end
                }
                {PETACTIONSTATUS} {
                    set statusLine ""
                    set line [split $line " "]
                    set index [lindex $line 1]
                    set status [lindex $line 2]
                    if {$index == "last"} {
                        set index end
                    } else {
                        set index $listboxIndex($index)
                    }
                    exec echo "index = $index" >> log
                    set previousLine [split [.frame5.frame0.frame2.listbox1 get $index] ":"]
                    set newLine [lreplace $previousLine 0 0 $status]
                    .frame5.frame0.frame2.listbox1 delete $index
                    .frame5.frame0.frame2.listbox1 insert $index [join $newLine ":"]
                    .frame5.frame0.frame2.listbox1 see $index
                    .frame5.frame0.frame2.listbox1 selection clear 0 end
                    .frame5.frame0.frame2.listbox1 selection set $index
                }
                {AVATARCREATED} {
                    #exec echo "(AVATARCREATED) line = $line" >> log
                    set id [lindex [split $line] 1]
                    .frame5.frame0.frame1.frame0.menubutton2.m add command -command {OptionButtonSet .frame5.frame0.frame1.frame0} -label "$id"
                }
                {PETCREATED} {
                    set id [lindex [split $line] 1]
                    .frame4.frame2.frame7.frame6.frame3.menubutton2.m add command -command {OptionButtonSet .frame4.frame2.frame7.frame6.frame3} -label "$id"
 
                }
                {PETUNAVAILABLE} {
                    set id [lindex [split $line] 1]
                    error "Pet <$id> not available."
                }
                {ROUTERUNAVAILABLE} {
                    error "Router not available."
                }
              
            }
        }
        after idle processInput
    }
}


# Procedure: sendAvatarAction
proc sendAvatarAction {} {
    
    global avatarActionName
    global avatarActionParameters

    set convertedActionName ""

	 set SendToAvatarId  [.frame5.frame0.frame1.frame0.value cget -text]
	 if { $SendToAvatarId == "" } {
		 error "You must select an avatar id"
		 return
	 }
     #exec echo "Send to AvatarID: $SendToAvatarId" >> log

    set tmp [split [string tolower $avatarActionName] "_"]
    for {set i 0} {$i < [llength $tmp]} {incr i} {
        set word [lindex $tmp $i]
        if {$i != 0} {
            set firstChar [string index $word 0]
            set word [string replace $word 0 0 [string toupper $firstChar]]
        }
        lappend convertedActionName $word
    }
    set convertedActionName [join $convertedActionName ""]

    set convertedActionParameters ""
    set tmp [split $avatarActionParameters " ,)(\[\]\{\};"]
    foreach s $tmp {
        if {$s != ""} {
            lappend convertedActionParameters $s
        }
    }
    set convertedActionParameters [join $convertedActionParameters " "]


    global pipeChannel

    set text [format "%s %s" $convertedActionName $convertedActionParameters]
    #exec echo $text >> log
    puts $pipeChannel "AVATARACTION $SendToAvatarId $text"
	 
    flush $pipeChannel
    DestroyWindow.topActionParameters
}


# Procedure: sendPredavese
proc sendPredavese { text} {

    if {$text == ""} {
        return
    }
    set petIdSelected [.frame4.frame2.frame7.frame6.frame3.value cget -text]

    global pipeChannel

    #exec echo $text >> log
    puts $pipeChannel "PREDAVESE $petIdSelected $text"
    flush $pipeChannel
}


# Procedure: showLog
proc showLog { log} {

    global env

    set fname [format "/tmp/%s/Petaverse/Logs/%s" $env(LOGNAME) $log]
    set f [open $fname "r"]
    .frame5.frame1.frame.text2 delete 1.0 end
    while {[gets $f line] != -1} {
        .frame5.frame1.frame.text2 insert end "$line\n"
    }
}


# Procedure: startPVPSimulator
proc startPVPSimulator {} {
    global pipeChannel
    set pipeChannel [open "|./pvpSimulator" "r+"]
    #set pipeChannel [open "|InterfaceUTest.sleep0" "r+"]
    after idle "processInput"
}


# Procedure: updatePetStatus
proc updatePetStatus {} {

    #exec echo updatePetStatus >> log
    global hungerLevel
    global thirstLevel
    global peeUrgencyLevel
    global pooUrgencyLevel
    global fitnessLevel
    global energyLevel

    drawBar .frame4.frame2.frame7.frame6.frame3.frame9.frame1.canvas9 $hungerLevel 0
    drawBar .frame4.frame2.frame7.frame6.frame3.frame9.frame2.canvas9 $thirstLevel 0
    drawBar .frame4.frame2.frame7.frame6.frame3.frame10.frame3.canvas9 $peeUrgencyLevel 0
    drawBar .frame4.frame2.frame7.frame6.frame3.frame10.frame4.canvas9 $pooUrgencyLevel 0
    drawBar .frame4.frame2.frame7.frame6.frame3.frame7.frame5.canvas9 $fitnessLevel 1
    drawBar .frame4.frame2.frame7.frame6.frame3.frame7.frame6.canvas9 $energyLevel 1
}


# User defined images


# Internal procedures


# Procedure: Alias
if {"[info procs Alias]" == ""} {
proc Alias { args} {
# xf ignore me 7
##########
# Procedure: Alias
# Description: establish an alias for a procedure
# Arguments: args - no argument means that a list of all aliases
#                   is returned. Otherwise the first parameter is
#                   the alias name, and the second parameter is
#                   the procedure that is aliased.
# Returns: nothing, the command that is bound to the alias or a
#          list of all aliases - command pairs. 
# Sideeffects: internalAliasList is updated, and the alias
#              proc is inserted
##########
  global internalAliasList

  if {[llength $args] == 0} {
    return $internalAliasList
  } {
    if {[llength $args] == 1} {
      set xfTmpIndex [lsearch $internalAliasList "[lindex $args 0] *"]
      if {$xfTmpIndex != -1} {
        return [lindex [lindex $internalAliasList $xfTmpIndex] 1]
      }
    } {
      if {[llength $args] == 2} {
        eval "proc [lindex $args 0] {args} {#xf ignore me 4
return \[eval \"[lindex $args 1] \$args\"\]}"
        set xfTmpIndex [lsearch $internalAliasList "[lindex $args 0] *"]
        if {$xfTmpIndex != -1} {
          set internalAliasList [lreplace $internalAliasList $xfTmpIndex $xfTmpIndex "[lindex $args 0] [lindex $args 1]"]
        } {
          lappend internalAliasList "[lindex $args 0] [lindex $args 1]"
        }
      } {
        error "Alias: wrong number or args: $args"
      }
    }
  }
}
}


# Procedure: Cat
if {"[info procs Cat]" == ""} {
proc Cat { filename} {
# xf ignore me 7
##########
# Procedure: Cat
# Description: emulate UNIX cat for one file
# Arguments: filename
# Returns: file contents
# Sideeffects: none
##########
global tcl_platform
if {$tcl_platform(platform) == "unix"} {exec cat $filename} {
   set fileid [open $filename "r"]
   set data [read $fileid]
   close $fileid
   return $data
}
}
}


# Procedure: Chmod
if {"[info procs Chmod]" == ""} {
proc Chmod { mode file} {
# xf ignore me 7
##########
# Procedure: Chmod
# Description: ignore UNIX chmod under DOS
# Arguments: file1 file2/directory
# Returns: nothing
# Sideeffects: none
##########
global tcl_platform
if {$tcl_platform(platform) == "unix"} {eval exec chmod $mode $file} {
   regsub -all {/} $file1 {\\\\} file1
   regsub -all {/} $file2 {\\\\} file2
   eval exec command.com /c copy /y $file1 $file2 >@stderr
}
}
}


# Procedure: Cp
if {"[info procs Cp]" == ""} {
proc Cp { file1 file2} {
# xf ignore me 7
##########
# Procedure: Cp
# Description: emulate UNIX cp with DOS COPY
# Arguments: file1 file2/directory
# Returns: nothing
# Sideeffects: none
##########
global tcl_platform
if {$tcl_platform(platform) == "unix"} {eval exec cp $file1 $file2} {
   regsub -all {/} $file1 {\\\\} file1
   regsub -all {/} $file2 {\\\\} file2
   eval exec command.com /c copy /y $file1 $file2 >@stderr
}
}
}


# Procedure: GetSelection
if {"[info procs GetSelection]" == ""} {
proc GetSelection {} {
# xf ignore me 7
##########
# Procedure: GetSelection
# Description: get current selection
# Arguments: none
# Returns: none
# Sideeffects: none
##########

  # the save way
  set xfSelection ""
  catch "selection get" xfSelection
  if {"$xfSelection" == "selection doesn't exist or form \"STRING\" not defined"} {
    return ""
  } {
    return $xfSelection
  }
}
}


# Procedure: Ls
if {"[info procs Ls]" == ""} {
proc Ls { args} {
# xf ignore me 7
##########
# Procedure: Ls
# Description: emulate UNIX ls
# Arguments: like UNIX (switches authorized -F -a [-a is ignored])
# Returns: directory list
# Sideeffects: none
##########
global tcl_platform
if {$tcl_platform(platform) == "unix"} {eval exec ls $args} {
   set last [lindex $args end]
   if {[lsearch $last -* ] >= 0 || $last == "" } {set path "*"} {set path $last/*}
   set lst [glob $path]
   set lst1 ""
   if {[lsearch -exact $args "-F"] >= 0} then {set car "/"} else {set car ""}
   foreach f $lst {
      if {[file isdirectory $f]}  {set f [file tail $f]$car} {set f [file tail $f]}
      lappend lst1 $f
   }
   return [lsort $lst1]
}
}
}


# Procedure: NoFunction
if {"[info procs NoFunction]" == ""} {
proc NoFunction { args} {
# xf ignore me 7
##########
# Procedure: NoFunction
# Description: do nothing (especially with scales and scrollbars)
# Arguments: args - a number of ignored parameters
# Returns: none
# Sideeffects: none
##########
}
}


# Procedure: Rm
if {"[info procs Rm]" == ""} {
proc Rm { filename} {
# xf ignore me 7
##########
# Procedure: Rm
# Description: emulate UNIX rm with DOS DEL
# Arguments: filename(s)
# Returns: nothing
# Sideeffects: none
##########
global tcl_platform
if {$tcl_platform(platform) == "unix"} {exec rm -f $filename} {
   regsub -all {/} $filename {\\\\} filename
   eval exec command.com /c del $filename >@stderr
}
}
}


# Procedure: SN
if {"[info procs SN]" == ""} {
proc SN { {xfName ""}} {
# xf ignore me 7
##########
# Procedure: SN
# Description: map a symbolic name to the widget path
# Arguments: xfName
# Returns: the symbolic name
# Sideeffects: none
##########

  SymbolicName $xfName
}
}


# Procedure: SymbolicName
if {"[info procs SymbolicName]" == ""} {
proc SymbolicName { {xfName ""}} {
# xf ignore me 7
##########
# Procedure: SymbolicName
# Description: map a symbolic name to the widget path
# Arguments: xfName
# Returns: the symbolic name
# Sideeffects: none
##########

  global symbolicName

  if {"$xfName" != ""} {
    set xfArrayName ""
    append xfArrayName symbolicName ( $xfName )
    if {![catch "set \"$xfArrayName\"" xfValue]} {
      return $xfValue
    } {
      if {"[info commands XFProcError]" != ""} {
        XFProcError "Unknown symbolic name:\n$xfName"
      } {
        puts stderr "XF error: unknown symbolic name:\n$xfName"
      }
    }
  }
  return ""
}
}


# Procedure: Unalias
if {"[info procs Unalias]" == ""} {
proc Unalias { aliasName} {
# xf ignore me 7
##########
# Procedure: Unalias
# Description: remove an alias for a procedure
# Arguments: aliasName - the alias name to remove
# Returns: none
# Sideeffects: internalAliasList is updated, and the alias
#              proc is removed
##########
  global internalAliasList

  set xfIndex [lsearch $internalAliasList "$aliasName *"]
  if {$xfIndex != -1} {
    rename $aliasName ""
    set internalAliasList [lreplace $internalAliasList $xfIndex $xfIndex]
  }
}
}



# application parsing procedure
proc XFLocalParseAppDefs {xfAppDefFile} {
  global xfAppDefaults tcl_platform

  # basically from: Michael Moore
  if {[file exists $xfAppDefFile] &&
      [file readable $xfAppDefFile] &&
      "[file type $xfAppDefFile]" == "link"} {
    catch "file type $xfAppDefFile" xfType
    while {"$xfType" == "link"} {
      if {[catch "file readlink $xfAppDefFile" xfAppDefFile]} {
        return
      }
      catch "file type $xfAppDefFile" xfType
    }
  }
  if {!("$xfAppDefFile" != "" &&
        [file exists $xfAppDefFile] &&
        [file readable $xfAppDefFile] &&
        "[file type $xfAppDefFile]" == "file")} {
    return
  }
  if {![catch "open $xfAppDefFile r" xfResult]} {
    set xfAppFileContents [read $xfResult]
    close $xfResult
    foreach line [split $xfAppFileContents "\n"] {
      # backup indicates how far to backup.  It applies to the
      # situation where a resource name ends in . and when it
      # ends in *.  In the second case you want to keep the *
      # in the widget name for pattern matching, but you want
      # to get rid of the . if it is the end of the name. 
      set backup -2  
      set line [string trim $line]
      if {[string index $line 0] == "#" || "$line" == ""} {
        # skip comments and empty lines
        continue
      }
      if {![string compare "windows" $tcl_platform(platform)]} {
        set list [split $line ";"]
      } {
        set list [split $line ":"]
      }
      set resource [string trim [lindex $list 0]]
      set i [string last "." $resource]
      set j [string last "*" $resource]
      if {$j > $i} { 
        set i $j
        set backup -1
      }
      incr i
      set name [string range $resource $i end]
      incr i $backup
      set widname [string range $resource 0 $i]
      set value [string trim [lindex $list 1]]
      if {"$widname" != "" && "$widname" != "*"} {
        # insert the widget and resourcename to the application
        # defaults list.
        if {![info exists xfAppDefaults]} {
          set xfAppDefaults ""
        }
        lappend xfAppDefaults [list $widname [string tolower $name] $value]
      }
    }
  }
}

# application loading procedure
proc XFLocalLoadAppDefs {{xfClasses ""} {xfPriority "startupFile"} {xfAppDefFile ""}} {
  global env tcl_platform

  if {![string compare "windows" $tcl_platform(platform)]} {
    set separator ";"
  } {
    set separator ":"
  }
  if {"$xfAppDefFile" == ""} {
    set xfFileList ""
    if {[info exists env(XUSERFILESEARCHPATH)]} {
      eval lappend xfFileList [split $env(XUSERFILESEARCHPATH) $separator]
    }
    if {[info exists env(XAPPLRESDIR)]} {
      eval lappend xfFileList [split $env(XAPPLRESDIR) $separator]
    }
    if {[info exists env(XFILESEARCHPATH)]} {
      eval lappend xfFileList [split $env(XFILESEARCHPATH) $separator]
    }
    append xfFileList " /usr/lib/X11/app-defaults"
    append xfFileList " /usr/X11/lib/X11/app-defaults"

    foreach xfCounter1 $xfClasses {
      foreach xfCounter2 $xfFileList {
        set xfPathName $xfCounter2
        if {[regsub -all "%N" "$xfPathName" "$xfCounter1" xfResult]} {
          set xfPathName $xfResult
        }
        if {[regsub -all "%T" "$xfPathName" "app-defaults" xfResult]} {
          set xfPathName $xfResult
        }
        if {[regsub -all "%S" "$xfPathName" "" xfResult]} {
          set xfPathName $xfResult
        }
        if {[regsub -all "%C" "$xfPathName" "" xfResult]} {
          set xfPathName $xfResult
        }
        if {[file exists $xfPathName] &&
            [file readable $xfPathName] &&
            ("[file type $xfPathName]" == "file" ||
             "[file type $xfPathName]" == "link")} {
          catch "option readfile $xfPathName $xfPriority"
          if {"[info commands XFParseAppDefs]" != ""} {
            XFParseAppDefs $xfPathName
          } {
            if {"[info commands XFLocalParseAppDefs]" != ""} {
              XFLocalParseAppDefs $xfPathName
            }
          }
        } {
          if {[file exists $xfCounter2/$xfCounter1] &&
              [file readable $xfCounter2/$xfCounter1] &&
              ("[file type $xfCounter2/$xfCounter1]" == "file" ||
               "[file type $xfCounter2/$xfCounter1]" == "link")} {
            catch "option readfile $xfCounter2/$xfCounter1 $xfPriority"
            if {"[info commands XFParseAppDefs]" != ""} {
              XFParseAppDefs $xfCounter2/$xfCounter1
            } {
              if {"[info commands XFLocalParseAppDefs]" != ""} {
                XFLocalParseAppDefs $xfCounter2/$xfCounter1
              }
            }
          }
        }
      }
    }
  } {
    # load a specific application defaults file
    if {[file exists $xfAppDefFile] &&
        [file readable $xfAppDefFile] &&
        ("[file type $xfAppDefFile]" == "file" ||
         "[file type $xfAppDefFile]" == "link")} {
      catch "option readfile $xfAppDefFile $xfPriority"
      if {"[info commands XFParseAppDefs]" != ""} {
        XFParseAppDefs $xfAppDefFile
      } {
        if {"[info commands XFLocalParseAppDefs]" != ""} {
          XFLocalParseAppDefs $xfAppDefFile
        }
      }
    }
  }
}

# application setting procedure
proc XFLocalSetAppDefs {{xfWidgetPath "."}} {
  global xfAppDefaults

  if {![info exists xfAppDefaults]} {
    return
  }
  foreach xfCounter $xfAppDefaults {
    if {"$xfCounter" == ""} {
      break
    }
    set widname [lindex $xfCounter 0]
    if {[string match $widname ${xfWidgetPath}] ||
        [string match "${xfWidgetPath}*" $widname]} {
      set name [string tolower [lindex $xfCounter 1]]
      set value [lindex $xfCounter 2]
      # Now lets see how many tcl commands match the name
      # pattern specified.
      set widlist [info command $widname]
      if {"$widlist" != ""} {
        foreach widget $widlist {
          # make sure this command is a widget.
          if {![catch "winfo id $widget"] &&
              [string match "${xfWidgetPath}*" $widget]} {
            catch "$widget configure -$name $value" 
          }
        }
      }
    }
  }
}

# initialize bindings for all widgets
proc XFInitAllBindings {} {
  # bindings
  bind all <<PrevWindow>> {tk::TabToWindow [tk_focusPrev %W]}
  bind all <Alt-Key> {
	tk::TraverseToMenu %W %A
    }
  bind all <Key-F10> {
	tk::FirstMenu %W
    }
  bind all <Key-Tab> {tk::TabToWindow [tk_focusNext %W]}
}


# startup source
proc StartupSrc {args} {
    nameAvatarActionButtons
}


# end source
proc EndSrc {} {

}

# prepare auto loading
global auto_path
global tk_library
global xfLoadPath
foreach xfElement [eval list [split $xfLoadPath :] $auto_path] {
  if {[file exists $xfElement/tclIndex]} {
    lappend auto_path $xfElement
  }
}

# startup source
StartupSrc

# initialize global variables
proc InitGlobals {} {
  global {actionPrototype}
  set {actionPrototype(ANTICIPATE_PLAY)} {{} {}}
  set {actionPrototype(BARE_TEETH)} {{} {{(string ID, string Type)} {float duration}}}
  set {actionPrototype(BARK)} {{} {{(string ID, string Type)} {float duration}}}
  set {actionPrototype(BEG)} {{} {}}
  set {actionPrototype(BLINK)} {{} {float duration}}
  set {actionPrototype(CHEW)} {{} {{(string ID, string Type)}}}
  set {actionPrototype(DIG)} {{} {float duration}}
  set {actionPrototype(DREAM)} {{} {{(string ID, string Type)}}}
  set {actionPrototype(DRINK)} {{{(string ID, string Type)}} float}
  set {actionPrototype(DROP)} {{} {}}
  set {actionPrototype(EARS_BACK)} {{} {float duration}}
  set {actionPrototype(EARS_PERK)} {{} {float duration}}
  set {actionPrototype(EARS_TWITCH)} {{} {float duration}}
  set {actionPrototype(EAT)} {{{(string ID, string Type)}} float}
  set {actionPrototype(FLY)} {{{(float X, float Y, float Z)} float} {{(float Pitch, float Roll, float Yaw)}}}
  set {actionPrototype(FOLLOW)} {{{(string ID, string Type)}} float}
  set {actionPrototype(GRAB)} {{{(string ID, string Type)}} float}
  set {actionPrototype(GROWL)} {{} {{(string ID, string Type)} {float duration}}}
  set {actionPrototype(HEEL)} {{} {float duration}}
  set {actionPrototype(HOWL)} {{} {float duration}}
  set {actionPrototype(JUMP_TOWARD)} {{{(float X, float Y, float Z)}} {}}
  set {actionPrototype(JUMP_UP)} {{} {}}
  set {actionPrototype(LICK)} {{} {{(string ID, string Type)}}}
  set {actionPrototype(LIE_DOWN)} {{} {float duration}}
  set {actionPrototype(MOVE_HEAD)} {{{(float X, float Y, float Z)} {(float Pitch, float Roll, float Yaw)} float} {}}
  set {actionPrototype(TAIL_FLEX)} {{{(float X, float Y, float Z)}} {float duration}}
  set {actionPrototype(NUDGE_TO)} {{} {{(string ID, string Type)} {(float X, float Y, float Z)} {(float Pitch, float Roll, float Yaw)}}}
  set {actionPrototype(PANT)} {{} {float duration}}
  set {actionPrototype(PAY_ATTENTION)} {{} {}}
  set {actionPrototype(PEE)} {{} {}}
  set {actionPrototype(PLAY_DEAD)} {{} {}}
  set {actionPrototype(POO)} {{} {}}
  set {actionPrototype(ROLL_OVER)} {{} {}}
  set {actionPrototype(SCRATCH_SELF_NOSE)} {{} {float duration}}
  set {actionPrototype(SCRATCH_OTHER)} {{{(string ID, string Type)}} {float duration}}
  set {actionPrototype(SIT)} {{} {float duration}}
  set {actionPrototype(SLEEP)} {{} {float duration}}
  set {actionPrototype(SNIFF)} {{} {{(string ID, string Type)}}}
  set {actionPrototype(STAY)} {{} {float duration}}
  set {actionPrototype(STRETCH)} {{} {}}
  set {actionPrototype(TURN)} {{} {{(float Pitch, float Roll, float Yaw)}}}
  set {actionPrototype(WAG)} {{} {float duration}}
  set {actionPrototype(WAKE)} {{} {}}
  set {actionPrototype(WALK)} {{{(float X, float Y, float Z)} float} {{(float Pitch, float Roll, float Yaw)}}}
  set {actionPrototype(WHINE)} {{} {{(string ID, string Type)} {float duration}}}
  set {actionPrototype(KICK)} {{{(string ID, string Type)}} float}
  set {actionPrototype(PET)} {{{(string ID, string Type)}} float}


  global {energyLevel}
  set {energyLevel} {0.0}
  global {entry_ENTITY_optional_id}
  set {entry_ENTITY_optional_id} {}
  global {entry_ENTITY_optional_type}
  set {entry_ENTITY_optional_type} {}
  global {entry_ENTITY_required_id}
  set {entry_ENTITY_required_id} {}
  global {entry_ENTITY_required_type}
  set {entry_ENTITY_required_type} {}
  global {entry_ROTATION_optional_pitch}
  set {entry_ROTATION_optional_pitch} {}
  global {entry_ROTATION_optional_roll}
  set {entry_ROTATION_optional_roll} {}
  global {entry_ROTATION_optional_yaw}
  set {entry_ROTATION_optional_yaw} {}
  global {entry_ROTATION_required_pitch}
  set {entry_ROTATION_required_pitch} {}
  global {entry_ROTATION_required_roll}
  set {entry_ROTATION_required_roll} {}
  global {entry_ROTATION_required_yaw}
  set {entry_ROTATION_required_yaw} {}
  global {entry_SINGLE_VALUE_optional}
  set {entry_SINGLE_VALUE_optional} {entry17}
  global {entry_SINGLE_VALUE_required}
  set {entry_SINGLE_VALUE_required} {entry17}
  global {entry_VECTOR_optional_x}
  set {entry_VECTOR_optional_x} {entry17}
  global {entry_VECTOR_optional_y}
  set {entry_VECTOR_optional_y} {entry17}
  global {entry_VECTOR_optional_z}
  set {entry_VECTOR_optional_z} {entry17}
  global {entry_VECTOR_required_x}
  set {entry_VECTOR_required_x} {entry17}
  global {entry_VECTOR_required_y}
  set {entry_VECTOR_required_y} {entry17}
  global {entry_VECTOR_required_z}
  set {entry_VECTOR_required_z} {entry17}
  global {fitnessLevel}
  set {fitnessLevel} {0.0}
  global {getNextAvatarButtonNameFirstTimeFlag}
  set {getNextAvatarButtonNameFirstTimeFlag} {1}
  global {hungerLevel}
  set {hungerLevel} {0.0}
  global {avatarActionButtonsList}
  set {avatarActionButtonsList} {}
  global {avatarActionName}
  set {avatarActionName} {ANTICIPATE_PLAY}
  global {avatarGetIdInputLabel}
  set {avatarGetIdInputLabel} {}
  global {createAvatarOrPetFlag}
  set {createAvatarOrPetFlag} {}
  global {avatarActionParameters}
  set {avatarActionParameters} {}
  global {avatarActionSyntaxExample}
  set {avatarActionSyntaxExample} {ANTICIPATE_PLAY()}
  global {avatarButton0}
  set {avatarButton0} {ANTICIPATE_PLAY}
  global {avatarButton1}
  set {avatarButton1} {BARE_TEETH}
  global {avatarButton10}
  set {avatarButton10} {EARS_BACK}
  global {avatarButton11}
  set {avatarButton11} {EARS_PERK}
  global {avatarButton12}
  set {avatarButton12} {EARS_TWITCH}
  global {avatarButton13}
  set {avatarButton13} {EAT}
  global {avatarButton14}
  set {avatarButton14} {FLY}
  global {avatarButton15}
  set {avatarButton15} {FOLLOW}
  global {avatarButton16}
  set {avatarButton16} {GRAB}
  global {avatarButton17}
  set {avatarButton17} {GROWL}
  global {avatarButton18}
  set {avatarButton18} {HEEL}
  global {avatarButton19}
  set {avatarButton19} {HOWL}
  global {avatarButton2}
  set {avatarButton2} {BARK}
  global {avatarButton20}
  set {avatarButton20} {JUMP_TOWARD}
  global {avatarButton21}
  set {avatarButton21} {JUMP_UP}
  global {avatarButton22}
  set {avatarButton22} {LICK}
  global {avatarButton23}
  set {avatarButton23} {LIE_DOWN}
  global {avatarButton24}
  set {avatarButton24} {MOVE_HEAD}
  global {avatarButton25}
  set {avatarButton25} {TAIL_FLEX}
  global {avatarButton26}
  set {avatarButton26} {NUDGE_TO}
  global {avatarButton27}
  set {avatarButton27} {PANT}
  global {avatarButton28}
  set {avatarButton28} {PEE}
  global {avatarButton29}
  set {avatarButton29} {PLAY_DEAD}
  global {avatarButton3}
  set {avatarButton3} {BEG}
  global {avatarButton30}
  set {avatarButton30} {POO}
  global {avatarButton31}
  set {avatarButton31} {ROLL_OVER}
  global {avatarButton32}
  set {avatarButton32} {SCRATCH_SELF_NOSE}
  global {avatarButton33}
  set {avatarButton33} {SCRATCH_OTHER}
  global {avatarButton34}
  set {avatarButton34} {SIT}
  global {avatarButton35}
  set {avatarButton35} {SLEEP}
  global {avatarButton36}
  set {avatarButton36} {SNIFF}
  global {avatarButton37}
  set {avatarButton37} {STAY}
  global {avatarButton38}
  set {avatarButton38} {STRETCH}
  global {avatarButton39}
  set {avatarButton39} {TURN}
  global {avatarButton4}
  set {avatarButton4} {BLINK}
  global {avatarButton40}
  set {avatarButton40} {WAG}
  global {avatarButton41}
  set {avatarButton41} {WAKE}
  global {avatarButton42}
  set {avatarButton42} {WALK}
  global {avatarButton43}
  set {avatarButton43} {WHINE}
  global {avatarButton44}
  set {avatarButton44} {GREET}
  global {avatarButton45}
  set {avatarButton45} {KICK}
  global {avatarButton46}
  set {avatarButton46} {PET}

  global {avatarButton5}
  set {avatarButton5} {CHEW}
  global {avatarButton6}
  set {avatarButton6} {DIG}
  global {avatarButton7}
  set {avatarButton7} {DREAM}
  global {avatarButton8}
  set {avatarButton8} {DRINK}
  global {avatarButton9}
  set {avatarButton9} {DROP}
  global {peeUrgencyLevel}
  set {peeUrgencyLevel} {0.0}
  global {pooUrgencyLevel}
  set {pooUrgencyLevel} {0.0}
  global {predaveseText}
  set {predaveseText} {}
  global {newAvatarId}
  set {newAvatarId} {}
  global {newPetId}
  set {newPetId} {}
  global {reqopt_SINGLE_VALUE}
  set {reqopt_SINGLE_VALUE} {required}
  global {singleValueReqOpt}
  set {singleValueReqOpt} {required}
  global {singleValueType}
  set {singleValueType} {Boolean (0 or 1):}
  global {statusLine}
  set {statusLine} {}
  global {textBox}
  set {textBox(activeBackground)} {}
  set {textBox(activeForeground)} {}
  set {textBox(background)} {}
  set {textBox(button)} {0}
  set {textBox(contents)} {}
  set {textBox(font)} {}
  set {textBox(foreground)} {}
  set {textBox(scrollActiveForeground)} {}
  set {textBox(scrollBackground)} {}
  set {textBox(scrollForeground)} {}
  set {textBox(scrollSide)} {left}
  set {textBox(state)} {disabled}
  set {textBox(toplevelName)} {.textBox}
  global {thirstLevel}
  set {thirstLevel} {0.0}
  global {type_SINGLE_VALUE}
  set {type_SINGLE_VALUE} {Boolean (0 or 1):}
  global {type_SINGLE_VALUE_optional}
  set {type_SINGLE_VALUE_optional} {Boolean (0 or 1):}
  global {type_SINGLE_VALUE_required}
  set {type_SINGLE_VALUE_required} {Boolean (0 or 1):}

  # please don't modify the following
  # variables. They are needed by xf.
  global {autoLoadList}
  set {autoLoadList(PVPSimulatorInterface.tcl)} {0}
  global {internalAliasList}
  set {internalAliasList} {}
  global {moduleList}
  set {moduleList(PVPSimulatorInterface.tcl)} {}
  global {preloadList}
  set {preloadList(xfInternal)} {}
  global {symbolicName}
  set {symbolicName(root)} {.}
  global {xfWmSetPosition}
  set {xfWmSetPosition} {}
  global {xfWmSetSize}
  set {xfWmSetSize} {.}
  global {xfAppDefToplevels}
  set {xfAppDefToplevels} {}
}

# initialize global variables
InitGlobals

# display/remove toplevel windows.
ShowWindow.

# load default bindings.
if {[info exists env(XF_BIND_FILE)] &&
    "[info procs XFShowHelp]" == ""} {
  source $env(XF_BIND_FILE)
}

# initialize bindings for all widgets.
XFInitAllBindings


# parse and apply application defaults.
XFLocalLoadAppDefs PVPSimulatorInterface
XFLocalSetAppDefs

# end source
EndSrc

# eof
#

