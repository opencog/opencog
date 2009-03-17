#!/usr/bin/wish -f
# Program: PetInterface
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


# procedure to show window .
proc ShowWindow. {args} {# xf ignore me 7

  # Window manager configurations
  wm positionfrom . user
  wm sizefrom . ""
  wm maxsize . 1265 770
  wm minsize . 1 1
  wm protocol . WM_DELETE_WINDOW {XFProcError {Application windows can not be destroyed.
Please use the "Current widget path:" to show/hide windows.}}
  wm title . {PetInterface.tcl}


  # build widget .frame0
  frame .frame0 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame0.frame2
  frame .frame0.frame2 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame0.frame2.label4
  label .frame0.frame2.label4 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {Basic goals:}

  # build widget .frame0.frame2.frame6
  frame .frame0.frame2.frame6 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame0.frame2.frame6.frame7
  frame .frame0.frame2.frame6.frame7 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame0.frame2.frame6.frame7.label10
  label .frame0.frame2.frame6.frame7.label10 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {AlleviateHunger:}

  # build widget .frame0.frame2.frame6.frame7.canvas11
  canvas .frame0.frame2.frame6.frame7.canvas11 \
    -background {#ffffff} \
    -height {16} \
    -highlightbackground {#ffffff} \
    -relief {raised} \
    -width {100}

  # build widget .frame0.frame2.frame6.frame3
  frame .frame0.frame2.frame6.frame3 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame0.frame2.frame6.frame3.label10
  label .frame0.frame2.frame6.frame3.label10 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {AlleviateThirst:}

  # build widget .frame0.frame2.frame6.frame3.canvas11
  canvas .frame0.frame2.frame6.frame3.canvas11 \
    -background {#ffffff} \
    -height {16} \
    -highlightbackground {#ffffff} \
    -relief {raised} \
    -width {100}

  # build widget .frame0.frame2.frame6.frame5
  frame .frame0.frame2.frame6.frame5 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame0.frame2.frame6.frame5.label10
  label .frame0.frame2.frame6.frame5.label10 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {SatisfyOwner:}

  # build widget .frame0.frame2.frame6.frame5.canvas11
  canvas .frame0.frame2.frame6.frame5.canvas11 \
    -background {#ffffff} \
    -height {16} \
    -highlightbackground {#ffffff} \
    -relief {raised} \
    -width {100}

  # build widget .frame0.frame0
  frame .frame0.frame0 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame0.frame0.label4
  label .frame0.frame0.label4 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {Feelings:}

  # build widget .frame0.frame0.frame6
  frame .frame0.frame0.frame6 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame0.frame0.frame6.frame7
  frame .frame0.frame0.frame6.frame7 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame0.frame0.frame6.frame7.label10
  label .frame0.frame0.frame6.frame7.label10 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {Hunger:}

  # build widget .frame0.frame0.frame6.frame7.canvas11
  canvas .frame0.frame0.frame6.frame7.canvas11 \
    -background {#ffffff} \
    -height {16} \
    -highlightbackground {#ffffff} \
    -relief {raised} \
    -width {100}

  # build widget .frame0.frame0.frame6.frame8
  frame .frame0.frame0.frame6.frame8 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame0.frame0.frame6.frame8.label10
  label .frame0.frame0.frame6.frame8.label10 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {Thirst:}

  # build widget .frame0.frame0.frame6.frame8.canvas11
  canvas .frame0.frame0.frame6.frame8.canvas11 \
    -background {#ffffff} \
    -height {16} \
    -highlightbackground {#ffffff} \
    -relief {raised} \
    -width {100}

  # build widget .frame0.frame4
  frame .frame0.frame4 \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame0.frame4.frame0
  frame .frame0.frame4.frame0 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame0.frame4.frame0.label6
  label .frame0.frame4.frame0.label6 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {Last selected goal:}

  # build widget .frame0.frame4.frame0.label2
  label .frame0.frame4.frame0.label2 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -text {Last selected schema:}

  # build widget .frame0.frame4.frame3
  frame .frame0.frame4.frame3 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame0.frame4.frame3.label0
  label .frame0.frame4.frame3.label0 \
    -activebackground {#eeeff2} \
    -anchor {nw} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -textvariable {lastSelectedGoal} \
    -width {15}

  # build widget .frame0.frame4.frame3.label7
  label .frame0.frame4.frame3.label7 \
    -activebackground {#eeeff2} \
    -anchor {sw} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2} \
    -textvariable {lastSelectedSchema} \
    -width {15}

  # build widget .frame0.frame1
  frame .frame0.frame1 \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame0.frame1.checkbutton4
  checkbutton .frame0.frame1.checkbutton4 \
    -activebackground {#eeeff2} \
    -background {#eeeff2} \
    -command {drawLastMap} \
    -highlightbackground {#eeeff2} \
    -text {Show names (map)} \
    -variable {showObjectNames}

  # build widget .frame0.frame3
  frame .frame0.frame3 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -highlightbackground {#eeeff2}

  # build widget .frame0.frame3.button5
  button .frame0.frame3.button5 \
    -activebackground {#bbbcc0} \
    -background {#eeeff2} \
    -command {exit} \
    -highlightbackground {#eeeff2} \
    -text {Close GUI} \
    -width {15}

  # build widget .frame1
  frame .frame1 \
    -borderwidth {2} \
    -background {#eeeff2} \
    -height {30} \
    -highlightbackground {#eeeff2} \
    -width {30}

  # build widget .frame1.canvas2
  canvas .frame1.canvas2 \
    -background {#44ff99} \
    -height {512} \
    -highlightbackground {#44ff99} \
    -width {1024}

  # pack master .frame0
  pack configure .frame0.frame2 \
    -anchor nw \
    -side left
  pack configure .frame0.frame0 \
    -anchor nw \
    -padx 32 \
    -side left
  pack configure .frame0.frame4 \
    -anchor nw \
    -side left
  pack configure .frame0.frame1 \
    -anchor nw \
    -padx 21 \
    -side left
  pack configure .frame0.frame3 \
    -anchor se \
    -side left

  # pack master .frame0.frame2
  pack configure .frame0.frame2.label4 \
    -anchor w
  pack configure .frame0.frame2.frame6 \
    -anchor e

  # pack master .frame0.frame2.frame6
  pack configure .frame0.frame2.frame6.frame7 \
    -anchor e
  pack configure .frame0.frame2.frame6.frame3 \
    -anchor e
  pack configure .frame0.frame2.frame6.frame5 \
    -anchor e

  # pack master .frame0.frame2.frame6.frame7
  pack configure .frame0.frame2.frame6.frame7.label10 \
    -side left
  pack configure .frame0.frame2.frame6.frame7.canvas11 \
    -side left

  # pack master .frame0.frame2.frame6.frame3
  pack configure .frame0.frame2.frame6.frame3.label10 \
    -side left
  pack configure .frame0.frame2.frame6.frame3.canvas11 \
    -side left

  # pack master .frame0.frame2.frame6.frame5
  pack configure .frame0.frame2.frame6.frame5.label10 \
    -side left
  pack configure .frame0.frame2.frame6.frame5.canvas11 \
    -side left

  # pack master .frame0.frame0
  pack configure .frame0.frame0.label4 \
    -anchor w
  pack configure .frame0.frame0.frame6

  # pack master .frame0.frame0.frame6
  pack configure .frame0.frame0.frame6.frame7 \
    -anchor e
  pack configure .frame0.frame0.frame6.frame8 \
    -anchor e

  # pack master .frame0.frame0.frame6.frame7
  pack configure .frame0.frame0.frame6.frame7.label10 \
    -side left
  pack configure .frame0.frame0.frame6.frame7.canvas11 \
    -side left

  # pack master .frame0.frame0.frame6.frame8
  pack configure .frame0.frame0.frame6.frame8.label10 \
    -side left
  pack configure .frame0.frame0.frame6.frame8.canvas11 \
    -side left

  # pack master .frame0.frame4
  pack configure .frame0.frame4.frame0 \
    -anchor nw \
    -side left
  pack configure .frame0.frame4.frame3 \
    -anchor nw \
    -side left

  # pack master .frame0.frame4.frame0
  pack configure .frame0.frame4.frame0.label6 \
    -anchor ne
  pack configure .frame0.frame4.frame0.label2 \
    -anchor ne

  # pack master .frame0.frame4.frame3
  pack configure .frame0.frame4.frame3.label0 \
    -anchor nw
  pack configure .frame0.frame4.frame3.label7 \
    -anchor nw

  # pack master .frame0.frame1
  pack configure .frame0.frame1.checkbutton4

  # pack master .frame0.frame3
  pack configure .frame0.frame3.button5 \
    -anchor se \
    -side left

  # pack master .frame1
  pack configure .frame1.canvas2 \
    -pady 19

  # pack master .
  pack configure .frame0 \
    -anchor w
  pack configure .frame1

  # build canvas items .frame0.frame2.frame6.frame7.canvas11
  # build canvas items .frame0.frame2.frame6.frame3.canvas11
  # build canvas items .frame0.frame2.frame6.frame5.canvas11
  # build canvas items .frame0.frame0.frame6.frame7.canvas11
  # build canvas items .frame0.frame0.frame6.frame8.canvas11
  # build canvas items .frame1.canvas2



  if {"[info procs XFEdit]" != ""} {
    catch "XFMiscBindWidgetTree ."
    after 2 "catch {XFEditSetShowWindows}"
  }
}


# User defined procedures


# Procedure: connectionManager
proc connectionManager { channel addr port} {
    #set f [open log "a"]; puts $f "Connected"; close $f
    fileevent $channel readable "readLine Server $channel"
}


# Procedure: convertCoords
proc convertCoords { coords} {

    set answer {}
    set isX 1
    foreach n $coords {
        if {$isX} {
            set x $n
            set isX 0
        } else {
            set isX 1
            set y $n
            #set newX [expr ($x + 128) * 4]
            #set newY [expr (256 - ($y + 128)) * 2]
            set newX [expr ($x + 64) * 8]
            set newY [expr (128 - ($y + 64)) * 4]
            # Uses only Q2
            #set newX [expr ($newX - (256 * 2)) * 2]
            #set newY [expr ($newY - (256 * 1)) * 2]
            lappend answer $newX
            lappend answer $newY
        }
    }

    return $answer
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

    set color {#0000ff}
    $cv addtag toDelete all
    $cv delete toDelete
    set xfTmpTag [$cv create rectangle 0.0 0.0 [expr $level * 100] 16.0]
    $cv itemconfigure $xfTmpTag  -fill $color -outline {}  -width {2.0}
}


# Procedure: drawLastMap
proc drawLastMap {} {

    global mapItens
    global lastMap
    global showObjectNames

    if {$lastMap == ""} {
        return
    }

    #set f [open log "a"]; puts $f "Processing local map: $lastMap"; close $f

    foreach item $mapItens {
        .frame1.canvas2 delete $item
    }

    set line [split $lastMap "\t"]
    foreach item $line {
        set item [split $item " "]
        set name [lindex $item 0]
        if {[lsearch [list N S E W] $name] == -1} {
            set coords [convertCoords [lrange $item 1 end]]
            #set f [open log "a"]; puts $f "name = $name"; puts $f "coords = $coords"; close $f
            set newItem [.frame1.canvas2 create polygon $coords -outline [getFillColor $name]  -width 5 -fill [getFillColor $name]]
            lappend mapItens $newItem
            set tagCoords [lrange [.frame1.canvas2 bbox $newItem] 2 3]
            if {$showObjectNames} {
                lappend mapItens [.frame1.canvas2 create text $tagCoords -text $name -fill black -anchor sw]
            }
        }
    }
}


# Procedure: getFillColor
proc getFillColor { name} {
    if {[string match -nocase "water*" $name]} {
        return {#55aaff}
    } else {
        if {[string match -nocase "*bowl*" $name]} {
            return {#aa7766}
        } else {
            if {([string match -nocase "ball*" $name] || [string match -nocase "stick*" $name] || [string match -nocase "doll*" $name])} {
                return {#885588}
            } else {
                if {[string match -nocase "food*" $name]} {
                    return {#ccbb22}
                } else {
                    if {([string match -nocase "Fido*" $name] || [string match -nocase "Wynx*" $name])} {
                        return {#ff0000}
                    } else {
                    }
                }
            }
        }
    }

    return {#000000}
}


# Procedure: processLastSelectedGoal
proc processLastSelectedGoal { line} {

    global lastSelectedGoal

    set lastSelectedGoal [lindex $line 0]
}


# Procedure: processLastSelectedSchema
proc processLastSelectedSchema { line} {

    global lastSelectedSchema

    set lastSelectedSchema [lindex $line 0]
}


# Procedure: processLine
proc processLine { line} {

    #set f [open log "a"]; puts $f "Processing line: $line"; close $f
    
    set line [split $line " "]
    set command [lindex $line 0]
    switch $command {
        {LOCALMAP} {
            processLocalMap [join [lrange $line 1 end] " "]
        }
        {GOAL} {
            processSignal [join [lrange $line 1 end] " "]
        }
        {FEELING} {
            processSignal [join [lrange $line 1 end] " "]
        }
        {LASTSELECTEDGOAL} {
            processLastSelectedGoal [join [lrange $line 1 end] " "]
        }
        {LASTSELECTEDSCHEMA} {
            processLastSelectedSchema [join [lrange $line 1 end] " "]
        }
    }
}


# Procedure: processLocalMap
proc processLocalMap { line} {
    
    global lastMap
    set lastMap $line
    drawLastMap
}


# Procedure: processSignal
proc processSignal { line} {

    #set f [open log "a"]; puts $f "Processing signal: $line"; close $f

    set canvas(AlleviateHunger) .frame0.frame2.frame6.frame7.canvas11
    set canvas(AlleviateThirst) .frame0.frame2.frame6.frame3.canvas11
    set canvas(SatisfyOwner) .frame0.frame2.frame6.frame5.canvas11

    set canvas(Hunger) .frame0.frame0.frame6.frame7.canvas11
    set canvas(Thirst) .frame0.frame0.frame6.frame8.canvas11
    #set canvas(OwnerSatisfaction) .frame0.frame0.frame6.frame8.canvas11

    set line [split $line " "]
    drawBar $canvas([lindex $line 0]) [lindex $line 1] 0
}


# Procedure: readLine
proc readLine { who channel} {
    if { [gets $channel line] < 0} {
        fileevent $channel readable {}
        after idle "close $channel"
        #set f [open log "a"]; puts $f "Connection closed"; close $f
    } else {
        #set f [open log "a"]; puts $f "READ LINE: $line"; close $f
        puts $channel "OK\n"
        flush $channel
        processLine $line
    }
}


# Procedure: startListener
proc startListener {} {
    socket -server connectionManager 11354
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


# end source
proc EndSrc {} {
    global mapItens
    set mapItens {}
    startListener
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

# initialize global variables
proc InitGlobals {} {
  global {checkbutton4}
  set {checkbutton4} {0}
  global {lastMap}
  set {lastMap} {}
  global {lastSelectedGoal}
  set {lastSelectedGoal} {}
  global {lastSelectedSchema}
  set {lastSelectedSchema} {}
  global {mapItens}
  set {mapItens} {}
  global {showObjectNames}
  set {showObjectNames} {0}
  global {tmplt}
  set {tmplt} {0}

  # please don't modify the following
  # variables. They are needed by xf.
  global {autoLoadList}
  set {autoLoadList(PetInterface.tcl)} {0}
  global {internalAliasList}
  set {internalAliasList} {}
  global {moduleList}
  set {moduleList(PetInterface.tcl)} {}
  global {preloadList}
  set {preloadList(xfInternal)} {}
  global {symbolicName}
  set {symbolicName(root)} {.}
  global {xfWmSetPosition}
  set {xfWmSetPosition} {}
  global {xfWmSetSize}
  set {xfWmSetSize} {}
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
XFLocalLoadAppDefs PetInterface
XFLocalSetAppDefs

# end source
EndSrc

# eof
#

