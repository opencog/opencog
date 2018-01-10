#
# Copyright (c) 2016 Amir Plivatsky <amirpli@gmail.com>
#

# In configure.in, use as:
#
#   # AM_ABC sets cache variables that prevent using it again,
#   # unless they are removed
#   abc_persistent=  # to be included in the checkpoint below
#   AM_VARNAMES_CHECKPOINT
#   AM_ABC
#   abc_persistent=$am_cv_abc
#   AM_VARNAMES_RESTORE
#   if test -n "$abc_persistent"; then
#       ...
#   fi
#   AM_ABC

# SYNOPSIS
#
#   AC_VARNAMES_CHECKPOINT
#
# DESCRIPTION
#
#   Save the current list of shell variables, to be restored later
#   by AC_VARNAMES_RESTORE.
#   Note that the filtering is not technically correct for eliminating the
#   continuation lines of some theoretically possible multi-line variable
#   values.
#   For example, x='ab\ncd=\n' (when \n is a real newline) may cause troubles.
#   For a general use, this has to be fixed. A fix is welcome.

#serial 1

AC_DEFUN([AC_VARNAMES_CHECKPOINT],
[
	am_varnames_checkpoint=`set | sed '/^[[a-zA-Z_]][[a-zA-Z0-9_]]*=/!d;s/=.*/\
/;/'\''/d'`
]
)

# SYNOPSIS
#
#   AC_VARNAMES_RESTORE
#
# DESCRIPTION
#
#   Restore the list of shell variable saved by AC_VARNAMES_CHECKPOINT.
#   Variables that got created after its call get unset.
#   See the above comment for a problem in the filter pattern.

AC_DEFUN([AC_VARNAMES_RESTORE],
[
	if test -z "$am_varnames_checkpoint"; then
		AC_MSG_ERROR([No prior variable checkpoint (already restored?)])

	fi

	am_varnames_curr=`set | sed '/^[[a-zA-Z_]][[a-zA-Z0-9_]]*=/!d;s/=.*/\
/;/'\''/d'`
	# A variable containing only a newline
	NL='
'
	am_varnames_combined="$am_varnames_checkpoint${NL}$am_varnames_curr"
	am_varnames_tounset=`echo "$am_varnames_combined" | sort | uniq -u`
	for am_varname_tmp in $am_varnames_tounset
	do
		unset $am_varname_tmp
	done
	# Unset all the variables created between the calls to AC_VARNAMES_CHECKPOINT
	# and AC_VARNAMES_RESTORE.
	# Especially be careful to unset am_varnames_checkpoint in order to detect
	# unmatched AC_VARNAMES_RESTORE calls.
	unset am_varnames_checkpoint am_varnames_curr am_varnames_combined
	unset am_varnames_tounset am_varname_tmp NL
]
)
