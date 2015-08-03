#! /usr/bin/env guile
!#
;
; Atomspace deduplication repair script
;
; Due to bugs, the SQL backend can end up with multiple copies of
; atoms. This script will find them, merge them, and sum the counts
; on the associated count truth values.  Its up to you to recompute
; anything else.
;
; This script focuses on accidentally having two ANY nodes.
; Viz if select * from atoms where type=89 and name='ANY';
; returns more than one row :-(

(load "common.scm")

