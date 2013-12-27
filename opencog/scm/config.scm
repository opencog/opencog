;
; config.scm
;
; General cog-server configuration, for scheme.
; XXX TODO This stuff really should be unified with the opencog.conf
; file aka the util/config.h mechanisms.  But, for now, these
; remain separate. I'm too lazy to "improve" this right now.
;
;--------------------------------------------------------------
; Turn on debugging prints -- this generally makes life easier.
; This only works for guile-1.8 and has no effect for guile-2.0
; and should be removed 'any day now'.
(turn-on-debugging)

;--------------------------------------------------------------
; The scheme shell listen port.
(define shell-port 18001)

;--------------------------------------------------------------
; The relex server host and port
(define relex-server-host "127.0.0.1")
(define relex-server-port 4444)
