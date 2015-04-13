;
; config.scm
;
; General cog-server configuration, for scheme.
; XXX TODO This stuff really should be unified with the opencog.conf
; file aka the util/config.h mechanisms.  But, for now, these
; remain separate. I'm too lazy to "improve" this right now.
;
;--------------------------------------------------------------
; The scheme shell listen port.
(define shell-port 18001)
; A plain simple prompt
; (define shell-prompt "opencog-scheme> ")
; An ANSI-terminal colorized prompt!  This one is blue.
(define shell-prompt "[0;34mopencog-scheme[1;34m> [0m")

;--------------------------------------------------------------
; The relex server host and port
(define relex-server-host "127.0.0.1")
(define relex-server-port 4444)
