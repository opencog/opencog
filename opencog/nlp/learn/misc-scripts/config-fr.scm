;
; config-fr.scm
;
; French parser config
; General cog-server configuration, for scheme.
;
;--------------------------------------------------------------
; Turn on debugging prints -- this generally makes life easier.
(turn-on-debugging)

;--------------------------------------------------------------
; The scheme shell listen port.
(define shell-port 18002)

;--------------------------------------------------------------
; The relex server host and port
(define relex-server-host "10.1.1.4")
(define relex-server-port 4444)
