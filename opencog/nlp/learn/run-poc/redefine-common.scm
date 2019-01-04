(define (count-one-atom ATM)
"
  count-one-atom ATM -- increment the count by one on ATM, and
  update the SQL database to hold that count.

  calls count-one-atom-times with count argument equal to 1.
"
	(count-one-atom-times ATM 1) ; call incrementer with parameter 1
)

; ---------------------------------------------------------------------

(define (count-one-atom-times ATM times)
"
  count-one-atom-times ATM times-- increment the count by times on ATM, and
  update the SQL database to hold that count.

  This will also automatically fetch the previous count from
  the SQL database, so that counting will work correctly, when
  picking up from a previous point.

  Warning: this is NOT SAFE for distributed processing! That is
  because this does NOT grab the count from the database every time,
  so if some other process updates the database, this will miss that
  update.
"
	(define (incr-times atom TIMES)
		; If the atom doesn't yet have a count TV attached to it,
		; then its probably a freshly created atom. Go fetch it
		; from SQL. Otherwise, assume that what we've got here,
		; in the atomspace, is the current copy.  This works if
		; there is only one process updating the counts.
		(if (not (cog-ctv? (cog-tv atom)))
			(fetch-atom atom)) ; get from SQL
		(cog-inc-count! atom TIMES) ; increment
	)
	(begin
		(incr-times ATM times) ; increment the count on ATM
		(store-atom ATM)) ; save to SQL
)

