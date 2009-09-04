; opencog/embodiment/scm/question-answering.scm
;
; Copyright (C) 2009 Novamente LLC
; All Rights Reserved
; Author(s): Samir Araujo
;
; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU Affero General Public License v3 as
; published by the Free Software Foundation and including the exceptions
; at http://opencog.org/wiki/Licenses
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU Affero General Public License
; along with this program; if not, write to:
; Free Software Foundation, Inc.,
; 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.


(define invalid-question-frames
  (list
   "#Attributes"
   "#Possibilities"
   "#Temporal_colocation"
   "#Emotion_directed"
   )
  )

(define word-node-map '())

(define (get-word-node-map)
  (if (null? word-node-map)
      (set! word-node-map
            (list
             (cons "you" agentSemeNode)
             )
            )
      )
  word-node-map
  )
