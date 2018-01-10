;;; -*- Mode: Lisp; Package: User; -*-

#|
  This is a hand-built Lisp interface to the Link Grammar Parser,
  following its API declarations in link-includes.h.  That parser is
  available from
  http://www.abisource.org/projects/link-grammar/
  and its API is described in
  http://www.abisource.com/projects/link-grammar/api/index.html
  as of March, 2009.

  That API supports five basic data structures: Dictionary, Sentence,
  Parse_Options, Linkage and PostProcessor.  In order to avoid memory
  leaks in creating these (and some other things) within Lisp, the
  appropriate (delete_...) calls should be made. We have tried to
  package these up in macros of the form (with-...), but for heavy
  weight items such as Dictionary, it may be more appropriate to
  create the structure and only delete it when it is no longer of
  use.  Note that only the five basic data structures and large
  strings returned by the linkage-print-... functions need to be
  deleted. The link parser appears to take care of managing memory for
  its short strings, such as individual tokens. In this interface, it
  is the functions that return :foreign-address that require special
  attention to potential memory leaks.

  An alternative to this implementation would be to wrap every
  link-parser object in a CLOS object and use the Lisp garbage
  collector's finalization mechanism to make sure that the link-parser
  object is deleted when the corresponding Lisp object is gc'd.  This
  would be more elegant and error-proof, but the with-... macros
  provide the same protection locally in code, with less overhead.

  This code was written in 2009 by Peter Szolovits (psz@mit.edu), and
  is made available under the "MIT License":

  Copyright (c) 2009 Peter Szolovits and Massachusetts Institute of
  Technology.  
  
  Permission is hereby granted, free of charge, to any person
  obtaining a copy of this software and associated documentation files
  (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge,
  publish, distribute, sublicense, and/or sell copies of the Software,
  and to permit persons to whom the Software is furnished to do so,
  subject to the following conditions: 

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
  ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  |#

(eval-when (:compile-toplevel :load-toplevel :execute)
  (require :foreign)
  )


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Dictionary
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ff:def-foreign-call dictionary_create
    ((dict :foreign-address)
     (knowledge :foreign-address)
     (post :foreign-address)
     (affix :foreign-address))
  :strings-convert nil
  :returning :foreign-address)

(defun dict-create (dictname knowlname postname affixname)
  (with-native-string (c_dn (or dictname ""))
    (with-native-string (c_kn (or knowlname ""))
      (with-native-string (c_pn (or postname ""))
	(with-native-string (c_an (or affixname ""))
	  (format t "wtf?~%")
	  (dictionary_create 
	   (if dictname c_dn 0)
	   (if knowlname c_kn 0)
	   (if postname c_pn 0)
	   (if affixname c_an 0)))))))

(ff:def-foreign-call dictionary_create_lang ((lang (* :char) string))
  :strings-convert t
  :returning :foreign-address)

(ff:def-foreign-call dictionary_create_default_lang (:void)
  :returning :foreign-address)

(ff:def-foreign-call dictionary_delete ((dict :foreign-address))
  :returning :int)

;;; with dictionary, execute the following program (in body b).
(defmacro with-dictionary ((name &optional d k p a) &body b)
  `(let ((,name nil))
     (unwind-protect
	 (unless
	     (zerop (setq ,name (dict-create ,d ,k ,p ,a)))
	   ,@b)
       (when (and ,name (not (zerop ,name)))
	 (dictionary_delete ,name)))))

(defmacro with-dictionary-lang
    ((name &optional (language nil lang-given)) &body b)
  `(let ((,name nil))
     (unwind-protect
	 (unless
	     (zerop (setq ,name
		      ,(if lang-given
			   `(dictionary_create_lang ,language)
			 '(dictionary_create_default_lang))))
	   ,@b)
       (when (and ,name (not (zerop ,name)))
	 (dictionary_delete ,name)))))

(ff:def-foreign-call dictionary_get_max_cost ((dict :foreign-address))
  :returning :int)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Parse_options
;;;
;;;
;;; Some of these options are numerical, others Boolean. We follow C
;;; conventions and require that the Boolean ones are set to 0 or 1 for
;;; FALSE or TRUE. Alternatively, we could declare these parameters to
;;; be of the form (argname :int boolean).
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ff:def-foreign-call parse_options_create (:void)
  :returning :foreign-address)

(ff:def-foreign-call parse_options_delete ((po :foreign-address))
  :returning :int)

;;; Macro to allow use of a parse-options, making sure to delete it
;;; when done.  The value of the last form in the body is
;;; returned. Options, if given, is a list of alternating keywords and
;;; values, appropriate for set-parse-options.
(defmacro with-parse-options ((name &optional options) &body b)
  `(let ((,name nil))
     (unwind-protect
	 (if (zerop (setq ,name (parse_options_create)))
	     (error "Unable to create parse_options.")
	   (progn
	     ,@(if options
		   (cons `(apply #'set-parse-options ,name ,options)
			 b)
		 b)))
       (when (and ,name (not (zerop ,name)))
	 (parse_options_delete ,name)))))

(ff:def-foreign-call parse_options_set_verbosity
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_verbosity
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_linkage_limit
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_linkage_limit
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_disjunct_cost
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_disjunct_cost
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_min_null_count
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_min_null_count
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_max_null_count
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_max_null_count
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_null_block
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_null_block
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_islands_ok
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_islands_ok
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_short_length
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_short_length
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_max_memory
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_max_memory
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_max_sentence_length
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_max_sentence_length
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_max_parse_time
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_max_parse_time
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_cost_model_type
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_cost_model_type
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_timer_expired
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_memory_exhausted
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_resources_exhausted
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_screen_width
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_screen_width
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_allow_null
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_allow_null
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_display_walls
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_display_walls
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_all_short_connectors
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_all_short_connectors
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_reset_resources
    ((po :foreign-address))
  :returning :void)

(ff:def-foreign-call parse_options_set_batch_mode
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_batch_mode
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_panic_mode
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_panic_mode
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_display_on
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_display_on
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_display_postscript
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_display_postscript
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_display_constituents
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_display_constituents
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_display_bad
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_display_bad
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_display_links
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_display_links
    ((po :foreign-address))
  :returning :int)

(ff:def-foreign-call parse_options_set_display_disjuncts
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_display_disjuncts
    ((po :foreign-address))
  :returning :int)


(ff:def-foreign-call parse_options_set_display_senses
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_display_senses
    ((po :foreign-address))
  :returning :int)


(ff:def-foreign-call parse_options_set_echo_on
    ((po :foreign-address)
     (param :int fixnum))
  :returning :void)

(ff:def-foreign-call parse_options_get_echo_on
    ((po :foreign-address))
  :returning :int)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Sentence
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; This is a hugely incomplete whack at trying to define a Lisp
;;; version of the Sentence structure.  If I were better at FFI, I
;;; would do this for all the public structures of LG, making them
;;; more easily manipulable in Lisp.
(ff:def-foreign-type Sentence
    (:struct (dict (* Dictionary))
	     (word (:array (* char)))
	     (deletable (* (* char)))
	     (dptr (* (* char)))
	     (effective_dist (* (* char)))
	     (num_linkages_found :int)
	     (num_linkages_allocated :int)
	     (num_linkages_post_processed :int)
	     (num_valid_linkages :int)
	     (null_links :int)
	     (null_count :int)
	     ;; There is actually more, but ...
	     ))

(ff:def-foreign-call sentence_create 
    ((sent (* :char) string)
     (dict :foreign-address))
  :strings-convert t
  :returning :foreign-address)

(ff:def-foreign-call sentence_delete ((sent :foreign-address))
  :returning :void)

(defmacro with-sentence ((name val dict) &body b)
  `(let ((,name nil))
     (unwind-protect
	 (unless
	     (zerop (setq ,name (sentence_create ,val ,dict)))
	   ,@b)
       (when (and ,name (not (zerop ,name)))
	 (sentence_delete ,name)))))

(ff:def-foreign-call sentence_parse
    ((sent :foreign-address)
     (opts :foreign-address))
  :strings-convert nil
  :returning :int)

(ff:def-foreign-call sentence_length ((sent :foreign-address))
  :returning :int)

(ff:def-foreign-call sentence_get_word ((sent :foreign-address)
					(wordnum :int))
  :returning ((* :char)))

(ff:def-foreign-call sentence_null_count ((sent :foreign-address))
  :returning :int)

(ff:def-foreign-call sentence_num_linkages_found ((sent :foreign-address))
  :returning :int)

(ff:def-foreign-call sentence_num_valid_linkages ((sent :foreign-address))
  :returning :int)

(ff:def-foreign-call sentence_num_linkages_post_processed
    ((sent :foreign-address))
  :returning :int)

(ff:def-foreign-call sentence_num_violations 
    ((sent :foreign-address)
     (i :int))
  :returning :int)

;; This is in the API, but not the documentation.
(ff:def-foreign-call sentence_disjunct_cost
    ((sent :foreign-address)
     (i :int))
  :returning :int)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Linkage
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ff:def-foreign-call linkage_create
    ((index :int)
     (sent :foreign-address)
     (opts :foreign-address))
  :returning :foreign-address)

(ff:def-foreign-call linkage_delete ((linkage :foreign-address))
  :returning :void)

(defmacro with-linkage ((name index sent opts) &body b)
  `(let ((,name nil))
     (unwind-protect
	 (unless
	     (zerop (setq ,name (linkage_create ,index ,sent ,opts)))
	   ,@b)
       (when (and ,name (not (zerop ,name)))
	 (linkage_delete ,name)))))

(ff:def-foreign-call linkage_get_num_words
    ((linkage :foreign-address))
  :returning :int)

(ff:def-foreign-call linkage_get_num_links
    ((linkage :foreign-address))
  :returning :int)

(ff:def-foreign-call linkage_get_link_lword
    ((linkage :foreign-address)
     (index :int))
  :returning :int)

(ff:def-foreign-call linkage_get_link_rword
    ((linkage :foreign-address)
     (index :int))
  :returning :int)

(ff:def-foreign-call linkage_get_link_length
    ((linkage :foreign-address)
     (index :int))
  :returning :int)

(ff:def-foreign-call linkage_get_link_label
    ((linkage :foreign-address)
     (index :int))
  :returning ((* :char)))

(ff:def-foreign-call linkage_get_link_llabel
    ((linkage :foreign-address)
     (index :int))
  :returning ((* :char)))

(ff:def-foreign-call linkage_get_link_rlabel
    ((linkage :foreign-address)
     (index :int))
  :returning ((* :char)))

(ff:def-foreign-call linkage_get_link_num_domains
    ((linkage :foreign-address)
     (index :int))
  :returning :int)


;;; linkage_get_link_domain_names returns char**, which I don't really
;;; understand how to handle in Lisp. It should be possible to define
;;; a foreign structure something like this:
;;; (ff:def-foreign-type StringVec (:array (* :char) *))
;;; but I need to know the length.
(ff:def-foreign-call linkage_get_link_domain_names
    ((linkage :foreign-address))
  :returning :foreign-address ;; StringVec
  )

;;; linkage_get_words returns char**, which I don't know how to handle.
(ff:def-foreign-call linkage_get_words ((linkage :foreign-address))
  :returning ((* (* :char)) (simple-array string (*)))) 

(ff:def-foreign-call linkage_get_word ((linkage :foreign-address)
				       (index :int))
  :returning ((* :char)))

(ff:def-foreign-call linkage_print_links_and_domains
    ((linkage :foreign-address))
  :returning :foreign-address)

(ff:def-foreign-call linkage_free_links_and_domains
    ((str :foreign-address))
  :returning :void)

(defun links-and-domains-str (linkage)
  "Retrieves and converts to a Lisp string the
  linkage_print_links_and_domains string from LP, and then frees
  it. The converted string is returned, as ordinary Lisp string."
  (let ((foreign-str nil))
    (unwind-protect
	(unless
	    (zerop
	     (setq foreign-str
	       (linkage_print_links_and_domains linkage)))
	  (native-to-string foreign-str))
      (when foreign-str
	(linkage_free_links_and_domains foreign-str)
	nil))))

(ff:def-foreign-call linkage_print_constituent_tree
    ((linkage :foreign-address)
     (mode :int))
  :returning :foreign-address)

(ff:def-foreign-call linkage_free_constituent_tree_str
    ((str :foreign-address))
  :returning :void)

(defun constituent-tree-str (linkage mode)
  "Retrieves and converts to a Lisp string the
  linkage_print_constituent_tree string from LP, and then frees
  it. The converted string is returned, as ordinary Lisp string."
  (let ((foreign-str nil))
    (unwind-protect
	(unless
	    (zerop
	     (setq foreign-str
	       (linkage_print_constituent_tree linkage mode)))
	  (native-to-string foreign-str))
      (when foreign-str
	(linkage_free_constituent_tree_str foreign-str)
	nil))))

(ff:def-foreign-call linkage_print_diagram ((linkage :foreign-address))
  :returning :foreign-address)

(ff:def-foreign-call linkage_free_diagram
    ((str :foreign-address))
  :returning :void)

(defun diagram-str (linkage)
  "Retrieves and converts to a Lisp string the
  linkage_print_diagram string from LP, and then frees
  it. The converted string is returned, as ordinary Lisp string."
  (let ((foreign-str nil))
    (unwind-protect
	(unless
	    (zerop
	     (setq foreign-str
	       (linkage_print_diagram linkage)))
	  (native-to-string foreign-str))
      (when foreign-str
	(linkage_free_diagram foreign-str)
	nil))))

(ff:def-foreign-call linkage_print_postscript ((linkage :foreign-address)
					       (mode :int))
  :returning :foreign-address)

(ff:def-foreign-call linkage_free_postscript
    ((str :foreign-address))
  :returning :void)

(defun postscript-str (linkage mode)
  "Retrieves and converts to a Lisp string the
  linkage_print_diagram string from LP, and then frees
  it. The converted string is returned, as ordinary Lisp string."
  (let ((foreign-str nil))
    (unwind-protect
	(unless
	    (zerop
	     (setq foreign-str
	       (linkage_print_postscript linkage mode)))
	  (native-to-string foreign-str))
      (when foreign-str
	(linkage_free_postscript foreign-str)
	nil))))

(ff:def-foreign-call linkage_unused_word_cost ((linkage :foreign-address))
  :returning :int)

(ff:def-foreign-call linkage_disjunct_cost ((linkage :foreign-address))
  :returning :int)

(ff:def-foreign-call linkage_link_cost ((linkage :foreign-address))
  :returning :int)

(ff:def-foreign-call linkage_get_violation_name
    ((linkage :foreign-address))
  :returning ((* :char)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; PostProcessor
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ff:def-foreign-call post_process_open ((name (* :char) string))
    :strings-convert nil
    :returning :foreign-address)

(ff:def-foreign-call post_process_close ((pp :foreign-address))
  :returning :void)

(ff:def-foreign-call linkage_post_process 
    ((linkage :foreign-address)
     (pp :foreign-address))
  :returning :void)

(defmacro with-post-processor (name &body b)
  `(let ((,name nil))
     (unwind-protect
	 (unless
	     (zerop (setq ,name (post_process_open name)))
	   ,@b)
       (when (and ,name (not (zerop ,name)))
	 (post_process_close ,name)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Utility
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ff:def-foreign-call linkgrammar_get_version (:void)
  :returning :foreign-address)

(defun linkgrammar-get-version ()
  (native-to-string (linkgrammar_get_version)))

(ff:def-foreign-call setlocale ((localename :string))
  :returning :void)

(provide :link)


