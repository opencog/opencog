(**
  Module [LinkGrammar]: Provides an Ocaml interface to LinkGrammar --
   A parser for sentences

   The Ocaml API maps almost 1-1 with the LinkGrammar C API -
   see description of the C API at: 
   http://abisource.com/projects/link-grammar/api/index.html

  Author: Ramu Ramamurthy ramu_ramamurthy at yahoo dot com   
  (C) 2006                                                     

  This software is released under the BSD license         
*)


(**
   {b ------parse options and operations on it------}
*)

type parseOptions

val poCreate : unit -> parseOptions

val poGetVerbosity : parseOptions -> int
val poSetVerbosity : parseOptions -> int -> unit
val poGetLinkageLimit : parseOptions -> int
val poSetLinkageLimit : parseOptions -> int -> unit
val poGetDisjunctCost : parseOptions -> int
val poSetDisjunctCost : parseOptions -> int -> unit
val poGetMinNullCount : parseOptions -> int
val poSetMinNullCount : parseOptions -> int -> unit
val poGetMaxNullCount : parseOptions -> int
val poSetMaxNullCount : parseOptions -> int -> unit
val poGetNullBlock : parseOptions -> int
val poSetNullBlock : parseOptions -> int -> unit
val poGetShortLength : parseOptions -> int
val poSetShortLength : parseOptions -> int -> unit
val poGetIslandsOk : parseOptions -> int
val poSetIslandsOk : parseOptions -> int -> unit
val poGetMaxParseTime : parseOptions -> int
val poSetMaxParseTime : parseOptions -> int -> unit
val poTimerExpired : parseOptions -> int
val poResetResources : parseOptions -> unit
val poGetAllowNull : parseOptions -> int
val poSetAllowNull : parseOptions -> int -> unit
val poGetAllShortConnectors : parseOptions -> int
val poSetAllShortConnectors : parseOptions -> int -> unit

(**
   {b -------dictionary and operations on it--------}
*)

type dictionary
(**    
       See notes in the README file on specifying paths to
       dictionary files
*)
val dictCreate : string -> dictionary


(**
   {b -------sentences and operations on it--------}
*)

type sentence

val sentCreate : dictionary -> string -> sentence
val sentParse : sentence -> parseOptions -> int
val sentLength : sentence -> int
val sentNullCount : sentence -> int
val sentNumLinkagesFound : sentence -> int
val sentNumValidLinkages : sentence -> int
val sentNumLinkagesPP : sentence -> int
val sentNumViolations : sentence -> int -> int
val sentDisjunctCost : sentence -> int -> int


(**
   {b -------linkage and operations on it--------}
*)

type linkage

val linkageCreate : sentence -> int -> parseOptions -> linkage
val linkageGetNumWords : linkage -> int
val linkageGetNumLinks : linkage -> int
val linkageGetLinkLength : linkage -> int -> int
val linkageGetLinkLword : linkage -> int -> int
val linkageGetLinkRword : linkage -> int -> int
val linkagePrintDiagram : linkage -> string
val linkagePrintPostscript : linkage -> int -> string
val linkagePrintLinksAndDomains : linkage -> string
val linkageGetLinkLabel : linkage -> int -> string
val linkageGetLinkLlabel : linkage -> int -> string
val linkageGetLinkRlabel : linkage -> int -> string
val linkageGetWords : linkage -> string list
val linkageGetWord : linkage -> int -> string
val linkageGetNumDomains : linkage -> int -> int
val linkageGetLinkDomainNames : linkage -> int -> string list
val linkageGetViolationName : linkage -> string
val linkageUnusedWordCost : linkage -> int
val linkageDisjunctCost : linkage -> int
val linkageLinkCost : linkage -> int

