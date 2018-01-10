(****************************************************************)
(* Module [Linkgrammar]: Provides an Ocaml interface to         *)
(* LinkGrammar - A parser for english sentences                 *)
(*                                                              *)
(* For more info on LinkGrammar                                 *)
(* refer: http://abisource.com/projects/link-grammar/           *)
(*                                                              *)
(* Author: Ramu Ramamurthy ramu_ramamurthy at yahoo dot com     *)
(* (C) 2006                                                     *)
(*                                                              *)
(* This is released under the BSD license                       *)
(****************************************************************)


(**
   {b ------parse options and operations on it------}
*)

type parseOptions
;;

external poCreateBase: unit -> parseOptions = "po_create";;
external freePo : parseOptions -> unit = "free_po";;

let poCreate () =
  let v = poCreateBase () in
  let () = Gc.finalise freePo v in
    v
;;

external poGetVerbosity : parseOptions -> int = "po_get_verbosity";;
external poSetVerbosity : parseOptions -> int -> unit = "po_set_verbosity";;
external poGetLinkageLimit : parseOptions -> int = "po_get_linkage_limit";;
external poSetLinkageLimit :  parseOptions -> int -> unit = "po_set_linkage_limit";;
external poGetDisjunctCost : parseOptions -> int = "po_get_disjunct_cost";;
external poSetDisjunctCost : parseOptions -> int -> unit = "po_set_disjunct_cost";;
external poGetMinNullCount : parseOptions -> int = "po_get_min_null_count";;
external poSetMinNullCount : parseOptions -> int -> unit = "po_set_min_null_count";;
external poGetMaxNullCount : parseOptions -> int = "po_get_max_null_count";;
external poSetMaxNullCount : parseOptions -> int -> unit = "po_set_max_null_count";;
external poGetNullBlock : parseOptions -> int = "po_get_null_block";;
external poSetNullBlock : parseOptions -> int -> unit = "po_set_null_block";;
external poGetShortLength : parseOptions -> int = "po_get_short_length";;
external poSetShortLength : parseOptions -> int -> unit = "po_set_short_length";;
external poGetIslandsOk : parseOptions -> int = "po_get_islands_ok";;
external poSetIslandsOk : parseOptions -> int -> unit = "po_set_islands_ok";;
external poGetMaxParseTime : parseOptions -> int = "po_get_max_parse_time";;
external poSetMaxParseTime : parseOptions -> int -> unit = "po_set_max_parse_time";;
external poTimerExpired : parseOptions -> int = "po_get_timer_expired";;
external poResetResources : parseOptions -> unit = "po_reset_resources";;
external poGetAllowNull : parseOptions -> int = "po_get_allow_null";;
external poSetAllowNull : parseOptions -> int -> unit = "po_set_allow_null";;
external poGetAllShortConnectors : parseOptions -> int = "po_get_all_short_connectors";;
external poSetAllShortConnectors : parseOptions -> int -> unit = "po_set_all_short_connectors";;

(**
   {b -------dictionary and operations on it--------}
*)
type dictionary;;

external dictCreateBase : string -> dictionary = "dict_create";;
external freeDict : dictionary -> unit = "free_dict";;

let dictCreate s1 =
  let v = dictCreateBase s1 in
  let () = Gc.finalise freeDict v in
    v
;;

(**
   {b -------sentences and operations on it--------}
*)
type sentence;;

external sentCreateBase : string -> dictionary -> sentence = "sent_create";;
external freeSent : sentence -> unit = "free_sentence";;

let sentCreate dict s =
  let v = sentCreateBase s dict in
  let () = Gc.finalise freeSent v in
    v
;;

external sentParse : sentence -> parseOptions -> int = "sent_parse";;
external sentLength : sentence -> int = "sent_length";;
external sentNullCount : sentence -> int = "sent_null_count";;
external sentNumLinkagesFound : sentence -> int = "sent_num_linkages_found";;
external sentNumValidLinkages : sentence -> int = "sent_num_valid_linkages";;
external sentNumLinkagesPP : sentence -> int = "sent_num_linkages_post_processed";;
external sentNumViolations : sentence -> int -> int = "sent_num_violations";;
external sentDisjunctCost : sentence -> int -> int = "sent_disjunct_cost";;

(**
   {b -------linkage and operations on it--------}
*)
type linkage

external linkageCreateBase : sentence -> int -> parseOptions -> linkage = "link_create";;
external freeLinkage : linkage -> unit = "free_linkage";;

let linkageCreate sent ith po =
  let v = linkageCreateBase sent ith po in
  let () = Gc.finalise freeLinkage v in
    v
;;


external linkageGetNumWords : linkage -> int = "link_get_num_words";;
external linkageGetWord : linkage -> int -> string = "link_get_word";;
let linkageGetWords link =
  let wList = ref [] in
  let () =
    for i = 0 to ((linkageGetNumWords link) - 1) do
      wList := List.append !wList [linkageGetWord link i]
    done
  in
    !wList
;;

external linkageGetNumLinks : linkage -> int = "link_get_num_links";;
external linkageGetLinkLength : linkage -> int -> int = "link_get_link_length";;
external linkageGetLinkLword : linkage -> int -> int = "link_get_link_lword";;
external linkageGetLinkRword : linkage -> int -> int = "link_get_link_rword";;
external linkageGetLinkLabel : linkage -> int -> string = "link_get_link_label";;
external linkageGetLinkLlabel : linkage -> int -> string = "link_get_link_llabel";;
external linkageGetLinkRlabel : linkage -> int -> string = "link_get_link_rlabel";;

external linkagePrintDiagram : linkage -> string = "link_print_diagram";;
external linkagePrintPostscript : linkage -> int -> string = "link_print_postscript";;
external linkagePrintLinksAndDomains : linkage -> string = "link_print_links_and_domains";;


external linkageGetNumDomains : linkage -> int -> int = "link_get_link_num_domains"
external linkageGetLinkDomainNameI : linkage -> int -> int -> string = "link_get_link_domain_name_i"
external linkageGetViolationName : linkage -> string = "link_get_violation_name"

let linkageGetLinkDomainNames linkage link_ind =
  let wList = ref [] in
  let () =
    for i = 0 to ((linkageGetNumDomains linkage link_ind) - 1) do
      wList := List.append !wList [linkageGetLinkDomainNameI linkage link_ind i]
    done
  in
    !wList
;;
  

external linkageUnusedWordCost : linkage -> int = "link_unused_word_cost"
external linkageDisjunctCost : linkage -> int = "link_disjunct_cost"
external linkageLinkCost : linkage -> int = "link_link_cost"

