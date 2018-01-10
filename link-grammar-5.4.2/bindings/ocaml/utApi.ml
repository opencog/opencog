(****************************************************************)
(* Unit Test for the LinkGrammar Ocaml API                      *)
(****************************************************************)

open Linkgrammar;;


let po = poCreate ();;
let () = poSetLinkageLimit po 1000;;

let dict = dictCreate ("en");;

let str = "Janet, who is an expert on dogs, helped me choose one";;

let sent = sentCreate dict str;;

let numLinks = sentParse sent po;;

let () = Printf.printf "sentence = %s\n" str;;
let () = Printf.printf "len = %d\n" (sentLength sent);;
let () = Printf.printf "nullcnt = %d\n" (sentNullCount sent);;
let () = Printf.printf "links = %d\n" (sentNumLinkagesFound sent);;
let () = Printf.printf "valid = %d\n" (sentNumValidLinkages sent);;
let () = Printf.printf "PP = %d\n" (sentNumLinkagesPP sent);;
let () = Printf.printf "vio = %d\n" (sentNumViolations sent 0);;
let () = Printf.printf "disjunct cost = %d\n" (sentDisjunctCost sent 0);;
let _ = sentGetWords sent;;


let link = linkageCreate sent 0 po;;
let _ = linkageGetNumWords link;;
let _ = linkageGetNumLinks link;;
let _ = linkageGetLinkLength link 0;;
let _ = linkageGetLinkLength link 1;;
let _ = linkageGetLinkLength link 2;;
let _ = linkageGetLinkLword link 2;;
let _ = linkageGetLinkRword link 2;;
let _ = linkageGetLinkLabel link 2;;
let _ = linkageGetLinkLlabel link 2;;
let _ = linkageGetLinkRlabel link 2;;
let ws = linkageGetWords link;;
let _ = linkageGetLinkDomainNames link 2;;
let _ = linkageGetLinkDomainNames link 3;;
let _ = linkageGetLinkDomainNames link 4;;
let _ = linkageUnusedWordCost link;;
let _ = linkageDisjunctCost link;;
let _ = linkageLinkCost link;;

let () = Printf.printf "linkage = \n%s\n" (linkagePrintDiagram link);;
let () = Printf.printf "linkage ps = \n%s\n" (linkagePrintPostscript link 0);;
let () = Printf.printf "links and domains \n%s\n" (linkagePrintLinksAndDomains link);;


