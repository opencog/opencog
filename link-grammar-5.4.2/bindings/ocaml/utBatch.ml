(****************************************************************)
(* Unit Test for the LinkGrammar Ocaml API                      *)
(* also doubles as an example of API usage                      *) 
(*                                                              *)
(* Given a file name, with one sentence per line, parses all    *)
(* sentences in the file -- it prints the linkage diagram, and  *)
(* the constituent tree for each linkage                        *)
(*                                                              *)
(* The unit test is to run this on the file "4.0.batch"         *)
(* 4.0.batch is a                                               *)
(* file containing about 900 sentences that is part of the      *)
(* linkgrammar distribution under "data" directory              *)
(*                                                              *)
(* Author: Ramu Ramamurthy ramu_ramamurthy at yahoo dot com     *)
(* (C) 2006                                                     *)
(*                                                              *)
(* This is released under the BSD license                       *)
(****************************************************************)

open Linkgrammar;;

(* LinkGrammar constituent tree algorithm has a bug
   on the 3rd linkage on the following sentence in
   4.0.batch
*)
let buggyStr = "This is the man whose dog I bought";;

let printLinkages sentparse po = 
  let numLinkages = sentNumValidLinkages sentparse in
  let () = Printf.printf "num of linkages = %d\n" numLinkages in
    if numLinkages > 0 then
      for i = 0 to (numLinkages-1)
      do
	let linkage = linkageCreate sentparse i po in
	let () = linkageSetSublinkage linkage 0 in (
	    Printf.printf "linkage %d is: %s\n" i (linkagePrintDiagram linkage);
	    flush stdout;
	    printConstituentTree linkage
	  );
      done
;;  

let parseAndPrint sent po = 
  let numLinkages = sentParse sent po in
  let () = Printf.printf "num of linkages = %d\n" numLinkages in
  let () = flush stdout in
    if numLinkages > 0 then
      printLinkages sent po
    else 
      let () = poSetMaxNullCount po (sentLength sent) in
      let () = poSetMinNullCount po 1 in
      let numLinkages = sentParse sent po in
      let () = Printf.printf "num of linkages = %d\n" numLinkages in
      let () = flush stdout in
	if numLinkages > 0 then
	  printLinkages sent po
;;

let parseFromFile dict po fname =
  let in_c = open_in fname in
    try
      while true do
	let str = input_line in_c in
	  if (String.length str) > 0 then
	    if str.[0] <> '!' && str <> buggyStr then
	      (
		let () = Printf.printf "---------- parsing %s\n" str in
		let sent = sentCreate dict str in
		  parseAndPrint sent po
	      );
      done
    with x -> ()
;;

let po = poCreate ();;
let () = poSetLinkageLimit po 1000;;

let defaultDict = dictCreate ("en");;

let () = Printf.printf "Enter input file:\n";;

let inp = read_line ();;

let () = parseFromFile defaultDict po inp;;
