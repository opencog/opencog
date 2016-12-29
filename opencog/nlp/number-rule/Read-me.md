# Number rule for OpenCog nlp pipeline

###The need for this rule

When a sentence that contains a number is parsed and goes through the OpenCog nlp pipeline, the number is not recognized as a NumberNode.</br> </br>
	For example: “I want to buy two refrigerators each costing less than fifty dollars.” </br></br>
	Here, "two" and "fifty" should have a reference to NumberNodes "2" and "50" respectively.</br></br>

###What Number-Rule does 
Given a SentenceNode sent, (num-rule sent) identifies every number (in string or numeric representation), creates a NumberNode for those numbers and creates a `ReferenceLink` to Word Instances reffering to the numbers in that sentence.

###Number-rule contains two Scheme Files

####num-rule.scm
- accepts a SentenceNode
- get the words (in their order) of a sentence
- trim the name of the word instance to get the pure word
- validate if that word is a number or not
- gather all words that validate as numbers
- create a Reference link for a list of the word instances that are numbers with a corresponding NumberNode

####strtonum.scm
- Provides utilities such as list iterators for num-rule to work with
- Does string manipulations
- Validates Number words
- Convers strings like "two hundred and twenty five" into numbers like 225.0 that can be used in numerical calculations.

####Usage:

- Start CogServer at port 17001
- Start relex server
- telnet localhost 17001
- Enter to the scheme shell 
	opencog> scm
- Load nlp modules	
	guile> (use-modules (opencog) (opencog nlp) (opencog nlp chatbot) (opencog nlp relex2logic))
- Load the number rule procedures, apply it to sent and see the result

	guile> (load-scm-from-file "opencog-master/opencog/opencog/nlp/Number-rule/strtonum.scm")
  
	guile> (load-scm-from-file "opencog-master/opencog/opencog/nlp/Number-rule/num-rule.scm")
  
- Parse the following sentence and apply num-rule to it

  	guile> (define sent (nlp-parse "Two fridges should cost no more than One thousand five hundred and forty dollars"))
	guile> (num-rule sent)
	
####Result:

	(ReferenceLink
	   (NumberNode "2.000000")
	   (ListLink
	      (WordInstanceNode "two@5548dc12-ceac-45d9-a869-1768a5cef4c5")
	   )
	)
	(ReferenceLink
	   (NumberNode "1540.000000")
	   (ListLink
	      (WordInstanceNode "One@239b5a62-f610-4de4-812d-76bc6d3abd5c")
	      (WordInstanceNode "thousand@2ba05746-0766-44c2-9b01-dbbe884b6d80")
	      (WordInstanceNode "five@07eb066e-08a2-406b-97e3-47fba0b8d038")
	      (WordInstanceNode "hundred@a3882137-10b1-4502-8dd7-e6b5e03ff88d")
	      (WordInstanceNode "and@45cb780f-af98-43ed-be7a-d320a62d2e23")
	      (WordInstanceNode "forty@63a48a2a-06b8-4f9e-9203-ddffd68f284b")
	   )
	)
	
	The `ReferenceLink` will just link the word instances to the NumberNode.

###More Examples:
	
	guile > (define sent (nlp-parse "I want 20 sinks."))
	guile > (num-rule sent)
	####Result: 
	(ReferenceLink
   		(NumberNode "20.000000")
   		(ListLink
      			(WordInstanceNode "20@5943eda7-9507-43e0-8bfb-65f4e8e954ae")
   		)
	)
	
	guile > (define sent (nlp-parse "I spent two hundred and ten dollars on two tables."))
	guile > (num-rule sent)
	####Result: 
	(ReferenceLink
	   (NumberNode "210.000000")
	   (ListLink
	      (WordInstanceNode "two@9a66cf52-7645-49c3-bf48-6ad81f25c6bf")
	      (WordInstanceNode "hundred@08256d80-6902-4b85-9147-7d897d1d7e93")
	      (WordInstanceNode "and@dd09fe08-6cca-4683-8d79-0a0a42120638")
	      (WordInstanceNode "ten@134557a5-9acc-4e63-bfd2-9eba59e1bb82")
	   )
	)
	(ReferenceLink
	   (NumberNode "2.000000")
	   (ListLink
	      (WordInstanceNode "two@78d47878-a4c2-4341-92ce-0af0b745306e")
	   )
	)
	
	Also works for the following string variations:
	- "I spent two hundred ten dollars."
	- "I spent two-hundred-ten Pandas."

###Next Steps
Sentences containing compund words such as "four-by-two" "two-way" etc...
should be handled differently.
	For example, "four-by-two" should also perhaps be linked to 8.0
