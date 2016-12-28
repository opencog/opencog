# Number rule for OpenCog nlp pipeline

###The need for this rule
- The relex and relex2logic result of sentences (with numeric values) parsed through OpenCog nlp pipeline is not recognizing a number as a NumberNode

Given a SentenceNode sent, (num-rule sent) identifies any number (in string or numeric representation) 
from the sentence parsed through the nlp-parse, creates a NumberNode for those numbers
and creates a `ReferenceLink` to Word Instances reffering to the numbers in a sentence.

num-rule.scm
 
- accepts a SentenceNode
- get the words (in their order) of a sentence
- trim the string of the word instances and get a string of the main word
- check the list of words for valid number strings, and create a NumberNode for each 
- Finding the valid words, stores the corresponding WordNode index to create a ReferenceLink.

strtonum.scm

- convers strings like "two hundred and twenty five" into
numbers like 225.0 that can be used in numerical calculations.

Example:

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

  	guile> (define sent (nlp-parse "The fridge should cost no more than fifteen thousand dollars"))
	
	guile> (num-rule sent)

####Result:
(ReferenceLink

   (NumberNode "15000.000000")
   
   (ListLink
   
      (WordInstanceNode "fifteen@0271e836-ea33-41d8-af43-973fdd6a230b")
      
      (WordInstanceNode "thousand@6e82f899-eabf-434b-b052-ee3a9751f6bb")
   )
)

The `ReferenceLink` will just link the word instances to the NumberNode.

###More Example Sentences:
	
	guile > (define sent (nlp-parse "I have 210 Pandas."))
	guile > (num-rule sent)
	####Result: 
	(ReferenceLink
   		(NumberNode "210.000000")
   		(ListLink
      			(WordInstanceNode "210@7a21f0d6-1ba1-45fe-9ff2-88e9ba43bf14")
   		)
	)
	
	guile > (define sent (nlp-parse "I have two hundred ten Pandas."))
	guile > (num-rule sent)
	####Result: 
	(ReferenceLink
   		(NumberNode "210.000000")
   		(ListLink
      			(WordInstanceNode "two@b4431c38-1500-4020-9070-6a54cb001a72")
      			(WordInstanceNode "hundred@853f72c2-e2ba-4476-96ae-da845ace82ef")
      			(WordInstanceNode "ten@772cf570-6d8d-473f-a488-2ea0a71c32db")
   		)
	)

	guile > (define sent (nlp-parse "I have two-hundred-ten Pandas."))
	guile > (num-rule sent)
	####Result:
	(ReferenceLink
   		(NumberNode "210.000000")
   		(ListLink
		      	(WordInstanceNode "two-hundred-ten@67622c33-d36e-4332-86b8-b4065379bb49")
   		)
	)
	
	guile > (define sent (nlp-parse "I have two hundred and ten Pandas."))
	guile > (num-rule sent)
	####Result: 
	(ReferenceLink
   	(NumberNode "210.000000")
   		(ListLink
      			(WordInstanceNode "two@6f4bd0b6-b3d6-49b1-ad9d-84c4f265a8e4")
      			(WordInstanceNode "hundred@411431d2-9d5e-4164-9a99-5f6c30ce28a6")
      			(WordInstanceNode "and@2698aa4a-1e10-4d07-937f-ef45103a0fe1")
      			(WordInstanceNode "ten@40990568-bc84-4081-88f5-9c821b17d482")
   		)
	)

	

###LIMITATIONS

Currently it can only be used to convert only one appearance of a number in a sentence. It can identify multiple numbers but it gives a wrong answer. (Working on it)

guile> (define sent2 (nlp-parse "Two fridges should cost no more than fifteen thousand dollars"))

guile> (num-rule sent2)

(ReferenceLink

   (NumberNode "17000.000000")
   
   (ListLink
   
      (WordInstanceNode "two@3567bd97-9113-40f5-ac86-8f9c5fa8e835")
      (WordInstanceNode "fifteen@189dfe53-36a9-42cf-80f5-c39be3b21f0b")
      (WordInstanceNode "thousand@cec50370-94f1-4db5-87b7-1dfd002b6ddb")
   )
)

###Next Steps
Sentences containing compund words such as "four-by-two" "two-way" etc...
should be handled differently.
	For example, "four-by-two" may be represented as 8.0








