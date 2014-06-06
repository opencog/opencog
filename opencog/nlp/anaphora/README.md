
Example:

Note that since GreaterThanLink and WordNumberNode have not been implemented yet, using generated
fake GreaterThanLinks(using ListLinks instead) and WordNumberLinks(using ListLinks instead) instead.
These fake links are located in "/opencog/opencog/nlp/anaphora/atomspace_data/generated_data.scm".

Sentences:

1) "Tom ate an apple under a tree."
2) "It was delicious."
3) "It was small."

1. Load rules and testing data

Change "YOUR_PATH" to the path to the opencog directory

(load-scm-from-file "YOUR_PATH/opencog/opencog/nlp/anaphora/atomspace_data/atomspace.scm")
(load-scm-from-file "YOUR_PATH/opencog/opencog/nlp/anaphora/atomspace_data/generated_data.scm")
(load-scm-from-file "YOUR_PATH/opencog/opencog/nlp/anaphora/rules/anaphora_resolution.scm")
(load-scm-from-file "YOUR_PATH/opencog/opencog/nlp/anaphora/rules/pronoun_finder.scm")

2. Finding pronouns

(cog-bind pronoun-finder)

3. Doing anaphora resolution

(cog-bind anaphora-resolution)

Testing result:

(ListLink
   (ReferenceLink
      (WordInstanceNode "Tom@765cc72e-56f7-4f43-b1e4-f3954091a5fb")
      (WordInstanceNode "it@e60e24e7-80f9-4818-b5e4-e5efc0695268")
   )
   (ReferenceLink
      (WordInstanceNode "apple@a96a5f2d-f168-494d-8240-3c622884bffc")
      (WordInstanceNode "it@e60e24e7-80f9-4818-b5e4-e5efc0695268")
   )
   (ReferenceLink
      (WordInstanceNode "tree@fb31eb83-0271-4349-9dd8-5d2a0d248c81")
      (WordInstanceNode "it@e60e24e7-80f9-4818-b5e4-e5efc0695268")
   )
   (ReferenceLink
      (WordInstanceNode "it@e60e24e7-80f9-4818-b5e4-e5efc0695268")
      (WordInstanceNode "it@c48321a8-1dca-456b-90e9-01e8b588419e")
   )
   (ReferenceLink
      (WordInstanceNode "Tom@765cc72e-56f7-4f43-b1e4-f3954091a5fb")
      (WordInstanceNode "it@c48321a8-1dca-456b-90e9-01e8b588419e")
   )
   (ReferenceLink
      (WordInstanceNode "apple@a96a5f2d-f168-494d-8240-3c622884bffc")
      (WordInstanceNode "it@c48321a8-1dca-456b-90e9-01e8b588419e")
   )
   (ReferenceLink
      (WordInstanceNode "tree@fb31eb83-0271-4349-9dd8-5d2a0d248c81")
      (WordInstanceNode "it@c48321a8-1dca-456b-90e9-01e8b588419e")
   )
)