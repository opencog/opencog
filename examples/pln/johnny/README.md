PLN demo
=========

This example shows basic inferencing pattern of a logic syllogism using some rules of PLN.

```
If Johnny is eating sweets every day, he is placing himself at rish for diabetes.
Johnny does not eat sweets everyday.
```

Using PLN on the logical representation on the following assertions, the following output is produced.


```
Johnny is not placing himself at risk for diabetes.
```

This output, however, is represented in AtomSpace notation.


PLN rules needed:

- ModusPonenRule
- NotRule

### Instructions to run it in the CogServer

Start the cogserver at ```/opencog/build``` with ```./opencog/server/cogserver```.
In the cogserver, type ```scm```.
In the cogserver, clear the atomspace with ```(clear)```.
Then run ```(load "../examples/Johnny_example/Assertions.scm")```.

### The inference process

#### AtomSpace starting contents:

##### If Johnny is eating sweets every day, he is placing himself at rish for diabetes.
```
(ImplicationLink
	(EvaluationLink
		(PredicateNode "every@111")
        (ListLink
			(ConceptNode "day@222")
			(EvaluationLink
				(PredicateNode "eating@333")
				(ListLink
					(ConceptNode "Johnny@444")
					(ConceptNode "Sweets@555")))))
	(EvaluationLink
		(PredicateNode "at@123")
		(ListLink
			(EvaluationLink
				(PredicateNode "placing@234")
				(ListLink
					(ConceptNode "he@345")
					(ConceptNode "himself@456")))
			(ConceptNode "risk@44"))))

(EvaluationLink
	(PredicateNode "for@55")
	(ListLink
		(ConceptNode "risk@44")
		(ConceptNode "diabetes@66")))
```
##### Johnny does not eat sweets everyday.
```
(NotLink
	(EvaluationLink
		(PredicateNode "every@111")
        (ListLink
			(ConceptNode "day@222")
			(EvaluationLink
				(PredicateNode "eating@333")
				(ListLink
					(ConceptNode "Johnny@444")
					(ConceptNode "Sweets@555"))))))
```

#### Inference steps

##### 1) NotRule

###### Input
```
(NotLink
	(EvaluationLink
		(PredicateNode "every@111")
        (ListLink
			(ConceptNode "day@222")
			(EvaluationLink
				(PredicateNode "eating@333")
				(ListLink
					(ConceptNode "Johnny@444")
					(ConceptNode "Sweets@555"))))))
```

##### Output
```
(EvaluationLink
	(PredicateNode "every@111")
    (ListLink
		(ConceptNode "day@222")
		(EvaluationLink
			(PredicateNode "eating@333")
			(ListLink
				(ConceptNode "Johnny@444")
				(ConceptNode "Sweets@555")))))
```

##### 2) ModusPonenRule

###### Input is previous output and the following atom
```
(ImplicationLink
	(EvaluationLink
		(PredicateNode "every@111")
        (ListLink
			(ConceptNode "day@222")
			(EvaluationLink
				(PredicateNode "eating@333")
				(ListLink
					(ConceptNode "Johnny@444")
					(ConceptNode "Sweets@555")))))
	(EvaluationLink
		(PredicateNode "at@123")
		(ListLink
			(EvaluationLink
				(PredicateNode "placing@234")
				(ListLink
					(ConceptNode "he@345")
					(ConceptNode "himself@456")))
			(ConceptNode "risk@44"))))

(EvaluationLink
	(PredicateNode "for@55")
	(ListLink
		(ConceptNode "risk@44")
		(ConceptNode "diabetes@66")))
```
 
###### Output
```
(EvaluationLink
	(PredicateNode "at@123")
	(ListLink
		(EvaluationLink
			(PredicateNode "placing@234")
			(ListLink
				(ConceptNode "he@345")
				(ConceptNode "himself@456")))
		(ConceptNode "risk@44")))
```

##### 3) NotRule

###### Input is previous output

###### Final Output
```
(NotLink
	(EvaluationLink
		(PredicateNode "at@123")
		(ListLink
			(EvaluationLink
				(PredicateNode "placing@234")
				(ListLink
					(ConceptNode "he@345")
					(ConceptNode "himself@456")))
			(ConceptNode "risk@44"))))

```

