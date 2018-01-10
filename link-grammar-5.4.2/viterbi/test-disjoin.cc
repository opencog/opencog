/*************************************************************************/
/* Copyright (c) 2012, 2013 Linas Vepstas <linasvepstas@gmail.com>       */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the Viterbi parsing system is subject to the terms of the      */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

/// This file provides a unit test for the operation of the viterbi parser.
#include "test-header.h"
#include "compile.h"

// ==================================================================
// Test some basic atomic functions.

bool test_operator_equals()
{
	Node* na = new Node("abc");
	Node* nb = new Node("abc", 0.8f);
	CHECK_NE(__FUNCTION__, na, nb);
}

bool test_operator_equals2()
{
	Node* na = new Node("abcd", 0.8f*0.8f);
	Node* nb = new Node("abcd", 0.6400001f);
	CHECK(__FUNCTION__, na, nb);
}

int ntest_core()
{
	size_t num_failures = 0;
	if (!test_operator_equals()) num_failures++;
	if (!test_operator_equals2()) num_failures++;
	return num_failures;
}

// ==================================================================
// Test the flatten function

bool test_flatten()
{
	Or* or_right = new Or(
  		ANODE(WORD, "AA1"),
		ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"))
	);
	Or* computed = or_right->flatten();

	Lynk* expected =
	ALINK3(OR,
		ANODE(WORD, "AA1"),
		ANODE(WORD, "BB2"),
		ANODE(WORD, "CC3")
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_flatten_rec()
{
	Or* or_right = new Or(
		ALINK2(OR,
			ANODE(WORD, "AA1"),
  		 	ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3")))
	);
	Or* computed = or_right->flatten();

	Lynk* expected =
	ALINK3(OR,
		ANODE(WORD, "AA1"),
		ANODE(WORD, "BB2"),
		ANODE(WORD, "CC3")
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_flatten_nest()
{
	And* and_right = new And(
		ALINK2(OR,
			ANODE(WORD, "AA1"),
  		 	ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3")))
	);
	Atom* computed = and_right->super_flatten();

	Lynk* expected =
	ALINK3(OR,
		ANODE(WORD, "AA1"),
		ANODE(WORD, "BB2"),
		ANODE(WORD, "CC3")
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_flatten_nest_deep()
{
	And* and_right = new And(
		ALINK3(OR,
			ANODE(WORD, "AA1"),
  		 	ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3")),
			ALINK3(AND,
				ANODE(WORD, "XAA1"),
  		 		ALINK2(AND, ANODE(WORD, "XBB2"), ANODE(WORD, "XCC3")),
  		 		ALINK2(AND, ANODE(WORD, "XDD4"), ANODE(WORD, "XEE5"))
			)
		)
	);
	Atom* computed = and_right->super_flatten();

	Lynk* expected =
	ALINK4(OR,
		ANODE(WORD, "AA1"),
		ANODE(WORD, "BB2"),
		ANODE(WORD, "CC3"),
		ALINK5(AND,
			ANODE(WORD, "XAA1"),
			ANODE(WORD, "XBB2"),
			ANODE(WORD, "XCC3"),
			ANODE(WORD, "XDD4"),
			ANODE(WORD, "XEE5")
		)
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_cost_flatten()
{
	Or* or_right = new Or(
  		ANODEC(WORD, "AA1", 0.01),
		ALINK2C(OR, ANODEC(WORD, "BB2", 0.02), ANODEC(WORD, "CC3", 0.03), 0.001),
	0.1f);
	Or* computed = or_right->flatten();

	Lynk* expected =
	ALINK3C(OR,
		ANODEC(WORD, "AA1", 0.01f),
		ANODEC(WORD, "BB2", 0.021f),
		ANODEC(WORD, "CC3", 0.031f),
	0.1f);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_cost_flatten_rec()
{
	Or* or_right = new Or(
		ALINK2C(OR,
			ANODEC(WORD, "AA1", 0.01f),
  		 	ALINK2C(OR,
				ANODEC(WORD, "BB2", 0.02f),
				ANODEC(WORD, "CC3",0.03f),
			0.003f),
		0.0004f),
	0.1f);
	Or* computed = or_right->flatten();

	Lynk* expected =
	ALINK3C(OR,
		ANODEC(WORD, "AA1", 0.0104f),
		ANODEC(WORD, "BB2", 0.0234f),
		ANODEC(WORD, "CC3", 0.0334f),
	0.1f);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_cost_flatten_nest()
{
	And* and_right = new And(
		ALINK2C(OR,
			ANODEC(WORD, "AA1", 0.01f),
  		 	ALINK2C(OR,
				ANODEC(WORD, "BB2", 0.02f),
				ANODEC(WORD, "CC3", 0.03f),
			0.003f),
		0.0004f),
	0.1f);
	Atom* computed = and_right->super_flatten();

	Lynk* expected =
	ALINK3C(OR,
		ANODEC(WORD, "AA1", 0.01f),
		ANODEC(WORD, "BB2", 0.023f),
		ANODEC(WORD, "CC3", 0.033f),
	0.1004f);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_cost_flatten_nest_deep()
{
	And* and_right = new And(
		ALINK3C(OR,
			ANODEC(WORD, "AA1", 0.01f),
  		 	ALINK2C(OR,
				ANODEC(WORD, "BB2", 0.02f),
				ANODEC(WORD, "CC3", 0.03f),
			0.003f),
			ALINK3C(AND,
				ANODEC(WORD, "XAA1", 0.00001f),
  		 		ALINK2C(AND,
					ANODEC(WORD, "XBB2", 0.00002f),
					ANODEC(WORD, "XCC3", 0.00003f),
				0.000003f),
  		 		ALINK2C(AND,
					ANODEC(WORD, "XDD4", 0.00004f),
					ANODEC(WORD, "XEE5", 0.00005f),
				0.000006f),
			0.5f),
		0.00007f),
	0.1f);
	Atom* computed = and_right->super_flatten();

	Lynk* expected =
	ALINK4C(OR,
		ANODEC(WORD, "AA1", 0.01f),
		ANODEC(WORD, "BB2", 0.023f),
		ANODEC(WORD, "CC3", 0.033f),
		ALINK5C(AND,
			ANODEC(WORD, "XAA1", 0.00001f),
			ANODEC(WORD, "XBB2", 0.000023f),
			ANODEC(WORD, "XCC3", 0.000033f),
			ANODEC(WORD, "XDD4", 0.000046f),
			ANODEC(WORD, "XEE5", 0.000056f),
		0.5f),
	0.10007f);

	CHECK(__FUNCTION__, expected, computed);
}

int ntest_flatten()
{
	size_t num_failures = 0;
	if (!test_flatten()) num_failures++;
	if (!test_flatten_rec()) num_failures++;
	if (!test_flatten_nest()) num_failures++;
	if (!test_flatten_nest_deep()) num_failures++;

	if (!test_cost_flatten()) num_failures++;
	if (!test_cost_flatten_rec()) num_failures++;
	if (!test_cost_flatten_nest()) num_failures++;
	if (!test_cost_flatten_nest_deep()) num_failures++;
	return num_failures;
}

// ==================================================================
// Make sure that the disjoined functions actually work.

bool test_and_dnf_single()
{
	And* and_singleton = new And(ANODE(WORD, "AA1"));
	Atom* computed = and_singleton->disjoin();

	Atom* expected = ANODE(WORD, "AA1");

	CHECK(__FUNCTION__, expected, computed);
}

bool test_and_dnf_double()
{
	And* and_two = new And(ANODE(WORD, "AA1"), ANODE(WORD, "BB2"));
	Atom* computed = and_two->disjoin();

	Lynk* expected =
	ALINK2(AND, ANODE(WORD, "AA1"), ANODE(WORD, "BB2"));

	CHECK(__FUNCTION__, expected, computed);
}

bool test_and_distrib_left()
{
	And* and_right = new And(
		ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3")),
		ANODE(WORD, "RR1"));
	Atom* computed = and_right->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK2(AND, ANODE(WORD, "BB2"), ANODE(WORD, "RR1")),
		ALINK2(AND, ANODE(WORD, "CC3"), ANODE(WORD, "RR1"))
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_and_distrib_right()
{
	And* and_right = new And(ANODE(WORD, "AA1"),
		ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3")));
	Atom* computed = and_right->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK2(AND, ANODE(WORD, "AA1"), ANODE(WORD, "BB2")),
		ALINK2(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3"))
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_and_distrib_middle()
{
	And* and_mid = new And(ANODE(WORD, "AA1"),
		ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3")),
		ANODE(WORD, "DD4"));
	Atom* computed = and_mid->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK3(AND, ANODE(WORD, "AA1"), ANODE(WORD, "BB2"), ANODE(WORD, "DD4")),
		ALINK3(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3"), ANODE(WORD, "DD4"))
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_and_distrib_quad()
{
	And* and_mid = new And(
		ALINK2(OR, ANODE(WORD, "AA1"), ANODE(WORD, "BB2")),
		ALINK2(OR, ANODE(WORD, "CC3"), ANODE(WORD, "DD4")));
	Atom* computed = and_mid->disjoin();

	Lynk* expected =
	ALINK4(OR,
		ALINK2(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3")),
		ALINK2(AND, ANODE(WORD, "BB2"), ANODE(WORD, "CC3")),
		ALINK2(AND, ANODE(WORD, "AA1"), ANODE(WORD, "DD4")),
		ALINK2(AND, ANODE(WORD, "BB2"), ANODE(WORD, "DD4"))
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_and_distrib_quad_right()
{
	And* and_mid = new And(
		ALINK2(OR, ANODE(WORD, "AA1"), ANODE(WORD, "BB2")),
		ALINK2(OR, ANODE(WORD, "CC3"), ANODE(WORD, "DD4")),
		ANODE(WORD, "EE5")
	);
	Atom* computed = and_mid->disjoin();

	Lynk* expected =
	ALINK4(OR,
		ALINK3(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3"), ANODE(WORD, "EE5")),
		ALINK3(AND, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), ANODE(WORD, "EE5")),
		ALINK3(AND, ANODE(WORD, "AA1"), ANODE(WORD, "DD4"), ANODE(WORD, "EE5")),
		ALINK3(AND, ANODE(WORD, "BB2"), ANODE(WORD, "DD4"), ANODE(WORD, "EE5"))
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_and_distrib_quad_left()
{
	And* and_mid = new And(
		ANODE(WORD, "EE5"),
		ALINK2(OR, ANODE(WORD, "AA1"), ANODE(WORD, "BB2")),
		ALINK2(OR, ANODE(WORD, "CC3"), ANODE(WORD, "DD4")));
	Atom* computed = and_mid->disjoin();

	Lynk* expected =
	ALINK4(OR,
		ALINK3(AND, ANODE(WORD, "EE5"), ANODE(WORD, "AA1"), ANODE(WORD, "CC3")),
		ALINK3(AND, ANODE(WORD, "EE5"), ANODE(WORD, "BB2"), ANODE(WORD, "CC3")),
		ALINK3(AND, ANODE(WORD, "EE5"), ANODE(WORD, "AA1"), ANODE(WORD, "DD4")),
		ALINK3(AND, ANODE(WORD, "EE5"), ANODE(WORD, "BB2"), ANODE(WORD, "DD4"))
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_or_dnf_single()
{
	Or* or_singleton = new Or(ANODE(WORD, "AA1"));
	Atom* computed = or_singleton->disjoin();

	Atom* expected = ANODE(WORD, "AA1");

	CHECK(__FUNCTION__, expected, computed);
}

bool test_or_dnf_double()
{
	Or* or_two = new Or(ANODE(WORD, "AA1"), ANODE(WORD, "BB2"));
	Atom* computed = or_two->disjoin();

	Lynk* expected =
	ALINK2(OR, ANODE(WORD, "AA1"), ANODE(WORD, "BB2"));

	CHECK(__FUNCTION__, expected, computed);
}

bool test_or_distrib_left()
{
	Or* or_right = new Or(
		ALINK2(AND,
			ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3")),
			ANODE(WORD, "RR1"))
	);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK2(AND, ANODE(WORD, "BB2"), ANODE(WORD, "RR1")),
		ALINK2(AND, ANODE(WORD, "CC3"), ANODE(WORD, "RR1"))
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_or_distrib_right()
{
	Or* or_right = new Or(
		ALINK2(AND,
			ANODE(WORD, "AA1"),
			ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3")))
	);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK2(AND, ANODE(WORD, "AA1"), ANODE(WORD, "BB2")),
		ALINK2(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3"))
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_or_distrib_nest()
{
	Or* or_right = new Or(
		ALINK1(OR,
			ALINK2(AND,
				ANODE(WORD, "AA1"),
	  		 	ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"))))
	);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK2(AND, ANODE(WORD, "AA1"), ANODE(WORD, "BB2")),
		ALINK2(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3"))
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_or_distrib_nest2()
{
	Or* or_right = new Or(
		ALINK3(OR,
			ANODE(WORD, "DD4"),
			ALINK2(AND,
				ANODE(WORD, "AA1"),
	  		 	ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"))),
			ANODE(WORD, "EE5"))
	);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK4(OR,
		ANODE(WORD, "DD4"),
		ALINK2(AND, ANODE(WORD, "AA1"), ANODE(WORD, "BB2")),
		ALINK2(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3")),
		ANODE(WORD, "EE5")
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_or_distrib_nest3()
{
	Or* or_right = new Or(
		ANODE(WORD, "AA1"),
  	 	ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3")));
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK3(OR,
		ANODE(WORD, "AA1"),
		ANODE(WORD, "BB2"),
		ANODE(WORD, "CC3")
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_or_distrib_nest4()
{
	Or* or_right = new Or(
		ALINK2(OR,
			ANODE(WORD, "AA1"),
  		 	ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3")))
	);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK3(OR,
		ANODE(WORD, "AA1"),
		ANODE(WORD, "BB2"),
		ANODE(WORD, "CC3")
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_or_distrib_nest5()
{
	And* and_right = new And(
		ALINK2(OR,
			ANODE(WORD, "AA1"),
  		 	ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3")))
	);
	Atom* computed = and_right->disjoin();

	Lynk* expected =
	ALINK3(OR,
		ANODE(WORD, "AA1"),
		ANODE(WORD, "BB2"),
		ANODE(WORD, "CC3")
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_or_distrib_nest6()
{
	Or* or_right = new Or(
		ALINK1(AND,
			ALINK2(OR,
				ANODE(WORD, "AA1"),
	  		 	ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3")))
			)
	);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK3(OR,
		ANODE(WORD, "AA1"),
		ANODE(WORD, "BB2"),
		ANODE(WORD, "CC3")
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_or_distrib_nest7()
{
	Or* or_right = new Or(
		ALINK3(AND,
			ANODE(WORD, "DD4"),
			ALINK2(OR,
				ANODE(WORD, "AA1"),
	  		 	ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"))),
			ANODE(WORD, "EE5"))
	);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK3(OR,
		ALINK3(AND, ANODE(WORD, "DD4"), ANODE(WORD, "AA1"), ANODE(WORD, "EE5")),
		ALINK3(AND, ANODE(WORD, "DD4"), ANODE(WORD, "BB2"), ANODE(WORD, "EE5")),
		ALINK3(AND, ANODE(WORD, "DD4"), ANODE(WORD, "CC3"), ANODE(WORD, "EE5"))
	);

	CHECK(__FUNCTION__, expected, computed);
}

int ntest_disjoin()
{
	size_t num_failures = 0;
	if (!test_and_dnf_single()) num_failures++;
	if (!test_and_dnf_double()) num_failures++;
	if (!test_and_distrib_left()) num_failures++;
	if (!test_and_distrib_right()) num_failures++;
	if (!test_and_distrib_middle()) num_failures++;
	if (!test_and_distrib_quad()) num_failures++;
	if (!test_and_distrib_quad_right()) num_failures++;
	if (!test_and_distrib_quad_left()) num_failures++;

	if (!test_or_dnf_single()) num_failures++;
	if (!test_or_dnf_double()) num_failures++;
	if (!test_or_distrib_left()) num_failures++;
	if (!test_or_distrib_right()) num_failures++;

	if (!test_or_distrib_nest()) num_failures++;
	if (!test_or_distrib_nest2()) num_failures++;
	if (!test_or_distrib_nest3()) num_failures++;
	if (!test_or_distrib_nest4()) num_failures++;
	if (!test_or_distrib_nest5()) num_failures++;
	if (!test_or_distrib_nest6()) num_failures++;
	if (!test_or_distrib_nest7()) num_failures++;
	return num_failures;
}


// ==================================================================
// Make sure that the disjoined functions actually work.
// Identical to the above, except this time, there are costs involved.

bool test_costly_and_dnf_single()
{
	And* and_singleton = new And(ANODEC(WORD, "AA1", 1.5f));
	Atom* computed = and_singleton->disjoin();

	Atom* expected = ANODEC(WORD, "AA1", 1.5f);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_and_dnf_single_ne()
{
	And* and_singleton = new And(ANODEC(WORD, "AA1", 1.5f));
	Atom* computed = and_singleton->disjoin();

	Atom* expected = ANODEC(WORD, "AA1", 31.6f);

	CHECK_NE(__FUNCTION__, expected, computed);
}

bool test_costly_and_dnf_single_sum()
{
	And* and_singleton = new And(ANODEC(WORD, "AA1", 1.5f), 0.3f);
	Atom* computed = and_singleton->disjoin();

	Atom* expected = ANODEC(WORD, "AA1", 1.8f);

	CHECK(__FUNCTION__, expected, computed);
}

// -----------------------------------------------
bool test_costly_and_dnf_double()
{
	And* and_two = new And(ANODE(WORD, "AA1"), ANODE(WORD, "BB2"), 1.6f);
	Atom* computed = and_two->disjoin();

	Lynk* expected =
	ALINK2C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "BB2"), 1.6f);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_and_dnf_double_w()
{
	And* and_two = new And(ANODE(WORD, "AA1"), ANODEC(WORD, "BB2", 2.8f));
	Atom* computed = and_two->disjoin();

	Lynk* expected =
	ALINK2(AND, ANODE(WORD, "AA1"), ANODEC(WORD, "BB2", 2.8f));

	CHECK(__FUNCTION__, expected, computed);
}

// -----------------------------------------------
bool test_costly_and_distrib_left()
{
	And* and_right = new And(
		ALINK2C(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 1.1f),
		ANODE(WORD, "RR1"));
	Atom* computed = and_right->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK2C(AND, ANODE(WORD, "BB2"), ANODE(WORD, "RR1"), 1.1f),
		ALINK2C(AND, ANODE(WORD, "CC3"), ANODE(WORD, "RR1"), 1.1f)
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_and_distrib_left_sum()
{
	And* and_right = new And(
		ALINK2C(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 1.1f),
		ANODE(WORD, "RR1"), 0.8f);
	Atom* computed = and_right->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK2C(AND, ANODE(WORD, "BB2"), ANODE(WORD, "RR1"), 1.9f),
		ALINK2C(AND, ANODE(WORD, "CC3"), ANODE(WORD, "RR1"), 1.9f)
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_and_distrib_left_w()
{
	And* and_right = new And(
		ALINK2(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3")),
		ANODEC(WORD, "RR1", 3.14f));
	Atom* computed = and_right->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK2(AND, ANODE(WORD, "BB2"), ANODEC(WORD, "RR1", 3.14f)),
		ALINK2(AND, ANODE(WORD, "CC3"), ANODEC(WORD, "RR1", 3.14f))
	);

	CHECK(__FUNCTION__, expected, computed);
}

// -----------------------------------------------
bool test_costly_and_distrib_right()
{
	And* and_right = new And(ANODE(WORD, "AA1"),
		ALINK2C(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 0.35));
	Atom* computed = and_right->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK2C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "BB2"), 0.35),
		ALINK2C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3"), 0.35)
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_and_distrib_right_sum()
{
	And* and_right = new And(ANODE(WORD, "AA1"),
		ALINK2C(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 0.35f), 0.5f);
	Atom* computed = and_right->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK2C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "BB2"), 0.85f),
		ALINK2C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3"), 0.85f)
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_and_distrib_middle()
{
	And* and_mid = new And(ANODE(WORD, "AA1"),
		ALINK2C(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 2.1f),
		ANODE(WORD, "DD4"), 0.6f);
	Atom* computed = and_mid->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK3C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "BB2"), ANODE(WORD, "DD4"), 2.7f),
		ALINK3C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3"), ANODE(WORD, "DD4"), 2.7f)
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_and_distrib_quad()
{
	And* and_mid = new And(
		ALINK2C(OR, ANODE(WORD, "AA1"), ANODE(WORD, "BB2"), 1.1f),
		ALINK2C(OR, ANODE(WORD, "CC3"), ANODE(WORD, "DD4"), 2.2f), 0.4f);
	Atom* computed = and_mid->disjoin();

	Lynk* expected =
	ALINK4(OR,
		ALINK2C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3"), 3.7f),
		ALINK2C(AND, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 3.7f),
		ALINK2C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "DD4"), 3.7f),
		ALINK2C(AND, ANODE(WORD, "BB2"), ANODE(WORD, "DD4"), 3.7f)
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_and_distrib_quad_right()
{
	And* and_mid = new And(
		ALINK2C(OR, ANODE(WORD, "AA1"), ANODE(WORD, "BB2"), 0.25f),
		ALINK2C(OR, ANODE(WORD, "CC3"), ANODE(WORD, "DD4"), 0.35f),
		ANODE(WORD, "EE5"), 0.5f
	);
	Atom* computed = and_mid->disjoin();

	Lynk* expected =
	ALINK4(OR,
		ALINK3C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3"), ANODE(WORD, "EE5"), 1.1f),
		ALINK3C(AND, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), ANODE(WORD, "EE5"), 1.1f),
		ALINK3C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "DD4"), ANODE(WORD, "EE5"), 1.1f),
		ALINK3C(AND, ANODE(WORD, "BB2"), ANODE(WORD, "DD4"), ANODE(WORD, "EE5"), 1.1f)
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_and_distrib_quad_left()
{
	And* and_mid = new And(
		ANODE(WORD, "EE5"),
		ALINK2C(OR, ANODE(WORD, "AA1"), ANODE(WORD, "BB2"), 0.11f),
		ALINK2C(OR, ANODE(WORD, "CC3"), ANODE(WORD, "DD4"), 0.22f), 0.1f);
	Atom* computed = and_mid->disjoin();

	Lynk* expected =
	ALINK4(OR,
		ALINK3C(AND, ANODE(WORD, "EE5"), ANODE(WORD, "AA1"), ANODE(WORD, "CC3"), 0.43f),
		ALINK3C(AND, ANODE(WORD, "EE5"), ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 0.43f),
		ALINK3C(AND, ANODE(WORD, "EE5"), ANODE(WORD, "AA1"), ANODE(WORD, "DD4"), 0.43f),
		ALINK3C(AND, ANODE(WORD, "EE5"), ANODE(WORD, "BB2"), ANODE(WORD, "DD4"), 0.43f)
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_or_dnf_single()
{
	Or* or_singleton = new Or(ANODE(WORD, "AA1"), 0.75f);
	Atom* computed = or_singleton->disjoin();

	Atom* expected = ANODEC(WORD, "AA1", 0.75f);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_or_dnf_double()
{
	Or* or_two = new Or(ANODE(WORD, "AA1"), ANODE(WORD, "BB2"), 0.65f);
	Atom* computed = or_two->disjoin();

	Lynk* expected =
	ALINK2C(OR, ANODE(WORD, "AA1"), ANODE(WORD, "BB2"), 0.65f);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_or_distrib_left()
{
	Or* or_right = new Or(
		ALINK2C(AND,
			ALINK2C(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 0.1f),
			ANODE(WORD, "RR1"), 0.22f),
	0.333f);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK2C(AND, ANODE(WORD, "BB2"), ANODE(WORD, "RR1"), 0.653f),
		ALINK2C(AND, ANODE(WORD, "CC3"), ANODE(WORD, "RR1"), 0.653f)
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_or_distrib_right()
{
	Or* or_right = new Or(
		ALINK2C(AND,
			ANODE(WORD, "AA1"),
			ALINK2C(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 0.111f), 0.222f),
	0.5f);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK2C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "BB2"), 0.833f),
		ALINK2C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3"), 0.833f)
	);

	CHECK(__FUNCTION__, expected, computed);
}

// -----------------------------------------------
bool test_costly_or_distrib_nest()
{
	Or* or_right = new Or(
		ALINK1C(OR,
			ALINK2C(AND,
				ANODE(WORD, "AA1"),
	  		 	ALINK2C(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 0.1f),
			0.02f),
		0.003f),
	0.0004f);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK2(OR,
		ALINK2C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "BB2"), 0.1234f),
		ALINK2C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3"), 0.1234f)
	);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_or_distrib_nest2()
{
	Or* or_right = new Or(
		ALINK1C(OR,
			ALINK2C(AND,
				ANODE(WORD, "AA1"),
	  		 	ANODEC(WORD, "BB2", 0.1f),
			0.02f),
		0.003f),
	0.0004f);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK2C(AND, ANODE(WORD, "AA1"), ANODEC(WORD, "BB2", 0.1f), 0.0234f);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_or_distrib_nest3()
{
	Or* or_right = new Or(
		ALINK1C(OR,
	  	 	ANODEC(WORD, "BB2", 0.1f),
		0.003f),
	0.0004f);
	Atom* computed = or_right->disjoin();

	Atom* expected = ANODEC(WORD, "BB2", 0.1034f);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_or_distrib_nest4()
{
	Or* or_right = new Or(
		ALINK3C(OR,
			ANODE(WORD, "DD4"),
			ALINK2C(AND,
				ANODE(WORD, "AA1"),
	  		 	ALINK2C(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 0.1f), 0.02f),
			ANODE(WORD, "EE5"), 0.003f),
	0.0004f);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK4C(OR,
		ANODEC(WORD, "DD4", 0.0f),
		ALINK2C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "BB2"), 0.12f),
		ALINK2C(AND, ANODE(WORD, "AA1"), ANODE(WORD, "CC3"), 0.12f),
		ANODEC(WORD, "EE5", 0.0f),
	0.0034f);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_or_distrib_nest5()
{
	Or* or_right = new Or(
		ANODE(WORD, "AA1"),
  	 	ALINK2C(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 0.01),
	0.1f);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK3C(OR,
		ANODE(WORD, "AA1"),
		ANODEC(WORD, "BB2", 0.01f),
		ANODEC(WORD, "CC3", 0.01f),
	0.1f);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_or_distrib_nest6()
{
	Or* or_right = new Or(
		ALINK2C(OR,
			ANODE(WORD, "AA1"),
  		 	ALINK2C(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 0.001f),
		0.01f),
	0.1f);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK3C(OR,
		ANODE(WORD, "AA1"),
		ANODEC(WORD, "BB2", 0.001f),
		ANODEC(WORD, "CC3", 0.001f),
	0.11f);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_or_distrib_nest7()
{
	And* and_right = new And(
		ALINK2C(OR,
			ANODE(WORD, "AA1"),
  		 	ALINK2C(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 0.001f),
		0.01f),
	0.1f);
	Atom* computed = and_right->disjoin();

	Lynk* expected =
	ALINK3C(OR,
		ANODE(WORD, "AA1"),
		ANODEC(WORD, "BB2", 0.001f),
		ANODEC(WORD, "CC3", 0.001f),
	0.11f);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_or_distrib_nest8()
{
	Or* or_right = new Or(
		ALINK1C(AND,
			ALINK2C(OR,
				ANODE(WORD, "AA1"),
	  		 	ALINK2C(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 0.0004f),
			0.003f),
		0.02f),
	0.1f);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK3C(OR,
		ANODE(WORD, "AA1"),
		ANODEC(WORD, "BB2", 0.0004f),
		ANODEC(WORD, "CC3", 0.0004f),
	0.123f);

	CHECK(__FUNCTION__, expected, computed);
}

bool test_costly_or_distrib_nest9()
{
	Or* or_right = new Or(
		ALINK3C(AND,
			ANODE(WORD, "DD4"),
			ALINK2C(OR,
				ANODE(WORD, "AA1"),
	  		 	ALINK2C(OR, ANODE(WORD, "BB2"), ANODE(WORD, "CC3"), 0.1f), 0.02f),
			ANODE(WORD, "EE5"), 0.003f),
	0.0004f);
	Atom* computed = or_right->disjoin();

	Lynk* expected =
	ALINK3C(OR,
		ALINK3C(AND, ANODE(WORD, "DD4"), ANODE(WORD, "AA1"), ANODE(WORD, "EE5"), 0.0234f),
		ALINK3C(AND, ANODE(WORD, "DD4"), ANODEC(WORD, "BB2", 0.1f), ANODE(WORD, "EE5"), 0.0234f),
		ALINK3C(AND, ANODE(WORD, "DD4"), ANODEC(WORD, "CC3", 0.1f), ANODE(WORD, "EE5"), 0.0234f),
	0.0f);

	CHECK(__FUNCTION__, expected, computed);
}

int ntest_costly_disjoin()
{
	size_t num_failures = 0;
	if (!test_costly_and_dnf_single()) num_failures++;
	if (!test_costly_and_dnf_single_ne()) num_failures++;
	if (!test_costly_and_dnf_single_sum()) num_failures++;

	if (!test_costly_and_dnf_double()) num_failures++;
	if (!test_costly_and_dnf_double_w()) num_failures++;

	if (!test_costly_and_distrib_left()) num_failures++;
	if (!test_costly_and_distrib_left_sum()) num_failures++;
	if (!test_costly_and_distrib_left_w()) num_failures++;

	if (!test_costly_and_distrib_right()) num_failures++;
	if (!test_costly_and_distrib_right_sum()) num_failures++;

	if (!test_costly_and_distrib_middle()) num_failures++;
	if (!test_costly_and_distrib_quad()) num_failures++;
	if (!test_costly_and_distrib_quad_right()) num_failures++;
	if (!test_costly_and_distrib_quad_left()) num_failures++;

	if (!test_costly_or_dnf_single()) num_failures++;
	if (!test_costly_or_dnf_double()) num_failures++;
	if (!test_costly_or_distrib_left()) num_failures++;
	if (!test_costly_or_distrib_right()) num_failures++;

	if (!test_costly_or_distrib_nest()) num_failures++;
	if (!test_costly_or_distrib_nest2()) num_failures++;
	if (!test_costly_or_distrib_nest3()) num_failures++;
	if (!test_costly_or_distrib_nest4()) num_failures++;
	if (!test_costly_or_distrib_nest5()) num_failures++;
	if (!test_costly_or_distrib_nest6()) num_failures++;
	if (!test_costly_or_distrib_nest7()) num_failures++;
	if (!test_costly_or_distrib_nest8()) num_failures++;
	if (!test_costly_or_distrib_nest9()) num_failures++;
	return num_failures;
}

// ==================================================================

int
main(int argc, char *argv[])
{
	size_t num_failures = 0;
	bool exit_on_fail = true;

	num_failures += ntest_core();
	report(num_failures, exit_on_fail);

	num_failures += ntest_flatten();
	report(num_failures, exit_on_fail);

	num_failures += ntest_disjoin();
	report(num_failures, exit_on_fail);

	num_failures += ntest_costly_disjoin();
	report(num_failures, exit_on_fail);

	exit (0);
}

