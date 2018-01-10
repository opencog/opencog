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

#include <link-grammar/link-includes.h>
#include <link-grammar/read-dict.h>

// ==================================================================
// A simple hello test; several different dictionaries
// should give exactly the same output.  The input sentence
// is just one word, it should connect to the left-wall in
// just one way. The result should be just one alternative:
// that alternatives has an empty state, and output with
// just one link.
bool test_hello(const char *id, const char *dict_str,
                bool empty_state, bool must_connect)
{
	total_tests++;

	Dictionary dict = dictionary_create_from_utf8(dict_str);

	// print_dictionary_data(dict);

	Parser parser(dict);
	parser.streamin("Hello");

	// This is the expected output, no matter what the
	// dictionary may be.  Its just one word, connected to the wall.
	Lynk* one_word =
	ALINK3(LING,
		ANODE(LING_TYPE, "Wd"),
		ALINK2(WORD_DISJ,
			ANODE(WORD, "LEFT-WALL"),
			ANODE(CONNECTOR, "Wd+")
		),
		ALINK2(WORD_DISJ,
			ANODE(WORD, "Hello"),
			ANODE(CONNECTOR, "Wd-")
		)
	);

	if (empty_state)
	{
		// This is the expected set of alternatives: just one alternative,
		// a single, empty state, and the output, above.
		Lynk* ans =
		ALINK1(SET,
			ALINK3(STATE_TRIPLE,
				ALINK0(SEQ),
				ALINK0(SEQ),
				ALINK1(SET, one_word)
			)
		);

		Lynk* alts = parser.get_alternatives();
		if (not (ans->operator==(alts)))
		{
			cout << "Error: test failure on test \"" << id << "\"" << endl;
			cout << "=== Expecting:\n" << ans << endl;
			cout << "=== Got:\n" << alts << endl;
			return false;
		}
	}
	else
	{
		// This test will have lots of alternatives. One should have
		// an empty state.
		Lynk* ans =
			ALINK3(STATE_TRIPLE,
				ALINK0(SEQ),
				ALINK0(SEQ),
				ALINK1(SET, one_word)
			);

		Lynk* out = new Set(one_word);

		bool pass_test = false;
		Lynk* alts = parser.get_alternatives();
		foreach_outgoing(StateTriple*, sp, alts)
		{
			// At least one alternative should have an empty state.
			if (ans->operator==(sp))
				pass_test = true;

			// In all cases, the output should be just the one word,
			// no matter what the state.
			if (must_connect and not sp->get_output()->operator==(out))
				pass_test = false;
		}
		if (pass_test)
		{
			cout<<"PASS: test_hello(" << id << ") " << endl;
			return true;
		}
		cout << "Error: test failure on test \"" << id << "\"" << endl;
		cout << "=== Expecting:\n" << ans << endl;
		cout << "=== Got:\n" << alts << endl;
		return false;
	}

	cout<<"PASS: test_hello(" << id << ") " << endl;
	return true;
}

bool test_simplest()
{
	return test_hello ("test_simplest",
		"LEFT-WALL: Wd+;"
		"Hello: Wd-;",
		true, true
	);
}

bool test_simple_left_disj()
{
	return test_hello ("simple left disj",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"Hello: Wd-;",
		true, true
	);
}

bool test_simple_optional_left_cset()
{
	return test_hello ("optional left cset",
		"LEFT-WALL: (Wd+ or Wi+ or Wq+) & {CP+} & {Xx+} & {RW+ or Xp+};"
		"Hello: Wd-;",
		false, true
	);
}

bool test_simple_right_disj()
{
	return test_hello ("simple right disj",
		"LEFT-WALL: Wd+;"
		"Hello: Wd- or Wi-;",
		true, true
	);
}

bool test_simple_right_required_cset()
{
	return test_hello ("required right cset",
		"LEFT-WALL: Wd+;"
		"Hello: Wd- or Xi- or (Xj- & (A+ or B+));",
		true, true
	);
}

bool test_simple_optional()
{
	return test_hello ("optionals in both csets",
		"LEFT-WALL: (Wd+ or Wi+ or Wq+) & {CP+} & {Xx+} & {RW+ or Xp+};"
		"Hello: Wd- or Xi- or (Xj- & {A+ or B+});",
		false, true
	);
}

bool test_simple_onereq()
{
	return test_hello ("one required link and opt righties (simple)",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"Hello: Wd- & {A+} & {B+} & {C+};",
		false, true
	);
}

bool test_simple_zeroreq()
{
	return test_hello ("zero required links and opt righties (simple)",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"Hello: {Wd-} & {A+} & {B+} & {C+};",
		false, false
	);
}

bool test_simple_onereq_and_left()
{
	return test_hello ("one required link and opt lefties (simple)",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"Hello: Wd- & {A-} & {B-} & {C+};",
		false, true
	);
}

int ntest_simple()
{
	size_t num_failures = 0;

	if (!test_simplest()) num_failures++;
	if (!test_simple_left_disj()) num_failures++;
	if (!test_simple_optional_left_cset()) num_failures++;
	if (!test_simple_right_disj()) num_failures++;
	if (!test_simple_right_required_cset()) num_failures++;
	if (!test_simple_optional()) num_failures++;
	if (!test_simple_onereq()) num_failures++;
	if (!test_simple_zeroreq()) num_failures++;
	if (!test_simple_onereq_and_left()) num_failures++;
	return num_failures;
}

// ==================================================================
// A test of two alternative parses of a sentence with single word in it.
// Expect to get back a set with two alternative parses, each parse is
// assigned a probability of 1/2.

bool test_alternative(const char *id, const char *dict_str, bool empty_state)
{
	total_tests++;

	Dictionary dict = dictionary_create_from_utf8(dict_str);

	// print_dictionary_data(dict);

	Parser parser(dict);
	parser.streamin("Hello");

	Lynk* alt_out_one =
	ALINK3(LING,
		ANODE(LING_TYPE, "Wd"),
		ALINK2(WORD_DISJ,
			ANODE(WORD, "LEFT-WALL"),
			ANODE(CONNECTOR, "Wd+")
		),
		ALINK2(WORD_DISJ,
			ANODE(WORD, "Hello"),
			ANODE(CONNECTOR, "Wd-")
		)
	);

	Lynk* alt_out_two =
	ALINK3(LING,
		ANODE(LING_TYPE, "Wi"),
		ALINK2(WORD_DISJ,
			ANODE(WORD, "LEFT-WALL"),
			ANODE(CONNECTOR, "Wi+")
		),
		ALINK2(WORD_DISJ,
			ANODE(WORD, "Hello"),
			ANODE(CONNECTOR, "Wi-")
		)
	);

	Lynk* alt_pair_one =
	ALINK3(STATE_TRIPLE,
		ALINK0(SEQ),
		ALINK0(SEQ),
		ALINK1(SET, alt_out_one)
	);

	Lynk* alt_pair_two =
	ALINK3(STATE_TRIPLE,
		ALINK0(SEQ),
		ALINK0(SEQ),
		ALINK1(SET, alt_out_two)
	);

	if (empty_state)
	{
		// This is the expected set of alternatives: two alternatives,
		// each with an empty state, and one of the two outputs, above.
		Lynk* ans = ALINK2(SET, alt_pair_one, alt_pair_two);

		Lynk* output = parser.get_alternatives();
		if (not (ans->operator==(output)))
		{
			cout << "Error: test failure on test \"" << id <<"\"" << endl;
			cout << "=== Expecting:\n" << ans << endl;
			cout << "=== Got:\n" << output << endl;
			return false;
		}
	}
	else
	{
		// The final state here might not be empty.  However, both
		// of the alternatives should show up somwhere in the output.

		bool found_one = false;
		bool found_two = false;
		Lynk* alts = parser.get_alternatives();
		foreach_outgoing(StateTriple*, sp, alts)
		{
			// At least one alternative should have an empty state.
			if (alt_pair_one->operator==(sp))
				found_one = true;

			if (alt_pair_two->operator==(sp))
				found_two = true;
		}

		// Both should have been found, somewhere.
		if (not alt_pair_one or not alt_pair_two)
		{
			cout << "Error: test failure on test \"" << id << "\"" << endl;
			cout << "=== Expecting this alt:\n" << alt_pair_one << endl;
			cout << "=== Expecting this alt:\n" << alt_pair_two << endl;
			cout << "=== Got:\n" << alts << endl;
			return false;
		}
	}

	cout<<"PASS: test_alternative(" << id << ") " << endl;
	return true;
}

bool test_two_alts()
{
	return test_alternative("two alternatives",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"Hello: Wd- or Wi-;",
		true
	);
}

bool test_two_opts()
{
	return test_alternative("two alts plus opts",
		"LEFT-WALL: (Wd+ or Wi+ or Wq+) & {A+};"
		"Hello: Wd- or Wi- or (Xj- & {A+ or B+});",
		false
	);
}

bool test_two_one_opts()
{
	return test_alternative("two alt, or one opt",
		"LEFT-WALL: (Wd+ or Wi+ or Wq+) & {A+};"
		"Hello: Wd- or {Wi-} or (Xj- & {A+ or B+});",
		false
	);
}

bool test_two_all_opts()
{
	return test_alternative("two alts, or all opt",
		"LEFT-WALL: (Wd+ or Wi+ or Wq+) & {A+};"
		"Hello: {Wd-} or {Wi-} or (Xj- & {A+ or B+});",
		false
	);
}

bool test_two_and_opts()
{
	return test_alternative("two alts, and an opt",
		"LEFT-WALL: (Wd+ or Wi+ or Wq+) & {A+};"
		"Hello: Wd- or (Wi- & {Xj- & {A+ or B+}} & {C+});",
		false
	);
}

bool test_two_and_no_opts()
{
	return test_alternative("two alt, and all opt",
		"LEFT-WALL: (Wd+ or Wi+ or Wq+) & {A+};"
		"Hello: Wd- or ({Wi-} & {Xj- & {A+ or B+}} & {C+});",
		false
	);
}

bool test_two_and_excess()
{
	return test_alternative("two alt, and excess reqs",
		"LEFT-WALL: (Wd+ or Wi+ or Wq+) & {A+};"
		"Hello: Wd- or (Wi- & Xj- & {A+ or B+} & {C+}) or Wi-;",
		false
	);
}

int ntest_two()
{
	size_t num_failures = 0;

	if (!test_two_alts()) num_failures++;
	if (!test_two_opts()) num_failures++;
	if (!test_two_one_opts()) num_failures++;
	if (!test_two_all_opts()) num_failures++;
	if (!test_two_and_opts()) num_failures++;
	if (!test_two_and_no_opts()) num_failures++;
	if (!test_two_and_excess()) num_failures++;

	return num_failures;
}

// ==================================================================

bool test_simple_state(const char *id, const char *dict_str)
{
	total_tests++;

	Dictionary dict = dictionary_create_from_utf8(dict_str);

	// print_dictionary_data(dict);

	Parser parser(dict);
	// Expecting more words to follow, so a non-trivial state.
	parser.streamin("this");

	Lynk* ans_state =
	ALINK2(SEQ,
		ALINK2(WORD_CSET,
			ANODE(WORD, "this"),
			ANODE(CONNECTOR, "Ss*b+")
		),
		ALINK2(WORD_CSET,
			ANODE(WORD, "LEFT-WALL"),
			ALINK3(OR,
				ANODE(CONNECTOR, "Wd+"),
				ANODE(CONNECTOR, "Wi+"),
				ANODE(CONNECTOR, "Wq+")
			)
		)
	);

	Lynk* ans =
	ALINK1(SET,
		ALINK3(STATE_TRIPLE,
			ALINK0(SEQ),
			ans_state,
			ALINK0(SET)
		)
	);

	Lynk* state = parser.get_alternatives();
	if (not (ans->operator==(state)))
	{
		cout << "Error: test failure on test " << id << endl;
		cout << "=== Expecting state:\n" << ans << endl;
		cout << "=== Got state:\n" << state << endl;
		return false;
	}

	cout<<"PASS: test_simple_state(" << id << ") " << endl;
	return true;
}

bool test_first_state()
{
	return test_simple_state("first state",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"this: Ss*b+;"
	);
}

bool test_first_opt_lefty()
{
	return test_simple_state("first state, left-going optional",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"this: Ss*b+ and {Xi-};"
	);
}

bool test_first_or_lefty()
{
	return test_simple_state("first state, OR left-going",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"this: Ss*b+ or Xi-;"
	);
}

bool test_first_or_multi_lefty()
{
	return test_simple_state("first state, multi-OR left-going",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"this: Ss*b+ or Xi- or Y- or Z-;"
	);
}

bool test_first_opt_cpx()
{
	return test_simple_state("first state, complex left-going optional",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"this: Ss*b+ and {Xi- or P- or {Q- & Z+}};"
	);
}

bool test_first_infer_opt()
{
	return test_simple_state("first state, complex infer optional",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"this: Ss*b+ and (Xi- or P- or {Q- & Z+});"
	);
}

int ntest_first()
{
	size_t num_failures = 0;

	if (!test_first_state()) num_failures++;
	if (!test_first_opt_lefty()) num_failures++;
	if (!test_first_or_lefty()) num_failures++;
	if (!test_first_or_multi_lefty()) num_failures++;
	if (!test_first_opt_cpx()) num_failures++;
	if (!test_first_infer_opt()) num_failures++;

	return num_failures;
}

// ==================================================================

bool test_short_sent(const char *id, const char *dict_str, bool empty_state)
{
	total_tests++;

	Dictionary dict = dictionary_create_from_utf8(dict_str);
	// print_dictionary_data(dict);

	Parser parser(dict);

	// Expecting more words to follow, so a non-trivial state.
	// In particular, the dictionary will link the left-wall to
	// "is", so "this" has to be pushed on stack until the "is"
	// shows up.  The test_seq_sent() below will link the other
	// way around.
	parser.streamin("this is");

	Lynk* alts = parser.get_alternatives();

	// At least one result should be this state pair.
	Lynk* sp =
		ALINK3(STATE_TRIPLE,
			ALINK0(SEQ),  // empty input
			ALINK0(SEQ),  // empty state
			ALINK2(SET,
				ALINK3(LING,
					ANODE(LING_TYPE, "Ss*b"),
					ALINK2(WORD_DISJ,
						ANODE(WORD, "this"),
						ANODE(CONNECTOR, "Ss*b+")),
					ALINK2(WORD_DISJ,
						ANODE(WORD, "is.v"),
						ANODE(CONNECTOR, "Ss-")))
				,
				ALINK3(LING,
					ANODE(LING_TYPE, "Wi"),
					ALINK2(WORD_DISJ,
						ANODE(WORD, "LEFT-WALL"),
						ANODE(CONNECTOR, "Wi+")),
					ALINK2(WORD_DISJ,
						ANODE(WORD, "is.v"),
						ANODE(CONNECTOR, "Wi-")))
			));

	if (empty_state)
	{
		Lynk* ans = ALINK1(SET, sp);
		if (not (ans->operator==(alts)))
		{
			cout << "Error: test failure on test \"" << id <<"\"" << endl;
			cout << "=== Expecting:\n" << ans << endl;
			cout << "=== Got:\n" << alts << endl;
			return false;
		}
	}
	else
	{
		// At least one alternative should be the desired state pair.
		bool found = false;
		foreach_outgoing(Atom*, a, alts)
		{
			if (sp->operator==(a))
				found = true;
		}
		if (not found)
		{
			cout << "Error: test failure on test \"" << id <<"\"" << endl;
			cout << "=== Expecting one of them to be:\n" << sp << endl;
			cout << "=== Got:\n" << alts << endl;
			return false;
		}
	}

	cout<<"PASS: test_short_sent(" << id << ") " << endl;
	return true;
}

bool test_short_this()
{
	return test_short_sent("short sent",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"this: Ss*b+;"
		"is.v: Ss- and Wi-;",
		true
	);
}

bool test_short_this_opt()
{
	return test_short_sent("short sent opt",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"this: Ss*b+;"
		"is.v: Ss- and Wi- and {O+};",
		false
	);
}

bool test_short_this_obj_opt()
{
	return test_short_sent("short sent with obj",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"this: Ss*b+ or Os-;"
		"is.v: Ss- and Wi- and {O+};",
		false
	);
}

bool test_short_this_costly()
{
	return test_short_sent("short sent with costly null",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"this: Ss*b+ or [[[()]]];"
		"is.v: Ss- and Wi-;",
		true
	);
}

bool test_short_this_complex()
{
	return test_short_sent("short sent complex",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		""
		"<CLAUSE>: {({@COd-} & (C-)) or ({@CO-} & (Wd- & {CC+})) or [Rn-]};"
		"<noun-main-h>:"
		"  (Jd- & Dmu- & Os-)"
		"  or (Jd- & Dmu- & {Wd-} & Ss*b+)"
		"  or (Ss*b+ & <CLAUSE>) or SIs*b- or [[Js-]] or [Os-];"
		""
		"this:"
		"  <noun-main-h>;"
		""
		"is.v: Ss- and Wi- and {O+};",
		false
	);
}

bool test_short_this_noun_dict()
{
	return test_short_sent("short sent realistic dict entry for noun",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"<costly-null>: [[[()]]];"
		""
		"<post-nominal-x>:"
		"  ({[B*j+]} & Xd- & (Xc+ or <costly-null>) & MX-);"
		""
		"<clause-conjoin>: RJrc- or RJlc+;"
		""
		"<CLAUSE>: {({@COd-} & (C- or <clause-conjoin>)) or ({@CO-} & (Wd- & {CC+})) or [Rn-]};"
		""
		"<noun-main-h>:"
		"  (Jd- & Dmu- & Os-)"
		"  or (Jd- & Dmu- & {Wd-} & Ss*b+)"
		"  or (Ss*b+ & <CLAUSE>) or SIs*b- or [[Js-]] or [Os-]"
		"  or <post-nominal-x>"
		"  or <costly-null>;"
		""
		"this:"
		"  <noun-main-h>;"
		""
		"is.v: Ss- and Wi- and {O+};",
		false
	);
}

int ntest_short()
{
	size_t num_failures = 0;

	if (!test_short_this()) num_failures++;
	if (!test_short_this_opt()) num_failures++;
	if (!test_short_this_obj_opt()) num_failures++;
	if (!test_short_this_costly()) num_failures++;
	if (!test_short_this_complex()) num_failures++;
	if (!test_short_this_noun_dict()) num_failures++;

	return num_failures;
}

// ==================================================================

bool test_seq_sent(const char *id, const char *dict_str, bool empty_state)
{
	total_tests++;

	Dictionary dict = dictionary_create_from_utf8(dict_str);

	// print_dictionary_data(dict);

	Parser parser(dict);
	// Expecting more words to follow, so a non-trivial state.
	// Unlike test_short_sent() above, here, we link "this" to
	// the left wall, followed by "is" to for a sequence.
	parser.streamin("this is");

	Lynk* alts = parser.get_alternatives();

	// At least one result should be this state pair.
	Lynk* sp =
		ALINK3(STATE_TRIPLE,
			ALINK0(SEQ),  // empty input
			ALINK0(SEQ),  // empty state
			ALINK2(SET,
				ALINK3(LING,
					ANODE(LING_TYPE, "Wd"),
					ALINK2(WORD_DISJ,
						ANODE(WORD, "LEFT-WALL"),
						ANODE(CONNECTOR, "Wd+")),
					ALINK2(WORD_DISJ,
						ANODE(WORD, "this"),
						ANODE(CONNECTOR, "Wd-"))),
				ALINK3(LING,
					ANODE(LING_TYPE, "Ss*b"),
					ALINK2(WORD_DISJ,
						ANODE(WORD, "this"),
						ANODE(CONNECTOR, "Ss*b+")),
					ALINK2(WORD_DISJ,
						ANODE(WORD, "is.v"),
						ANODE(CONNECTOR, "Ss-")))));

	if (empty_state)
	{
		Lynk* ans = ALINK1(SET, sp);
		if (not (ans->operator==(alts)))
		{
			cout << "Error: test failure on test \"" << id <<"\"" << endl;
			cout << "=== Expecting:\n" << ans << endl;
			cout << "=== Got:\n" << alts << endl;
			return false;
		}
	}
	else
	{
		// At least one alternative should be the desired state pair.
		bool found = false;
		foreach_outgoing(Atom*, a, alts)
		{
			if (sp->operator==(a))
				found = true;
		}
		if (not found)
		{
			cout << "Error: test failure on test \"" << id <<"\"" << endl;
			cout << "=== Expecting one of them to be:\n" << sp << endl;
			cout << "=== Got:\n" << alts << endl;
			return false;
		}
	}

	cout<<"PASS: test_short_sent(" << id << ") " << endl;
	return true;
}

bool test_seq_this()
{
	return test_seq_sent("short seq sent",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"this: Wd- and Ss*b+;"
		"is.v: Ss-;",
		true
	);
}

bool test_seq_this_opt()
{
	return test_seq_sent("short seq sent opt",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"this: Wd- and Ss*b+;"
		"is.v: Ss- and {O+};",
		false
	);
}

bool test_seq_this_obj_opt()
{
	return test_seq_sent("short seq sent with obj",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"this: Wd- and (Ss*b+ or Os-);"
		"is.v: Ss- and {O+};",
		false
	);
}

bool test_seq_this_costly()
{
	return test_seq_sent("short seq sent with costly null",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"this: Wd- and (Ss*b+ or [[[()]]]);"
		"is.v: Ss-;",
		true
	);
}

bool test_seq_this_complex()
{
	return test_seq_sent("short seq sent complex",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		""
		"<CLAUSE>: {({@COd-} & (C-)) or ({@CO-} & (Wd- & {CC+})) or [Rn-]};"
		"<noun-main-h>:"
		"  (Jd- & Dmu- & Os-)"
		"  or (Jd- & Dmu- & {Wd-} & Ss*b+)"
		"  or (Ss*b+ & <CLAUSE>) or SIs*b- or [[Js-]] or [Os-];"
		""
		"this:"
		"  <noun-main-h>;"
		""
		"is.v: Ss- and {O+};",
		false
	);
}

bool test_seq_this_noun_dict()
{
	return test_seq_sent("short seq sent realistic dict entry for noun",
		"LEFT-WALL: Wd+ or Wi+ or Wq+;"
		"<costly-null>: [[[()]]];"
		""
		"<post-nominal-x>:"
		"  ({[B*j+]} & Xd- & (Xc+ or <costly-null>) & MX-);"
		""
		"<clause-conjoin>: RJrc- or RJlc+;"
		""
		"<CLAUSE>: {({@COd-} & (C- or <clause-conjoin>)) or ({@CO-} & (Wd- & {CC+})) or [Rn-]};"
		""
		"<noun-main-h>:"
		"  (Jd- & Dmu- & Os-)"
		"  or (Jd- & Dmu- & {Wd-} & Ss*b+)"
		"  or (Ss*b+ & <CLAUSE>) or SIs*b- or [[Js-]] or [Os-]"
		"  or <post-nominal-x>"
		"  or <costly-null>;"
		""
		"this:"
		"  <noun-main-h>;"
		""
		"is.v: Ss- and {O+};",
		false
	);
}


bool test_seq_this_verb_dict()
{
	return test_seq_sent("short seq sent realistic dict entry for verb",
		"LEFT-WALL:"
		"  (Wd+ or Wq+ or Ws+ or Wj+ or Wc+ or Wi+ or We+ or Qd+)"
		"    & {CP+} & {Xx+} & {RW+ or Xp+};"
		""
		"<costly-null>: [[[()]]];"
		""
		"<post-nominal-x>:"
		"  ({[B*j+]} & Xd- & (Xc+ or <costly-null>) & MX-);"
		""
		"<clause-conjoin>: RJrc- or RJlc+;"
		""
		"<CLAUSE>: {({@COd-} & (C- or <clause-conjoin>)) or ({@CO-} & (Wd- & {CC+})) or [Rn-]};"
		""
		"<noun-main-h>:"
		"  (Jd- & Dmu- & Os-)"
		"  or (Jd- & Dmu- & {Wd-} & Ss*b+)"
		"  or (Ss*b+ & <CLAUSE>) or SIs*b- or [[Js-]] or [Os-]"
		"  or <post-nominal-x>"
		"  or <costly-null>;"
		""
		"this:"
		"  <noun-main-h>;"
		""
		"<verb-and-s->: {@E-} & VJrs-;"
		"<verb-and-s+>: {@E-} & VJls+;"
		""
		"<verb-x-s,u>: {@E-} & (Ss- or SFs- or SFu- or (RS- & Bs-));"
		""
		"<vc-be-obj>:"
		"  {@EBm+} & O*t+ & {@MV+};"
		""
		"<vc-be-no-obj>:"
		"  ({@EBm+} & ((([B**t-] or [K+] or BI+ or OF+ or PF- or"
		"      (Osi+ & R+ & Bs+) or"
		"      (Opi+ & R+ & Bp+) or"
		"      [[()]]) & {@MV+}) or"
		"    (Pp+ & {THi+ or @MV+}) or"
		"    THb+ or"
		"    TO+ or"
		"    Pa+)) or"
		"  ({N+} & (AF- or Pv+ or I*v+)) or"
		"  (({N+} or {Pp+}) & Pg*b+);"
		""
		"<vc-be>: <vc-be-no-obj> or <vc-be-obj>;"
		""
		"is.v:"
		"  (<verb-x-s,u> & <vc-be>) or"
		"  (<verb-and-s-> & <vc-be>) or (<vc-be> & <verb-and-s+>) or"
		"  (((Rw- or ({Ic-} & Q-) or [()]) & (SIs+ or SFIs+)) & <vc-be>);"
		"",
		false
	);
}

int ntest_short_seq()
{
	size_t num_failures = 0;

	if (!test_seq_this()) num_failures++;
	if (!test_seq_this_opt()) num_failures++;
	if (!test_seq_this_obj_opt()) num_failures++;
	if (!test_seq_this_costly()) num_failures++;
	if (!test_seq_this_complex()) num_failures++;
	if (!test_seq_this_noun_dict()) num_failures++;
	if (!test_seq_this_verb_dict()) num_failures++;

	return num_failures;
}

// ==================================================================

bool test_state_sent(const char *id, const char *dict_str)
{
	total_tests++;

	Dictionary dict = dictionary_create_from_utf8(dict_str);

	// print_dictionary_data(dict);

	Parser parser(dict);
	// Expecting more words to follow, so a non-trivial state.
	parser.streamin("this is a test");

	Lynk* alts = parser.get_alternatives();

	// We expect no output, and a crazy state:
	// The provided dictionary will not allow a linkage to happen;
	// this is really just testing the push of stack state.
	Lynk* sp =
		ALINK3(STATE_TRIPLE,
			ALINK0(SEQ),      // empyt input
			ALINK5(SEQ,
				ALINK2(WORD_CSET,
					ANODE(WORD, "test.n"),
					ALINK2(OR,
						ANODE(CONNECTOR, "XXXGIVEN+"),
						ANODE(CONNECTOR, "AN+")))
				,
				ALINK2(WORD_CSET,
					ANODE(WORD, "a"),
					ANODE(CONNECTOR, "Ds+"))
				,
				ALINK2(WORD_CSET,
					ANODE(WORD, "is.v"),
					ANODE(CONNECTOR, "SIs+"))
				,
				ALINK2(WORD_CSET,
					ANODE(WORD, "this.J2"),
					ANODE(CONNECTOR, "JDBKQ+"))
				,
				ALINK2(WORD_CSET,
					ANODE(WORD, "LEFT-WALL"),
					ANODE(CONNECTOR, "Wq+"))
			),
			ALINK0(SET));  // empty output

	Lynk* ans = ALINK1(SET, sp);
	if (not (ans->operator==(alts)))
	{
		cout << "Error: test failure on test \"" << id <<"\"" << endl;
		cout << "=== Expecting:\n" << ans << endl;
		cout << "=== Got:\n" << alts << endl;
		return false;
	}
	cout<<"PASS: test_state_sent(" << id << ") " << endl;
	return true;
}

bool test_state_order()
{
	return test_state_sent("short state sent",
		"LEFT-WALL: Wq+;"
		"this.J2: JDBKQ+;"
		"is.v: SIs+;"
		"a: Ds+;"
		"test.n: XXXGIVEN+ or AN+;"
	);
}

bool test_state_order_left()
{
	return test_state_sent("short state sent leftwards",
		"LEFT-WALL: Wq+;"
		"this.J2: JDBKQ+ or JAAA-;"
		"is.v: SIs+ or KBB-;"
		"a: Ds+ & {Junk-} ;"
		"test.n: XXXGIVEN+ or BOGUS- or (AN+ & {GLOP-});"
	);
}

int ntest_short_state()
{
	size_t num_failures = 0;

	if (!test_state_order()) num_failures++;
	if (!test_state_order_left()) num_failures++;
	return num_failures;
}

// ==================================================================

bool test_right_wall(const char *id, const char *dict_str, bool empty_state)
{
	total_tests++;

	Dictionary dict = dictionary_create_from_utf8(dict_str);

	// print_dictionary_data(dict);
	Parser parser(dict);
	// Expecting more words to follow, so a non-trivial state.
	parser.streamin("this is .");

	Lynk* alts = parser.get_alternatives();

	// We expect empty final state.
	Lynk* sp =
		ALINK3(STATE_TRIPLE,
			ALINK0(SEQ),  // empty input
			ALINK0(SEQ),  // empty state
			ALINK3(SET,
				ALINK3(LING,
					ANODE(LING_TYPE, "Wd"),
					ALINK2(WORD_DISJ,
						ANODE(WORD, "LEFT-WALL"),
						ANODE(CONNECTOR, "Wd+")),
					ALINK2(WORD_DISJ,
						ANODE(WORD, "this"),
						ANODE(CONNECTOR, "Wd-")))
				,
				ALINK3(LING,
					ANODE(LING_TYPE, "Ss*b"),
					ALINK2(WORD_DISJ,
						ANODE(WORD, "this"),
						ANODE(CONNECTOR, "Ss*b+")),
					ALINK2(WORD_DISJ,
						ANODE(WORD, "is.v"),
						ANODE(CONNECTOR, "Ss-")))
				,
				ALINK3(LING,
					ANODE(LING_TYPE, "Xp"),
					ALINK2(WORD_DISJ,
						ANODE(WORD, "LEFT-WALL"),
						ANODE(CONNECTOR, "Xp+")),
					ALINK2(WORD_DISJ,
						ANODE(WORD, "."),
						ANODE(CONNECTOR, "Xp-")))
			));


	Lynk* ans = ALINK1(SET, sp);
	if (not (ans->operator==(alts)))
	{
		cout << "Error: test failure on test \"" << id <<"\"" << endl;
		cout << "=== Expecting:\n" << ans << endl;
		cout << "=== Got:\n" << alts << endl;
		return false;
	}
	cout<<"PASS: test_right_wall(" << id << ") " << endl;
	return true;
}

bool test_period()
{
	return test_right_wall("period",
		"LEFT-WALL: (Wd+ or Wi+ or Wq+) & {Xp+};"
		"this: Wd- and Ss*b+;"
		"is.v: Ss-;"
		"\".\": Xp-;",
		false
	);
}

int ntest_right_wall()
{
	size_t num_failures = 0;

	if (!test_period()) num_failures++;
	return num_failures;
}

// ==================================================================

int
main(int argc, char *argv[])
{
	size_t num_failures = 0;
	bool exit_on_fail = true;

	num_failures += ntest_simple();
	report(num_failures, exit_on_fail);

	num_failures += ntest_two();
	report(num_failures, exit_on_fail);

	num_failures += ntest_first();
	report(num_failures, exit_on_fail);

	num_failures += ntest_short();
	report(num_failures, exit_on_fail);

	num_failures += ntest_short_seq();
	report(num_failures, exit_on_fail);

	num_failures += ntest_short_state();
	report(num_failures, exit_on_fail);

	num_failures += ntest_right_wall();
	report(num_failures, exit_on_fail);

	exit (0);
}

