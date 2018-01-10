--
-- dict.sql
--
-- SQLite3 style SQL commands to create the tables holding the various
-- dictionary bits and pieces.  Right now, this is just a very simple
-- pair of tables, one to hold word classes, and another to hold all
-- the disjuncts for that class.
--
-- To create a new database, simply say:
--    cat dict.sql | sqlite3 dict.db
-- To populate it with the demo data:
--    cat demo.sql | sqlite3 dict.db
--

CREATE TABLE Morphemes
(
	-- For English, the 'morpheme' is the 'word'. A given morpheme
	-- may appear multiple times in this table.  This is the field that
	-- the tokenizer uses to determine if a token is in the dictionary.
	morpheme TEXT NOT NULL,

	-- The subscripted form of the above.  The subscripted forms are
	-- always unique for the dictionary. They serve as a debugging tool,
	-- unique identifier for the database.
	subscript TEXT UNIQUE NOT NULL,

	-- The classname is the set that the subscripted 'word' belongs to.
	-- All members of the class share a common set of disjuncts, with
	-- a common set of costs.
	classname TEXT NOT NULL
);

-- We want fast lookup of words.
CREATE INDEX morph_idx ON Morphemes(morpheme);

CREATE TABLE Disjuncts
(
	-- All words/morphemes sharing this classname also share this
	-- disjunct and cost.
	classname TEXT NOT NULL,

	-- The standard Link Grammar disjunct, expressed as an ASCII string.
	-- The disjunct can be composed of the & operator, and the optional
	-- connectors i.e. {} and the multiple connector i.e. @. The and
	-- operator is NOT allowed. This means that the grouping parents () 
	-- must also not appear in the expression.  The cost operators [] are
	-- also not allowed; costs are to be expressed using the cost field.
	--
	-- An example of a valid disjunct:
	--     A+ & B- & {Ca*bc*f+} & @Mpqr-
	--
	-- An example of an INVALID disjunct:
	--     (A+ & B-) & {Ca*bc*f+ or [D-]} & @Mpqr-
	--
	disjunct TEXT NOT NULL,

	-- Cost of using this disjunct.
	cost REAL
);

-- We want fast lookup of classnames.
CREATE INDEX class_idx ON Disjuncts(classname);
