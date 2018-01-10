--
-- demo.sql
--
-- Inserts some word and disjunct data into the database.
-- This is for demonstration purposed only: it inserts just enough to
-- parse the sentences "this is a test" "this is a dog" "this is
-- a cat", "that is a test", "that is another dog", etc.  The disjuncts
-- are a small subset of the correct ones for English.
--
-- To create a new database, simply say:
--    cat dict.sql | sqlite3 dict.db
-- To populate it with the demo data:
--    cat demo.sql | sqlite3 dict.db
--

-- These first few lines identify the dictionary version number,
-- and the locale that applies. The are optional but strongly
-- recommended.  The locale is used, among other things, to identify
-- capitalized words during tokenization.
--
INSERT INTO Morphemes VALUES ('<dictionary-version-number>', '<dictionary-version-number>', '<dictionary-version-number>');
INSERT INTO Disjuncts VALUES ('<dictionary-version-number>', 'V5v4v0+', 0.0);

INSERT INTO Morphemes VALUES ('<dictionary-locale>', '<dictionary-locale>', '<dictionary-locale>');
INSERT INTO Disjuncts VALUES ('<dictionary-locale>', 'EN4us+', 0.0);

-- The UNKNOWN-WORD device is needed in order to allow the wild-card
-- query of dictionary contents to work. That is, the user can use the
-- command-line client to type in `!!blah*` and this will search the
-- dictionary for all words whos first four letters are `blah`. You
-- are free to replace `XXXBOGUS+` by something useful, for example,
-- If you use `Ds- & Os-`, than any word will be accepted as the object!
INSERT INTO Morphemes VALUES ('UNKNOWN-WORD', 'UNKNOWN-WORD', 'UNKNOWN-WORD');
INSERT INTO Disjuncts VALUES ('UNKNOWN-WORD', 'XXXBOGUS+', 0.0);
-- INSERT INTO Disjuncts VALUES ('UNKNOWN-WORD', 'Ds- & Os-', 0.0);

-- The following should look familier, if you already understand
-- link-grammar basics.
INSERT INTO Morphemes VALUES ('LEFT-WALL', 'LEFT-WALL', 'LEFT-WALL');
INSERT INTO Disjuncts VALUES ('LEFT-WALL', 'Wd+ & WV+', 0.0);

INSERT INTO Morphemes VALUES ('this', 'this.p', 'this');
INSERT INTO Disjuncts VALUES ('this', 'Wd- & Ss*b+', 0.1);

INSERT INTO Morphemes VALUES ('is', 'is.v', 'is');
INSERT INTO Disjuncts VALUES ('is', 'Ss- & WV- & O*m+', 0.214159265358979);

INSERT INTO Morphemes VALUES ('a', 'a', '(article)');
INSERT INTO Morphemes VALUES ('another', 'another', '(article)');
INSERT INTO Disjuncts VALUES ('(article)', 'Ds+', 0.1);

INSERT INTO Morphemes VALUES ('test', 'test.n', '(noun)');
INSERT INTO Morphemes VALUES ('dog', 'dog.n', '(noun)');
INSERT INTO Morphemes VALUES ('cat', 'cat.n', '(noun)');
INSERT INTO Disjuncts VALUES ('(noun)', 'Ds- & Os-', 0.0);
