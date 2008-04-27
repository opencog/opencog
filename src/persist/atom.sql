
--
-- Create a table representation for an opencog Atom
--

CREATE TABLE Atoms (
	-- the uuid maps to the atom handle
	uuid	INT PRIMARY KEY,

	type  INT,

	-- maps to TruthValue ID
	-- tvid INT, -- not used, just inline the truth value

	-- Inlined truth values
	stv_mean FLOAT,
	stv_count FLOAT

	-- maps to AttentionValue
	attention FLOAT,

   -- The node name
	name    TEXT
);

--
-- Simple truth values
-- This would store truth values out-of-line with the atom,
-- but this seems very ineffcient, as it wastes index space, 
-- requires extra queries, and so on. 
-- So its commented out below, and left behind as FYI.
--
-- CREATE TABLE SimpleTVs (
-- 	tvid INT PRIMARY KEY,
-- 	mean FLOAT,
-- 	count FLOAT
-- );
-- 
-- 
-- INSERT INTO SimpleTVs VALUES (0, 0.0, 0.0);     -- NULL_TV
-- INSERT INTO SimpleTVs VALUES (1, 0.0, 0.0);     -- DEFAULT_TV
-- INSERT INTO SimpleTVs VALUES (2, 0.0, 10000.0); -- FALSE_TV
-- INSERT INTO SimpleTVs VALUES (3, 1.0, 10000.0); -- TRUE_TV
-- INSERT INTO SimpleTVs VALUES (4, 1.0, 0.0);     -- TRIVIAL_TV
-- 
-- CREATE SEQUENCE tvid_seq START WITH 5;
-- 


-- Table of the edges of the Levi craph corresponding 
-- to the hypergraph. An edge is a (src,dst) pair. The
-- pair, understood as going src->dst, for a fixed src,
-- is the set of outgoing edges of the atom. Understood
-- as dst<-src, with fixed dst, are the incoming edges.
--
-- Outgoing edges are understood to be ordered. "pos" is
-- is the order, starting with 0.
CREATE TABLE Edges (
	src_uuid  INT,
	dst_uuid  INT,
	pos INT
);
