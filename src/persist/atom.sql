
--
-- Create a table representation for an opencog Atom
--

CREATE TABLE OpenCogAtom (
	-- the uuid maps to the atom handle
	uuid	INT PRIMARY KEY,

	type  INT,

	-- maps to TruthValue
	truth FLOAT,

	-- maps to AttentionValue
	attention FLOAT,

   -- The node name
	name    TEXT
);


-- Table of the edges of the Levi craph corresponding 
-- to the hypergraph. An edge is a (src,dst) pair. The
-- pair, understood as going src->dst, for a fixed src,
-- is the set of outgoing edges of the atom. Understood
-- as dst<-src, with fixed dst, are the incoming edges.
--
CREATE TABLE OpenCogEdge (
	src_uuid  INT,
	dst_uuid  INT
);
