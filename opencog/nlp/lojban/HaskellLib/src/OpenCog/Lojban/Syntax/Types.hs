{-# LANGUAGE RankNTypes                 #-}
{-# LANGUAGE FlexibleContexts           #-}
{-# LANGUAGE DeriveFunctor              #-}
{-# LANGUAGE DeriveFoldable             #-}
module OpenCog.Lojban.Syntax.Types where

import Prelude hiding (id,(.),(<*>),(<$>),pure,(*>),(<*),foldl)


import Data.List (partition,isPrefixOf,isSuffixOf,nub,any,intercalate)
import Data.Maybe (fromJust)
import qualified Data.Map as M

import Iso
import qualified Syntax

import Lojban hiding (State(..))

import OpenCog.AtomSpace
import OpenCog.Lojban.Util

import Control.Monad.RWS

import qualified Data.ListTrie.Patricia.Set.Ord as TS

type StringSet = TS.TrieSet Char

--The firs element of the tuple is a Atom that is part of the main Sentence/Link
--The List are other atoms which have to be added to the Atomspace or are needed for printing
type Tag = String

--Sumtis are tagged atoms where the tag descripes their place in the predicate
type Sumti = Tagged Atom

--Selbri are Atoms with a TruthVal they can also be tagged with a strength modifier
type Selbri = (TruthVal,Atom)
type SelbriNA = (Maybe NA,Selbri)

type NA = (String,[UI])

--Attitude
type UI = (Atom,TruthVal)

type Tagged a = (a,Maybe Tag)

--A Bridi consits of a some sumti/arguments in the front
--Then the Selbri/Predicate
--finally some more sumti/arguments
type Bridi = ([Sumti],BTCT)

--The State
--sFlags : A list of flags then can be used to pass information along
--sAtoms : A list of Atoms that will be generated by parsing
--sText  : The actuall text to be parsed
type Flag = String
data State = State { sFlags :: M.Map String String
                   , sAtoms :: [Atom]
                   , sText  :: String
                   , sSeed  :: Int
                   , sNow   :: Atom
                   , sCtx   :: [Atom]
                   , sJAI   :: Maybe JJCTTS
                -- , sPro   :: M.Map String String
                   , sXU    :: [Atom]
                   } deriving Show

--They Iso we are using
--We use a RWST monad over (Either String) for failurs
--currently the writer part is unused
type SynIso a b = Syntax.SynIso (RWST (WordList State) () State) a b
type Syntax a   = SynIso () a

--The parser Requirse that the State is a SyntaxState
instance Syntax.SyntaxState State where
    getText = sText
    addText str sta = sta {sText = str ++ sText sta}
    setText str sta = sta {sText = str}

-------------------------------------------------------------------------------
--Connective Types
-------------------------------------------------------------------------------

data ConnectorTree c e = CTNode c (ConnectorTree c e,ConnectorTree c e)
                       | CTLeaf e
                       deriving (Show,Eq,Functor,Foldable)

type JJCTTS = ConnectorTree JOIK_JEK (Tagged SelbriNA)

type BT = (SelbriNA,[Sumti])

type BTCT = ConnectorTree Con BT

--Con possible contains a Logical Connective and one based on a predicate
type Con = (Maybe JOIK_EK,Maybe JJCTTS)

type EK = (Bool,(Bool,(String,Bool)))

data JOIK = JOI (Bool,(String,Bool))
          | INT (Bool,(String,Bool))
          | INTGAhO (String,((Bool,(String,Bool)),String))
          deriving (Show,Eq)

type JOIK_JEK = Either JOIK EK
type JOIK_EK = Either JOIK EK
