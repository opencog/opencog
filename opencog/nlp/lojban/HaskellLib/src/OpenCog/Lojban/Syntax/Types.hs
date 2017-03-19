{-# LANGUAGE RankNTypes                 #-}
{-# LANGUAGE FlexibleContexts           #-}
module OpenCog.Lojban.Syntax.Types where

import Prelude hiding (id,(.),(<*>),(<$>),pure,(*>),(<*),foldl)


import Data.List (partition,isPrefixOf,isSuffixOf,nub,any,intercalate)
import Data.Maybe (fromJust)
import qualified Data.Map as M

import qualified Iso

import OpenCog.AtomSpace
import OpenCog.Lojban.Util

import Control.Monad.RWS

import qualified Data.ListTrie.Patricia.Set.Ord as TS

type StringSet = TS.TrieSet Char

--The firs element of the tuple is a Atom that is part of the main Sentence/Link
--The List are other atoms which have to be added to the Atomspace or are needed for printing
type Tag = String
type Sumti = Tagged Atom
type Selbri = (TruthVal,Atom) --STring represents TV
type Tagged a = (a,Maybe Tag)

type LCON = (Maybe String,(String,Maybe String))
type Con = (Maybe LCON,Maybe (Tagged Selbri))
type Bridi = ([Sumti],((Maybe Atom,(Maybe String,Tagged Selbri)),[Sumti]))

data WordList = WordList { cmavos :: M.Map String StringSet
                         , gismus :: StringSet
                         , bai    :: SynIso String String
                         , seed   :: Int
                         }

data State = State { sFlags :: [String]
                   , sAtoms :: [Atom]
                   , sText :: String
                   } deriving Show

type SynIso a b = Iso.SynIso (RWST WordList () State) a b
type Syntax a   = SynIso () a

instance Iso.SyntaxState State where
    getText = sText
    addText str sta = sta {sText = str ++ (sText sta)}
    setText str sta = sta {sText = str}
