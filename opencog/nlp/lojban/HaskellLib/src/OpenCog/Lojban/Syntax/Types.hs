{-# LANGUAGE RankNTypes                 #-}
{-# LANGUAGE TemplateHaskell            #-}
module OpenCog.Lojban.Syntax.Types where

import Prelude hiding (id,(.),(<*>),(<$>),pure,(*>),(<*),foldl)

import qualified Data.Map as M

import Data.List (partition,isPrefixOf,isSuffixOf,nub,any,intercalate)
import Data.Maybe (fromJust)

import Control.Category (id,(.))
import Control.Isomorphism.Partial
import Control.Isomorphism.Partial.Derived
import Control.Isomorphism.Partial.Unsafe
import Control.Isomorphism.Partial.TH
import Text.Syntax

import OpenCog.AtomSpace
import OpenCog.Lojban.Util

import Control.Monad.Trans.Reader

import qualified Data.ListTrie.Patricia.Set.Ord as TS

type StringSet = TS.TrieSet Char

--The firs element of the tuple is a Atom that is part of the main Sentence/Link
--The List are other atoms which have to be added to the Atomspace or are needed for printing
type State a = (a,[Atom])
type Tag = String
type Sumti = Tagged Atom
type Selbri = (TruthVal,Atom) --STring represents TV
type Tagged a = (a,Maybe Tag)

type LCON = (Maybe String,(String,Maybe String))
type Con = (Maybe LCON,Maybe (Tagged Selbri))
type Bridi = ([Sumti],((Maybe Atom,(Maybe String,Tagged Selbri)),[Sumti]))

type WordList = (M.Map String StringSet,StringSet,Iso String String,Int)
type SyntaxReader a = forall delta. Syntax delta => ReaderT WordList delta a

instance IsoFunctor f => IsoFunctor (ReaderT a f) where
    iso <$> r
        = ReaderT (\e -> iso <$> runReaderT r e)

instance ProductFunctor f => ProductFunctor (ReaderT a f) where
    a <*> b
        = ReaderT (\e -> runReaderT a e <*> runReaderT b e)

instance Alternative f => Alternative (ReaderT a f) where
    a <|> b
        = ReaderT (\e -> runReaderT a e <|> runReaderT b e)
    a <||> b
        = ReaderT (\e -> runReaderT a e <||> runReaderT b e)
    empty = ReaderT (const empty)

instance Syntax f => Syntax (ReaderT a f) where
    pure x = ReaderT (\e -> pure x)
    token  = ReaderT (const token)
    withText r = ReaderT (withText . runReaderT r)
    ptp r1 iso r2 = ReaderT (\e -> ptp (runReaderT r1 e) iso (runReaderT r2 e))
    withOut r1 r2 = ReaderT (\e -> withOut (runReaderT r1 e) (runReaderT r2 e))

$(defineIsomorphisms ''Atom)
