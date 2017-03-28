{-# LANGUAGE LambdaCase                 #-}
module OpenCog.Lojban
( WordList
, initParserPrinter
, lojbanToAtomese
, atomeseToLojban
, loadWordLists
) where

import OpenCog.Lojban.Syntax
import OpenCog.Lojban.Util
import OpenCog.Lojban.WordList
import OpenCog.Lojban.Syntax.Types

import OpenCog.AtomSpace

import Control.Monad.IO.Class
import Control.Monad.RWS
import Control.Exception
import System.Random
import Data.Char (chr)
import Data.Maybe
import qualified Data.Map as M

import Control.Monad.RWS
import Control.Monad.Trans.Class

import Iso hiding (Syntax,SynIso)

initParserPrinter :: String -> String
                  -> IO (String -> Either String Atom, Atom -> Either String String)
initParserPrinter cmavoSrc gismuSrc = do
    wordlist <- loadWordLists cmavoSrc gismuSrc
    return (lojbanToAtomese wordlist,atomeseToLojban wordlist)

lojbanToAtomese :: WordList -> String -> Either String Atom
lojbanToAtomese rstate text = wrapAtom . fst <$> evalRWST (apply lojban ()) rstate state
    where state = State {sFlags = [],sAtoms = [],sText = text++" "}

wrapAtom :: Atom -> Atom
wrapAtom atom@(Link "SatisfactionLink" _ _) = cLL [cAN "QuestionAnchor" , atom]
wrapAtom atom@(Link "PutLink" _ _)          = cLL [cAN "QuestionAnchor" , atom]
wrapAtom atom                               = cLL [cAN "StatementAnchor", atom]

atomeseToLojban :: WordList -> Atom -> Either String String
atomeseToLojban rstate a@(LL [_an,s]) = sText . fst <$> execRWST (unapply lojban s) rstate state
    where state = State {sFlags = [],sAtoms = [],sText = ""}

{-tvToLojban :: TruthVal -> String
tvToLojban tv
    | tvMean tv > 0.5 = "go'i"
    | tvMean tv <= 0.5 = "nago'i"-}


{-EquivalenceLink
    EvaluationLink
        VariableNode "var1!!!"
        ListLink
            VariableNode "var2"
            ConceptNode "vo'a"
    EvaluationLink
        PredicateNode "sumti1"
        ListLink "var2"
            VariableNode "var2"
            ConceptNode "something"
-}
