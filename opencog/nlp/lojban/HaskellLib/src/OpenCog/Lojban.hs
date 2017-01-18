{-# LANGUAGE LambdaCase                 #-}
module OpenCog.Lojban
( WordList
, initParserPrinter
, lojbanToAtomese
, lojbanToAtomeseRaw
, atomeseToLojban
, loadWordLists
) where

import OpenCog.Lojban.Syntax
import OpenCog.Lojban.Util
import OpenCog.Lojban.WordList
import OpenCog.Lojban.Syntax.Types (WordList)

import OpenCog.AtomSpace

import Control.Monad.IO.Class
import Control.Monad.Trans.Reader
import Control.Exception
import System.Random
import Data.Char (chr)
import Data.Maybe
import qualified Data.Map as M

import Text.Syntax.Parser.Naive
import qualified Text.Syntax.Printer.Naive as P

initParserPrinter :: String -> IO (String -> Maybe Atom, Atom -> Maybe String)
initParserPrinter path = do
    wordlist <- loadWordLists path
    return (lojbanToAtomese wordlist,atomeseToLojban wordlist)

lojbanToAtomese :: WordList -> String -> Maybe Atom
lojbanToAtomese state text =
    wrapAtom <$> listToMaybe (parse (runReaderT lojban state) (text++" "))

lojbanToAtomeseRaw :: WordList -> String -> Maybe (Atom,String)
lojbanToAtomeseRaw state text =
    listToMaybe (rawparse (runReaderT lojban state) (text++" "))

wrapAtom :: Atom -> Atom
wrapAtom atom@(Link "SatisfactionLink" _ _) = cLL [cAN "QuestionAnchor" , atom]
wrapAtom atom@(Link "PutLink" _ _)          = cLL [cAN "QuestionAnchor" , atom]
wrapAtom atom                               = cLL [cAN "StatementAnchor", atom]

atomeseToLojban :: WordList -> Atom -> Maybe String
atomeseToLojban state a@(LL [_an,s]) = P.print (runReaderT preti state) s

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
