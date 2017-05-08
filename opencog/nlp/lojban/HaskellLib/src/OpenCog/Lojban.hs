{-# LANGUAGE LambdaCase                 #-}
module OpenCog.Lojban
( initParserPrinter
, lojbanToAtomese
, atomeseToLojban
, loadWordLists
) where

import Lojban

import OpenCog.Lojban.Syntax
import OpenCog.Lojban.Util
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
    seed <- randomIO
    return (lojbanToAtomese wordlist seed,atomeseToLojban wordlist seed)

lojbanToAtomese :: (WordList State) -> Int -> String -> Either String Atom
lojbanToAtomese rstate seed text = wrapAtom . fst <$> evalRWST (apply lojban ()) rstate state
    where state = State {sFlags = [],sAtoms = [],sText = text++" ",sSeed = seed}

wrapAtom :: Atom -> Atom
wrapAtom atom@(Link "SatisfactionLink" _ _) = cLL [cAN "QuestionAnchor" , atom]
wrapAtom atom@(Link "PutLink" _ _)          = cLL [cAN "QuestionAnchor" , atom]
wrapAtom atom                               = cLL [cAN "StatementAnchor", atom]

atomeseToLojban :: (WordList State) -> Int -> Atom -> Either String String
atomeseToLojban rstate seed a@(LL [_an,s]) = sText . fst <$> execRWST (unapply lojban s) rstate state
    where state = State {sFlags = [],sAtoms = [],sText = "",sSeed = seed}
