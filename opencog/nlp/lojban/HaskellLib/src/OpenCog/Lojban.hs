{-# LANGUAGE LambdaCase                 #-}
module OpenCog.Lojban
( initParserPrinter
, lojbanToAtomese
, atomeseToLojban
, loadWordLists
, State
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

initParserPrinter :: IO ( String -> Either String (Maybe Atom)
                        ,Atom -> Either String String)

initParserPrinter = do
    wordlist <- loadWordLists
    seed <- randomIO
    return (lojbanToAtomese wordlist seed,atomeseToLojban wordlist seed)

lojbanToAtomese :: (WordList State) -> Int -> String -> Either String (Maybe Atom)
lojbanToAtomese rstate seed text = (fmap wrapAtom) . fst <$> evalRWST (apply lojban ()) rstate state
    where state = State {sFlags = M.empty
                        ,sAtoms = []
                        ,sTVLs = []
                        ,sText = text++" "
                        ,sSeed = seed
                        ,sNow = now_here
                        ,sCtx = [now_here]
                        ,sJAI = Nothing
                        ,sXU = []}

wrapAtom :: Atom -> Atom
wrapAtom atom@(Link "SatisfactionLink" _ _) = cLL [cAN "QuestionAnchor" , atom]
wrapAtom atom@(Link "PutLink" _ _)          = cLL [cAN "QuestionAnchor" , atom]
wrapAtom atom                               = cLL [cAN "StatementAnchor", atom]

atomeseToLojban :: (WordList State) -> Int -> Atom -> Either String String
atomeseToLojban rstate seed a@(LL [_an,s]) = sText . fst <$> execRWST (unapply lojban (Just s)) rstate state
    where state = State {sFlags = M.empty
                        ,sAtoms = []
                        ,sTVLs = []
                        ,sText = ""
                        ,sSeed = seed
                        ,sNow = now_here
                        ,sCtx = [now_here]
                        ,sJAI = Nothing
                        ,sXU = []}

now_here = cCN "NowAndHere" noTv
