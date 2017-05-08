{-# LANGUAGE ScopedTypeVariables #-}
module Test where

import Prelude hiding (id,(.),(<*>),(<$>),pure,(*>),(<*),foldl,print)
import qualified Prelude as P

import Lojban

import OpenCog.Lojban
import OpenCog.Lojban.Util
import OpenCog.Lojban.Syntax
import OpenCog.Lojban.Syntax.Util
import OpenCog.Lojban.Syntax.Types
import OpenCog.Lojban.Syntax.AtomUtil

import OpenCog.AtomSpace

import Iso hiding (Syntax,SynIso)

import Control.Applicative ((<$>))
import Control.Monad.RWS
import Control.Category (id,(.))

import qualified Data.ListTrie.Patricia.Set.Ord as TS

import Data.Maybe
import qualified Data.Map as M

--import Text.XML.HXT.Core

mystate s = State {sFlags = [],sAtoms = [],sText = s,sSeed = 0}

loadwl = do
    (wl :: WordList State) <- loadWordLists "cmavo.csv" "gismu.csv"
    return wl

mpag :: WordList State -> Syntax a -> String -> Either String (a,State,())
mpag wl x y = runRWST (apply x ()) wl (mystate y)

mpa :: WordList State -> Syntax Atom -> String -> IO ()
mpa wl x y = do
    case runRWST (apply x ()) wl (mystate y) of
        Left s -> putStrLn s
        Right (a,s,_) -> printAtom a >> P.print s
