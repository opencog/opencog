module Test where

import Prelude hiding (id,(.),(<*>),(<$>),pure,(*>),(<*),foldl,print)
import qualified Prelude as P

import OpenCog.Lojban
import OpenCog.Lojban.Util
import OpenCog.Lojban.WordList
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

mystate s = State {sFlags = [],sAtoms = [],sText = s}

init = do
    wl <- loadWordLists "cmavo.csv" "gismu.csv"
    return wl

mpa :: WordList -> Syntax a -> String -> Either String (a,State,())
mpa wl x y = runRWST (apply x ()) wl (mystate y)
