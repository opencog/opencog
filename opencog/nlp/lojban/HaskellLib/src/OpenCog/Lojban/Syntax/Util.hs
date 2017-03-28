{-# LANGUAGE RankNTypes #-}
{-# LANGUAGE LambdaCase #-}
{-# LANGUAGE FlexibleContexts           #-}
module OpenCog.Lojban.Syntax.Util where

import Prelude hiding (id,(.),(<*>),(<$>),(*>),(<*),foldl)

import Data.List.Split (splitOn)
import Data.List (partition,intercalate,find)
import Data.Char (chr,isLetter,isDigit)
import Data.Maybe
import Data.Map (findWithDefault)

import Control.Category
import Control.Applicative hiding (many,some,optional)
import Control.Monad
import Control.Monad.RWS.Class
import Control.Monad.Trans.Class

import System.Random

import Iso hiding (SynIso,Syntax)

import OpenCog.AtomSpace (Atom)
import OpenCog.Lojban.Syntax.Types

import qualified Data.ListTrie.Patricia.Set.Ord as TS

-------------------------------------------------------------------------------
--Iso Util
-------------------------------------------------------------------------------

mapIso :: Traversable t => SynIso a b -> SynIso (t a) (t b)
mapIso iso = Iso f g where
    f = traverse (apply iso)
    g = traverse (unapply iso)

choice :: [SynIso (c,a) b] -> SynIso (c,a) b
choice lst = Iso f g where
    f i = foldl1 mplus $ map (`apply` i) lst
    g i = foldl1 mplus $ map (`unapply` i) lst

infix 8 .>
infix 8 .<
infix 8 >.
infix 8 <.

(<.) :: SynIso b d -> SynIso a (b,c) -> SynIso a (d,c)
(<.) i j = first i . j

(>.) :: SynIso b d -> SynIso a (c,b) -> SynIso a (c,d)
(>.) i j = second i . j

(.<) :: SynIso (a,b) c -> SynIso d a -> SynIso (d,b) c
(.<) i j = i . first j

(.>) :: SynIso (a,b) c -> SynIso d b -> SynIso (a,d) c
(.>) i j = i . second j

infix 8 |^|

(|^|) :: SynIso gamma alpha -> SynIso gamma beta -> SynIso gamma (Either alpha beta)
(|^|) i j = inverse (inverse i ||| inverse j)

showReadIso :: (Read a, Show a) => SynIso a String
showReadIso = mkIso show read

partitionIso :: (a -> Bool) -> SynIso [a] ([a],[a])
partitionIso p = mkIso f g where
    f = partition p
    g = uncurry (++)

isoConcat :: String -> SynIso [String] String
isoConcat x = mkIso (intercalate x) (splitOn x)

isoConcat2 :: SynIso [[a]] [a]
isoConcat2 = mkIso f g where
    f = concat
    g = map (: [])

isoDrop :: Int -> SynIso String String
isoDrop i = mkIso (drop i) id

isoReverse :: SynIso [a] [a]
isoReverse = mkIso reverse reverse

stripSpace :: SynIso String String
stripSpace = mkIso f g where
    f [] = []
    f (' ':xs) = f xs
    f (x:xs) = x : f xs
    g x = x

--For converting elements or tuples into lists
--Lists are needed as arguments to form Link Atoms
tolist1 :: Show a => SynIso a [a]
tolist1 = Iso f g where
    f a   = pure [a]
    g [a] = pure a
    g a   = lift $ Left $ "Expecting List with exaclty two elements but got" ++ show a

tolist2 :: Show a => SynIso (a,a) [a]
tolist2 = Iso f g where
    f (a,b) = pure [a,b]
    g [a,b] = pure (a,b)
    g a     = lift $ Left $ "Expecting List with exaclty two elements but got" ++ show a

isoZip :: SynIso ([a],[b]) [(a,b)]
isoZip = mkIso (uncurry zip) unzip

isoDistribute :: SynIso (a,[b]) [(a,b)]
isoDistribute = isoZip . reorder
    where reorder = Iso f g
          f (a,b)   = pure (replicate (length b) a,b)
          g (a:_,b) = pure (a,b)
          g ([],_)  = lift $ Left "Got Empty list but need at least 1 elem."


mkSynonymIso :: (Eq a, Show a, Eq b, Show b) => [(a,b)] -> SynIso a b
mkSynonymIso ls = Iso f g where
    f e = case snd `fmap` find (\(a,b) -> a == e) ls of
            Just r -> pure r
            Nothing -> lift $ Left $ "No synoyme for " ++ show e
    g e = case fst `fmap` find (\(a,b) -> b == e) ls of
            Just r -> pure r
            Nothing -> lift $ Left $ "No synoyme for " ++ show e

try :: SynIso a a -> SynIso a a
try iso = Iso f g where
    f a = apply iso a <|> pure a
    g a = unapply iso a <|> pure a

--For dealing with maybes from/for Optional in the first or second position
ifJustA :: SynIso (Maybe a,b) (Either (a,b) b)
ifJustA = mkIso f g where
    f (Just a,b)  = Left (a,b)
    f (Nothing,b) = Right b
    g (Left (a,b))= (Just a,b)
    g (Right b)   = (Nothing,b)

ifJustB :: SynIso (a,Maybe b) (Either (a,b) a)
ifJustB = mkIso f g where
    f (a,Just b)  = Left (a,b)
    f (a,Nothing) = Right a
    g (Left (a,b))= (a,Just b)
    g (Right a)   = (a,Nothing)
-------------------------------------------------------------------------------
--SyntaxReader Util
-------------------------------------------------------------------------------

letter, digit :: Syntax Char
letter = token (\x -> isLetter x || x=='\'' || x=='.')
digit  = token isDigit

anyWord :: Syntax String
anyWord = some letter <&& sepSpace

--any :: Syntax String
--any = many token

word :: String -> Syntax String
word s = string s <&& sepSpace

mytext :: String -> Syntax ()
mytext s = text s <&& sepSpace

--For text that is both optional and should be parsed into ()
optext :: String -> Syntax ()
optext t = (text t <&& sepSpace) <+> (text "" <&& optSpace)

--Handles 1 of many options from a list
oneOfS :: (a -> Syntax b) -> [a] -> Syntax b
oneOfS f = foldr ((<+>) . f) zeroArrow

{-multipleOf :: StringSet -> Syntax [String]
multipleOf ss = isoReverse . memberIso2 ss . anyWord

memberIso2 :: StringSet -> SynIso String [String]
memberIso2 sss = Iso (f sss [] []) g where
    f ss [] erg [] = Just erg
    f ss l erg [] = if TS.member l ss
                       then Just (l:erg)
                     else Nothing
    f ss l erg (x:xs) = let key= l++[x]
                            ns = TS.lookupPrefix key ss
                        in case TS.size ns of
                            0 -> if TS.member l ss
                                    then f sss [] (l:erg) (x:xs)
                                      else Nothing
                              1 -> if TS.member key ns
                                      then f sss [] (key:erg) xs
                                      else f ns key erg xs
                              _ -> f ns key erg xs
    g ls = Just $ concat ls --FIXME
-}

gismu :: Syntax String
gismu = Iso f f . anyWord
    where f word = do
                gismu <- asks gismus
                if TS.member word gismu
                    then pure word
                    else lift $ Left $ "'" ++ word ++ "' is not a gismu"

selmaho :: String -> Syntax String
selmaho s = _selmaho s . anyWord

_selmaho :: String -> SynIso String String
_selmaho s = Iso f f
    where f word = do
            cmavo <- asks cmavos
            let selmaho = findWithDefault TS.empty s cmavo
            if TS.member word selmaho
                then pure word
                else lift $ Left $ "'" ++ word ++ "' is not a cmavo of class: " ++ s

{-joiSelmaho :: String -> Syntax [String]
joiSelmaho s = ReaderT (\(cmavo,_,_,_) -> multipleOf (findWithDefault TS.empty s cmavo))
-}

sepSelmaho :: String -> Syntax ()
sepSelmaho s = ignoreAny "sepSelmaho FIXME" . selmaho s

optSelmaho :: String -> Syntax ()
optSelmaho s = handle . optional (selmaho s)
    where handle = Iso f g where
            f _ = pure ()
            g () = pure Nothing

-------------------------------------------------------------------------------
--State Helpers
-------------------------------------------------------------------------------

setAtoms :: SynMonad t State => [Atom] -> (t ME) ()
setAtoms a = modify (\s -> s {sAtoms = a})

pushAtom :: SynMonad t State => Atom -> (t ME) ()
pushAtom a = modify (\s -> s {sAtoms = a : sAtoms s})

pushAtoms :: SynMonad t State => [Atom] -> (t ME) ()
pushAtoms a = modify (\s -> s {sAtoms = a ++ sAtoms s})

popAtom :: SynMonad t State => (t ME) Atom
popAtom = do
    atoms <- gets sAtoms
    let ([a],as) = splitAt 1 atoms
    setAtoms as
    pure a

popAtoms :: SynMonad t State => Int -> (t ME) [Atom]
popAtoms i = do
    atoms <- gets sAtoms
    let (a,as) = splitAt i atoms
    setAtoms as
    pure a

consAtoms :: SynIso Atom [Atom]
consAtoms = Iso f g where
    f a = do
        atoms <- gets sAtoms
        pure (a:atoms)
    g (a:atoms) = do
        setAtoms atoms
        pure a

appendAtoms :: Int -> SynIso [Atom] [Atom]
appendAtoms i = Iso f g where
    f as = do
        atoms <- gets sAtoms
        pure (as++atoms)
    g atoms = do
        let (a,as) = splitAt i atoms
        setAtoms as
        pure a

-- Applies an iso with an empty state
-- Assumes inverse doesn't need special state handling
withEmptyState :: SynIso a b -> SynIso a b
withEmptyState iso = Iso f g
  where
    f a = do
       atoms <- gets sAtoms
       setAtoms []
       b <- apply iso a
       pushAtoms atoms
       pure b
    g = unapply iso
