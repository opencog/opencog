{-# LANGUAGE RankNTypes #-}
{-# LANGUAGE LambdaCase #-}
module OpenCog.Lojban.Syntax.Util where

import Prelude hiding (id,(.),(<*>),(<$>),pure,(*>),(<*),foldl)

import qualified Data.Map as M
import qualified Data.List.Split as S
import qualified Data.Foldable as F
import Data.List (partition,intercalate)
import Data.Char (chr,isLetter,isDigit)
import Data.Maybe


import Control.Monad.Trans.Reader
import Control.Monad
import qualified Control.Applicative as A

import System.Random

import Control.Category (id,(.))
import Control.Isomorphism.Partial
import Control.Isomorphism.Partial.Unsafe
import Text.Syntax

import OpenCog.Lojban.Syntax.Types

import qualified Data.ListTrie.Patricia.Set.Ord as TS

-------------------------------------------------------------------------------
--Iso Util
-------------------------------------------------------------------------------

isofmap :: Functor f => Iso a b -> Iso (f a) (f b)
isofmap iso = Iso f g where
    f = Just . fmap (fromJust . apply iso)
    g = Just . fmap (fromJust . unapply iso)

second :: Iso a b -> Iso (c,a) (c,b)
second a = id *** a

first :: Iso a b -> Iso (a,c) (b,c)
first  a = a *** id

insert :: a -> Iso () a
insert = inverse . ignore

iunit = inverse unit

ciunit = iunit . commute

addfst :: a -> Iso b (a,b)
addfst a = inverse $ rmfst a

addsnd :: a -> Iso b (b,a)
addsnd a = inverse $ rmsnd a

rmfst :: a -> Iso (a,b) b
rmfst a = iunit . commute .< ignore a

rmsnd :: a -> Iso (b,a) b
rmsnd a = iunit .> ignore a

choice :: Eq c => [(c,Iso a b)] -> Iso (c,a) b
choice lst = Iso f g where
    f (c,a) = let found = F.find (\(k,_) -> c == k) lst
              in case found of
                Just (_,iso) -> apply iso a
                Nothing -> Nothing
    g b = let Just (c,iso) = F.find (\(c,iso) -> isJust $ unapply iso b) lst
          in Just (c,fromJust $ unapply iso b)

infixr 8 .>
infixr 8 .<
infixr 8 >.
infixr 8 <.

(<.) :: Iso b d -> Iso a (b,c) -> Iso a (d,c)
(<.) i j = first i . j

(>.) :: Iso b d -> Iso a (c,b) -> Iso a (c,d)
(>.) i j = second i . j

(.<) :: Iso (a,b) c -> Iso d a -> Iso (d,b) c
(.<) i j = i . first j

(.>) :: Iso (a,b) c -> Iso d b -> Iso (a,d) c
(.>) i j = i . second j

(<^.) :: Iso b (State c) -> Iso a (State (b,d)) -> Iso a (State (c,d))
(<^.) i j = reorder . (first i <. j)
    where reorder = Iso (Just . f) (Just . g)
          f (((c,s1),d),s2) = ((c,d),s1++s2)
          g ((c,d),s)       = (((c,s),d),s)

(>^.) :: Iso b (State c) -> Iso a (State (d,b)) -> Iso a (State (d,c))
(>^.) i j =  reorder . (second i <. j)
    where reorder = Iso (Just . f) (Just . g)
          f ((c,(d,s1)),s2) = ((c,d),s1++s2)
          g ((c,d),s)       = ((c,(d,s)),s)

infixr 9 =.

(=.) :: Iso b (State c) -> Iso a (State b) -> Iso a (State c)
(=.) i j = collapsState . (i <. j)

(*^*) :: Iso a (State b) -> Iso c (State d) -> Iso (State (a,c)) (State (b,d))
(*^*) i j = reorder .< iso
    where iso = Iso f g
          f (a,c) = liftM2 (,) (apply i a) (apply j c)
          g (b,d) = liftM2 (,) (unapply i b) (unapply j d)
          reorder = Iso rf rg
          rf (((b,s1),(d,s2)),s3) = Just ((b,d),s1++s2++s3)
          rg ((b,d),s)            = Just (((b,s),(d,s)),s)

infix 8 |^|

(|^|) :: Iso gamma alpha -> Iso gamma beta -> Iso gamma (Either alpha beta)
(|^|) i j = inverse (inverse i ||| inverse j)

showReadIso :: (Read a, Show a) => Iso a String
showReadIso = Iso (Just . show) (Just . read)

partitionIso :: (a -> Bool) -> Iso [a] ([a],[a])
partitionIso p = Iso f g where
    f ls = Just $ partition p ls
    g (xs,ys) = Just $ xs ++ ys

isoConcat :: String -> Iso [String] String
isoConcat x = Iso (Just . intercalate x) (Just . S.splitOn x)

isoConcat2 :: Iso [[a]] [a]
isoConcat2 = Iso f g where
    f = Just . concat
    g = Just . map (\x -> [x])

isoDrop :: Int -> Iso String String
isoDrop i = Iso (Just . drop i) (Just . id)

isoReverse :: Iso [a] [a]
isoReverse = Iso (Just . reverse) (Just . reverse)

stripSpace :: Iso String String
stripSpace = Iso (Just . f) (Just . g) where
    f [] = []
    f (' ':xs) = f xs
    f (x:xs) = x : f xs
    g x = x

--For converting elements or tuples into lists
--Lists are needed as arguments to form Link Atoms
tolist1 :: Iso a [a]
tolist1 = Iso (\a -> Just [a]) (\[a] -> Just a)

tolist2 :: Show a => Iso (a,a) [a]
tolist2 = Iso (\(a1,a2) -> Just [a1,a2])
              (\case {[a1,a2] -> Just (a1,a2); a -> error $ "tolist2: " ++ show a})

isoZip :: Iso ([a],[b]) [(a,b)]
isoZip = Iso (Just . uncurry zip) (Just . unzip)

isoDistribute :: Iso (a,[b]) [(a,b)]
isoDistribute = isoZip . reorder
    where reorder = Iso f g
          f (a,b)   = Just (replicate (length b) a,b)
          g (a:_,b) = Just (a,b)
          g ([],_)  = error $ "This can't happen can it?"


mapIso :: Iso a b -> Iso [a] [b]
mapIso iso = Iso f g where
    f = mapM $ apply iso
    g = mapM $ unapply iso

mkSynonymIso :: (Eq a, Eq b) => [(a,b)] -> Iso a b
mkSynonymIso ls = Iso f g where
    f e = snd `fmap` F.find (\(a,b) -> a == e) ls
    g e = fst `fmap` F.find (\(a,b) -> b == e) ls

try :: Iso a a -> Iso a a
try iso = Iso f g where
    f a = apply iso a A.<|> Just a
    g a = unapply iso a A.<|> Just a

--For dealing with maybes from/for Optional in the first or second position
ifJustA :: Iso (Maybe a,b) (Either (a,b) b)
ifJustA = Iso (\case {(Just a,b) -> Just $ Left (a,b) ; (Nothing,b) -> Just $  Right b})
              (\case {Left (a,b) -> Just (Just a,b) ;  Right b  -> Just (Nothing,b)})

ifJustB :: Iso (a,Maybe b) (Either (a,b) a)
ifJustB = Iso (\case {(a,Just b) -> Just $ Left (a,b) ; (a,Nothing) -> Just $  Right a})
              (\case {Left (a,b) -> Just (a,Just b) ;  Right a  -> Just (a,Nothing)})
-------------------------------------------------------------------------------
--State Util
-------------------------------------------------------------------------------

infixr 6 <&>

(<&>) :: Syntax delta => delta (State a) -> delta (State b) -> delta (State (a,b))
a <&> b =  mergeState <$> a <*> b

mergeState :: Iso (State a,State b) (State (a,b)) --((Atom,Atom),[Atoms])
mergeState = Iso f g where
    f ((a1,s1),(a2,s2)) = Just ((a1,a2),s1++s2)
    g ((a1,a2),s)       = Just ((a1,s) ,(a2,s))

optState :: Syntax delta => delta (State a) -> delta (State (Maybe a))
optState syn = iso <$> optional syn
    where iso = Iso (Just . f) (Just . g)
            where f (Just (a,as)) = (Just a, as)
                  f Nothing       = (Nothing, [])
                  g (Nothing,_)   =  Nothing
                  g (Just a ,as)  = Just (a,as)

stateMany :: (Eq alpha,Syntax delta) => delta (State alpha)
                                     -> delta (State [alpha])
stateMany p = (first cons <$> p <&> stateMany p) <|> pure ([],[])

stateMany1 :: (Eq alpha,Syntax delta) => delta (State alpha)
                                      -> delta (State [alpha])
stateMany1 p = first cons <$> p <&> stateMany p

stateList :: Iso [State a] (State [a])
stateList = foldl ff . init
    where ff = Iso (Just . f) (Just . g)
          f ((a,xs),(b,ys)) = (b:a,xs++ys)
          g (b:a,xs) = ((a,xs),(b,xs))
          init = Iso (\ls -> Just (([],[]),ls))
                     (\(_,ls) -> Just ls)

collapsState :: Iso (State (State a)) (State a)
collapsState = Iso (Just . f) (Just . g) where
    f ((a,ys),xs) = (a,xs++ys)
    g (a,xs) = ((a,xs),xs)

expandState :: Iso (State a,b) (State (a,b))
expandState = Iso (Just . f) (Just . g) where
    f ((a,s),b) = ((a,b),s)
    g ((a,b),s) = ((a,s),b)

joinState :: Iso (State a,State b) (State (a,b))
joinState = Iso (Just . f) (Just . g) where
    f ((a,s1),(b,s2)) = ((a,b),s1++s2)
    g ((a,b),s)       = ((a,s),(b,s))

reorder0 :: Iso a (State a)
reorder0 = Iso (\a -> Just (a,[]))
               (\(a,_) -> Just a)


-------------------------------------------------------------------------------
--SyntaxReader Util
-------------------------------------------------------------------------------


withSeed :: SyntaxReader a -> SyntaxReader (a,Int)
withSeed r = ReaderT (\e@(_,_,_,seed) ->
                            runReaderT r e <*> pure seed) ---XXX can't work with printing as we don't no seed make pure just swallo it

withSeedState :: SyntaxReader (State a) -> SyntaxReader (State (a,Int))
withSeedState r = ReaderT (\e@(_,_,_,seed) ->
                                runReaderT r e <&> pure (seed,[])) ---XXX can't work with printing as we don't no seed make pure just swallo it


------------------------------------------------------------------------
letter, digit :: Syntax delta => delta Char
letter  =  subset (\x -> isLetter x || x=='\'' || x=='.') <$> token
digit   =  subset isDigit <$> token

anyWord :: Syntax delta => delta String
anyWord = many1 letter <* sepSpace

--any :: Syntax delta => delta String
--any = many token

--Handling for simple Strings
{-# ANN module "HLint: ignore Use foldr" #-}
string :: Syntax delta => String -> delta String
string [] =  pure []
string (x:xs) = cons <$> (subset (== x) <$> token) <*> string xs

word :: Syntax delta => String -> delta String
word s = string s <* sepSpace

mytext :: Syntax delta => String -> delta ()
mytext s = text s <* sepSpace

--For text that is both optional and should be parsed into ()
optext :: Syntax delta => String -> delta ()
optext t = (text t <* sepSpace) <|> (text "" <* optSpace)

--Handles 1 of many options from a list
oneOfS :: Syntax delta => (a -> delta b) -> [a] -> delta b
oneOfS f [] = empty
oneOfS f (x:xs) = f x <|> oneOfS f xs

oneOf :: Syntax delta => StringSet -> delta String
oneOf ss = memberIso ss <$> anyWord

memberIso :: StringSet -> Iso String String
memberIso ss = Iso f f where
    f e = if TS.member e ss
             then Just e
             else Nothing

multipleOf :: Syntax delta => StringSet -> delta [String]
multipleOf ss = isoReverse . memberIso2 ss <$> anyWord

memberIso2 :: StringSet -> Iso String [String]
memberIso2 sss = Iso (f sss [] []) (g) where
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

gismu :: SyntaxReader String
gismu = ReaderT (\(_,gismus,_,_) -> oneOf gismus)

selmaho :: String -> SyntaxReader String
selmaho s = ReaderT (\(cmavo,_,_,_) -> oneOf (M.findWithDefault TS.empty s cmavo))

joiSelmaho :: String -> SyntaxReader [String]
joiSelmaho s = ReaderT (\(cmavo,_,_,_) -> multipleOf (M.findWithDefault TS.empty s cmavo))

sepSelmaho :: String -> SyntaxReader ()
sepSelmaho s = ReaderT (\(cmavo,_,_,_) ->
    oneOfS mytext (TS.toList $ M.findWithDefault TS.empty s cmavo))

optSelmaho :: String -> SyntaxReader ()
optSelmaho s = handle <$> optional (selmaho s)
    where handle = Iso f g where
            f _ = Just ()
            g () = Just Nothing

