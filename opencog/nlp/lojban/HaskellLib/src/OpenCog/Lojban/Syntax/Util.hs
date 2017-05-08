{-# LANGUAGE RankNTypes #-}
{-# LANGUAGE LambdaCase #-}
{-# LANGUAGE FlexibleContexts           #-}
module OpenCog.Lojban.Syntax.Util where

import Prelude hiding (id,(.),(<*>),(<$>),(*>),(<*),foldl)

import Data.List.Split (splitOn)
import Data.List (intercalate,find,delete)
import Data.Char (chr,isLetter,isDigit)
import Data.Maybe
import Data.Map (findWithDefault)

import Control.Category
import Control.Arrow
import Control.Applicative hiding (many,some,optional)
import Control.Monad
import Control.Monad.RWS.Class
import Control.Monad.Trans.Class

import System.Random

import Iso
import Syntax hiding (SynIso,Syntax)

import OpenCog.AtomSpace (Atom(..))
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

isoIntercalate :: String -> SynIso [String] String
isoIntercalate x = mkIso (intercalate x) (splitOn x)

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

setSeed :: SynMonad t State => Int -> (t ME) ()
setSeed i = modify (\s -> s {sSeed = i})

setFlag :: SynMonad t State => Flag -> (t ME) ()
setFlag f = modify (\s -> s {sFlags = f : sFlags s})

rmFlag :: SynMonad t State => Flag -> (t ME) ()
rmFlag f = modify (\s -> s {sFlags = delete f (sFlags s)})

setFlagIso :: Flag -> SynIso a a
setFlagIso flag = Iso f g
    where f a = setFlag flag >> pure a
          g a = rmFlag flag >> pure a

withFlag :: Flag -> SynIso a b -> SynIso a b
withFlag flag syn = inverse (setFlagIso flag) . syn . setFlagIso flag

ifFlag :: Flag -> SynIso a a
ifFlag flag = Iso f f where
    f a = do
        flags <- gets sFlags
        if flag `elem` flags
           then pure a
           else lift $ Left $ "Flag: " ++ flag ++ " is not set."

ifNotFlag :: Flag -> SynIso a a
ifNotFlag flag = Iso f f where
    f a = do
        flags <- gets sFlags
        if flag `elem` flags
           then lift $ Left $ "Flag: " ++ flag ++ " is set."
           else pure a

switchOnFlag :: Flag -> SynIso a (Either a a)
switchOnFlag flag = Iso f g where
    f a = do
        flags <- gets sFlags
        if flag `elem` flags
           then pure (Left a)
           else pure (Right a)
    g (Left a)  = setFlag flag >> pure a
    g (Right a) = pure a
