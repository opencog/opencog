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

import qualified Data.Map as M
import qualified Data.ListTrie.Patricia.Set.Ord as TS

-------------------------------------------------------------------------------
--Iso Util
-------------------------------------------------------------------------------

mapIso :: Traversable t => SynIso a b -> SynIso (t a) (t b)
mapIso iso = Iso f g where
    f = traverse (apply iso)
    g = traverse (unapply iso)

choice :: [SynIso a b] -> SynIso a b
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

isoPrepend :: String -> SynIso String String
isoPrepend s1 = mkIso f g where
    f s2 = s1++s2
    g s  = drop (length s1) s

isoAppend :: String -> SynIso String String
isoAppend s1 = mkIso f g where
    f s2 = s2++s1
    g s  = take (length s - length s1) s

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

--Parser ID that fails when printing
--Use to force a path in (maybe . pid &&& something)
pid :: SynIso a a
pid = Iso f g where
    f a = pure a
    g _ = lift $ Left "Only succeds when parsing."

-------------------------------------------------------------------------------
--State Helpers
-------------------------------------------------------------------------------

setJai :: SynMonad t State => JJCTTS -> (t ME) ()
setJai a = modify (\s -> s {sJAI = Just a})

addXUIso :: SynIso Atom Atom
addXUIso = Iso f g where
    f a = addXU a >> pure a
    g a = rmXU a >> pure a

addXU :: SynMonad t State => Atom -> (t ME) ()
addXU a = modify (\s -> s {sXU = a:(sXU s)})

rmXU :: SynMonad t State => Atom -> (t ME) ()
rmXU a = modify (\s -> s {sXU = delete a (sXU s)})

rmJai :: SynMonad t State => (t ME) ()
rmJai = modify (\s -> s {sJAI = Nothing})

setCtx :: SynMonad t State => [Atom] -> (t ME) ()
setCtx a = modify (\s -> s {sCtx = a})

setNow :: SynMonad t State => Atom -> (t ME) ()
setNow a = modify (\s -> s {sNow = a})

setPrimaryCtx :: SynMonad t State => Atom -> (t ME) ()
setPrimaryCtx a = modify (\s -> s {sCtx = a : sCtx s})

addCtx :: SynMonad t State => Atom -> (t ME) ()
addCtx a = modify (\s -> s {sCtx = (sCtx s) ++ [a]})

setAtoms :: SynMonad t State => [Atom] -> (t ME) ()
setAtoms a = modify (\s -> s {sAtoms = a})

getAtoms :: SynMonad t State => (t ME) [Atom]
getAtoms = gets sAtoms

gsAtoms :: SynIso () [Atom]
gsAtoms = Iso f g where
    f _ = do
        as <- getAtoms
        setAtoms []
        pure as
    g a = setAtoms a

rmAtom ::  SynMonad t State => Atom -> (t ME) ()
rmAtom a = modify (\s -> s {sAtoms = delete a (sAtoms s)})

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

pushTVLs :: SynMonad t State => [Atom] -> (t ME) ()
pushTVLs a = modify (\s -> s {sTVLs = a ++ sTVLs s})

setTVLs :: SynMonad t State => [Atom] -> (t ME) ()
setTVLs a = modify (\s -> s {sTVLs = a})

--FIXME???
withCleanState :: Syntax a -> Syntax a
withCleanState syn = Iso f g where
    f () = do
        state <- get
        put (cleanState state)
        res <- apply syn ()
        state2 <-get
        put (mergeState state state2)
        pure res
    g a = unapply syn a

cleanState :: State -> State
cleanState s = State {sFlags = M.empty
                     ,sAtoms = []
                     ,sTVLs = []
                     ,sText = sText s
                     ,sSeed = sSeed s
                     ,sNow = sNow s
                     ,sCtx = sCtx s
                     ,sJAI = Nothing
                     ,sXU  = []}

mergeState :: State -> State -> State
mergeState s1 s2 = State {sFlags = sFlags s1
                         ,sAtoms = sAtoms s1
                         ,sTVLs = sTVLs s2
                         ,sText = sText s2
                         ,sSeed = sSeed s2
                         ,sNow = sNow s1
                         ,sCtx = sCtx s1
                         ,sJAI = sJAI s1
                         ,sXU  = sXU s1}

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
setFlag f = modify (\s -> s {sFlags = M.insert f "" (sFlags s) })

setFlagValue :: SynMonad t State => Flag -> String -> (t ME) ()
setFlagValue f v = modify (\s -> s {sFlags = M.insert f v (sFlags s) })

getFlagValue :: SynMonad t State => Flag -> (t ME) String
getFlagValue f = do
    flags <- gets sFlags
    case M.lookup f flags of
        Just a -> pure a
        Nothing -> lift $ Left $ "Flag: " ++ f ++ " doesn't exist."

getFlagValueIso :: Flag -> SynIso () String
getFlagValueIso flag = Iso f g where
    f () = getFlagValue flag
    g s  = setFlagValue flag s

setFlagValueIso :: Flag -> SynIso String ()
setFlagValueIso flag = inverse (getFlagValueIso flag)

rmFlag :: SynMonad t State => Flag -> (t ME) ()
rmFlag f = modify (\s -> s {sFlags = M.delete f (sFlags s)})

setFlagIso :: Flag -> SynIso a a
setFlagIso flag = Iso f g
    where f a = setFlag flag >> pure a
          g a = rmFlag flag >> pure a

rmFlagIso :: Flag -> SynIso a a
rmFlagIso flag = inverse $ setFlagIso flag

withFlag :: Flag -> SynIso a b -> SynIso a b
withFlag flag syn = inverse (setFlagIso flag) . syn . setFlagIso flag

toggleFlag :: Flag -> SynIso a a
toggleFlag f = setFlagIso f . ifNotFlag f <+> rmFlagIso f . ifFlag f

ifFlag :: Flag -> SynIso a a
ifFlag flag = Iso f f where
    f a = do
        flags <- gets sFlags
        if flag `M.member` flags
           then pure a
           else lift $ Left $ "Flag: " ++ flag ++ " is not set."

ifFlagVlaue :: Flag -> String -> SynIso a a
ifFlagVlaue flag val  = Iso f f where
    f a = do
        value <- getFlagValue flag
        if value == val
           then pure a
           else lift $ Left $ "Flag: " ++ flag ++ " doesn't have value" ++ val

ifNotFlag :: Flag -> SynIso a a
ifNotFlag flag = Iso f f where
    f a = do
        flags <- gets sFlags
        if flag `M.member` flags
           then lift $ Left $ "Flag: " ++ flag ++ " is set."
           else pure a

switchOnFlag :: Flag -> SynIso a (Either a a)
switchOnFlag flag = Iso f g where
    f a = do
        flags <- gets sFlags
        if flag `M.member` flags
           then pure (Left a)
           else pure (Right a)
    g (Left a)  = setFlag flag >> pure a
    g (Right a) = pure a

switchOnValue :: Eq a => a -> SynIso a (Either a a)
switchOnValue val = mkIso f g where
    f a = if a == val
             then Left a
             else Right a
    g (Left a)  = a
    g (Right a) = a

toState :: Int -> SynIso [Atom] ()
toState i = Iso f g where
    f as =
        if length as == i
           then pushAtoms as
           else lift $ Left "List has wrong lenght, is this intended?"
    g () = do
        allatoms <- gets sAtoms
        let (as,atoms) = splitAt i allatoms
        modify (\s -> s {sAtoms = atoms})
        pure as

fstToState :: Int -> SynIso ([Atom],a) a
fstToState i = iunit . commute . first (toState i)

sndToState :: Int -> SynIso (a,[Atom]) a
sndToState i = iunit . second (toState i)
