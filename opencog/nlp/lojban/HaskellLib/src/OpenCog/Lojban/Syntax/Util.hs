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

--Try all SynIsos in the list and return result of the first one that works
choice :: [SynIso a b] -> SynIso a b
choice lst = Iso f g where
    f i = foldl1 mplus $ map (`apply` i) lst
    g i = foldl1 mplus $ map (`unapply` i) lst

infix 8 |^|

--Reverse of |||
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

--Try SynIso on failure keep original value
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

--Sep Isos consume input but have () as result
--using the default we would get [()]
--The count of Units is often not usefull so this just returns ()
--While consuming many sepIsos
manySep :: Syntax () -> Syntax ()
manySep iso = (manySep iso <+> id) . iso <+> insert ()

-------------------------------------------------------------------------------
--State Helpers
-------------------------------------------------------------------------------

--JAI

setJai :: SynMonad t State => JJCTTS -> (t ME) ()
setJai a = modify (\s -> s {sJAI = Just a})

rmJai :: SynMonad t State => (t ME) ()
rmJai = modify (\s -> s {sJAI = Nothing})

--XU

addXU :: SynMonad t State => Atom -> (t ME) ()
addXU a = modify (\s -> s {sXU = a:(sXU s)})

rmXU :: SynMonad t State => Atom -> (t ME) ()
rmXU a = modify (\s -> s {sXU = delete a (sXU s)})

addXUIso :: SynIso Atom Atom
addXUIso = Iso f g where
    f a = addXU a >> pure a
    g a = rmXU a >> pure a

--CTX

setCtx :: SynMonad t State => [Atom] -> (t ME) ()
setCtx a = modify (\s -> s {sCtx = a})

setNow :: SynMonad t State => Atom -> (t ME) ()
setNow a = modify (\s -> s {sNow = a})

setPrimaryCtx :: SynMonad t State => Atom -> (t ME) ()
setPrimaryCtx a = modify (\s -> s {sCtx = a : sCtx s})

addCtx :: SynMonad t State => Atom -> (t ME) ()
addCtx a = modify (\s -> s {sCtx = (sCtx s) ++ [a]})

--Atoms

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

--Add a fixed number of Atoms to the State
toState :: Int -> SynIso [Atom] ()
toState i = Iso f g where
    f as =
        if length as == i
           then pushAtoms as
           else lift $ Left "List has wrong lenght, is this intended?"
    g () = popAtoms i

fstToState :: Int -> SynIso ([Atom],a) a
fstToState i = iunit . commute . first (toState i)

sndToState :: Int -> SynIso (a,[Atom]) a
sndToState i = iunit . second (toState i)

--TypedVariableLinks

pushTVLs :: SynMonad t State => [Atom] -> (t ME) ()
pushTVLs a = modify (\s -> s {sTVLs = a ++ sTVLs s})

setTVLs :: SynMonad t State => [Atom] -> (t ME) ()
setTVLs a = modify (\s -> s {sTVLs = a})

--Clean State

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
                     ,sDA = sDA s
                     ,sDaM = sDaM s
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
                         ,sDA = sDA s2
                         ,sDaM = sDaM s2
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

--Seed

setSeed :: SynMonad t State => Int -> (t ME) ()
setSeed i = modify (\s -> s {sSeed = i})

-- DA Function

setDAF :: SynMonad t State => (Atom -> Atom) -> (t ME) ()
setDAF f = modify (\s -> s {sDA = f})

--DA instantiations

setDa :: SynMonad t State => String -> Atom -> (t ME) ()
setDa da a = modify (\s -> s {sDaM = M.insert da a (sDaM s)})

unsetDA :: SynMonad t State => String -> (t ME) ()
unsetDA da = modify (\s -> s {sDaM = M.delete da (sDaM s)})

getDA :: SynMonad t State => String -> (t ME) (Maybe Atom)
getDA da = do
        das <- gets sDaM
        pure $ M.lookup da das

--Flags

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

--Set or Read the Flag Value
setFlagValueIso :: Flag -> SynIso String ()
setFlagValueIso flag = inverse (getFlagValueIso flag)

rmFlag :: SynMonad t State => Flag -> (t ME) ()
rmFlag f = modify (\s -> s {sFlags = M.delete f (sFlags s)})

--Set or Delete the Flag Value
setFlagValueIso2 :: Flag -> String -> SynIso a a
setFlagValueIso2 flag val = Iso f g
    where f a = setFlagValue flag val >> pure a
          g a = rmFlag flag >> pure a

--Set or Delete the Flat with no Value
setFlagIso :: Flag -> SynIso a a
setFlagIso flag = Iso f g
    where f a = setFlag flag >> pure a
          g a = rmFlag flag >> pure a

rmFlagIso :: Flag -> SynIso a a
rmFlagIso flag = inverse $ setFlagIso flag

--Run SynIso with Flag set and remove the Flag afterwards
withFlag :: Flag -> SynIso a b -> SynIso a b
withFlag flag syn = rmFlagIso flag . syn . setFlagIso flag

--Run SynIso with Flag and Value set and remove the Flag afterwards
withFlagValue :: Flag -> String -> SynIso a b -> SynIso a b
withFlagValue flag val syn = inverse (setFlagValueIso2 flag val)
                           . syn
                           . setFlagValueIso2 flag val

toggleFlag :: Flag -> SynIso a a
toggleFlag f = setFlagIso f . ifNotFlag f <+> rmFlagIso f . ifFlag f

--Iso that fails if flag is not Set
ifFlag :: Flag -> SynIso a a
ifFlag flag = Iso f f where
    f a = do
        flags <- gets sFlags
        if flag `M.member` flags
           then pure a
           else lift $ Left $ "Flag: " ++ flag ++ " is not set."

--Iso that fails if flag doesn't have Value
ifFlagVlaue :: Flag -> String -> SynIso a a
ifFlagVlaue flag val  = Iso f f where
    f a = do
        value <- getFlagValue flag
        if value == val
           then pure a
           else lift $ Left $ "Flag: " ++ flag ++ " doesn't have value" ++ val

--Iso that fails if flag is set
ifNotFlag :: Flag -> SynIso a a
ifNotFlag flag = Iso f f where
    f a = do
        flags <- gets sFlags
        if flag `M.member` flags
           then lift $ Left $ "Flag: " ++ flag ++ " is set."
           else pure a

--Left if Flag else Right
switchOnFlag :: Flag -> SynIso a (Either a a)
switchOnFlag flag = Iso f g where
    f a = do
        flags <- gets sFlags
        if flag `M.member` flags
           then pure (Left a)
           else pure (Right a)
    g (Left a)  = setFlag flag >> pure a
    g (Right a) = pure a

--Left if a has Value else Right
switchOnValue :: Eq a => a -> SynIso a (Either a a)
switchOnValue val = mkIso f g where
    f a = if a == val
             then Left a
             else Right a
    g (Left a)  = a
    g (Right a) = a

--Left if Flag has Value else Right
switchOnFlagValue :: Eq a => Flag -> String -> SynIso a (Either a a)
switchOnFlagValue flag val = Iso f g where
    f a = do
        flagval <- getFlagValue flag
        if flagval == val
           then pure (Left a)
           else pure (Right a)
    g (Left a)  = setFlagValue flag val >> pure a
    g (Right a) = pure a
