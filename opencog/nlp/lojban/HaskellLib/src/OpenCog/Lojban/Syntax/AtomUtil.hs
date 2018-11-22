{-# LANGUAGE LambdaCase #-}
{-# LANGUAGE FlexibleContexts           #-}
module OpenCog.Lojban.Syntax.AtomUtil where

import Prelude hiding (id,(.))

import Control.Category
import Control.Arrow
import Control.Monad.Trans.Class
import Control.Monad.RWS

import Iso
import Syntax hiding (SynIso,Syntax)

import qualified Data.Map as M
import qualified Data.Map.Lazy as M
import qualified Data.Foldable as F
import Data.Maybe (fromJust,isJust)
import Data.List (isSuffixOf,nub,isInfixOf)
import Data.Hashable
import Data.Char (chr)

import System.Random

import OpenCog.AtomSpace (Atom(..),noTv,TruthVal(..),stv,atomType
                         ,atomGetAllNodes,atomElem,nodeName,showAtom)

import OpenCog.Lojban.Syntax.Types
import OpenCog.Lojban.Syntax.Util
import OpenCog.Lojban.Util

import Lojban.Syntax.Util

--Iso for creating Atomese Links given a Name and TV
linkIso :: String -> TruthVal -> SynIso [Atom] Atom
linkIso n tv = Iso f g where
    f as = pure $ Link n as tv
    g (Link t o _) --TODO: should we also check the tv?
        | t == n = pure o
        | otherwise = lift $ Left $ "Wrong LinkType was expecting " ++ n ++
                                    " but got " ++ t

--Iso for creating Atomese Links given a Name
linkIso2 :: String -> SynIso (TruthVal,[Atom]) Atom
linkIso2 n  = Iso f g where
    f (tv,as) = pure $ Link n as tv
    g (Link t o tv)
        | t == n = pure (tv,o)
        | otherwise = lift $ Left $ "Wrong LinkType was expecting " ++ n ++
                                    " but got " ++ t

--Iso for creating Atomese Nodes
nodeIso :: String -> TruthVal -> SynIso String Atom
nodeIso t tv = Iso f g where
    f n = pure $ Node t n tv
    g (Node tt n _) --TODO: should we also check the tv?
        | t == tt = pure n
        | otherwise = lift $ Left $ "Wrong LinkType was expecting " ++ t ++
                                    " but got " ++ tt

--Various Isos for create Nodes

concept :: SynIso String Atom
concept = nodeIso "ConceptNode" noTv

wordNode :: SynIso String Atom
wordNode = nodeIso "WordNode" noTv

predicate :: SynIso String Atom
predicate = nodeIso "PredicateNode" noTv

varnode :: SynIso String Atom
varnode = nodeIso "VariableNode" noTv

randvarnode :: SynIso String Atom
randvarnode = varnode . Iso f g where
    f s = do
        n <- randName s
        pure (n ++ s)
    g s = pure $ drop 20 s

numberNode :: SynIso String Atom
numberNode = nodeIso "NumberNode" noTv


--Various semi-isos to easily transfrom Certain Links
_eval :: SynIso (Atom,[Atom]) Atom
_eval = eval . tolist2 . second listl

_evalTv :: SynIso (TruthVal,(Atom,[Atom])) Atom
_evalTv = evalTv . second (tolist2 . second listl)

_ctx :: SynIso (Maybe Atom,Atom) Atom
_ctx = ((ctx . tolist2) ||| id) . ifJustA

_ctxold :: SynIso (Atom,(Atom,[Atom])) Atom
_ctxold = ctx . tolist2 . second _eval

_ssl :: SynIso Atom Atom
_ssl = ssl . tolist2 . addfst (Node "VariableNode" "$var" noTv)

_meml :: SynIso (Atom, Atom) Atom
_meml = meml . tolist2

_equivl :: SynIso (Atom, Atom) Atom
_equivl = equivl . tolist2

_typedvarl :: SynIso (Atom, Atom) Atom
_typedvarl = typedvarl . tolist2

_exl :: SynIso (Atom, Atom) Atom
_exl = exl . tolist2

_satl :: SynIso Atom Atom
_satl = satl . tolist1

_iimpl :: SynIso (Atom,Atom) Atom
_iimpl = iimpl . tolist2

--Primitive variants (less used directly)

satl :: SynIso [Atom] Atom
satl = linkIso "SatisfactionLink" noTv

ctx :: SynIso [Atom] Atom
ctx = linkIso "ContextLink" noTv

eval :: SynIso [Atom] Atom
eval = linkIso "EvaluationLink" noTv

evalTv :: SynIso (TruthVal,[Atom]) Atom
evalTv = linkIso2 "EvaluationLink"

ssl :: SynIso [Atom] Atom
ssl = linkIso "SatisfyingSetScopeLink" noTv

exl :: SynIso [Atom] Atom
exl = linkIso "ExistsLink"  noTv

meml :: SynIso [Atom] Atom
meml = linkIso "MemberLink" noTv

equivl :: SynIso [Atom] Atom
equivl = linkIso "EquivalenceLink" noTv

setl :: SynIso [Atom] Atom
setl = linkIso "SetLink" noTv

subsetL :: SynIso (Atom,Atom) Atom
subsetL = linkIso "SubsetLink" noTv . tolist2

setTypeL  :: SynIso [Atom] Atom
setTypeL = linkIso "SetTypeLink" noTv

sizeL  :: SynIso [Atom] Atom
sizeL = linkIso "SetSizeLink" noTv

inhl :: SynIso [Atom] Atom
inhl = linkIso "InheritanceLink" noTv

impl :: SynIso [Atom] Atom
impl = linkIso "ImplicationLink" noTv

iimpl :: SynIso [Atom] Atom
iimpl = linkIso "IntensionalImplicationLink" noTv

listl :: SynIso [Atom] Atom
listl = linkIso "ListLink" noTv

--varl :: SynIso [Atom] Atom
--varl = linkIso "VariableLink" noTv

typedvarl :: SynIso [Atom] Atom
typedvarl = linkIso "TypedVariableLink" noTv

varll :: SynIso [Atom] Atom
varll = linkIso "VariableList" noTv

notl :: SynIso [Atom] Atom
notl = linkIso "NotLink" noTv

andl :: SynIso [Atom] Atom
andl = linkIso "AndLink" noTv

orl :: SynIso [Atom] Atom
orl = linkIso "MyOrLink" noTv

exel :: SynIso [Atom] Atom
exel = linkIso "ExecutionLink" noTv

--Logical Links partially selfmade

--There is no actuall SchemaLink instead it's a ExecutionLink with a
--DefinedSchemaNode used for Logical connectives
definedSchemaLink :: String -> SynIso [Atom] Atom
definedSchemaLink s = exel . tolist2 . addfst dsn . listl
    where dsn = Node "DefinedSchemaNode" s noTv

iffl :: SynIso [Atom] Atom
iffl = definedSchemaLink "IfAndOnlyIf"

uL :: SynIso [Atom] Atom
uL = definedSchemaLink "WetherOrNot"

--FIXME this is completely broken would need use TypeNode
--Which wouldn't allow using a1 and a2 makes is kinda pointless
--Find other solution
--Supposed to represent a question to which link was used.
jiL :: SynIso [Atom] Atom
jiL = Iso f g where
    f [a1,a2] = do
        rndname1 <- randName $ show a1 ++ show a2
        rndname2 <- randName $ show a1 ++ show a2
        rndnamel <- randName $ show a1 ++ show a2
        let v1 = cVN rndname1
            v2 = cVN rndname2
            v  = cVN rndnamel
            tvl= [cTVL v1
                  (cTCL [a1,cNL noTv a1])
                 ,cTVL v2
                  (cTCL [a2,cNL noTv a2])
                 ,cTVL v
                  (cTCL [cAL noTv [v1,v2]
                        ,cOL noTv [v1,v2]
                        ,cEXL noTv (cDSN "IfAndOnlyIf") (cLL [v1,v2])
                        ,cEXL noTv (cDSN "WetherOrNot") (cLL [v1,v2])
                        ])
                 ]
        pushTVLs tvl
        pure v
    g = error "AtomUtil.jiL.g not defined"

--Logical connectives have modifies deal with them
handleEKMods :: SynIso (EK,(Atom,Atom)) (String,(Atom,Atom))
handleEKMods = mkIso f g where
    f ((bna,(bse,(c,bnai))),(a1,a2)) = let na1 = if bna
                                                 then cMNL noTv a1
                                                 else a1
                                           na2 = if bnai
                                                 then cMNL noTv a2
                                                 else a2
                                       in if bse
                                             then (c,(na2,na1))
                                             else (c,(na1,na2))
    g (s,(na1,na2)) = let (bna,a1) = case na1 of
                                  (NL [a1]) -> (True,a1)
                                  _         -> (False,na1)
                          (bnai,a2) = case na2 of
                                  (NL [a2]) -> (True ,a2)
                                  _         -> (False,na2)
                      in ((bna,(False,(s,bnai))),(a1,a2))

conLink :: SynIso (EK,(Atom,Atom)) Atom
conLink = conLink' . second tolist2 . handleEKMods
    where conLink' :: SynIso (String,[Atom]) Atom
          conLink' = choice conHandlers
          conHandlers = [andl   . rmfst "e"
                        ,orl    . rmfst "a"
                        ,iffl   . rmfst "o"
                        ,uL     . rmfst "u"
                        {-,jiL    . rmfst "ji"-}
                        ]

--There are multiple logical connectives in Lojban connecting different things
--But they have the same representation in Atomese so map them to the base case
_JAtoA :: SynIso String String
_JAtoA = mkSynonymIso [("je","e")
                      ,("ja","a")
                      ,("jo","o")
                      ,("ju","u")
                      ,("je'i","ji")]
_GIhAtoA :: SynIso String String
_GIhAtoA = mkSynonymIso [("gi'e","e")
                        ,("gi'a","a")
                        ,("gi'o","o")
                        ,("gi'u","u")
                        ,("gi'i","ji")]
_GAtoA :: SynIso String String
_GAtoA = mkSynonymIso [("ge","e")
                      ,("ga","a")
                      ,("go","o")
                      ,("gu","u")
                      ,("ge'i","ji")]
_GUhAtoA :: SynIso String String
_GUhAtoA = mkSynonymIso [("gu'e","e")
                        ,("gu'a","a")
                        ,("gu'o","o")
                        ,("gu'u","u")
                        ,("gu'i","ji")]


_frames :: SynIso (Selbri,[(Atom,Tag)]) Atom
_frames = handleXU
        . second ((id ||| andl)
                  . isSingle
                  . mapIso _frame
                  . isoDistribute) --Selbri get's applied to each Sumti
        . reorder
    where isSingle = mkIso f g where
            f [a] = Left a
            f as  = Right as
            g (Left a) = [a]
            g (Right as) = as

          reorder = mkIso f g where
            f a@((_,sa),_) = (sa,a)
            g (sa,a) = a

handleXU :: SynIso (Atom,Atom) Atom
handleXU = Iso f g where
    f (a,e) = do
        xus <- gets sXU
        if a `elem` xus
           then pure $ Link "SatisfactionLink" [e] noTv
           else pure e
    g (Link "SatisfactionLink" [e] _) = pure (cCN "dummy" noTv,e)
    g e                               = pure (cCN "dummy" noTv,e)

--Get the argument location of all Sumties
handleTAG :: SynIso [(Atom,Maybe String)] [(Atom,String)]
handleTAG = post . isoFoldl tagOne . init
    where startMap = M.fromList [("1",True),("2",True),("3",True),("4",True),("5",True)]
          init = mkIso f g where
              f a     = (([],("0",startMap)),a)
              g (_,a) = a
          post = mkIso f g where
              f (l,(_,_)) = l
              g l         = (l,(show $ length l,M.empty))
          -- r : Result List
          -- p : Current Possiton
          -- u : map of used positions
          -- a : atom
          -- s : position tag /ms : maybe s
          --tagOne :: ((r,(p,u)),(a,ms)) (r,(p,u))
          tagOne = mkIso f g where
              f ((r,(p,u)),(a,Just s))
                --Tags longer then one are special and don't need to be tracked
                | length s >  1 = ((a,s):r,(p,u))
                | length s == 1 = ((a,s):r,(s,M.update (\_ -> Just False) s u))
              --If there is no tag use the next avaible positon after the current
              f ((r,(p,u)),(a,Nothing)) =
                                  ((a,t):r,(t,M.update (\_ -> Just False) t u))
                    where next s = show (read s + 1)
                          t = findNext p
                          findNext s = let t = next s
                                       in case M.lookup t u of
                                           Just True  -> t
                                           Just False -> findNext t
                                           Nothing -> t --FIXME could this casue issues???
              g ((a,s):r,(p,u))
                | length s >  1 = ((r,(p     ,u)), (a,Just s ))
                | s == p        = ((r,(prev p,u)), (a,Nothing))
                | otherwise     = ((r,(prev p,u)), (a,Just s ))
                    where prev s = show (read s - 1 )

-- Create the EvaluationLink for a Selbr Sumti Pair
--        Iso       Selbri          Stumti       Atom
_frame :: SynIso (Selbri,(Atom,Tag)) Atom
_frame = handleXU . second (_evalTv . (second.first) _framePred) . reorder
    where reorder = mkIso f g
          f ((tv,s),(a,t))     = (a,(tv,(t,[s,a])))
          g (_,(tv,(t,[s,a]))) = ((tv,s),(a,t))

          --Given the Tag of the Sumti create the approriate frame Predicate
          _framePred :: SynIso Tag Atom
          _framePred = handleVar <+> specialTag <+> predicate . isoPrepend "sumti"
              where handleVar :: SynIso Tag Atom
                    handleVar = Iso f g where
                        f "?" = cVN <$> randName "?"
                        f _ = lift $ Left "Not a VariableTag"
                        g (VN name) = pure "?"
                        g a = lift $ Left "Not a VariableNode"

                    specialTag :: SynIso Tag Atom
                    specialTag = Iso f g where
                        f s = if "_sumti" `isInfixOf` s
                                 then pure (cPN s noTv)
                                 else lift $ Left "Not a special Tag."
                        g (PN s) = pure "fix_reverse_specialTag_handeling"

--Generate a random Name given some Salt inside our SynMonad
randName :: SynMonad t State => String -> (t ME) String
randName salt = do
    seed1 <- gets sSeed
    let (n,seed2) = randName' seed1 salt
    setSeed seed2
    pure n

--Generate a random Name given a Seed and some Salt
randName' :: Int -> String -> (String,Int)
randName' = ((take 20 . map chr . filter pred . randomRs (33,126)) *** (fst . random)) . split . mkStdGen ... hashWithSalt
    where pred i = (i >= 48 && i <= 57)
                    || (i >= 65 && i <= 90)
                    || (i >= 97 && i <= 122)


--Most pronouns are instances of a more general concept
--This will create the inheritance link to show this relation
instanceOf :: SynIso Atom Atom
instanceOf = genInstance "InheritanceLink"

iInstanceOf :: SynIso Atom Atom
iInstanceOf = genInstance "IntensionalInheritanceLink"

implicationOf ::  SynIso Atom Atom
implicationOf = genInstance "ImplicationLink"

genInstance :: String -> SynIso Atom Atom
genInstance typeL = Iso f g where
    f a = do
        let (t,name) = if "Link" `isSuffixOf` atomType a
                          then ("ConceptNode","")
                          else (atomType a,nodeName a)
        rndname <- randName $ show a
        let i = Node t (rndname  ++ "___" ++ name) noTv
            l = case a of
            --Variable Links require special handeling
            --For Bound variables like da/de/di the concept should inherit those
            --For Question Variables the other way around
                    VN name -> if name `elem` ["da","de","di"]
                                  then Link typeL [cCN (nodeName i) noTv,a] highTv
                                  else Link typeL [i,cCN name noTv] highTv
                    _       -> Link typeL [i,a] highTv
        pushAtom l
        pure i
    g n = do
        atoms <- gets sAtoms
        let ml = (\(Link _ [_,i] _) -> i) <$> F.find (ff n) atoms
        case ml of
            Just v -> pure v
            Nothing ->  lift $ Left $ show n ++ " is not an Instance"

    ff n (Link "InheritanceLink" [b,_] _) = n == b
    ff n a = False

--Remove all atoms from the State that are not used to define a
filterState :: SynIso Atom Atom
filterState = Iso f g where
    f   = pure
    g a = do
        modify (\s -> s {sAtoms = getDefinitions [a] (sAtoms s)})
        pure a


--Given a List of Nodes and a List of Links
--Find all links that are used to define the Nodes
getDefinitions :: [Atom] -> [Atom] -> [Atom]
getDefinitions ns ls = if ns == nns then links else getDefinitions nns ls
    where links = filter ff ls --Get all links that contain a node from ns
          --The node to be defined should be the first element of  a link
          ff (Link _ ls _) = (head ls) `elem` ns
          ff _ = False
          nodes = concatMap atomGetAllNodes links --Get all Nodes from the links
          nns   = nub $ ns ++ nodes --Remove duplicates

--Given a Set/Concept that is define via Set_Type/Size_Link find it's type
findSetType :: SynIso Atom (Either (Atom,Atom) Atom)
findSetType = Iso f g where
    f a = do
        atoms <- gets sAtoms
        case fmap getType $ F.find (pat a) atoms of
            Just t -> pure $ Left (t,a)
            Nothing -> pure $ Right a
    g (Left (t,a)) = pure a
    g (Right a) = pure a

    getType (Link "SetTypeLink" [s,t] _) = t

    pat a (Link "SetTypeLink" [s,t] _) = s == a
    pat _ _ = False

--Map the Iso over all Atoms in the Atom Graph
atomIsoMap :: SynIso Atom Atom -> SynIso Atom Atom
atomIsoMap iso = Iso f g where
  f (Link t ls tv) = do ls' <- apply (mapIso (atomIsoMap iso)) ls
                        apply iso $ Link t ls' tv
  f n@(Node _ _ _) = apply iso n
  g (Link t ls tv) = do ls' <- unapply (mapIso (atomIsoMap iso)) ls
                        unapply iso $ Link t ls' tv
  g n@(Node _ _ _) = unapply iso n

--Remove duplicates in a Link
atomNub :: SynIso Atom Atom
atomNub = atomIsoMap (mkIso f id) where
  f (Link t ls tv) = Link t (nub ls) tv
  f n@(Node _ _ _) = n
