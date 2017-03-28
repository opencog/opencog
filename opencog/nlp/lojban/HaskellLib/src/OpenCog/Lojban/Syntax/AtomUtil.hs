{-# LANGUAGE LambdaCase #-}
{-# LANGUAGE FlexibleContexts           #-}
module OpenCog.Lojban.Syntax.AtomUtil where

import Prelude hiding (id,(.))

import Control.Category (id,(.))
import Control.Monad.Trans.Class
import Control.Monad.RWS

import Iso hiding (SynIso,Syntax)

import qualified Data.Map as M
import qualified Data.Foldable as F
import Data.Maybe (fromJust,isJust)
import Data.List (isSuffixOf,nub)
import Data.Hashable
import Data.Char (chr)

import System.Random

import OpenCog.AtomSpace (Atom(..),noTv,TruthVal(..),stv,atomType
                         ,atomGetAllNodes,atomElem,nodeName)

import OpenCog.Lojban.Syntax.Types
import OpenCog.Lojban.Syntax.Util
import OpenCog.Lojban.Util

--Various semi-isos to easily transfrom Certain Atom types
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

_exl :: SynIso (Atom, Atom) Atom
_exl = exl . tolist2

_meml :: SynIso (Atom, Atom) Atom
_meml = meml . tolist2

_equivl :: SynIso (Atom, Atom) Atom
_equivl = equivl . tolist2

_typedvarl :: SynIso (Atom, Atom) Atom
_typedvarl = typedvarl . tolist2

_satl :: SynIso (String,Atom) Atom
_satl = satl . tolist1 . listl . consAtoms . rmfst "xu"

satl :: SynIso [Atom] Atom
satl = linkIso "SatisfactionLink" noTv

_iil :: SynIso (Atom,Atom) Atom
_iil = iil . tolist2

ctx :: SynIso [Atom] Atom
ctx = linkIso "ContextLink" noTv

eval :: SynIso [Atom] Atom
eval = linkIso "EvaluationLink" noTv


--Iso Atom Atom
--eval . node x . listl .a pred . addAsnd arg

--addAsnd :: Iso c b -> c -> Iso a (a,b)
--addAsnd iso c = iso >. addsnd c

--addAfst :: Iso c b -> c -> Iso a (b,a)
--addAfst iso c = iso <. addfst c

--(.a) :: Iso [a] a -> Iso b (a,a) -> Iso b a
--(.a) iso1 iso2 = iso1 . tolist2 iso2

evalTv :: SynIso (TruthVal,[Atom]) Atom
evalTv = linkIso2 "EvaluationLink"

ssl :: SynIso [Atom] Atom
ssl = linkIso "SatisfyingSetLink" noTv

exl :: SynIso [Atom] Atom
exl = linkIso "ExistsLink" noTv

meml :: SynIso [Atom] Atom
meml = linkIso "MemberLink" noTv

equivl :: SynIso [Atom] Atom
equivl = linkIso "EquivalenceLink" noTv

setTypeL  :: SynIso [Atom] Atom
setTypeL = linkIso "SetTypeLink" noTv

subsetL :: SynIso (Atom,Atom) Atom
subsetL = linkIso "SubSetLink" noTv . tolist2

sizeL  :: SynIso [Atom] Atom
sizeL = linkIso "SetSizeLink" noTv

iil :: SynIso [Atom] Atom
iil = linkIso "IntensionalImplicationLink" noTv

listl :: SynIso [Atom] Atom
listl = linkIso "ListLink" noTv

--varl :: SynIso [Atom] Atom
--varl = linkIso "VariableLink" noTv

typedvarl :: SynIso [Atom] Atom
typedvarl = linkIso "TypedVariableLink" noTv

varll :: SynIso [Atom] Atom
varll = linkIso "VariableListLink" noTv

notl :: SynIso [Atom] Atom
notl = linkIso "NotLink" noTv

andl :: SynIso [Atom] Atom
andl = linkIso "AndLink" noTv

orl :: SynIso [Atom] Atom
orl = linkIso "OrLink" noTv

exel :: SynIso [Atom] Atom
exel = linkIso "ExecutionLink" noTv

definedSchemaLink :: String -> SynIso [Atom] Atom
definedSchemaLink s = exel . tolist2 . addfst dsn . listl
    where dsn = Node "DefinedSchemaNode" s noTv

iffl :: SynIso [Atom] Atom
iffl = definedSchemaLink "IfAndOnlyIf"

uL :: SynIso [Atom] Atom
uL = definedSchemaLink "WetherOrNot"

anotbl :: SynIso [Atom] Atom
anotbl = andl . tolist2 . second (notl . tolist1) . inverse tolist2

onlyif :: SynIso [Atom] Atom
onlyif = orl . tolist2 . (andl *** notl) . reorder
    where reorder = mkIso f g
          f [a,b] = ([a,b],[a])
          g (a,_) = a

xorl :: SynIso [Atom] Atom
xorl = orl . tolist2
     . (myand . first mynot *** myand . second mynot)
     . reorder
    where reorder = Iso (pure . f) (pure . g)
          f [a,b] = ((a,b),(a,b))
          g ((a,b),_) = [a,b]
          myand = andl .tolist2
          mynot = notl . tolist1


handleConNeg :: SynIso (LCON,[Atom]) (String,[Atom])
handleConNeg = Iso (pure . f) (pure . g)
    where f ((mna,(s,mnai)),[a1,a2]) = let na1 = if isJust mna
                                                 then cNL noTv a1
                                                 else a1
                                           na2 = if isJust mnai
                                                 then cNL noTv a2
                                                 else a2
                                       in (s,[na1,na2])
          g (s,[na1,na2]) = let (mna,a1) = case na1 of
                                        (NL [a1]) -> (Just "na",a1)
                                        _ -> (Nothing,na1)
                                (mnai,a2) = case na2 of
                                        (NL [a2]) -> (Just "nai",a2)
                                        _ -> (Nothing,na2)
                            in ((mna,(s,mnai)),[a1,a2])


conLink :: SynIso (LCON,[Atom]) Atom
conLink = conLink' . handleConNeg

conLink' :: SynIso (String,[Atom]) Atom
conLink' = choice conHandlers
    where conHandlers = [andl   . rmfst "e"
                        ,orl    . rmfst "a"
                        ,iffl   . rmfst "o"
                        ,uL     . rmfst "u"
                --FIXME:,varl   . rmfst "ji"
                        ,anotbl . rmfst "enai"
                        ,xorl   . rmfst "onai"
                        ,onlyif . rmfst "na.a"
                        ]

_JAtoA :: SynIso String String
_JAtoA = mkSynonymIso [("je","e")
                      ,("ja","a")
                      ,("jo","o")
                      ,("ju","u")
                      ,("jonai","onai")
                      ,("jenai","enai")
                      ,("naja","na.a")
                      ,("je'i","ji")]

_GIhAtoA :: SynIso String String
_GIhAtoA = mkSynonymIso [("gi'e","e")
                        ,("gi'a","a")
                        ,("gi'o","o")
                        ,("gi'u","u")
                        ,("gi'enai","enai")
                        ,("gi'onai","onai")
                        ,("nagi'a","na.a")
                        ,("gi'i","ji")]

_GAtoA :: SynIso String String
_GAtoA = mkSynonymIso [("ge","e")
                      ,("ga","a")
                      ,("go","o")
                      ,("gu","u")
                      ,("ganai","na.a")
                      ,("gonai","onai")
                      ,("ge'i","ji")]



linkIso :: String -> TruthVal -> SynIso [Atom] Atom
linkIso n tv = Iso f g where
    f as = pure $ Link n as tv
    g (Link t o _) --TODO: should we also check the tv?
        | t == n = pure o
        | otherwise = lift $ Left $ "Wrong LinkType was expecting " ++ n ++
                                    " but got " ++ t

linkIso2 :: String -> SynIso (TruthVal,[Atom]) Atom
linkIso2 n  = Iso f g where
    f (tv,as) = pure $ Link n as tv
    g (Link t o tv)
        | t == n = pure (tv,o)
        | otherwise = lift $ Left $ "Wrong LinkType was expecting " ++ n ++
                                    " but got " ++ t

nodeIso :: String -> TruthVal -> SynIso String Atom
nodeIso t tv = Iso f g where
    f n = pure $ Node t n tv
    g (Node tt n _) --TODO: should we also check the tv?
        | t == tt = pure n
        | otherwise = lift $ Left $ "Wrong LinkType was expecting " ++ t ++
                                    " but got " ++ tt

concept :: SynIso String Atom
concept = nodeIso "ConceptNode" noTv

wordNode :: SynIso String Atom
wordNode = nodeIso "WordNode" noTv

predicate :: SynIso String Atom
predicate = nodeIso "PredicateNode" noTv

varnode :: SynIso String Atom
varnode = nodeIso "VariableNode" noTv

number :: SynIso String Atom
number = nodeIso "VariableNode" noTv


_frames :: SynIso (Tagged Selbri,[Sumti]) Atom
_frames = (id ||| andl) . isSingle . mapIso (handleDA . _frame) . isoDistribute . handleTAG
    where isSingle = mkIso f g
          f [a] = Left a
          f as  = Right as
          g (Left a) = [a]
          g (Right as) = as

handleDA :: SynIso Atom Atom
handleDA = mkIso f g where
    f (EvalL tv ps (LL [p1,CN n]))
        | n == "da" || n == "de" || n == "di"
            = let i = cVN (randName 0 (show p1) ++ "___" ++ n)
              in cExL tv i (cEvalL tv ps (cLL [p1,i]))
    f a = a
    g (ExL _ _ (EvalL tv ps (LL [p1,VN name])))
        = let n = drop 23 name
              da = cCN n lowTv
          in cEvalL tv ps (cLL [p1,da])
    g a = a

handleTAG :: SynIso (Tagged Selbri,[Sumti]) (Selbri,[(Atom,Tag)])
handleTAG = handleTAGupdater . second tagger
    where handleTAGupdater = mkIso f g
          f ((s,Nothing),args) = (s,args)
          f ((s,Just u) ,args) = (s,map (mapf u) args)
          g (s,args)           = ((s,Nothing),args) --TODO: Should we really just ingore that?
          mapf = mapSnd . tagUpdater

tagUpdater :: String -> (Tag -> Tag)
tagUpdater t = case t of
                    "se" -> f [("1","2"),("2","1")]
                    "te" -> f [("1","3"),("3","1")]
                    "ve" -> f [("1","4"),("4","1")]
                    "xe" -> f [("1","5"),("5","1")]
    where f ls e = maybe e snd $ F.find (\(a,b) -> a == e) ls

--Get the argumetn location of all Sumties
tagger :: SynIso [(Atom,Maybe String)] [(Atom,String)]
tagger = post . isoFoldl tagOne . init
    where startMap = M.fromList [("1",True),("2",True),("3",True),("4",True),("5",True)]
          init = mkIso f g where
              f a     = (([],("0",startMap)),a)
              g (_,a) = a
          post = mkIso f g where
              f (l,(_,_)) = l
              g l         = (l,(show $ length l,M.empty))
          tagOne = mkIso f g where
              f ((r,(p,u)),(a,Just s))
                | length s >  1 = ((a,s):r,(p,u))
                | length s == 1 = ((a,s):r,(s,M.update (\_ -> Just False) s u))
              f ((r,(p,u)),(a,Nothing)) =
                                  ((a,t):r,(t,M.update (\_ -> Just False) t u))
                    where next s = show (read s + 1)
                          t = findNext p
                          findNext s = let t = next s
                                       in if u M.! t then t else findNext t
              g ((a,s):r,(p,u))
                | length s >  1 = ((r,(p     ,u)), (a,Just s ))
                | s == p        = ((r,(prev p,u)), (a,Nothing))
                | otherwise     = ((r,(prev p,u)), (a,Just s ))
                    where prev s = show (read s - 1 )

--        Iso       Selbri          Stumti       Atom
_frame :: SynIso (Selbri,(Atom,Tag)) Atom
_frame = _evalTv . (id *** (_framePred *** tolist2)) . reorder
    where reorder = mkIso f g
          f ((tv,s),(a,t))     = (tv,((s,t),(s,a)))
          g (tv,((_,t),(s,a))) = ((tv,s),(a,t))

node :: SynIso (String,(String,TruthVal)) Atom
node = mkIso f g where
    f (t,(n,tv))    = Node t n tv
    g (Node t n tv) = (t,(n,tv))

_framePred :: SynIso (Atom,Tag) Atom
_framePred = handleVar $ node . second (first (isoConcat "_sumti". tolist2 .< isoDrop 23)) . reorder .< inverse node
    where reorder = mkIso f g where
                f ((t,(n,tv)),tag) = (t,((n,tag),tv))
                g (t,((n,tag),tv)) = ((t,(n,tv)),tag)
          handleVar iso = Iso f g where
              f (n,"?") = pure $ cVN (nodeName n)
              f a = apply iso a
              g (VN name) = pure (cPN name noTv,"$var")
              g a = unapply iso a

randName :: Int -> String -> String
randName = take 20 . map chr . randomRs (33,126) . mkStdGen ... hashWithSalt

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
        seed <- asks seed
        let salt = show a
            (t,name) = if "Link" `isSuffixOf` atomType a
                   then ("ConceptNode","")
                   else (atomType a,nodeName a)
            fullname = randName seed salt ++ "___"++ name
            i = Node t fullname noTv
            l = Link typeL [i,a] highTv
        modify (\s -> s {sAtoms = l : sAtoms s})
        pure i
    g n = do
        atoms <- gets sAtoms
        let ml = (\(Link _ [_,i] _) -> i) <$> F.find (ff n) atoms
        case ml of
            Just v -> pure v
            Nothing ->  lift $ Left $ show n ++ " is not an Instance"

    ff n (Link "InheritanceLink" [b,_] _) = n == b
    ff n a = False

filterState :: SynIso Sumti Sumti
filterState = Iso f g where
    f       = pure
    g (a,t) = do
        modify (\s -> s {sAtoms = getDefinitions [a] (sAtoms s)})
        pure (a,t)


getDefinitions :: [Atom] -> [Atom] -> [Atom]
getDefinitions ns ls = if ns == nns then links else getDefinitions nns ls
    where links = filter ff ls --Get all links that contain a node from ns
          ff l = any (`atomElem` l) ns --Check if the link contains a node from ns
          nodes = concatMap atomGetAllNodes links --Get all Nodes from the links
          nns   = nub $ ns ++ nodes --Remove duplicates

findSetType :: Atom -> [Atom] -> Maybe Atom
findSetType a = fmap getType . F.find f
    where f (Link "SetTypeLink" [s,t] _) = s == a
          f _ = False
          getType (Link "SetTypeLink" [s,t] _) = t
