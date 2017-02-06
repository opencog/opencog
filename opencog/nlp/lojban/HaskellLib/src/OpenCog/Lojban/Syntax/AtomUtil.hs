{-# LANGUAGE LambdaCase #-}
module OpenCog.Lojban.Syntax.AtomUtil where

import Prelude hiding (id,(.),(<*>),(<$>),pure,(*>),(<*),foldl)

import Control.Category (id,(.))
import Control.Isomorphism.Partial
import Control.Isomorphism.Partial.Derived
import Control.Isomorphism.Partial.Unsafe
import Text.Syntax

import qualified Data.Map as M
import qualified Data.Foldable as F
import Data.Maybe (fromJust)
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
_eval :: Iso (Atom,[Atom]) Atom
_eval = eval . tolist2 . second list

_evalTv :: Iso (TruthVal,(Atom,[Atom])) Atom
_evalTv = evalTv . second (tolist2 . second list)

_ctx :: Iso (Maybe Atom,Atom) Atom
_ctx = ((ctx . tolist2) ||| id) . ifJustA

_ctxold :: Iso (Atom,(Atom,[Atom])) Atom
_ctxold = ctx . tolist2 . second _eval

_ssl :: Iso Atom Atom
_ssl = ssl . tolist2 . addVarNode

addVarNode :: Iso Atom (Atom,Atom)
addVarNode = Iso (\a -> Just (Node "VariableNode" "$var" noTv,a))
                 (\(_,a) -> Just a)

_satl :: Iso ((String,Atom),[Atom]) Atom
_satl = Iso (\((_,a),s) -> let all = Link "ListLink" (a:s) noTv
                           in Just $ Link "SatisfactionLink" [all] noTv)
            (\case {(Link "SatisfactionLink" (a:s) _) -> Just (("xu",a),s)
                   ;_ -> Nothing})

_iil :: Iso (Atom,Atom) Atom
_iil = iil . tolist2

ctx :: Iso [Atom] Atom
ctx = linkIso "ContextLink" noTv

eval :: Iso [Atom] Atom
eval = linkIso "EvaluationLink" noTv

evalTv :: Iso (TruthVal,[Atom]) Atom
evalTv = linkIso2 "EvaluationLink"

ssl :: Iso [Atom] Atom
ssl = linkIso "SatisfyingSetLink" noTv

setTypeL  :: Iso [Atom] Atom
setTypeL = linkIso "SetTypeLink" noTv

sizeL  :: Iso [Atom] Atom
sizeL = linkIso "SizeLink" noTv

iil :: Iso [Atom] Atom
iil = linkIso "IntensionalImplicationLink" noTv

list :: Iso [Atom] Atom
list = linkIso "ListLink" noTv

varl :: Iso [Atom] Atom
varl = linkIso "VariableLink" noTv

notl :: Iso [Atom] Atom
notl = linkIso "NotLink" noTv

andl :: Iso [Atom] Atom
andl = linkIso "AndLink" noTv

orl :: Iso [Atom] Atom
orl = linkIso "OrLink" noTv

iffl :: Iso [Atom] Atom
iffl = orl . tolist2 . (andl *** andl) . reorder
    where reorder = Iso (Just . f) (Just . g)
          f ls     = (ls,map (fromJust . apply (notl . tolist1)) ls)
          g (ls,_) = ls

limpl :: Iso [Atom] Atom
limpl = orl . tolist2 . (andl *** andl) . reorder
    where reorder = Iso (Just . f) (Just . g)
          f [a,b] = let nb = fromJust $ apply notl [b]
                    in ([a,b],[a,nb])
          f a = error $ show a ++ " is not a accepted value for limpl"
          g (ls,_) = ls

conLink :: Iso (String,[Atom]) Atom
conLink = Iso (\(s,args) -> case s of
                             "e"  -> apply andl args
                             "a"  -> apply orl args
                             "o"  -> apply iffl args
                             "u"  -> apply limpl args
                             "ji" -> apply varl args
                             _    -> error $ "Can't handle conLink: " ++ show s)
              (\a -> case a of
                        Link "OrLink"
                            [Link "AndLink" args _
                            ,Link "AndLink"
                                [Link "NotLink" _arg1 _
                                ,Link "NotLink" _arg2 _
                                ]_
                            ] _ -> Just ("o",args)
                        Link "OrLink"
                            [Link "AndLink" args _
                            ,Link "AndLink"
                                [arg1
                                ,Link "NotLink" _arg2 _
                                ]_
                            ] _ -> Just ("u",args)
                        Link "AndLink" args _ -> Just ("e",args)
                        Link "OrLink" args _ -> Just ("a",args)
                        Link "VariableLink" args _ -> Just ("ji",args)
                        _ -> Nothing)

conLinkJA :: Iso (String,[Atom]) Atom
conLinkJA = conLink .< _JAtoA

conLinkGIhA :: Iso (String,[Atom]) Atom
conLinkGIhA = conLink .< _GIhAtoA

_JAtoA :: Iso String String
_JAtoA = mkSynonymIso [("je","e")
                      ,("ja","a")
                      ,("jo","o")
                      ,("ju","u")
                      ,("je'i","ji")]

_GIhAtoA :: Iso String String
_GIhAtoA = mkSynonymIso [("gi'e","e")
                        ,("gi'a","a")
                        ,("gi'o","o")
                        ,("gi'u","u")
                        ,("gi'i","ji")]

_GAtoA :: Iso String String
_GAtoA = mkSynonymIso [("ge","e")
                      ,("ga","a")
                      ,("go","o")
                      ,("gu","u")
                      ,("ge'i","ji")]



linkIso :: String -> TruthVal -> Iso [Atom] Atom
linkIso n t = link . Iso (\l -> Just (n,(l,t)))
                         (\(an,(l,at)) -> if an == n
                                           then Just l
                                           else Nothing)

linkIso2 :: String -> Iso (TruthVal,[Atom]) Atom
linkIso2 n = link . Iso (\(t,l) -> Just (n,(l,t)))
                        (\(an,(l,t)) -> if an == n
                                          then Just (t,l)
                                          else Nothing)

nodeIso :: String -> TruthVal -> Iso String Atom
nodeIso n t = node . Iso (\l -> Just (n,(l,t)))
                         (\(an,(l,at)) -> if an == n
                                           then Just l
                                           else Nothing)


concept :: Iso String Atom
concept = nodeIso "ConceptNode" noTv

wordNode :: Iso String Atom
wordNode = nodeIso "WordNode" noTv

predicate :: Iso String Atom
predicate = nodeIso "PredicateNode" noTv

varnode :: Iso String Atom
varnode = nodeIso "VariableNode" noTv

number :: Iso String Atom
number = nodeIso "VariableNode" noTv


_frames :: Iso (Tagged Selbri,[Sumti]) Atom
_frames = (id ||| andl) . isSingle . mapIso _frame . isoDistribute . handleTAG
    where isSingle = Iso (Just . f) (Just . g)
          f [a] = Left a
          f as  = Right as
          g (Left a) = [a]
          g (Right as) = as

handleTAG :: Iso (Tagged Selbri,[Sumti]) (Selbri,[(Atom,Tag)])
handleTAG = handleTAGupdater . second tagger
    where handleTAGupdater = Iso (Just . f) (Just . g)
          f ((s,Nothing),args) = (s,args)
          f ((s,Just u) ,args) = (s,map (mapf u) args)
          g (s,args)           = ((s,Nothing),args)
          mapf u = mapSnd $ fromJust . apply (tagUpdater u)

tagUpdater :: String -> Iso Tag Tag
tagUpdater "se" = try $ mkSynonymIso [("1","2"),("2","1")]
tagUpdater "te" = try $ mkSynonymIso [("1","3"),("3","1")]
tagUpdater "ve" = try $ mkSynonymIso [("1","4"),("4","1")]
tagUpdater "xe" = try $ mkSynonymIso [("1","5"),("5","1")]

--Get the argumetn location of all Sumties
tagger :: Iso [(Atom,Maybe String)] [(Atom,String)]
tagger = post . foldl tagOne . init
    where init = Iso (\a     -> Just (([],("0",startMap)),a))
                     (\(_,a) -> Just a)
          startMap = M.fromList [("1",True),("2",True),("3",True),("4",True),("5",True)]
          post = Iso (\(l,(_,_)) -> Just l)
                     (\l         -> Just (l,(show $ length l,M.empty)))
          tagOne = Iso (Just . f) (Just . g)
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
_frame :: Iso ((TruthVal,Atom),(Atom,Tag)) Atom
_frame = _evalTv . (id *** (_framePred *** tolist2)) . reorder
    where reorder = Iso f g
          f ((tv,s),(a,t))      = Just (tv,((s,t),(s,a)))
          g (tv,((_,t),(s,a)))  = Just ((tv,s),(a,t))

_framePred :: Iso (Atom,Tag) Atom
_framePred = handleVar $ node . second (first (isoConcat "_sumti". tolist2 .< isoDrop 23)) . reorder .< (inverse node)
    where reorder = Iso (Just . f) (Just . g) where
                f ((t,(n,tv)),tag) = (t,((n,tag),tv))
                g (t,((n,tag),tv)) = ((t,(n,tv)),tag)
          handleVar iso = Iso f g where
              f (n,"?") = Just $ cVN (nodeName n)
              f a = apply iso a
              g (VN name) = Just (cPN name noTv,"$var")
              g a = unapply iso a

randName :: Int -> String -> String
randName = take 20 . map chr . randomRs (33,126) . mkStdGen ... hashWithSalt

--Most pronouns are instances of a more general concept
--This will create the inheritance link to show this relation
instanceOf :: Iso (Atom,Int) (State Atom)
instanceOf = genInstance "InheritanceLink"

iInstanceOf :: Iso (Atom,Int) (State Atom)
iInstanceOf = genInstance "IntensionalInheritanceLink"

implicationOf ::  Iso (Atom,Int) (State Atom)
implicationOf = genInstance "ImplicationLink"

genInstance :: String -> Iso (Atom,Int) (State Atom)
genInstance typeL = Iso f g where
    f (e,seed) = let salt = show e
                     (t,name) = if "Link" `isSuffixOf` atomType e
                            then ("ConceptNode","")
                            else (atomType e,nodeName e)
                     fullname = (randName seed salt) ++ "___"++ name
                     i = Node t fullname noTv
                     l = Link typeL [i,e] highTv
                 in Just (i,[l])
    g (n,ls) = (\(Link _ [_,i] _) -> (i,0)) `fmap` F.find (ff n) ls
    ff n (Link "InheritanceLink" [b,_] _) = n == b
    ff n a = False

filterState :: Iso (State Sumti) (State Sumti)
filterState = Iso f g where
    f           = apply id
    g ((a,t),s) = Just ((a,t),getDefinitons [a] s)

getDefinitons :: [Atom] -> [Atom] -> [Atom]
getDefinitons ns ls = if ns == nns then links else getDefinitons nns ls
    where links = filter ff ls
          nodes = concatMap atomGetAllNodes links
          nns   = nub $ ns ++ nodes
          ff l = any (`atomElem` l) ns
