{-# LANGUAGE NoMonomorphismRestriction  #-}
{-# LANGUAGE RelaxedPolyRec             #-}
{-# LANGUAGE LambdaCase                 #-}
{-# LANGUAGE TypeFamilies               #-}
{-# LANGUAGE RankNTypes #-}

module OpenCog.Lojban.Syntax where

import Prelude hiding (id,(.),(<*>),(<$>),pure,(*>),(<*),foldl)

import qualified Data.List.Split as S
import Data.Maybe (fromJust)
import Data.Hashable

import System.Random

import Control.Monad.Trans.Reader
import Control.Category (id,(.))
import Control.Isomorphism.Partial
import Control.Isomorphism.Partial.Derived
import Control.Isomorphism.Partial.Unsafe
import Text.Syntax

import OpenCog.AtomSpace (Atom(..),TruthVal(..),noTv,stv,atomFold,nodeName)
import OpenCog.Lojban.Util

import OpenCog.Lojban.Syntax.Types
import OpenCog.Lojban.Syntax.Util
import OpenCog.Lojban.Syntax.AtomUtil

import Debug.Trace

mytrace a = traceShow a a
mytrace2 s a = trace (s ++(' ':show a)) a

--TODO
-- FIX: pa
--      da
--      is GIhA NAI correct?
--      ZAhO explicit representation
--      me'au
--      leP inner qunantifier
--      tenseSumti
--      nup
--      GOhA
--      vo'a rule see ../Lojban.hs

-- argslots
-- Synonims for tenses and gismu

--Post parser
-- Multi Sentences Parsing
-- ki
-- references


-- ConceptAnd/OrLink isntead of boolea and/or

---------
--General
---------

_con_BO :: SyntaxReader String -> SyntaxReader (String,Maybe(Tagged Selbri))
_con_BO ss = (id *** isofmap toSelbri) <$> ss <*> optional (_anytense <* sepSelmaho "BO")
    where toSelbri = Iso (Just . f) (Just . g) where
              f (CN name) = ((highTv,cPN name lowTv),Nothing)
              g ((_,PN n),_) = cCN n lowTv
-------------------------------------------------------------------------------
--Sumti
-------------------------------------------------------------------------------

_LE = reorder0 <$> selmaho "LE"

--Handles anykind of LE phrases
-- State ((le,(s,(v,[a]))),Int)
-- State (le,((s,(v,[a])),Int))

-- State ((le,(v,(s,[a]))),Int)
-- State (le,((v,(s,[a])),Int))
leP :: SyntaxReader (State Atom)
leP = collapsState
      .<
      (choice leHandlers .> first (_ssl . handleBRIDI))
      .
      inverse associate
      <$> withSeedState (_LE
                      <&> (first tolist1 <$> pureVarNode)
                      <&> selbriAll
                      <&> beP
                      <* optSelmaho "KU")
    where pureVarNode :: SyntaxReader (State Sumti)
          pureVarNode = pure ((Node "VariableNode" "$var" noTv,Nothing),[])

          --Handles addtional arguments in a leP
          beP :: SyntaxReader (State [Sumti])
          beP = (first cons <$>            sepSelmaho "BE"  *> sumtiAllUI
                            <&> stateMany (sepSelmaho "BEI" *> sumtiAllUI)
                                        <* optSelmaho "BEhO"
                ) <|> (stateMany sumtiAllUI)

          handleBE :: Iso (Sumti,Maybe [Sumti]) [Sumti]
          handleBE = (cons ||| tolist1) . ifJustB

leHandlers = [("le"  , genInstance "IntensionalInheritanceLink")
             ,("lo"  , genInstance "InheritanceLink"           )
             ,("lei" , massOf "IntensionalInheritanceLink"     )
             ,("loi" , massOf "InheritanceLink"                )
             ,("le'i", setOf "IntensionalInheritanceLink"      )
             ,("lo'i", setOf "InheritanceLink"                 )
             ,("le'e", genInstance "IntensionalInheritanceLink")
             ,("lo'e", genInstance "InheritanceLink"           )
             ]

massOf :: String -> Iso (Atom,Int) (State Atom)
massOf itype = instanceOf =. (first (_ssl . _frames) . addStuff <. (implicationOf *^* genInstance itype)) . reorder

    where pred = ((noTv,Node "PredicateNode" "gunma" noTv),tag)
          pp = Node "PredicateNode" "gunma" noTv
          arg1 = (Node "VariableNode" "$var" noTv,tag)
          tag = Nothing

          reorder = Iso (Just . f) (Just . g) where
              f (a,i)     = (((pp,i),(a,i)),[])
              g ((_,m),_) = m

          addStuff = Iso (Just . f) (Just . g) where
              f (p,a) = ((((noTv,p),tag),[arg1,(a,tag)]),0)
              g ((((_,p),_),[_,(a,_)]),_) = (p,a)


setOf :: String -> Iso (Atom,Int) (State Atom)
setOf itype = setTypeL . tolist2 . makeSet <. genInstance itype
    where makeSet = Iso (Just . f) (Just . g)
              where f a = let salt = show a
                              set  = cCN (randName 1 salt) noTv
                          in (set,a)
                    g (_,a) = a

--Handle anykind of LA phrase
laP :: SyntaxReader (State Atom)
laP = handleName . first wordNode <$> withSeed (between (sepSelmaho "LA")
                                                        (optSelmaho "KU")
                                                        anyWord
                                               )
    where handleName :: Iso (Atom,Int) (State Atom)
          handleName = Iso (\(a,i) -> let na = nodeName a
                                          name = (randName i na) ++ "___"++ na
                                          c = cCN name lowTv
                                          pred = cPN "cmene" lowTv
                                          Just (p,ps) = apply instanceOf (pred,i)
                                          ct = (c,Nothing)
                                          at = (a,Nothing)
                                          pt = ((highTv,p),Nothing)
                                          (Just l) = apply _frames (pt,[at,ct])
                                      in Just (c,[l]))
                           (\(_,[EvalL _ _ (LL [a,_])]) -> Just (a,0))



liP :: SyntaxReader (Atom,[Atom])
liP = sepSelmaho "LI" *> (xo <|> pa) <* optSelmaho "LOhO"

xo :: SyntaxReader (State Atom)
xo = reorder0 . varnode <$> word "xo"

pa :: SyntaxReader (State Atom)
pa = reorder0
    . (         number       |||    concept   )
    . (showReadIso . paToNum |^| isoConcat " ")
    . isoConcat2 <$> many1 (joiSelmaho "PA")
    where paToNum :: Iso [String] Int
          paToNum = foldl (digitsToNum . second paToDigit) . addfst 0

          digitsToNum :: Iso (Int,Int) Int
          digitsToNum = Iso f g where
              f (i,j) = Just (i*10+j)
              g 0     = Nothing
              g n     = Just (n `div` 10,n `mod` 10)

          paToDigit :: Iso String Int
          paToDigit = mkSynonymIso [("no",0),("pa",1)
                                   ,("re",2),("ci",3)
                                   ,("vo",4),("mu",5)
                                   ,("xa",6),("ze",7)
                                   ,("bi",8),("so",9)]


zoP :: SyntaxReader (State Atom)
zoP = instanceOf .< wordNode . ciunit <$> withSeed (mytext "zo" <*> anyWord)

--KohaPharse for any kind of Pro-Noune
kohaP :: SyntaxReader (Atom,[Atom])
kohaP = reorder0 <$> (ma <|> koha)
    where koha = concept <$> selmaho "KOhA"
          ma   = varnode <$> word "ma"

luP' :: SyntaxReader Atom
luP' = sepSelmaho "LU" *> lojban <* optSelmaho "LIhU"

luP :: SyntaxReader (Atom,[Atom])
luP = reorder0 <$> luP'

sumti :: SyntaxReader (State Atom)
sumti = kohaP <|> leP <|> laP <|> liP <|> zoP <|> luP

--This Handles relative phrases
noiP :: SyntaxReader (State Atom)
noiP = sepSelmaho "NOI" *> bridi <* optSelmaho "KUhO"

noi_short :: SyntaxReader (State Atom)
noi_short = sepSelmaho "NOI" *> ptp selbriUI addKEhA bridi <* optSelmaho "KUhO"
    where addKEhA = Iso (Just . f) (Just . g)
          f s = "ke'a " ++ s
          g s = drop 4 s

goiP :: SyntaxReader (State Atom)
goiP = ptp (selmaho "GOI") goiToNoi noiP
    where goiToNoi = mkSynonymIso [("pe "  ,"poi ke'a srana ")
                                  ,("po "  ,"poi ke'a se steci srana ")
                                  ,("po'e ","poi jinzi ke se steci srana ")
                                  ,("po'u ","poi ke'a du ")
                                  ,("ne "  ,"noi ke'a srana ")
                                  ,("no'u ","noi ke'a du ")
                                  ,("goi " ,"poi ke'a du ")]

sumtiNoi :: SyntaxReader (State Atom)
sumtiNoi = (kehaToSesku  ||| id) . reorder <$> sumti
                                           <&> optState (noiP <|> noi_short <|> goiP)
    where reorder = Iso (Just . f) (Just . g)
          f ((a,Just n ),s) = Left (a,n:s)
          f ((a,Nothing),s) = Right (a,s)
          g (Left (a,n:s))  = ((a,Just n ),s)
          g (Right (a,s))   = ((a,Nothing),s)
          --If we have a relative clause
          --this will switch the ke'a with the actually concept
          --TODO: Do we always have a ke'a in a relative clause?
          kehaToSesku :: Iso (State Atom) (State Atom)
          kehaToSesku = Iso f g
              where f (c,l) = let nl = map (switch kea c) l
                              in if nl /= l
                                  then Just (c,nl)
                                  else Nothing
                    g (c,l) = let nl = map (switch c kea) l
                              in if nl /= l
                                  then Just (c,nl)
                                  else Nothing
                    kea = Node "ConceptNode" "ke'a" noTv
                    switch a b l@(InhL [x,c] tv) = if c == a
                                                  then cInhL tv x b
                                                  else l
                    switch _ _ l = l

sumtiLaiP :: SyntaxReader (State Atom)
sumtiLaiP = ptp (pa <*> selbri) id (ptp pa addLE sumtiLai) <|> sumtiLai
    where addLE = Iso (Just . f) (Just . g)
          f s = s ++ " lo "
          g s = take (length s - 4) s

sumtiLai :: SyntaxReader (State Atom)
sumtiLai = collapsState
           . first ((instanceWithSize ||| instanceOf) . reorder)
           <$> withSeedState (optState pa <&> sumtiNoi)

    where reorder = Iso (Just . f) (Just . g)
              where f ((Just pa,sumti),seed)   = Left ((pa,seed),sumti)
                    f ((Nothing,sumti),seed)   = Right (sumti,seed)
                    g (Left ((pa,seed),sumti)) = ((Just pa,sumti),seed)
                    g (Right (sumti,seed))     = ((Nothing,sumti),seed)

          instanceWithSize :: Iso ((Atom,Int),Atom) (State Atom)
          instanceWithSize = second (tolist2 . (sizeL *** setTypeL)) . makeSet
              where makeSet = Iso (Just . f) (Just . g)
                    f ((a,i),b) = let salt = show a ++ show b
                                      set  = cCN (randName i salt) noTv
                                  in (set,([set,a],[set,b]))
                    g (_,([_,a],[_,b])) = ((a,0),b)



--Connective

_A_BO :: SyntaxReader (State (String,Maybe (Tagged Selbri)))
_A_BO = reorder0 <$> _con_BO (selmaho "A")

--AndLink
--  AndLink
--  EvaluationLink
--      PredicateNode "tense"
--      ListLink

--This Handels Sumti connected by logical connectives
sumtiC :: SyntaxReader (State Atom)
sumtiC = first ((handleCon . reorder ||| id) . ifJustB)
        <$> sumtiLaiP <&> optState (_A_BO <&> sumtiC)
    where reorder = Iso (\(a1,(con,a2)) -> Just (con,[a1,a2]))
                        (\(con,[a1,a2]) -> Just (a1,(con,a2)))

handleCon :: Iso ((String,Maybe (Tagged Selbri)),[Atom]) (Atom)
handleCon = (andl . tolist2 . (conLink *** _frames) ||| conLink) . reorder
    where reorder = Iso (Just . f) (Just . g) where
            f ((s,Just ts),as) = Left ((s,as),(ts,map (\x -> (x,Nothing)) as))
            f ((s,Nothing),as) = Right (s,as)
            g (Left ((s,as),(ts,_)))= ((s,Just ts),as)
            g (Right (s,as))        = ((s,Nothing),as)

placeholder :: SyntaxReader (State Sumti)
placeholder = expandState . first instanceOf . handle
            <$> withSeed (letter <*> digit <* sepSpace)
    where handle = Iso (Just . f) (Just . g)
          f ((c,d),i) = ((cCN [c,d] noTv,i),Just [d])
          g ((CN [c,_],i),Just [d]) = ((c,d),i)

sumtiT :: SyntaxReader (State Sumti)
sumtiT = (first handleFA <$> optState _FA
                         <&> sumtiC
         ) <|> placeholder
    where handleFA = commute . first (isofmap faToPlace)
          _FA = reorder0 <$> selmaho "FA"

          faToPlace :: Iso String String
          faToPlace = mkSynonymIso [("fa","1")
                                   ,("fe","2")
                                   ,("fi","3")
                                   ,("fo","4")
                                   ,("fu","5")
                                   ,("fi'a","?")]

modalSumti :: SyntaxReader (State Sumti)
modalSumti = reorder . first handleFIhO <$> (fihoP <|> baiP)
                                        <&> sumtiT
    where handleFIhO = (fi'otag &&& _frame) . second (inverse tolist1)
                                            . handleTAG . second tolist1
          fi'otag = Iso (Just . f) (Just . g)
              where f ((tv,PN name),(s,tag)) = (s,Just $ name++tag)
                    g (s,Just nametag) =
                        let [name,tag,tv] = S.split (S.oneOf "12345") nametag
                        in ((read tv,cPN name lowTv),(s,tag))
          reorder = Iso (\((a,b),c) -> Just (a,b: c))
                        (\ (a,b: c) -> Just ((a,b),c))

          fihoP :: SyntaxReader (State (Tagged Selbri))
          fihoP = sepSelmaho "FIhO" *> selbriUI <* optSelmaho "FEhU"

baiP :: SyntaxReader (State (Tagged Selbri))
baiP = ReaderT (\wl@(_,_,btf,_) -> runReaderT (ptp _bai btf selbriUI) wl)
  where _bai = optional (selmaho "SE") <*> selmaho "BAI"

tenseSumti :: SyntaxReader (State Sumti)
tenseSumti = first reorder <$> anytense <&> sumtiC <* optSelmaho "KU"
    where reorder = Iso (Just . f) (Just . g)
          f (t,s) = (s,Just $ nodeName t)
          g (s,Just n) = (cCN n noTv,s)

tenseSumtiP :: SyntaxReader (State Sumti)
tenseSumtiP = tenseSumti <|> ptp (anytense `withOut` selbriAll) addti tenseSumti
    where addti = Iso (Just . f) (Just . g)
          f s = s ++ "ti "
          g s = s --TODO: Maybe remoe ti again

sumtiAll :: SyntaxReader (State Sumti)
sumtiAll = filterState <$> modalSumti <|> sumtiT <|> tenseSumtiP

sumtiAllUI :: SyntaxReader (State Sumti)
sumtiAllUI = withAttitude sumtiAll

-------------------------------------------------------------------------------
--Selbri
-------------------------------------------------------------------------------

gismuP :: SyntaxReader (State Atom)
gismuP = implicationOf .< predicate <$> withSeed gismu

tanru :: SyntaxReader (State Atom)
tanru = handleTanru <$> gismuP
                    <&> optState tanru
    where handleTanru = (second (cons . first _iil) ||| id ) . reorder
          reorder = Iso (Just . f) (Just . g)
          f ((g,Just t),s)  = Left (g,((t,g),s))
          f ((g,Nothing),s) = Right (g,s)
          g (Left (g,((t,_),s)))  = ((g,Just t),s)
          g (Right (g,s))         = ((g,Nothing),s)


--meP :: SyntaxReader (State Atom)
--meP =  handleME <$> selmaho "ME" <*> sumtiAll <*> optSelmaho "MEhU"

nuP :: SyntaxReader (State Atom)
nuP = choice nuHandlers <$> selmaho "NU" <*> withText bridiPMI <* optSelmaho "KEI"

nuHandlers = [("nu"     ,handleNU)
             ,("du'u"   ,handleNU)
             ,("ka"     ,handleKA)]

handleNU :: Iso ((State Atom),String) (State Atom)
handleNU = Iso f g where
    f ((atom,as),name) = let pred = cPN name lowTv
                             link = mkCtxPre pred atom
                         in Just (pred,link:as)
    g (PN name,CtxPred atom : as) = Just ((atom,as),name)

handleKA :: Iso ((State Atom),String) (State Atom)
handleKA = Iso (Just . f) (Just . g) where
    f ((atom,as),name) = let pred = cPN name lowTv
                             link = mkPropPre pred atom name
                         in (pred,link:as)
    g (PN name,PropPred atom : as) = ((atom,as),name)


bridiPMI :: SyntaxReader (State Atom)
bridiPMI = (ptp (optional uiP <*> selbriAll) addmi bridiUI) <|> bridiUI <|> (ptp (optional anytense <*> selbriAll) addmi bridiUI)
    where addmi = Iso (Just . f) (Just . g)
          f s = s ++ "mi "
          g s = s --TODO: Maybe remoe ti again


_MOI :: SyntaxReader (State String)
_MOI = reorder0 <$> selmaho  "MOI"

moiP :: SyntaxReader (State Atom)
moiP = collapsState . first implicationOf .< first (predicate . handleMOI)
     <$> withSeedState (pa <&> _MOI)
    where handleMOI = Iso (Just . f) (Just . g)
          f (a,s) = let nn = nodeName a
                    in nn ++ '-':s
          g name  = let nn = takeWhile (/= '-') name
                        s  = drop 1 $ dropWhile (/= '-') name
                    in case isNumeric nn of
                        True  -> (fromJust $ apply number nn,s)
                        False -> (fromJust $ apply concept nn,s)

_ME = reorder0 <$> mytext "me"

meP :: SyntaxReader (State Atom)
meP = collapsState . first implicationOf .< first (predicate . handleME)
     <$> withSeedState (_ME <&> sumtiC)
    where handleME = Iso (Just . f) (Just . g)
          f (s,a) = show a
          g name  = ((),read name)

_NAhE :: SyntaxReader (State TruthVal)
_NAhE = reorder0 . naheToTV <$> (selmaho "NAhE" <|> pure "")
    where naheToTV = mkSynonymIso [("je'a",stv 1    0.9)
                                  ,(""    ,stv 0.75 0.9)
                                  ,("no'e",stv 0.5  0.9)
                                  ,("na'e",stv 0.25 0.9)
                                  ,("to'e",stv 0    0.9)]


_MO :: SyntaxReader (State Atom)
_MO = reorder0 . varnode <$> word "mo"

_GOhA :: SyntaxReader (State Atom)
_GOhA = implicationOf .< predicate <$> withSeed (selmaho "GOhA")

gohaP = _MO <|> _GOhA

selbri :: SyntaxReader (State (Tagged Atom))
selbri = filterState . first commute . associate <$> optional (selmaho "SE")
                                                 <*> (tanru <|> nuP <|> meP <|> moiP <|> gohaP)
selbriP :: SyntaxReader (State (Tagged Selbri))
selbriP = first associate <$> _NAhE <&> selbri

selbriUI :: SyntaxReader (State (Tagged Selbri))
selbriUI = collapsState . first ((merge . first (second handleUI) ||| id) . reorder)
        <$> withSeedState (selbriP <&> optState uiP)
    where reorder = Iso (Just . f) (Just . g)
              where f ((((tv,p),t),Just ui),i)    = Left  ((tv,((ui,p),i)),t)
                    f ((p,Nothing),i)             = Right (p,[])
                    g (Left  ((tv,((ui,p),i)),t)) = ((((tv,p),t),Just ui),i)
                    g (Right (p,_))               = ((p,Nothing),0)
          merge = Iso (Just . f) (Just . g)
              where f ((tv,(a,s)),t) = (((tv,a),t),s)
                    g (((tv,a),t),s) = ((tv,(a,s)),t)

_PU :: SyntaxReader String
_PU = selmaho "PU"

_ZI :: SyntaxReader String
_ZI = selmaho "ZI"

_VA :: SyntaxReader String
_VA = selmaho "VA"

_VIhA :: SyntaxReader String
_VIhA = selmaho "VIhA"

_FAhA :: SyntaxReader String
_FAhA = selmaho "FAhA"

_ZAhO :: SyntaxReader String
_ZAhO = selmaho "ZAhO"

_ZEhA :: SyntaxReader String
_ZEhA = selmaho "ZEhA"

_CAhA :: SyntaxReader String
_CAhA = selmaho "CAhA"

_anytense :: SyntaxReader (Atom)
_anytense = concept <$> (_PU   <|> _ZI   <|> _VA <|> _FAhA <|> _ZAhO <|>
                         _ZEhA <|> _VIhA <|> _CAhA)

anytense :: SyntaxReader (State Atom)
anytense = reorder0 <$> _anytense

_trati :: SyntaxReader (State (Maybe Atom))
_trati = first handleTrati
      <$> stateMany anytense
    where handleTrati = Iso (Just . f) (Just . g)
          f [] = Nothing
          f [x]= Just x
          f xs = apply andl xs
          g Nothing   = []
          g (Just xs@(AL _)) = fromJust $ unapply andl xs
          g (Just x) = [x]

_NA :: SyntaxReader (State String)
_NA = reorder0 <$> selmaho "NA"

selbriAll =  _trati
         <&> optState _NA
         <&> selbriUI

-------------------------------------------------------------------------------
--bacru
-------------------------------------------------------------------------------

--                            trati        na
bridi_tail :: SyntaxReader (State ((Maybe Atom, (Maybe String, Tagged Selbri)),[Sumti]))
bridi_tail = selbriAll <&> stateMany sumtiAllUI

-- (a,(mp,(ma,(s,aa))))
-- (mp,(ma,(s,a++aa)))
-- ((mp,(ma,(s,a))),as)
-- (bridi,as)

_GIhA ::  SyntaxReader String
_GIhA = _GIhAtoA <$> selmaho "GIhA"

_GIhA_BO :: SyntaxReader (State Con)
_GIhA_BO = reorder0 <$> _con_BO _GIhA

--(s,(bt,(giha,bt)))
--((s,bt),[(giha,(s,bt))])

_bridi :: SyntaxReader (State Atom)
_bridi = first (foldl handleGIhA . (handleBRIDI *** distribute) . reorder)
        <$> stateMany sumtiAllUI
        <*  optext "cu"
        <&> bridi_tail
        <&> stateMany (_GIhA_BO <&> bridi_tail)

    where reorder = Iso (Just . f) (Just . g) where
            f (s,(bt,ls)) = ((s,bt),(s,ls))
            g ((s,bt),(_,ls)) = (s,(bt,ls))
          distribute = Iso (Just . f) (Just . g) where
            f (s,[]) = []
            f (s,(giha,bt):xs) = (giha,(s,bt)):(f (s,xs))
            --g [] = Nothing -- let this fail with a bang so we notice
            g ((giha,(s,bt)):xs) = (s,(giha,bt):g' xs)
            g' [] = []
            g' ((giha,(s,bt)):xs) = (giha,bt):g' xs


handleGIhA :: Iso (Atom,(Con,Bridi)) (Atom)
handleGIhA = handleCon . second (tolist2 .> handleBRIDI) . reorder
    where reorder = Iso (Just . f) (Just . g) where
            f (a,(giha,br)) = (giha,(a,br))
            g (giha,(a,br)) = (a,(giha,br))

-- ((ma,(ms,ts)),[a])
-- (((ma,ms),ts),[a])
-- ((ma,ms),(ts,[a]))

handleBRIDI :: Iso Bridi Atom
handleBRIDI = handleNA
            . second _ctx
            . inverse associate
            . first commute
            . second _frames
            . inverse associate
            . first associate
            . mergeSumti

-- ((MPU,(MNA,(Selbri,Args)))   ,Atoms)
-- (((MPU,MNA),(Selbri,Args))   ,Atoms)
-- (((MPU,MNA),frames)            ,Atoms)
-- (((MNA,MPU),frames)            ,Atoms)
-- ((MNA,(MPU,frames))            ,Atoms)
-- ((MNA,MCtxL)                 ,Atoms)
-- (bridi                       ,Atoms)

handleNA :: Iso (Maybe String,Atom) Atom
handleNA = Iso f g where
    f (Nothing,a)    = Just a
    f (Just n, a)    = apply _eval (cGPN n lowTv,[a])
    g (EvalL _ (GPN n) a) = Just (Just n,a)
    g a = Just (Nothing,a)

--For mergin sumties before and after the selbri into a single list
mergeSumti :: (a ~ aa) => Iso ([a],(s,[aa])) (s,[a])
mergeSumti = Iso f g where
    f ([],(_,[])) = Nothing
    f (a1,(s,a2)) = Just (s,a1++a2)
    g     (s,a)   = case a of
                       [] -> Nothing
                       (x:xs) -> Just ([x],(s,xs))


_GA :: SyntaxReader (State String)
_GA = reorder0 . _GAtoA <$> selmaho "GA"

bridiGA :: SyntaxReader (State Atom)
bridiGA = first handleGA <$> _GA <&> _bridi <* sepSelmaho "GI" <&> _bridi
    where handleGA = handleCon . (addsnd Nothing *** tolist2)

bridi = (bridiGA <|> _bridi)

bridiUI :: SyntaxReader (State Atom)
bridiUI = collapsState . (first handleUI ||| id) . reorder . expandState
        <$> withSeed (optState uiP <&> bridi)
    where reorder = Iso (Just . rf) (Just . rg)
          rf (((Just ui,b),i),s)   = Left  (((ui,b),i),s)
          rf (((Nothing,b),i),s)   = Right ((b,[]),s)
          rg (Left (((ui,b),i),s)) = (((Just ui,b),i),s)
          rg (Right((b,_),s)     ) = (((Nothing,b),0),s)

bridiP :: SyntaxReader (State Atom)
bridiP = (ptp (uiP <*> selbriAll) addti bridiUI) <|> bridiUI <|> (ptp (anytense <*> selbriAll) addti bridiUI)
    where addti = Iso (Just . f) (Just . g)
          f s = s ++ "ti "
          g s = s --TODO: Maybe remoe ti again

preti :: SyntaxReader Atom
preti = ((_satl . associate) ||| handleMa) . ifJustA <$> optional (word "xu") <*> bridiP
    where handleMa =
              Iso (\(a,s) ->
                    let x = atomFold (\r a -> r || isMa a) False a
                        isMa (Node "VariableNode" x noTv) = x /= "$var"
                        isMa _ = False
                        all = Link "ListLink" (a:s) noTv
                        na = Link "PutLink" [all,Link "GetLink" [all] noTv] noTv
                    in Just (x ? na $ all))
                  (\ma -> case ma of
                      (Link "PutLink"  [LL (a:s),_] _) -> Just (a,s)
                      (Link "ListLink" (a:s) _) -> Just (a,s))


------------------------------------
--Free
-----------------------------------

_COI :: SyntaxReader (State Atom)
_COI = instanceOf .< concept <$> withSeed (selmaho "COI")

vocative :: SyntaxReader Atom
vocative = list . reorder <$> _COI <&> sumtiC
    where reorder = Iso (Just . f) (Just . g) where
              f ((c,a),s) = c:a:s
              g (c:a:s) = ((c,a),s)

          --Does Work
          ui = ptp uiP addti (first dropTag <$> sumtiAllUI)
          addti = Iso (Just . f) (Just . g) where
              f s = "ti " ++ s
              g s = drop 3 s
          dropTag = Iso (Just . f) (Just . g) where
              f (s,_) = s
              g s = (s,Nothing)




freeuiP :: SyntaxReader Atom
freeuiP = reorder <$> _UI <*> caiP
    where reorder = Iso (Just . f) (Just . g)
          f (a,tv) = cCN (nodeName a) tv
          g c@(Node "ConceptNode" name tv) = (c,tv)

freeSumti ::SyntaxReader Atom
freeSumti = list . cons .< reorder <$> sumtiAllUI
    where reorder = Iso (Just . f) (Just . g)
          f (s,_) = s
          g s = (s,Nothing)

free :: SyntaxReader Atom
free = vocative <|> freeuiP <|> freeSumti <|> luP'

--jufra = list <$> many1 (sepSelmaho "I" *> preti)

_JA :: SyntaxReader String
_JA = _JAtoA <$> selmaho "JA"

_JA_BO :: SyntaxReader (String,Maybe(Tagged Selbri))
_JA_BO = _con_BO _JA

{-_BAI_BO :: SyntaxReader Atom
_BAI_BO = <$> baiP <* sepSelmaho "BO"
-}

selbriToEval :: Iso Selbri Atom
selbriToEval = Iso (Just . f) (Just . g) where
    f (tv,atom) = cEvalL tv atom (cLL [])
    g (EvalL tv p _) = (tv,p)

_jufra :: SyntaxReader (Maybe (String,Maybe (Tagged Selbri)),[Atom])
_jufra = (id *** ((handle ||| tolist1) . ifJustB))
     <$> sepSelmaho "I" *> optional _JA_BO <*> preti <*> optional _jufra
    where handle = Iso (Just . f) (Just . g) where
              f (p,(mc,(a:as))) = case mc of
                                    Just c ->
                                        let Just x = apply handleCon (c,[p,a])
                                        in (x:as)
                                    Nothing -> (p:a:as)
              g (x:as) = case unapply handleCon x of
                            Just (c,[p,a]) -> (p,(Just c,(a:as)))
                            Nothing -> (x,(Nothing,as))

jufra :: SyntaxReader Atom
jufra = list . rmfst Nothing <$> _jufra

jufmei = list . reorder <$> sepSelmaho "NIhO" *> preti
                        <*> many1 (sepSelmaho "I" *> preti)
    where reorder = Iso (Just . f) (Just . g) where
            f (a,as) = a:as
            g (a:as) = (a,as)

lojban = jufmei <|> jufra <|> preti <|> free

------------------------------------
--Attitude
-----------------------------------

_UI :: SyntaxReader Atom
_UI = concept <$> selmaho "UI"

_CAI :: SyntaxReader String
_CAI = selmaho "CAI"

_NAI :: SyntaxReader String
_NAI = selmaho "NAI"

caiP :: SyntaxReader TruthVal
caiP = naicaiToTV . isoConcat "|" . tolist2 <$> (_NAI <|> pure "")
                                            <*> (_CAI <|> pure "")

naicaiToTV :: Iso String TruthVal
naicaiToTV = mkSynonymIso [("|cai"    ,stv 1     0.9)
                          ,("|sai"    ,stv 0.833 0.9)
                          ,("|"       ,stv 0.75  0.9)
                          ,("|ru'e"   ,stv 0.666 0.9)
                          ,("|cu'i"   ,stv 0.5   0.9)
                          ,("nai|ru'e",stv 0.333 0.9)
                          ,("nai|"    ,stv 0.250 0.9)
                          ,("nai|sai" ,stv 0.166 0.9)
                          ,("nai|cai" ,stv 0     0.9)]


uiP :: SyntaxReader (State (Atom,TruthVal))
uiP = reorder0 <$> _UI <*> caiP

withAttitude :: SyntaxReader (State Sumti) -> SyntaxReader (State Sumti)
withAttitude syn = merge . first ((first handleUI ||| id) . reorder) . expandState
                <$> withSeed (syn <&> optState uiP)
    where reorder = Iso (Just . rf) (Just . rg)
          rf (((a,mt),Just ui),i)   = Left (((ui,a),i),mt)
          rf (((a,mt),Nothing),i)   = Right ((a,[]),mt)
          rg (Left (((ui,a),i),mt)) = (((a,mt),Just ui),i)
          rg (Right ((a,_),mt)    ) = (((a,mt),Nothing),0)
          merge = Iso (Just . mf) (Just . mg)
          mf (((a,s1),mt),s2) = ((a,mt),s1++s2)
          mg ((a,mt),s)       = (((a,s),mt),s)

handleUI :: Iso (((Atom,TruthVal),Atom),Int) (State Atom)
handleUI = second (cons . first _frames . joinState
                        . (selbri *** (sumti1 *** (sumti2 *** sumti3))))
                     . reorder
    where joinState = Iso (Just . jf) (Just . jg)
          jf ((tv,(a1,s1)),((a2,s2),((a3,s3),(a4,s4)))) = ((((tv,a1),Nothing),[a2,a3,a4]),s1++s2++s3++s4)
          jg ((((tv,a1),_),[a2,a3,a4]),s) = ((tv,(a1,s)),((a2,s),((a3,s),(a4,s))))
          reorder = Iso (Just . rf) (Just . rg)
          rf (((ui,tv),a),i) = (a,((tv,((),i)),(((),i),((ui,i),(a,i)))))
          rg (a,((tv,((),_)),(((),_),((ui,_),(_,_))))) = (((ui,tv),a),0)
          sumti3 = first (addsnd $ Just "3") . instanceOf
          sumti2 = first (addsnd $ Just "2") . instanceOf
          sumti1 = first (addsnd $ Just "1") . instanceOf
                    . first (insert (Node "ConceptNode" "mi" noTv))
          selbri = second ( instanceOf
                          . first (insert (Node "PredicateNode" "cinmo" noTv))
                          )

------------------------------------
--Pre Parse XXX would need to consider all words to not parse things it shouldnt'
-----------------------------------

{-preParser :: SyntaxReader String
preParser =  isoConcat "" . tolist2
          <$> ((preGOI <|> prePA <|> (tolist1 <$> token)) <*> preParser)
          <|> (pure "")

preGOI :: SyntaxReader String
preGOI = goiToNoi <$> selmaho "GOI"
    where goiToNoi = mkSynonymIso [("pe"  ,"poi ke'a srana ")
          ,("po"  ,"poi ke'a se steci srana ")
          ,("po'e","poi jinzi ke se steci srana ")
          ,("po'u","poi du ")
          ,("ne"  ,"noi ke'a srana ")
          ,("no'u","noi du ")]

prePA :: SyntaxReader String
prePA = ptp (pa <*> selbri) id (addLE . isoConcat "" <$> joiSelmaho "PA")
    where addLE = Iso (Just . f) (Just . g)
          f s = s ++ " lo "
          g s = take (length s - 4) s

-}

{-
andExpansion :: Iso Atom Atom
andExpansion = Iso (\a -> atomMap a)

eval (Link "EvaluationLink" e _)

data Linked a b = NotLinked a | Linked a b (Linked a b)
addElem :: a -> Linked a b -> Linked a b
addElem e (NotLinked a)    = NotLinked (a:e)
addElem e (Linked a b l) = Linked (a:e) b $ addElem e l

addLink :: s -> Linked a b -> Linked a b
addLink e (NotLinked a) = Linked a e a
addLink e (Linked a b l) = Linked a b $ addLink e l

func :: Linked [Atom] String -> Atom -> Linked [Atom] String
func al@(NotLinked a) (Link "AndLink" [e1,e2] noTv) = Linked (a:e1) "AndLink" $ addElem e2 al
func al@(Linked a1 b l) (Link "AndLink" [e1,e2] noTv) =
    Linked (a1:e1) "AndLink" $ addElem e2 al
func l b = addElem b l
-}
