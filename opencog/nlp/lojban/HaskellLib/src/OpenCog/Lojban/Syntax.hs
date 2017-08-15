{-# LANGUAGE NoMonomorphismRestriction  #-}
{-# LANGUAGE RelaxedPolyRec             #-}
{-# LANGUAGE TypeFamilies               #-}
{-# LANGUAGE FlexibleContexts           #-}
{-# LANGUAGE RankNTypes                 #-}
{-# LANGUAGE LambdaCase                 #-}

module OpenCog.Lojban.Syntax where

import Prelude hiding (id,(.),(<*>),(<$>),(*>),(<*))

import qualified Data.Map as M
import qualified Data.List.Split as S
import Data.List (nub,partition)
import Data.Foldable (find)
import Data.Maybe (fromJust,listToMaybe)
import Data.List (isInfixOf,isPrefixOf,(\\))
import Data.Hashable

import System.Random

import Control.Category
import Control.Arrow hiding (left,right)
import Control.Applicative hiding (many,some,optional)
import Control.Monad.RWS
import Control.Monad.Trans.Class

import Iso
import Syntax hiding (SynIso,Syntax,text)

import Lojban
import Lojban.Syntax.Util

import OpenCog.AtomSpace (Atom(..),TruthVal(..),noTv,stv,atomFold,nodeName,atomElem,atomMap)
import OpenCog.Lojban.Util

import OpenCog.Lojban.Syntax.Types
import OpenCog.Lojban.Syntax.Util
import OpenCog.Lojban.Syntax.AtomUtil

import Debug.Trace

mytrace a = traceShow a a
mytrace2 s a = trace (s ++(' ':show a)) a

--ReadMe:
--It is recommend to first take a look at Syntax.Types

--http://wiki.opencog.org/w/Claims_and_contexts#An_Example_of_Moderately_Complex_Semantic_Embedding

--TODO Parser
-- FIX: pa + ki'o
--      da + quantifier
--      ZAhO explicit representation
--      vo'a
--      make statments explicit
--      ge'e cai on logical conectives and else?
--      Does SEI have implicit args?
--      What if you have SEI and UI

-- argslots
-- Synonims for tenses and gismu

-- ki
-- references

-- ConceptAnd/OrLink NotLink isntead of boolea and/or

--TODO Printer
-- da,de,di differentiation


--List of Flags
--NA_FLAG keeps track of the current NA state
--FEhE switch between temporal/spatioal interval_propertys

lojban = finalCheck <<< text

finalCheck :: SynIso a a
finalCheck = Iso f g where
    f a = do
        text <- gets getText
        if text == ""
           then pure a
           else lift $ Left $ "Incomplete parse: " ++ text
    g = pure

{-
text = handle
    <<< many (selmaho "NAI")
    &&& many (cmene <+> indicators)
    &&& optional (joik_jek)
    &&& text_1

text_1 :: Syntax Atom
text_1 = handle
    <<< optional (many (sepSelmaho "I")
                    --   &&> optional joik_jek
                    --   &&& optional (optional stag
                    --                 <&& sepSelmaho "BO"
                    --                )
                    -- )
                  <+>
                  many (sepSelmaho "NIhO")
                 )

    &&& optional paragraphs
-}

text :: Syntax Atom
text = (text_1 <+> (filterDummy . handleFREEs . addfst dummy . frees))
    where filterDummy = mkIso f id where
            f (LL ls) = cLL (filter (\a -> not $ atomAny (== dummy) a) ls)
          dummy = cCN "dummy" noTv

text_1 :: Syntax Atom
text_1 = paragraphs

manySep :: Syntax () -> Syntax ()
manySep iso = ((manySep iso <+> id) . iso <+> insert ())

paragraphs :: Syntax Atom
paragraphs = listl . cons . addfst (cAN "paragraphs") . cons
    <<< ((handleFREEs . commute ||| id) . ifJustA
         <<< optional (sepSelmaho "NIhO" &&> frees)
         &&& paragraph
        )
    &&& many (handleFREEs . commute <<< sepSelmaho "NIhO"
                                    &&> frees
                                    &&& paragraph)

paragraph :: Syntax Atom
paragraph = listl . cons . addfst (cAN "paragraph") . cons
    <<< (handleFREEs . commute
         <<< optSelmaho "I"
         &&> frees
         &&& (statement <+> fragment)
        )
    &&& many (handleFREEs . commute <<< sepSelmaho "I"
                                    &&> frees
                                    &&& (statement <+> fragment)
             )

statement :: Syntax Atom
statement = handleMa <<< statement'
    where handleMa :: SynIso Atom Atom
          handleMa = Iso f g where
              f a = do
                  let x = atomFold (\r a -> r || isMa a) False a
                      list = "$var" : ((('$':).show) <$> [0..10])
                      isMa (Node "VariableNode" x noTv) = x `notElem` list
                      isMa _ = False
                      inter = cCN "Interrogative" noTv
                  interinst <- apply instanceOf inter
                  let na = cInhL noTv interinst (cSSScL noTv [a])
                  pure (x ? na $ a)
              g (InhL _ _ (SSScL [a])) = pure a
              g a                      = pure a

statement' :: Syntax Atom
statement' = listl . cons . addfst (cAN "statement") . cons
        <<< (statement_1 <+> (prenex &&> statement)) &&& gsAtoms

statement_1 :: Syntax Atom
statement_1 = isoFoldl handleCon2
    <<< statement_2 &&& many (sepSelmaho "I"
                              &&>
                              (just.joik_jek &&& insert Nothing)
                              &&&
                              statement_2 --FIXME officaly optional
                             )

statement_2 :: Syntax Atom
statement_2 = (handleCon2 ||| id) . ifJustB
    <<< statement_3 &&& optional (sepSelmaho "I"
                                  &&> (just.joik_jek &&& optional stag)
                                  &&& sepSelmaho "BO"
                                  &&> statement_2 --FIXME officaly optional
                                 )

statement_3 :: Syntax Atom
statement_3 = sentence
 -- <+> optional tag &&& sepSelmaho "TUhE" &&> text_1 <&&& optSelmaho "TUhU"

--FIXME
fragment :: Syntax Atom
fragment = listl . cons . addfst (cAN "fragment") . gsAtoms
    -- <<< ek
    -- <+> gihek
       <<< (toState 1 . tolist1 . quantifier)
    -- <+> selmaho "NA"
       <+> (termsToState <<< terms <&& optSelmaho "VAU")
       <+> prenex
       <+> (toState 1 . tolist1 . listl <<< relative_clauses)
       <+> (termsToState <<< linkargs)
       <+> (termsToState <<< links)
    where termsToState = toState 1 . tolist1 . listl . mapIso (rmsndAny Nothing)

--FIXME
prenex :: Syntax ()
prenex = toState 1 . tolist1 . listl . tolist2 . addfst (cAN "ZOhU")
       . listl . mapIso (rmsndAny Nothing)
       <<< terms <&& sepSelmaho "ZOhU"

sentence :: Syntax Atom
sentence = withCleanState sentence'

sentence' :: Syntax Atom
sentence' = handleCTX . handleBTCT
    <<< ((terms <&& optSelmaho "CU") <+> insert []) &&& bridi_tail
    where handleCTX = Iso f g where
              f a = do
                  atoms <- gets sAtoms
                  now <- gets sNow
                  case now of
                      CN "NOCTX" -> pure (cSL ((cAN "RelativePhrase"):a:atoms))
                      _ -> do
                          ctxs <- filter (/= now) <$> gets sCtx
                          setAtoms []
                          setCtx [now]
                          ctx <- if ctxs /= []
                                    then pure $ cSL ctxs
                                    else do
                                        name <- randName ""
                                        pure $ cSL [cCN name noTv]
                          pure $ cCtxL noTv ctx (cSL (a:atoms))
              g (CtxL (SL ctxs) (SL (a:atoms))) = do
                  setCtx ctxs
                  setAtoms atoms
                  pure a
              g (SL (_:a:atoms)) = do --Check that now is "NOCTX"???
                  setAtoms atoms
                  pure a

subsentence :: Syntax Atom
subsentence = sentence
          <+> (prenex &&> subsentence)

handleBTCT :: SynIso ([Sumti],BTCT) Atom
handleBTCT = Iso f g where
    f (sumti1,CTLeaf (selb,sumti2)) = apply handleBRIDI (selb,sumti1++sumti2)
    f (sumti1,CTNode con (st1,st2)) = do
        a1 <- apply handleBTCT (sumti1,st1)
        a2 <- apply handleBTCT (sumti1,st2)
        apply handleCon (con,(a1,a2))
    g = error "Not implemented g handleBTCT"

 -- Isoier version:
 -- (handleBRIDI . manageLeaf. inverse ctLeaf
 -- <+> handleCon . second (handleBTCT *** handleBTCT) . manageNode . inverse ctNode)

 -- manageLeaf = Iso f g where
 --     f (s1,(s,s2)) = (s,s1++s2)
 --     g (s1,s) = ([],(s,s1)) --Leafe it and split later

handleBRIDI :: SynIso BT Atom
handleBRIDI = handleNA
            -- frames
            . handleSelbriSumtis
            -- (Selbri,sumti)

handleNA :: SynIso Atom Atom
handleNA = rmFlagIso "NA_FLAG" . iso . second (getFlagValueIso "NA_FLAG") . unit
        <+> id
    where iso = Iso f g
          f (a,n)               = apply _eval (cGPN n lowTv,[a])
          g (EvalL _ (GPN n) a) = pure (a,n)


handleSelbriSumtis :: SynIso (Selbri,[Sumti]) Atom
handleSelbriSumtis = merge
                   . (_frames *** handleModalSumti)
                   . reorder
                   . second (splitSumti
                            . (mapIso handleJAI . ifFlag "JAI" <+> id)
                            . handleTAG)

    where splitSumti :: SynIso [(Atom,Tag)] ([(Atom,Tag)],[Atom])
          splitSumti = mkIso f g where
              f s = f' ([],[]) s
              g (ls,rs) = ls ++ map (\a -> (a,"ModalSumti")) rs

              f' (ls,rs) [] = (ls,rs)
              f' (ls,rs) ((a,"ModalSumti"):xs) = f' (ls,a:rs) xs
              f' (ls,rs) (a:xs)                = f' (a:ls,rs) xs

          reorder = mkIso f g where
              f ((tv,s),(ls,rs)) = (((tv,s),ls),(s,rs))
              g (((tv,s),ls),(_,rs)) = ((tv,s),(ls,rs))

          merge :: SynIso (Atom,Maybe Atom) Atom
          merge = Iso f g where
              f (a1,Just a2) = apply andl [a1,a2]
              f (a1,Nothing) = pure a1
              g (AL [a1@(AL _),a2@(AL _)]) = pure (a1,Just a2)
              g a1 = pure (a1,Nothing)

          handleJAI :: SynIso (Atom,Tag) (Atom,Tag)
          handleJAI = Iso f g where
              f (a,"fai") = pure (a,"1")
              f (a,"1") = do
                  mjai <- gets sJAI
                  case mjai of
                      Just jai -> do
                          (na,Just t) <- apply modalSumti (jai,a)
                          pure (na,t)
                      Nothing  -> lift $ Left "No JAI in state."
              f a = pure a
              g a = pure a --Loosing information


bridi_tail :: Syntax BTCT
bridi_tail = (extendBTCT ||| id) . ifJustB
        <<< bridi_tail_1
        &&& optional (((just.right.gihek &&& optional stag)
                       &&& sepSelmaho "KE" &&> bridi_tail <&& optSelmaho "KEhE"
                      )
                      &&& tail_terms
                     )

bridi_tail_1 :: Syntax BTCT
bridi_tail_1 = isoFoldl extendBTCT
            <<< bridi_tail_2
            &&& many (((just.right.gihek &&& insert Nothing)
                       &&& bridi_tail_2
                      )
                      &&& tail_terms
                     )

bridi_tail_2 :: Syntax BTCT
bridi_tail_2 = (extendBTCT ||| id) . ifJustB
    <<< ctLeaf . bridi_tail_3
    &&& optional (((just.right.gihek &&& optional stag)
                   &&& sepSelmaho "BO"
                   &&> bridi_tail_2
                  )
                  &&& tail_terms
                 )

extendBTCT :: SynIso (BTCT,((Con,BTCT),[Sumti])) BTCT
extendBTCT = appendSumtiToBTCT . first manage . associate

    where manage :: SynIso (BTCT,(Con,BTCT)) BTCT
          manage = mkIso f g where
              f (l1,(con,l2)) = CTNode con (l1,l2)
              g (CTNode con (l1,l2)) = (l1,(con,l2))

          appendSumtiToBTCT :: SynIso (BTCT,[Sumti]) BTCT
          appendSumtiToBTCT = mkIso f g where
              f (btct,sumti) = fmap (aSTBTCT sumti) btct
              g (btct)       = let sumti = reverse $ foldl (ff) [] btct
                                   nbtct = fmap (tCTBTSa sumti) btct
                               in (nbtct,sumti)

              aSTBTCT sumti2 (selbri,sumti1) = (selbri,sumti1++sumti2)
              tCTBTSa sumti2 (selbri,sumti1) = (selbri,sumti1 \\ sumti2)

              ff ls (selbri,sumti) = fun (reverse sumti) ls
              fun a [] = a
              fun (a:as) (b:bs) | a == b    = a : (fun as bs)
                                | otherwise = []

bridi_tail_3 :: Syntax (Selbri,[Sumti])
bridi_tail_3 = (second handleKO . selbri) &&& tail_terms
        -- <+> gek_sentence
    where handleKO :: SynIso Atom Atom
          handleKO = (sndToState 1 . second (tolist1 . impl) . reorder . ifFlag "ko") <+> id
              where reorder = mkIso f g where
                      f selbri = (selbri,[selbri,cPN "Imperative" lowTv])
                      g (selbri,_) = selbri


gek_sentence :: Syntax Atom
gek_sentence = handleCon . handleGIK
           <<< gek &&& subsentence
                   &&& gik
                   &&& subsentence
--                  &&& tail_terms
--    <+> optional tag &&& sepSelmaho "KE"
--                     &&> gek_sentence
--                     &&& optSelmaho "KEhE"
--    <+> selmaho "NA" &&& gek_sentence

-------------------------------------------------------------------------------
--Sumti
-------------------------------------------------------------------------------

tail_terms :: Syntax [Sumti]
tail_terms = termsM <&& optSelmaho "VAU"

terms :: Syntax [Sumti]
terms = some term

termsM :: Syntax [Sumti]
termsM = many term

--FIXME Implement TermSets
--terms_1 :: Syntax [Sumti]
--terms_1 = terms_2 &&& many (sepSelmaho "PEhE" &&> joik_jek &&& terms_2)

--terms_2 :: Syntax [Sumti]
--terms_2 = cons <<< term &&& many (sepSelmaho "CEhE" &&& term)

term :: Syntax Sumti
term = (sumti &&& insert Nothing)
    <+> (modalSumti <<< tag &&& sumti <&& optSelmaho "KU")
    <+> (commute    <<< _FA &&& sumti <&& optSelmaho "KU")
 -- <+> termset
 -- <+> selmaho "NA" <&& sepSelmaho "KU"
    <+> placeholder --Not normal Lojban

    where _FA = just . faToPlace . selmaho "FA"

          faToPlace :: SynIso String String
          faToPlace = mkSynonymIso [("fa","1")
                                   ,("fe","2")
                                   ,("fi","3")
                                   ,("fo","4")
                                   ,("fu","5")
                                   ,("fi'a","?")]

          --For parsing placeholder like "x1" as a sumti
          --The 1 will be used as the place tag
          placeholder :: Syntax Sumti
          placeholder = first instanceOf . handle
                     <<< letter &&& digit <&& sepSpace
              where handle = mkIso f g
                    f (c,d)               = (cCN [c,d] noTv,Just [d])
                    g (CN [c,_],Just [d]) = (c,d)

modalSumti :: SynIso (JJCTTS,Atom) Sumti
modalSumti = addsnd (Just "ModalSumti")
           . sndToState 1
           . (pid &&& tolist1)
           . handleJJCTTS
           . second tolist1

sumti :: Syntax Atom
sumti = handleFREEs2 <<< sumti' &&& frees

sumti' :: Syntax Atom
sumti' = (isoFoldl handleKEhA ||| id) . ifJustB
    <<< sumti_1 &&& optional (sepSelmaho "VUhO" &&> relative_clauses)

sumti_1 :: Syntax Atom
sumti_1 = (handleCon2 ||| id) . ifJustB
     <<< sumti_2 &&& optional ((just . joik_ek &&& optional stag)
                                <&& sepSelmaho "KE"
                                &&& sumti
                                <&& optSelmaho "KEhE"
                               )

sumti_2 :: Syntax Atom
sumti_2 = isoFoldl handleCon2
      <<< sumti_3 &&& many ((just . joik_ek &&& insert Nothing) &&& sumti_3)

sumti_3 :: Syntax Atom
sumti_3 = (handleCon2 ||| id) . ifJustB
     <<< sumti_4 &&& optional ((just . joik_ek &&& optional stag)
                                <&& sepSelmaho "BO"
                                &&& sumti_3
                              )

sumti_4 :: Syntax Atom
sumti_4 = sumti_5
       <+> (handleCon . handleGIK <<< gek &&& sumti &&& gik &&& sumti_4)

--FIXME missing optional "KU" after selbri
sumti_5 :: Syntax Atom
sumti_5 = ptp (quantifier &&& lookahead selbri) (isoAppend " lo ") sumti_5Q
         <+> sumti_5Q

-- Quantiviers
sumti_5Q :: Syntax Atom
sumti_5Q = (handleSubSet . second setWithSize ||| maybeinof)
        . manage
       <<< (optional quantifier &&& sumti_5R)

    where manage :: SynIso (Maybe Atom,Atom) (Either (Atom,(Atom,Atom)) Atom)
          manage = Iso f g where
              f (Just pa,sumti) = do
                        atoms <- gets sAtoms
                        case findSetType sumti atoms of
                            Just t  -> pure $ Left (sumti,(pa,t    ))
                            Nothing -> pure $ Left (sumti,(pa,sumti))
              f (Nothing,sumti) = pure $ Right sumti
              g (Left (sumti,(pa,t))) = pure (Just pa,sumti)
              g (Right sumti)         = pure (Nothing,sumti)

          handleSubSet = sndToState 1 . second (tolist1 . subsetL) . reorder2

          maybeinof = Iso f g where
              f a = do
                  atoms <- gets sAtoms
                  case getDefinitions [a] atoms of
                      [] -> apply instanceOf a
                      _  -> pure a              --We have a lep which is already an instance
              g a = error "Check for lep somehow"

          reorder2 = mkIso f g where
              f (t,s)     = (s,(s,t))
              g (s,(_,t)) = (t,s)



-- Relative clauses
sumti_5R :: Syntax Atom
sumti_5R = (isoFoldl handleKEhA ||| id) . ifJustB
        <<< sumti_6 &&& optional relative_clauses

handleKEhA :: SynIso (Atom,Atom) Atom
handleKEhA = Iso f g where
    f (c,a) = do
        pushAtom $ atomMap (switch c) a
        pure c
    g c = error "Not Implemented"
    switch c (Node "ConceptNode" "ke'a" _) = c
    switch _ a = a

sumti_6 :: Syntax Atom
sumti_6 = kohaP
       <+> le
       <+> laP
       <+> liP
       <+> zoP
       <+> luP

relative_clauses :: Syntax [Atom]
relative_clauses = cons <<< relative_clause &&& many (sepSelmaho "ZIhE" &&> relative_clause)

relative_clause :: Syntax Atom
relative_clause = (goi <&& optSelmaho "GEhU") <+> (noi <&& optSelmaho "KUhO")
    where goi :: Syntax Atom
          goi = ptp (selmaho "GOI") goiToNoi noi where
              goiToNoi = mkSynonymIso [("pe "  ,"poi ke'a srana ")
                                      ,("po "  ,"poi ke'a se steci srana ")
                                      ,("po'e ","poi jinzi ke se steci srana ")
                                      ,("po'u ","poi ke'a du ")
                                      ,("ne "  ,"noi ke'a srana ")
                                      ,("no'u ","noi ke'a du ")
                                      ,("goi " ,"poi ke'a du ")]

          noi :: Syntax Atom
          noi = sepSelmaho "NOI" &&> ((hasKEhA <<< relSentence)
                                      <+>
                                      ptp bridi_tail addKEhA relSentence)
              where hasKEhA = Iso f f
                    f a = if atomAny (\case { Link _ _ _ -> False
                                            ; (Node _ n _) -> "ke'a" `isInfixOf` n
                                            }) a
                             then pure a
                             else lift $ Left "No ke'a in bridi."

                    addKEhA = mkIso f g where
                      f = (++) "ke'a "
                      g = drop 4

                    relSentence = withNoCTX subsentence

                    withNoCTX syn = Iso f g where
                        f a = do
                            now <- gets sNow
                            setNow $ cCN "NOCTX" noTv
                            res <- apply syn a
                            setNow now
                            pure res
                        g a = do
                            now <- gets sNow
                            setNow $ cCN "NOCTX" noTv
                            res <- unapply syn a
                            setNow now
                            pure res

le :: Syntax Atom
le = (handleFREEs2 ||| id) . ifJustB
   <<< setFlagValueIso "LE_FLAG" . (selmaho "LA" <+> selmaho "LE")
   &&> sumti_tail
   &&& optional (sepSelmaho "KU" &&> frees)

sumti_tail :: Syntax Atom
sumti_tail = sumti_tail_1
        <+> ((handleKEhA . commute ||| id) . ifJustA
            <<< optional (reparse relative_clause . insert "pe ")
            &&& sumti_tail_1)
         <+> (isoFoldl handleKEhA . commute <<< relative_clauses
                                            &&& sumti_tail_1)

sumti_tail_1 :: Syntax Atom
sumti_tail_1 =
             (
              (setWithSize ||| id) . ifJustA
              <<<
              optional quantifier
              &&&
              ((isoFoldl handleKEhA ||| id) . ifJustB
               <<< (handle <<< selbri &&& varNode)
               &&& optional relative_clauses)
             )
          -- <+>
          -- (setWithSize <<< quantifier &&& sumti)
    where varNode = insert [(Node "VariableNode" "$var" noTv,Nothing)]
          handle = choice leHandlers . first (getFlagValueIso "LE_FLAG")
                 . commute . unit . _ssl . handleBRIDI

          leHandlers = [genInstance "IntensionalInheritanceLink" . rmfst "le"
                       ,genInstance "SubsetLink"                 . rmfst "lo"
                       ,massOf "IntensionalInheritanceLink"      . rmfst "lei"
                       ,massOf "SubsetLink"                      . rmfst "loi"
                       ,setOf "IntensionalInheritanceLink"       . rmfst "le'i"
                       ,setOf "SubsetLink"                       . rmfst "lo'i"
                       ,genInstance "IntensionalInheritanceLink" . rmfst "le'e"
                       ,genInstance "SubsetLink"                 . rmfst "lo'e"
                       ]

-- A Mass according to Lojban is a thing that has
-- all properties of it's parts
massOf :: String -> SynIso Atom Atom
massOf itype = instanceOf . _ssl . _frames . addStuff
             . (implicationOf *** genInstance itype) . addfstAny pp
    where pp = Node "PredicateNode" "gunma" noTv
          var = Node "VariableNode" "$var" noTv
          addStuff = mkIso f g where
              f (p,a) = ((noTv,p),[(var,"1"),(a,"2")])
              g ((_,p),[_,(a,_)]) = (p,a)

setOf :: String -> SynIso Atom Atom
setOf itype = sndToState 1 . second (tolist1 . setTypeL . tolist2)
            . makeSet . genInstance itype
    where makeSet = Iso f g
              where f a = do
                        name <- randName (show a)
                        let set = cCN name noTv
                        pure (set,(set,a))
                    g (_,(_,a)) = pure a

laP :: Syntax Atom
laP = handleName . wordNode <<< sepSelmaho "LA"
                            &&> anyWord
                            <&& optSelmaho "KU"
    where handleName :: SynIso Atom Atom
          handleName = Iso f g where
              f a = do
                  p <- apply instanceOf (cPN "cmene" lowTv)
                  name <- randName (nodeName a ++ "___" ++ nodeName a)
                  let c = cCN name lowTv
                      at = (a,"1")
                      ct = (c,"2")
                      pt = (highTv,p)
                  l <- apply _frames (pt,[at,ct])
                  pushAtom l
                  pure c
              g _ = do
                  (EvalL _ _ (LL [a,_])) <- popAtom
                  pure a

liP :: Syntax Atom
liP = sepSelmaho "LI" &&> (xo <+> number) <&& optSelmaho "LOhO"

xo :: Syntax Atom
xo = varnode <<< word "xo"

quantifier :: Syntax Atom
quantifier = number <&& optSelmaho "BOI"
       --FIXME <+> sepSelmaho "VEI" &&& mex &&& optSelmaho "VEhO"

number :: Syntax Atom
number =  (    numberNode    |||    concept   )
    . (showReadIso . paToNum |^| isoIntercalate " ")
    <<< some (selmaho "PA")
    where paToNum :: SynIso [String] Int
          paToNum = isoFoldl (digitsToNum . second paToDigit) . addfst 0

          digitsToNum :: SynIso (Int,Int) Int
          digitsToNum = Iso f g where
              f (i,j) = pure (i*10+j)
              g 0     = lift $ Left "Last Digit"
              g n     = pure (n `div` 10,n `mod` 10)

          paToDigit :: SynIso String Int
          paToDigit = mkSynonymIso [("no",0),("pa",1)
                                   ,("re",2),("ci",3)
                                   ,("vo",4),("mu",5)
                                   ,("xa",6),("ze",7)
                                   ,("bi",8),("so",9)]


zoP :: Syntax Atom
zoP = instanceOf . wordNode <<< mytext "zo" &&> anyWord

--KohaPharse for any kind of Pro-Noune
kohaP :: Syntax Atom
kohaP = da <+> ma <+> ko <+> keha <+> koha
    where koha = concept . selmaho "KOhA"
          ma   = varnode . word "ma"
          da   = concept . oneOfS word ["da","de","di"]
          ko   = setFlagIso "ko" . concept . word "ko"
          keha = concept . word "ke'a"

luP' :: Syntax Atom
luP' = sepSelmaho "LU" &&> lojban <&& optSelmaho "LIhU"

luP :: Syntax Atom
luP = instanceOf . luP'

setWithSize :: SynIso (Atom,Atom) Atom
setWithSize = sndToState 2 . second (tolist2 . (sizeL *** setTypeL)) . makeSet
    where makeSet = Iso f g
          f (a,b) = do
              name <- randName $ show a ++ show b
              let set = cCN name noTv
              pure (set,([set,a],[set,b]))
          g (_,([_,a],[_,b])) = pure (a,b)

ctLeaf :: SynIso a (ConnectorTree c a)
ctLeaf = Iso f g where
    f a = pure $ CTLeaf a
    g (CTLeaf a) = pure $ a
    g _ = lift $ Left "Not a CTLeaf."

tagPat :: Syntax (Tagged Selbri) -> Syntax JJCTTS
tagPat syn = isoFoldl toTree <<< ctLeaf . syn
                             &&& many (joik_jek &&& syn)
    where toTree :: SynIso (JJCTTS,(JOIK_JEK,Tagged Selbri)) JJCTTS
          toTree = Iso f g
          f (a,(c,b)) = pure $ CTNode c (a,CTLeaf b)
          g (CTNode c (a,CTLeaf b)) = pure (a,(c,b))
          g _ = lift $ Left "toTree.g: Doesn't match expected pattern."

tag :: Syntax JJCTTS
tag = tagPat tense_modal

stag :: Syntax JJCTTS
stag = tagPat simple_tense_modal

tense_modal :: Syntax (Tagged Selbri)
tense_modal = simple_tense_modal
            <+> addsnd Nothing . (sepSelmaho "FIhO" &&> selbri <&& optSelmaho "FEhU")

simple_tense_modal :: Syntax (Tagged Selbri)
simple_tense_modal = _bai
                  <+> _space_time
                  <+> _CAhA
--FIXME: Can we really just use tanru_unit_2 or do we need sothing specific
_bai :: Syntax (Tagged Selbri)
_bai = addsnd Nothing . withFlag "WITH_BAI" tanru_unit_2

_space_time :: Syntax (Tagged Selbri)
_space_time = (addsnd (Just "space_time") . addfstAny noTv . space_time)

_CAhA :: Syntax (Tagged Selbri)
_CAhA = addsndAny Nothing . addfstAny noTv
      . implicationOf . predicate . selmaho "CAhA"

--Fails when the Syntax Succeds and the other way arround
--Either the syn succeds then we fail with the zeroArrow
--Or the right . insertc succeds because syn failed then we do nothing
notsyn :: Syntax a -> Syntax ()
notsyn x = (zeroArrow ||| id) . ((left <<< lookahead x) <+> (right . insert ()))

-------------------------------------------------------------------------------
--Selbri
-------------------------------------------------------------------------------

handleTanru :: SynIso (Selbri,Selbri) Selbri
handleTanru = sndToState 1 . second (sel_iimpl) . reorder
    where reorder = mkIso f g where
            f (g,t) = (t,(t,g))
            g(t,(_,g)) = (g,t)
          sel_iimpl = Iso f g where
            --FIXME do something sensible with the tvs
            f ((tv1,s1),(tv2,s2)) = apply (tolist1 . _iimpl) (s1,s2)
            g a = do
                (s1,s2) <- unapply (tolist1 . _iimpl) a
                pure ((noTv,s1),(noTv,s2))

selbri :: Syntax Selbri
selbri = second filterState . (handle ||| id) . ifJustA
      <<< optional tag &&& selbri_1
    where handle = commute . first handleJJCTTS_Selbri . associate . second commute

selbri_1 :: Syntax Selbri
selbri_1 = selbri_2 <+> ((handleNAFlag . selmaho "NA") &&> selbri)
    where handleNAFlag = Iso f g where
            f na = do
                flags <- gets sFlags
                if "NA_FLAG" `M.member` flags
                   then case M.lookup "NA_FLAG" flags of
                       Just "na" -> if na == "na"
                                      then setFlagValue "NA_FLAG" "ja'a"
                                      else pure ()
                       Just "ja'a" -> if na == "na"
                                        then setFlagValue "NA_FLAG" "na"
                                        else pure ()
                   else setFlagValue "NA_FLAG" na
            g () = getFlagValue "NA_FLAG"

selbri_2 :: Syntax Selbri
selbri_2 = (handleTanru . commute ||| id) . ifJustB
        <<< selbri_3 &&& optional (sepSelmaho "CO" &&> selbri_2)

selbri_3 :: Syntax Selbri
selbri_3 = isoFoldl handleTanru . (inverse cons) <<< some selbri_4

selbri_4 :: Syntax Selbri
selbri_4 = ( handleSelbri4 ||| id) . ifJustB
        <<< selbri_5
        &&& optional (some (((just . joik_jek    &&& insert Nothing)
                              &&& selbriToEval . selbri_5)
                        <+> ((just . left . joik &&& optional stag)
                             &&& sepSelmaho "KE"
                             &&> selbriToEval . selbri_3
                             <&& optSelmaho "KEhE")))

    where reorder = mkIso f g
          f (a,(c,b)) = (c,(a,b))
          g (c,(a,b)) = (a,(c,b))

          handleSelbri4 = addfst selbriDefaultTV
                        . isoFoldl (rmfst selbriDefaultTV
                                    . manageSelbriCon
                                    . handleCon
                                    . reorder)
                        . first selbriToEval

selbri_5 :: Syntax Selbri
selbri_5 = (handleSelbri5 ||| id) . ifJustB
        <<< selbri_6
        &&& optional ((
                        (just . joik_jek)
                        &&& optional stag
                        <&& sepSelmaho "BO"
                      )
                     &&& selbriToEval . selbri_5)
    where handleSelbri5 = manageSelbriCon . handleCon2 .< selbriToEval


selbri_6 :: Syntax Selbri
selbri_6 = tanruBO <+> tanruGUHEK

tanruBO :: Syntax Selbri
tanruBO = (handleTanru ||| id) . ifJustB
       <<< tanru_unit &&& optional (sepSelmaho "BO" &&> selbri_6)

tanruGUHEK :: Syntax Selbri
tanruGUHEK = manageSelbriCon . handleCon . handleGIK
         <<< guhek &&& (selbriToEval . selbri) &&& gik &&& (selbriToEval . selbri_6)

tanru_unit :: Syntax Selbri
tanru_unit = isoFoldl handleCEI <<< tanru_unit_1
                                &&& many (sepSelmaho "CEI" &&> tanru_unit_1)
    where handleCEI = Iso f g
          f ((tv1,a1),(tv2,a2)) = do
              pushAtom $ cImpL noTv a2 a1
              pure (tv1,a1)
          g (tv1,a1) = lift $ Left "FIXME: not unpacking of tanru_unit yet"

tanru_unit_1 :: Syntax Selbri
tanru_unit_1 = (handleLinkArgs ||| id) . ifJustB
            <<< tanru_unit_2 &&& optional linkargs
    where handleLinkArgs :: SynIso (Selbri,[Sumti]) Selbri
          handleLinkArgs = iunit . commute
                         . (toState 1 . tolist1 . _frames . second handleTAG
                            &&& rmsndAny [])

linkargs :: Syntax [Sumti]
linkargs = (handleBEhOFREEs ||| id) . ifJustB
    <<< sepSelmaho "BE"
    &&> (cons <<<
              (handleBEFREEs <<< frees &&& tag2 . term)
              &&& (links <+> insert [])
        )
    &&& optional (sepSelmaho "BEhO" &&> frees)
    where tag2 = second (mkIso f g)
          f Nothing = Just "2"
          f b = b
          g (Just "2") = Nothing
          g b = b

          handleBEFREEs = first (handleFREEs2 . commute) . associate
          handleBEhOFREEs = isoZip . first (inverse setl) . commute
                          . second handleFREEs2
                          . inverse associate
                          . first (commute . first setl . inverse isoZip)

links :: Syntax [Sumti]
links = some links'
    where links' :: Syntax Sumti
          links' = first (handleFREEs2 . commute) . associate
              <<< sepSelmaho "BEI" &&> frees
                                   &&& term

--Also block selbri_4 when selbri_5+6 ???
tanru_unit_2 :: Syntax Selbri
tanru_unit_2 = addfst selbriDefaultTV
             . ((brivla
                <+> nuP
                <+> moiP
                <+> gohaP
                <+> meP) . ifNotFlag "WITH_BAI"
               <+> bai . ifFlag "WITH_BAI")
            <+> tanruSE
            <+> tanruKE
            <+> tanruNAhE

bai :: Syntax Atom
bai = (ptp (selmaho "BAI") iso brivla) . ifFlag "WITH_BAI"
    where iso = Iso f g where
            f a = do
                btf <- asks wBai
                apply btf a
            g b = do
                btf <- asks wBai
                unapply btf b

selbriDefaultTV = stv 0.75 0.9

selbriToEval :: SynIso Selbri Atom
selbriToEval = mkIso f g where
    f (tv,p) = cEvalL tv (cVN "$arg_place") (cLL [p,cVN "$arg"])
    g (EvalL tv _ (LL [p,_])) = (tv,p)

manageSelbriCon :: SynIso Atom Selbri
manageSelbriCon = Iso f g where
    f a = do
        name <- randName (show a)
        let s = (selbriDefaultTV,cPN name noTv)
        eval <- apply selbriToEval s
        pushAtom $ cImpL noTv eval a
        pure s
    g s = do
        atoms <- gets sAtoms
        eval <- apply selbriToEval s
        let ml = find (ff eval) atoms
        case ml of
           Nothing -> lift $ Left "No ImpL for guhek."
           Just l@(ImpL _ _ a) -> rmAtom l >> pure a

    ff eval (ImpL _ eval2 _) = eval == eval2
    ff _ _ = False

--Nahe with pred ohter then main??? influnce impl link???
tanruNAhE :: Syntax Selbri
tanruNAhE = (handleNAhE <<< _NAhE &&& tanru_unit_2)
    where handleNAhE = mkIso f g
          f (tv,(_,s)) = (tv,s)
          g (tv,s) = (tv,(noTv,s))

tanruJAI :: Syntax Selbri
tanruJAI = (sepSelmaho "JAI"
                &&> setFlagIso "JAI" . jaiFlag . tag
                &&> tanru_unit_2)
    where jaiFlag :: SynIso JJCTTS ()
          jaiFlag = Iso f g where
              f t = setJai t
              g () = do
                  mjai <- gets sJAI
                  case mjai of
                      Just jai -> rmJai >> pure jai
                      Nothing  -> lift $ Left "No JAI in state."


_NAhE :: Syntax TruthVal
_NAhE = naheToTV <<< (selmaho "NAhE" <+> insert "")
    where naheToTV = mkSynonymIso [("je'a",stv 1    0.9)
                                --,(""    ,stv 0.75 0.9)
                                  ,("no'e",stv 0.5  0.9)
                                  ,("na'e",stv 0.25 0.9)
                                  ,("to'e",stv 0    0.9)]

brivla :: Syntax Atom
brivla = handleFREEs2 <<< brivla' &&& frees

brivla' :: Syntax Atom
brivla' = implicationOf . predicate . gismu

meP :: Syntax Atom
meP = implicationOf . predicate . showReadIso
     <<< sepSelmaho "ME" &&> sumti

_MO :: Syntax Atom
_MO = varnode . word "mo"

_GOhA :: Syntax Atom
_GOhA = implicationOf . predicate <<< selmaho "GOhA"

gohaP :: Syntax Atom
gohaP = _MO <+> _GOhA

tanruKE :: Syntax Selbri
tanruKE = sepSelmaho "KE" &&> selbri_3 <&& optSelmaho "KEhE"

--FIXME: Should use DefineLink instead of EquivalenceLink but that doesnt' accept a
--PredicateNode only DefinedPredicateNode which messus patter matching in the rest
-- of the code
tanruSE :: Syntax Selbri
tanruSE = handle <<< selmaho "SE" &&& tanru_unit_2
    where handle = Iso f g where
            f (se,(tv,t@(PN name))) = let dpred = cPN (name ++ "_"++ se) noTv
                                          dsch  = cDSN se
                                          defl  = cEquivL noTv
                                                     dpred
                                                     (cEXOL noTv
                                                         [dsch
                                                         ,t
                                                         ])
                                      in do
                                          pushAtom defl
                                          pure (tv,dpred)
            --FIXME: all popAtom's should be findAtom's
            g (tv,dpred) = do
                (EquivL _ (EXOL [dsch,t])) <- popAtom
                let (DSN se) = dsch
                pure (se,(tv,t))

nuP :: Syntax Atom
nuP = maybeImpl . isoFoldl handleCon2 . manage
    <<< (selmaho "NU" &&& many ((just.joik_jek &&& insert Nothing)
                                &&& selmaho "NU"))
    &&& subsentence
    <&& optSelmaho "KEI"
    where manage = Iso f g
          f ((s,ls),a) = do
              a1 <- apply handleNU (s,a)
              as <- apply (mapIso (manage2 a)) ls
              pure (a1,as)
          g (a1,as) = do
              (s,a) <- unapply handleNU a1
              ls    <- unapply (mapIso (manage2 a)) as
              pure ((s,ls),a)

          manage2 :: Atom -> SynIso (Con,String) (Con,Atom)
          manage2 a = Iso f g where
              f (con,s) = do
                  newa <- apply handleNU (s,a)
                  pure (con,newa)
              g (con,newa) = do
                  (s,_) <- unapply handleNU (newa)
                  pure (con,s)

          handleNU = withEmptyState $ choice nuHandlers

          maybeImpl :: SynIso Atom Atom
          maybeImpl = Iso f g where
             f l@(Link _ _ _) = do
                 name <- randName (show l)
                 let pred = cPN name noTv
                 pushAtom $ cImpL noTv pred l
                 pure pred
             f a@(Node _ _ _) = pure a
             g pred = do
                 atoms <- gets sAtoms
                 case find (ff pred) atoms of
                     Just i@(ImpL _ _ l) -> rmAtom i >> pure l
                     Nothing             -> pure pred

             ff predP (ImpL _ predD _) = predP == predD
             ff _ _ = False


nuHandlers :: [SynIso (String,Atom) Atom]
nuHandlers = [handleNU "du'u" (mkNuEventLabel "du'u")           . rmfst "du'u",
              handleNU "su'u" (mkNuEventLabel "su'u")           . rmfst "su'u",
              handleNU "nu"   (mkNuEvent ["fasnu"])             . rmfst "nu",
              handleNU "mu'e" (mkNuEvent ["fasnu", "mokca"])    . rmfst "mu'e",
              handleNU "zu'o" (mkNuEvent ["zumfau"])            . rmfst "zu'o",
              handleNU "za'i" (mkNuEvent ["tcini"])             . rmfst "za'i",
              handleNU "ka"   (mkNuEvent ["ckaji"])             . rmfst "ka",
              handleNU "ni"   (mkNuEvent ["klani"])             . rmfst "ni",
              handleNU "si'o" (mkNuEvent ["sidbo"])             . rmfst "si'o",
              handleNU "li'i" (mkNuEventLabel "is_experience")  . rmfst "li'i",
              handleNU "pu'u" (mkNuEventLabel "is_point_event") . rmfst "pu'u",
              handleNU "jei"  (mkNuEventLabel "is_truth_value") . rmfst "jei"]
  where
    -- Functions that specify the type of the abstraction
    -- Note: atomNub may leave AndLink with only one atom
    mkNuEventLabel :: String -> String -> SynIso Atom [Atom]
    mkNuEventLabel eventName _ = tolist2 . (mkEval . atomNub . mkEvent &&& id)
      where mkEval = _eval . addfst (cPN eventName highTv)
                   . tolist2 . addfstAny (cCN "$2" highTv)

    mkNuEvent :: [String] -> String -> SynIso Atom [Atom]
    mkNuEvent (nuType:nts) name = isoConcat . tolist2 . addfstAny nuImps
                                            . tolist2 . (wrapNuVars &&& id)
     where
       nuPred = cPN (nuType ++ "_" ++ name) highTv

       nuImps = (cImpL highTv nuPred (cPN nuType highTv))
            :(case nts of
                [] -> []
                [nts] -> let nuSec = (cPN (nts ++ "_" ++ name) highTv)
                         in [(cIImpL highTv nuPred nuSec)
                            ,(cImpL highTv nuSec (cPN nts highTv))])

       wrapNuVars = andl . tolist2 . (mkEval "1" *** mkEval "2")
                         . addfstAny (cCN "$2" highTv) . atomNub . mkEvent

       mkEval num = _evalTv . addfstAny highTv
                            . addfstAny (cPN ("sumti" ++ num) highTv)
                            . tolist2
                            . addfstAny nuPred

    --Turns a Sentence into a conjunction of predicates
    mkEvent = atomIsoMap (mkIso f id) where
     f (CtxL _ (SL (evals:_))) = evals
     f (EvalL _ (PN _) (LL [vn@(VN _), _])) = vn -- can be PN:CN, PN:WN, anything else?
     f a = a

--As for "pu'u", "pruce" and "farvi" don't seem quite right

handleNU :: String -> (String -> SynIso Atom [Atom]) -> SynIso Atom Atom
handleNU abstractor nuTypeMarker = Iso f g where
  f atom = do
    rname <- randName $ (show atom)
    let name = rname ++ "___" ++ abstractor
        pred = cPN name highTv
    link <- apply (mkLink pred name nuTypeMarker) atom
    pushAtom link
    pure pred

  g (pred@(PN name)) = do
    state <- gets sAtoms
    atom <- case find (atomElem pred) state of -- remove "is_event" atoms
        Just l  -> unapply (mkLink pred name nuTypeMarker) l
        Nothing -> lift $ Left $ (show pred) ++ " can't be found in state."
    pure atom --should only be one. Check? Instatiate VNs???

mkLink :: Atom -> String -> (String -> SynIso Atom [Atom]) -> SynIso Atom Atom
mkLink pred name nuTypeMarker = mkLink' . addfstAny pred
                              . second (nuTypeMarker name)
                              . mkNuState . getPredVars

-- Extract predicateNodes from the atom and state
getPredVars :: SynIso Atom (([Atom], [Atom]), Atom)
getPredVars = mkIso f g where
  f atom =
    let predicateNodes =
          nub $ atomFold (\ns a -> case a of (EvalL _ _ (LL (pn@(PN _):_))) -> pn:ns
                                             (ImpL _ pn@(PN _) (PN _)) -> pn:ns
                                             (InhL _ pn@(PN _) (PN _)) -> pn:ns
                                             a -> ns) [] atom
        predicateVars = map (cVN.("$"++).show) [3..(length predicateNodes) + 2]
    in ((predicateVars,predicateNodes), atom)
  g (_, atom) = atom --FIXME, can't assume first atom is the atom

mkNuState :: SynIso (([Atom], [Atom]), Atom) ([Atom], Atom)
mkNuState = second replacePredicatesIso
          . inverse associate
          . first (rmsndAny [] . pid &&& isoZip . commute)

replacePredicatesIso :: SynIso ([(Atom,Atom)],Atom) Atom
replacePredicatesIso = mkIso f g where
  f (nodeVarMap,a) = atomMap (mapf nodeVarMap) a
  g a = ([],a) -- i.e., don't instantiate vars for now

  mapf nvm pn@(PN _) =
    case lookup pn nvm of
      Just vn -> vn
      Nothing -> pn
  mapf _ a = a

-- (pred, (typedPredicateVars, eventAtom:state')
mkLink' :: SynIso (Atom, ([Atom], [Atom])) Atom
mkLink' = _equivl
         . first  (_evalTv . addfst highTv  . addsnd [cVN "$1"])
         . second
             (_meml . addfst (cVN "$1") . ssl . tolist2 . addfst (cVN "$2")
             . _exl . first (varll . mapIso (_typedvarl . addsnd (cTN "PredicateNode")))
             . second andl)

_MOI :: Syntax String
_MOI = selmaho "MOI"

moiP :: Syntax Atom
moiP = implicationOf . predicate . handleMOI
     <<< (number &&& _MOI)
    where handleMOI = mkIso f g
          f (a,s) = nodeName a ++ '-':s
          g name  = let nn = takeWhile (/= '-') name
                        s  = drop 1 $ dropWhile (/= '-') name
                    in if isNumeric nn
                          then (Node "NumberNode"  nn noTv,s)
                          else (Node "ConceptNode" nn noTv,s)


-------------------------------------------------------------------------------
--bacru
-------------------------------------------------------------------------------


handleModalSumti :: SynIso (Atom,[Atom]) (Maybe Atom)
handleModalSumti = handle . mapIso handleModalSumti' . isoDistribute
    where handle = Iso f g
          f [] = pure Nothing
          f as = Just <$> apply andl as
          g Nothing   = pure []
          g (Just al) = unapply andl al

handleModalSumti' :: SynIso (Atom,Atom) Atom
handleModalSumti' = mkIso f g where
    f (pred,a) = atomMap (fun pred) a
    g atom = (cVN "ignore when printing",cVN "ignore when priting2")

    fun pred1 (EvalL _ _ (LL [pred2@(PN _),_])) = cImpL noTv pred1 pred2
    fun pred1 (EvalL _ pred2 _) = cImpL noTv pred1 pred2 --Time/Space
    fun _ a = a

--For mergin sumties before and after the selbri into a single list
mergeSumti :: (a ~ aa) => SynIso ([a],(s,[aa])) (s,[a])
mergeSumti = Iso f g where
    f ([],(_,[])) = lift $ Left "No Sumti to merge."
    f (a1,(s,a2)) = pure (s,a1++a2)
    g     (s,a)   = case a of
                       [] -> lift $ Left "No Sumti to reverse merge."
                       (x:xs) -> pure ([x],(s,xs))

addti :: SynIso String String
addti = mkIso f g
    where f s = s ++ "ti "
          g s = s --TODO: Maybe remoe ti again

------------------------------------
--Free
-----------------------------------

data Free = FNull | FUI [UI]
          deriving (Show,Eq)

fNull :: SynIso () Free
fNull = Iso f g where
    f () = pure FNull
    g FNull = pure ()
    g a = lift $ Left (show a ++ "is not a FNull")

fUI :: SynIso [UI] Free
fUI = Iso f g where
    f ui  = pure (FUI ui)
    g (FUI ui) = pure ui
    g a = lift $ Left (show a ++ "is not a FUI")

handleFREEs2 :: SynIso (Atom,[Free]) Atom
handleFREEs2 = isoFoldl handleFREE

handleFREEs :: SynIso (Atom,[Free]) Atom
handleFREEs = listl . cons . addfst (cAN "frees")
                    . cons . second gsAtoms . unit
                    . isoFoldl handleFREE

handleFREE :: SynIso (Atom,Free) Atom
handleFREE = (iunit     . second (inverse fNull))
         <+> (handleUIs . second (inverse fUI))

frees :: Syntax [Free]
frees = many free

free :: Syntax Free
free = atomToFNull . sei
  -- <+> soi
  -- <+> vocative &&& optional relative_clauses
  --              &&& selbri
  --              &&& optional relative_clauses
  --              <&& optSelmaho "DOhU"

  -- <+> vocative &&& optional relative_clauses
  --              &&& some cmene
  --              &&& optional relative_clauses
  --              <&& optSelmaho "DOhU"

   <+> atomToFNull . voc1
   <+> fUI . indicators
    where atomToFNull = fNull . toState 1 . tolist1

voc1 :: Syntax Atom
voc1 = (listl . cons . addfst (cAN "vocative1")
        . (mapIso handleVOC1 . isoDistribute . commute ||| id) . ifJustB
       <<< vocatives &&& optional sumti <&& optSelmaho "DOhU"
       )
    where handleVOC1 = _eval . second tolist1 . commute


--Vice Versa
--soi :: Syntax Atom
--soi = sepSelmaho "soi" &&> sumti &&& optional sumti <&& optSelmaho "SEhU"

{-
free :: SyntaxState s => Syntax s [ADT]
free = adtSyntax "free" <<<
    <+> (number <+> lerfu_string) &+& adtSelmaho "MAI"
    <+> adtSelmaho "TO" &+& text
                        &+& listoptional (adtSelmaho "TOI")
    <+> adtSelmaho "XI" &+& listoptional (concatSome free)
                        &+& (number <+> lerfu_string)
                        &+& listoptional (adtSelmaho "BOI")
    <+> adtSelmaho "XI" &+& listoptional (concatSome free)
                        &+& adtSelmaho "VEI"
                        &+& listoptional (concatSome free)
                        &+& mex
                        &+& listoptional (adtSelmaho "VEhO")
-}

------------------------------------
--Second Order Statments
-----------------------------------

type SEI = Atom

--SEIs are second order Statments that can appear almost anywhere
sei :: Syntax Atom
sei = handleBRIDI . commute
   <<< sepSelmaho "SEI" &&> ((terms <&& optSelmaho "CU") <+> insert [])
                        &&& selbri
                        <&& optSelmaho "SEhU"

vocatives :: Syntax [Atom]
vocatives = mapIso (implicationOf . predicate) . merge
    <<< oooob (some (handle <<< selmaho "COI" &&& optional (selmaho "NAI"))) []
              (tolist1 . selmaho "DOI")                                      []
    where handle :: SynIso (String,Maybe String) String
          handle = mkIso f g
          f (c,Just n)  = c ++ n
          f (c,Nothing) = c
          g s = let (ms,mn) = splitAt (length s - 3) s
                in case mn of
                    "nai" -> (ms,Just mn)
                    _     -> (s,Nothing)

          merge = mkIso f g where
            f (a,b) = a ++ b
            g _ = error $ "Not implemented vocative merge g."
--Attitude
type UI = (Atom,TruthVal)

indicators :: Syntax [UI]
indicators = some indicator
    -- sepSelmaho "FUhE" &&> some indicator

indicator :: Syntax UI
indicator = uiP
 -- <+> adtSelmaho "Y" FIXME??? Not relevant for text
 -- <+> adtSelmaho "DAhO" FIXME resets various things to default
 -- <+> adtSelmaho "FUhO"

uiP :: Syntax (Atom,TruthVal)
uiP = handle <<< oooob (_UI &&& naiP) (gehe,1) caiP 0.5
    where handle = second handleNAICAI . inverse associate
          handleNAICAI = mkIso f g where
              f (n,c) = stv ((n*c)/2+0.5) 0.9
              g (SimpleTV s _) = let v = (s-0.5)*2
                             in if v >= 0
                                then (1,v)
                                else (-1,-v)
          gehe = cCN "ge'e" noTv

_UI :: Syntax Atom
_UI = concept <<< (xu <+> selmaho "UI")
    where xu = setFlagIso "xu" <<< word "xu"

naiP :: Syntax Double
naiP = handleNAI <<< (selmaho "NAI" <+> insert "")
    where handleNAI = mkSynonymIso [("nai"  , -1)
                                   ,(""     , 1)
                                   ,("ja'ai", 1)
                                   ]
caiP :: Syntax Double
caiP = handleCAI <<< (selmaho "CAI" <+> (insert "" . ifFlag "HaveUI"))
    where handleCAI :: SynIso String Double
          handleCAI = mkSynonymIso [("cai"    ,0.99)
                                   ,("sai"    ,0.75)
                                   ,(""       ,0.5 )
                                   ,("ru'e"   ,0.25)
                                   ,("cu'i"   ,0.01)
                                   ]

handleUIs :: SynIso (Atom,[UI]) Atom
handleUIs = isoFoldl (handleUI . commute)

handleUI :: SynIso ((Atom,TruthVal),Atom) Atom
handleUI = (handleXU ||| handleUI') . switchOnFlag "xu"
    where handleXU = rmFlagIso "xu" . addXUIso . rmfstAny (xu,tv)
          xu = Node "ConceptNode" "xu" noTv
          tv = stv 0.75  0.9

handleUI' :: SynIso ((Atom,TruthVal),Atom) Atom
handleUI' = sndToState 1
          . second (tolist1 . _frames . first selbri)
          . manage
    where manage = mkIso f g where
              f ((ui,tv),a)     = (a,((tv,ui),[(getPred a,"1")]))
              g (a,((tv,ui),_)) = ((ui,tv),a)

          getPred (LL [_,CtxL _ (SL ((EvalL _ _ (LL [pred,_])):_)) ]) = pred
          getPred a = a

          selbri  = second instanceOf

handleSEI :: SynIso (Atom,Atom) Atom
handleSEI = fstToState 1 . first tolist1

-------------------------------------------------------------------------------
--Space Time Utils
-------------------------------------------------------------------------------

oooob :: (Eq a,Show a,Eq b,Show b) => Syntax a -> a -> Syntax b -> b -> Syntax (a,b)
oooob syna defa synb defb =
    rmFlagIso "SynA" <<< (setFlagIso "SynA" . syna <+> insert defa)
                     &&& (synb <+> insert defb . ifFlag "SynA")

oooobm :: (Eq a,Show a,Eq b,Show b) => Syntax a -> Syntax b -> Syntax (Maybe a,Maybe b)
oooobm syna synb =
    rmFlagIso "SynA" <<< (setFlagIso "SynA" . just . syna <+> insert Nothing)
                     &&& (just . synb <+> insert Nothing . ifFlag "SynA")

--FIXME This is wrong use the slow version below???
mergeMaybe :: (Eq a,Show a) => SynIso (a,a) a
           -> Syntax (Maybe a,Maybe a)
           -> Syntax a
mergeMaybe iso syn = handle . syn
    where handle = Iso f g
          f (Just a,Just b)  = apply iso (a,b)
          f (Just a,Nothing) = pure a
          f (Nothing,Just a) = pure a
          f (Nothing,Nothing)= lift $ Left "Need at least one to mergeMaybe"
          g _ = error $ "mergeMaybe g using wrong but fast implementeation"

--FIXME super slow tyr to get syn out of the alternatives
--mergeMaybe iso syn = (iso . (inverse just *** inverse just) . syn)
--                   <+> (inverse just . rmsnd Nothing . syn)
--                   <+> (inverse just . rmfst Nothing . syn)

mergePredicates :: SynIso (Atom,Atom) Atom
mergePredicates = Iso f g where
    f (p1,p2) = do
        name <- randName (show p1 ++ show p2)
        let pred = cPN name noTv
        pushAtom $ cImpL noTv pred (cAL noTv [p1,p2])
        pure pred
    g pred = error $ "Not Implemented g mergePredicates"

mergePredicatesSeq :: SynIso (Atom,Atom) Atom
mergePredicatesSeq = Iso f g where
    f (p1,p2) = do
        let p1name = nodeName p1
            p2name = nodeName p2
            p1pred = drop 20 p1name
            p2pred = drop 20 p2name
        name <- randName (p1name ++ p2name)
        name2 <- randName (p1pred ++ p2pred)
        let pred = cPN (name ++ "___" ++ name2) noTv
        pushAtom $ cEquivL noTv (cEvalL noTv pred (cLL [cVN "$1",cVN "$3"]))
                                (cAL noTv [ cEvalL noTv p1 (cLL [cVN "$1",cVN "$2"])
                                          , cEvalL noTv p2 (cLL [cVN "$2",cVN "$3"])
                                          ])
        pure pred
    g pred = error $ "Not Implemented g mergePredicatesSeq"

imply :: String -> SynIso Atom Atom
imply string = Iso f g where
     f a = pushAtom (cImpL noTv a (cPN string noTv)) >> pure a
     g a = popAtom >> pure a

selmahoPred :: String -> Syntax Atom
selmahoPred s = implicationOf . imply s . predicate . selmaho s


-------------------------------------------------------------------------------
--SpaceTime
-------------------------------------------------------------------------------

space_time :: Syntax Atom
space_time = mergeMaybe mergePredicates ((just . space &&& optional time)
                                         <+>
                                         (optional space &&& just . time))

time :: Syntax Atom
time = mergeMaybe mergePredicatesSeq (oooobm time_offset time_interval)

time_offset :: Syntax Atom
time_offset = general_offset "PU" "ZI"

time_interval :: Syntax Atom
time_interval = mergeMaybe mergePredicates (oooobm time_interval' interval_property)

time_interval' :: Syntax Atom
time_interval' = handle_interval <<< selmahoPred "ZEhA" &&& optional (selmahoPred "PU")
    where handle_interval = Iso f g where
          f (zeha,pu) = do
              let zehaname = nodeName zeha
                  puname = maybe "" nodeName pu
              name <- randName (zehaname ++('_':puname))
              let pred = cPN (name ++ "interval") noTv
              pushAtom $ cImpL noTv pred zeha
              case pu of
                  Just pu -> pushAtom $ cImpL noTv pred pu
                  Nothing -> pushAtom $ cImpL noTv pred (cPN "PU" noTv)
              pure pred
          g _ = error "Not implemented g time_interval'"


space :: Syntax Atom
space = mergeMaybe mergePredicatesSeq (oooobm space_offset space_interval)

space_offset :: Syntax Atom
space_offset = general_offset "FAhA" "VA"

general_offset :: String -> String -> Syntax Atom
general_offset dir mag = isoFoldl mergePredicates . inverse cons . mapIso (handle_offset dir mag)
            <<< some (oooobm (selmahoPred dir) (selmahoPred mag))
            <+> (insert [(Nothing,Nothing)] . ifFlag "WithDefaultTenses")

handle_offset :: String -> String -> SynIso (Maybe Atom,Maybe Atom) Atom
handle_offset dirC magC = Iso f g
    where f (mdir,mmag) = do
              let dirname = maybe "" nodeName mdir
                  magname = maybe "" nodeName mmag
              name <- randName (dirname ++('_':magname))
              let pred = cPN (name ++ "___offset") noTv
              case mdir of
                  Just dir -> pushAtom $ cImpL noTv pred dir
                  _ -> pushAtom $ cImpL noTv pred (cPN dirC noTv)
              case mmag of
                  Just mag -> pushAtom $ cImpL noTv pred mag
                  _ -> pushAtom $ cImpL noTv pred (cPN magC noTv)
              pure pred
          g pred = error $ "Not Implemented g handle_offset"


space_interval :: Syntax Atom
space_interval = mergeMaybe mergePredicates (oooobm space_interval' space_int_prop)

space_interval' :: Syntax Atom
space_interval' = handle_interval
              <<< mergeMaybe mergePredicates (oooobm (selmahoPred "VEhA")
                                                     (selmahoPred "VIhA"))
              &&& optional (selmahoPred "FAhA")
    where handle_interval = Iso f g where
            f (pred,mfaha) = do
                case mfaha of
                    Just faha -> pushAtom $ cImpL noTv pred faha
                    Nothing -> pushAtom $ cImpL noTv pred (cPN "FAhA" noTv)
                pure pred
            g _ = error "Not Implemented"

space_int_prop :: Syntax Atom
space_int_prop = (setFlagIso "FEhE" . sepSelmaho "FEhE") &&> interval_property

interval_property :: Syntax Atom
interval_property = handle <<< handleROI . (number &&& selmahoPred "ROI")
                           <+> selmahoPred "TAhE"
                           <+> selmahoPred "ZAhO"
    where handle = Iso f g where
            f pred = do
                  flags <- gets sFlags
                  if "FEhE" `elem` flags
                     then pushAtom $ cImpL noTv pred (cPN "Spatial" noTv)
                     else pushAtom $ cImpL noTv pred (cPN "Temporal" noTv)
                  pushAtom $ cImpL noTv
                                    (cEvalL noTv pred
                                                 (cLL [cVN "$1",cVN "$2"]))
                                    (cSubL noTv (cVN "$1") (cVN "$2"))
                  pure pred
            g _ = error "Reverse Interval Property Not implemented."
          handleROI = Iso f g where
            f (pa,roi) = do
                  pushAtom $ cImpL noTv
                                    (cEvalL noTv roi
                                                 (cLL [cVN "$1",cVN "$2"]))
                                    (Link "SetSizeLink"  [cVN "$1",pa] noTv)
                  pure roi
            g roi = error "Handle Roi g not implemented"

-------------------------------------------------------------------------------
--Connective Utils
-------------------------------------------------------------------------------

optBool :: String -> Syntax Bool
optBool s = insert True . mytext s <+> insert False

ekPat :: Syntax String -> Syntax EK
ekPat syn = optBool "na"
        &&& optBool "se"
        &&& syn
        &&& optBool "nai"

_BO :: Syntax JJCTTS
_BO = stag <&& sepSelmaho "BO"

--type EK = (Bool,(Bool,(String,Bool)))

ek :: Syntax EK
ek = ekPat (selmaho "A")

jek :: Syntax EK
jek = ekPat (_JAtoA . selmaho "JA")

gihek :: Syntax EK
gihek = ekPat (_GIhAtoA . selmaho "GIhA")

--data JOIK = JOI (Bool,(String,Bool))
--          | INT (Bool,(String,Bool))
--          | INTGAhO (String,((Bool,(String,Bool)),String))
--          deriving (Show,Eq)

gek :: Syntax (Bool,(String,Bool))
gek = optBool "se" &&& (_GAtoA . selmaho "GA") &&& optBool "nai"
--  <+> joik &&& selmaho "GI"
--  <+> stag &&& gik

guhek :: Syntax (Bool,(String,Bool))
guhek = optBool "se" &&& (_GUhAtoA . selmaho "GUhA") &&& optBool "nai"

gik :: Syntax (Bool)
gik = sepSelmaho"GI" &&> optBool "nai"

--                   ( gek / guhek      )  a  gik  a
handleGIK :: SynIso ((Bool,(String,Bool)),(a,(Bool,a))) (Con,(a,a))
handleGIK = Iso f g where
    f ((bse,(s,bna)),(bridi1,(bnai,bridi2))) =
      pure ((Just $ Right(bna,(bse,(s,bnai))),Nothing),(bridi1,bridi2))
    g ((Just (Right (bna,(bse,(s,bnai)))),Nothing),(bridi1,bridi2)) =
      pure ((bse,(s,bna)),(bridi1,(bnai,bridi2)))

joik_JOI :: SynIso (Bool,(String,Bool)) JOIK
joik_JOI = Iso f g where
    f a = pure $ JOI a
    g (JOI a) = pure $ a
    g _ = lift $ Left "Not a JOI."

joik_INT :: SynIso (Bool,(String,Bool)) JOIK
joik_INT = Iso f g where
    f a = pure $ INT a
    g (INT a) = pure $ a
    g _ = lift $ Left "Not a INT."

joik_INTGAhO :: SynIso (String,((Bool,(String,Bool)),String)) JOIK
joik_INTGAhO = Iso f g where
    f a = pure $ INTGAhO a
    g (INTGAhO a) = pure $ a
    g _ = lift $ Left "Not a INTGAhO."

joik :: Syntax JOIK
joik = joik_JOI . (optBool "se" &&& selmaho "JOI" &&& optBool "nai")
   <+> joik_INT . interval
   <+> joik_INTGAhO . (selmaho "GAhO" &&& interval &&& selmaho "GAhO")
    where interval = optBool "se"
                   &&& selmaho "BIhI"
                   &&& optBool "nai"

joik_jek :: Syntax JOIK_JEK
joik_jek = left . joik <+> right . jek

joik_ek :: Syntax JOIK_EK
joik_ek = left . joik <+> right . ek

handleJJCTTS :: SynIso (JJCTTS,[Atom]) Atom
handleJJCTTS = Iso f g where
    f (CTLeaf (pred,Nothing),as) = do
        apply (_frames . second (handleTAG . toSumti)) (pred,as)

    f (CTLeaf (pred,Just "space_time"),as) = do
        apply handleSpaceTime (pred,as)

    f (CTNode joik_jek (x1,x2),as) = do
        a1 <- f (x1,as)
        a2 <- f (x2,as)
        case joik_jek of
            Right jek -> apply conLink (jek,(a1,a2))
            Left joik -> apply handleJOIK (joik,(a1,a2))
    g _ = error $ "handleJJCTTS g: not implemented."

    toSumti :: SynIso [Atom] [Sumti]
    toSumti = mkIso f g where
        f = map (\x -> (x,Nothing))
        g = map fst

    handleSpaceTime :: SynIso (Selbri,[Atom]) Atom
    handleSpaceTime = Iso f g where
        f ((tv,st),as) = do
            case as of
                [s] -> do
                    nctx <- (\x -> cCN x noTv) <$> randName (show s)
                    addCtx nctx
                    pure $ cEvalL tv st (cLL [nctx,s])
                _ -> pure $ cEvalL tv st (cLL as)
        g _ = error $ "Not Implemented g handleSpaceTime"

handleJJCTTS_Selbri :: SynIso (JJCTTS,Atom) Atom
handleJJCTTS_Selbri = Iso f g where
    f (CTLeaf ((_tv,pred),Nothing),selb) = do
        pushAtom $ cImpL noTv selb pred
        pure selb

    f (CTLeaf ((_tv,pred),Just "space_time"),selb) = do
        atom <- apply handleSpaceTime pred
        pushAtom atom
        pure selb

    f (CTNode joik_jek (x1,x2),s) = do
        f (x1,s)
        a1 <- popAtom
        f (x2,s)
        a2 <- popAtom
        case joik_jek of
            Right jek -> apply conLink (jek,(a1,a2)) >> pure s
            Left joik -> apply handleJOIK (joik,(a1,a2)) >> pure s
    g _ = error $ "handleJJCTTS g: not implemented."

    toSumti :: SynIso [Atom] [Sumti]
    toSumti = mkIso f g where
        f = map (\x -> (x,Nothing))
        g = map fst

    handleSpaceTime :: SynIso Atom Atom
    handleSpaceTime = Iso f g where
        f pred = do
            ctx <- gets (head.sCtx)
            nctx <- (\x -> cCN x noTv) <$> randName (show ctx)
            setPrimaryCtx nctx
            pure (cEvalL noTv
                      pred
                      (cLL [nctx,ctx])
                 )
        g _ = error "Not Implemented g handleSpaceTime_selbri"


--HandleCon Connectes the Atoms with 2 possible connectives
--Since booth connectives are in a Maybe we first lift the Atoms into the Maybe
--Then we fmap the isos for each Connective Type over the Maybes
--Finally we merge the results together or pick one
handleCon2 :: SynIso (Atom,(Con,Atom)) Atom
handleCon2 = handleCon . reorder
    where reorder = mkIso f g
          f (a1,(con,a2)) = (con,(a1,a2))
          g (con,(a1,a2)) = (a1,(con,a2))

handleCon :: SynIso (Con,(Atom,Atom)) Atom
handleCon = merge
          . (mapIso handle_joik_ek *** mapIso handle_jjctts)
          . reorder
    where handle_joik_ek = (handleJOIK ||| conLink) . expandEither
          handle_jjctts = handleJJCTTS .> tolist2

          reorder = mkIso f g where
              f ((s,ts),as)                 = (eM (s,as),eM (ts,as))
              g (Just (s,as) ,Just (ts,_))  = ((Just s,Just ts),as)
              g (Nothing     ,Just (ts,as)) = ((Nothing,Just ts),as)
              g (Just (s,as) ,Nothing)      = ((Just s,Nothing),as)
              eM (Just a,b)  = Just (a,b) --expand Maybe
              eM (Nothing,b) = Nothing

          toSumti = mkIso f g where
              f = map (\x -> (x,Nothing))
              g = map fst

          merge = mkIso f g where
              f (Just a,Just b) = Link "AndLink" [a,b] highTv
              f (Nothing,Just b) = b
              f (Just a,Nothing) = a
              f (Nothing,Nothing) = error "not allowed to happen."
              g l@EvalL{} = (Nothing,Just l)
              g (AL [a,b@EvalL{}]) = (Just a,Just b)
              g l = (Just l,Nothing)

          expandEither = mkIso f g where
              f (Left a,c)  = Left (a,c)
              f (Right b,c) = Right (b,c)
              g (Left (a,c) ) = (Left a,c)
              g (Right (b,c)) = (Right b,c)

handleJOIK :: SynIso (JOIK,(Atom,Atom)) Atom
handleJOIK = Iso f g where
    f (JOI (b1,(s,b2)),(a1,a2)) = do
        (s,(na1,na2)) <- apply handleEKMods ((b1,(False,(s,b2))),(a1,a2))
        pred <- apply implicationOf (cPN s noTv)
        name <- randName (show s)
        let new = case a1 of
                    PN _ -> cPN name noTv
                    _ -> cCN name noTv
            selbri = (selbriDefaultTV,pred)
        atom <- apply _frames (selbri,[(new,"1"),(na1,"2"),(na2,"3")])
        pushAtom atom
        pure new
    f (INT (b1,(s,b2)),(a1,a2)) = lift $ Left "handleJOIK not implemented"
    f (INTGAhO (s1,((b1,(s2,b2)),s3)),(a1,a2)) = lift $ Left "handleJOIK not implemented"
    g _ = error "handleJOIK g not implemented"














