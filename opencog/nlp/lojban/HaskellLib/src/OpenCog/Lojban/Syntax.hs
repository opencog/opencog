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
import Data.List (nub,partition,isInfixOf,isPrefixOf,(\\))
import Data.Foldable (find)
import Data.Maybe (fromMaybe,fromJust,listToMaybe)
import Data.Hashable

import System.Random

import Control.Category
import Control.Arrow hiding (left,right)
import Control.Applicative hiding (many,some,optional)
import Control.Monad.RWS
import Control.Monad.Trans.Class

import Iso
import Syntax hiding (SynIso,Syntax,text)

import Lojban hiding (brivla,cmevla)
import qualified Lojban as Morph (brivla,cmevla)
import Lojban.Syntax.Util hiding (brivla,cmevla)

import OpenCog.AtomSpace (Atom(..),TruthVal(..),noTv,stv,atomFold,nodeName,atomElem,atomMap,showAtom)
import OpenCog.Lojban.Util

import OpenCog.Lojban.Syntax.Types
import OpenCog.Lojban.Syntax.Util
import OpenCog.Lojban.Syntax.AtomUtil

import Debug.Trace

{-# ANN module "HLint: ignore Use camelCase" #-}
{-# ANN module "HLint: ignore Reduce duplication" #-}
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

lojban :: Syntax (Maybe Atom)
lojban = finalCheck <<< (just . text <+> insert Nothing) <&& optMorph "FAhO"

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
    <<< many (morph "NAI")
    &&& many (cmene <+> indicators)
    &&& optional (joik_jek)
    &&& text_1

text_1 :: Syntax Atom
text_1 = handle
    <<< optional (many (sepMorph "I")
                    --   &&> optional joik_jek
                    --   &&& optional (optional stag
                    --                 <&& sepMorph "BO"
                    --                )
                    -- )
                  <+>
                  many (sepMorph "NIhO") &&& frees
                 )

    &&& optional paragraphs
-}

text :: Syntax Atom
text = text_1 <+> free_frees

free_frees :: Syntax Atom
free_frees = filterDummy . handleFREEs . addfst dummy . ifEmptyFail . frees
    where filterDummy = mkIso f id where
            f (LL ls) = cLL (filter (not . atomAny (== dummy)) ls)
          dummy = cCN "dummy" noTv
          ifEmptyFail = Iso f g
          f [] = lift $ Left "Empty free_frees"
          f a = pure a
          g a = pure a

text_1 :: Syntax Atom
text_1 = ((handleCON ||| handleNIhO) . distribute ||| id) . ifJustA
        <<< optional (left . checkEmpty . (manySep (sepMorph "I")
                      &&> optional joik_jek
                      &&& (optional (stag <&& sepMorph "BO")
                          <+> nothing <<< optMorph "BO"))
                      <+>
                      right . (someNIhO &&& frees))
        &&& paragraphs
    where handleNIhO = listl . tolist2
                             . second (handleFREEs.commute)
                             . inverse associate
          handleCON = handleCon . second (addfst $ cCN "dummy" noTv) --FIXME no dummy
          someNIhO = mkIso f g . countNulls . some (sepMorph "NIhO")
          f i = cAN ("paragraphLevel" ++ show i)
          g (AN name) = read $ drop 14 name

checkEmpty :: SynIso Con Con
checkEmpty = Iso f g where
    f (Nothing,Nothing) = lift $ Left "No connectives"
    f a = pure a
    g a = pure a

countNulls :: SynIso [()] Int
countNulls = mkIso f g where
    f = length
    g n = replicate n ()

manySep :: Syntax () -> Syntax ()
manySep iso = (manySep iso <+> id) . iso <+> insert ()

paragraphs :: Syntax Atom
paragraphs = listl . cons . addfst (cAN "paragraphs") . cons
    <<< ((handleFREEs . commute ||| id) . ifJustA
         <<< optional (sepMorph "NIhO" &&> frees)
         &&& paragraph
        )
    &&& many (handleFREEs . commute <<< sepMorph "NIhO"
                                    &&> frees
                                    &&& paragraph)

paragraph :: Syntax Atom
paragraph = listl . cons . addfst (cAN "paragraph") . cons
    <<< (handleFREEs . commute
         <<< optMorph "I"
         &&> frees
         &&& (statement <+> fragment)
        )
    &&& many (handleFREEs . commute <<< sepMorph "I"
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
                  if x
                     then do
                         interinst <- apply instanceOf inter
                         tvls <- gets sTVLs
                         setTVLs []
                         case tvls of
                             [] -> pure $ cInhL noTv interinst
                                                     (cSSScL noTv [a])
                             ls -> pure $ cInhL noTv interinst
                                                     (cSSScL noTv [cLL ls,a])
                     else pure a
              g (InhL _ _ (SSScL [a])) = pure a
              g a                      = pure a

statement' :: Syntax Atom
statement' = listl . cons . addfst (cAN "statement") . cons
        <<< (statement_1 <+> (prenex &&> statement)) &&& gsAtoms

statement_1 :: Syntax Atom
statement_1 = isoFoldl handleCon2
    <<< statement_2 &&& many (sepMorph "I"
                              &&>
                              (just.joik_jek &&& insert Nothing)
                              &&&
                              statement_2 --FIXME officaly optional
                             )

statement_2 :: Syntax Atom
statement_2 = (handleCon2 ||| id) . ifJustB
    <<< statement_3 &&& optional (sepMorph "I"
                                  &&> (checkEmpty <<< optional joik_jek
                                                  &&& optional stag)
                                  &&& sepMorph "BO"
                                  &&> statement_2 --FIXME officaly optional
                                 )

statement_3 :: Syntax Atom
statement_3 = sentence
            <+> (handle <<< optional tag <&& sepMorph "TUhE" &&& frees
                                         &&& text_1
                                         &&& ((sepMorph "TUhU" &&> frees)
                                              <+> insert [])
                )
    where handle = (handleJJCTTS .> tolist1 ||| id) . ifJustA
                 . second handleFREEs2 . reorder
          reorder = mkIso f g where
              f (mt,(f1,(t,f2))) = (mt,(t,f1++f2))
              g (mt,(t,f)) = (mt,(f,(t,[])))

--FIXME
fragment :: Syntax Atom
fragment = listl . cons . addfst (cAN "fragment") . gsAtoms
    -- <<< ek
    -- <+> gihek
       <<< (toState 1 . tolist1 . quantifier)
    -- <+> morph "NA"
       <+> (termsToState <<< terms <&& optMorph "VAU")
       <+> prenex
       <+> (toState 1 . tolist1 . listl <<< relative_clauses)
       <+> (termsToState <<< linkargs)
       <+> (termsToState <<< links)
    where termsToState = toState 1 . tolist1 . listl . mapIso (rmsndAny Nothing)

--FIXME find best/correct representation
prenex :: Syntax ()
prenex = toState 1 . tolist1 . handleFREEs . first zohuAnchor
       <<< terms <&& sepMorph "ZOhU" &&& frees
    where zohuAnchor = listl . tolist2 . addfst (cAN "ZOhU")
                             . listl . mapIso (rmsndAny Nothing)


sentence :: Syntax Atom
sentence = withCleanState sentence'

sentence' :: Syntax Atom
sentence' = handleCTX . handleFREEs2 . first handleBTCT
          . associate . second commute
    <<< termsM <&& optMorph "CU" &&& frees &&& bridi_tail
    where handleCTX = Iso f g where
              f a = do
                  atoms <- gets sAtoms
                  now <- gets sNow
                  case now of
                      CN "NOCTX" -> pure (cSL (cAN "RelativePhrase" : a : atoms))
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
            -- (NA,frames)
            . second handleSelbriSumtis
            -- (NA,(Selbri,sumti))
            . inverse associate
            -- ((NA,Selbri),sumti)

handleNA :: SynIso (Maybe NA,Atom) Atom
handleNA = (iso ||| id) . ifJustA
    where iso = Iso f g
          f ((n,[(CN "ge'e",tv)]),a) = apply _evalTv (tv,(cGPN n lowTv,[a]))
          f _ = lift $ Left "handleNA.iso.f can't handle non ge'e UIs"
          g (EvalL tv (GPN n) a) = pure ((n,[(cCN "ge'e" noTv,tv)]),a)

handleSelbriSumtis :: SynIso (Selbri,[Sumti]) Atom
handleSelbriSumtis = merge
                   . (_frames *** handleModalSumti)
                   . reorder
                   . second (splitSumti
                            . (mapIso handleJAI . ifFlag "JAI" <+> id)
                            . handleTAG)

    where splitSumti :: SynIso [(Atom,Tag)] ([(Atom,Tag)],[Atom])
          splitSumti = mkIso f g where
              f = f' ([],[])
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
                      Nothing  -> do
                          name <- randName (showAtom a)
                          pred <- apply implicationOf (cPN "ckini" noTv)
                          e <- apply concept name
                          f1 <- apply _frame ((noTv,pred),(a,"1"))
                          f2 <- apply _frame ((noTv,pred),(e,"2"))
                          pushAtoms [f1,f2]
                          pure (e,"jai")
              f a = pure a
              g = pure --Loosing information


bridi_tail :: Syntax BTCT
bridi_tail = (extendBTCT ||| id) . ifJustB
        <<< bridi_tail_1
        &&& optional (((just.right.gihek &&& optional stag)
                       &&& sepMorph "KE" &&> bridi_tail <&& optMorph "KEhE"
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
                   &&& sepMorph "BO"
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
              g  btct        = let sumti = reverse $ foldl ff [] btct
                                   nbtct = fmap (tCTBTSa sumti) btct
                               in (nbtct,sumti)

              aSTBTCT sumti2 (selbri,sumti1) = (selbri,sumti1++sumti2)
              tCTBTSa sumti2 (selbri,sumti1) = (selbri,sumti1 \\ sumti2)

              ff ls (selbri,sumti) = fun (reverse sumti) ls
              fun a [] = a
              fun (a:as) (b:bs) | a == b    = a : fun as bs
                                | otherwise = []

bridi_tail_3 :: Syntax (SelbriNA,[Sumti])
bridi_tail_3 = (((second.second) handleKO . selbri) &&& tail_terms)
             <+> gek_sentence
    where handleKO :: SynIso Atom Atom
          handleKO = (sndToState 1 . second (tolist1 . impl) . reorder . ifFlag "ko") <+> id
              where reorder = mkIso f g where
                      f selbri = (selbri,[selbri,cPN "Imperative" lowTv])
                      g (selbri,_) = selbri


gek_sentence :: Syntax (SelbriNA,[Sumti])
gek_sentence = (addfstAny Nothing . addfstAny selbriDefaultTV
               . handleCon . handleGIK
           <<< gek &&& subsentence
                   &&& gik
                   &&& subsentence)
          &&& tail_terms
--    <+> optional tag &&& sepMorph "KE"
--                     &&> gek_sentence
--                     &&& optMorph "KEhE"
--    <+> morph "NA" &&& gek_sentence

-------------------------------------------------------------------------------
--Sumti
-------------------------------------------------------------------------------

tail_terms :: Syntax [Sumti]
tail_terms = termsM <&& optMorph "VAU"

terms :: Syntax [Sumti]
terms = some term

termsM :: Syntax [Sumti]
termsM = many term

--FIXME Implement TermSets
--terms_1 :: Syntax [Sumti]
--terms_1 = terms_2 &&& many (sepMorph "PEhE" &&> joik_jek &&& terms_2)

--terms_2 :: Syntax [Sumti]
--terms_2 = cons <<< term &&& many (sepMorph "CEhE" &&& term)

zohe = cCN "zo'e" noTv

optSelWithFrees :: String -> Syntax [Free]
optSelWithFrees sel = mltol <<< optional (sepMorph sel &&> frees)
    where mltol = mkIso f g
          f (Just a)  = a
          f (Nothing) = []
          g [] = Nothing
          g a  = Just a

term :: Syntax Sumti
term = (sumti &&& insert Nothing)

    <+> (modalSumti . second handleFREEs2
         <<< tag
         &&& ((sumti &&& insert [])
              <+>
              (addfst zohe . optSelWithFrees "KU")
             )
        )
    <+> (commute . second handleFREEs2
         <<< _FA
         &&& ((sumti &&& insert [])
              <+>
              (addfst zohe . optSelWithFrees "KU")
             )
        )
 -- <+> termset
 -- <+> morph "NA" <&& sepMorph "KU"
    <+> placeholder --Not normal Lojban

    where _FA = just . faToPlace . morph "FA"

          faToPlace :: SynIso String String
          faToPlace = mkSynonymIso [("fa","1")
                                   ,("fe","2")
                                   ,("fi","3")
                                   ,("fo","4")
                                   ,("fu","5")
                                   ,("fai","fai")
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
    <<< sumti_1 &&& optional (sepMorph "VUhO" &&> relative_clauses)

sumti_1 :: Syntax Atom
sumti_1 = (handleCon2 ||| id) . ifJustB
     <<< sumti_2 &&& optional ((just . joik_ek &&& optional stag)
                                <&& sepMorph "KE"
                                &&& sumti
                                <&& optMorph "KEhE"
                               )

sumti_2 :: Syntax Atom
sumti_2 = isoFoldl handleCon2
      <<< sumti_3 &&& many ((just . joik_ek &&& insert Nothing) &&& sumti_3)

sumti_3 :: Syntax Atom
sumti_3 = (handleCon2 ||| id) . ifJustB
     <<< sumti_4 &&& optional ((just . joik_ek &&& optional stag)
                                <&& sepMorph "BO"
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

handleRelClause :: SynIso (Atom,Maybe [Atom]) Atom
handleRelClause = (isoFoldl handleKEhA ||| id) . ifJustB

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
       <+> laheP
       <+> le
       <+> laP
       <+> liP
       <+> zoP
       <+> luP
       <+> lerfuP

laheP :: Syntax Atom
laheP = handleRelClause
      . first (handleFREEs2 . first (handle . first (choice ls)))
      . reorder
        <<<
        ((morph "LAhE" &&& frees)
         <+>
         (morph "NAhE" <&& sepMorph "BO" &&& frees))
        &&& optional relative_clauses
        &&& sumti
        &&& optional (sepMorph "LUhU" &&> frees)
    where reorder = mkIso f g where
            f ((lahe,f1),(mr,(s,Just f2))) = (((lahe,s),f1++f2),mr)
            f ((lahe,f1),(mr,(s,Nothing))) = (((lahe,s),f1),mr)
            g (((lahe,s),f),mr) = ((lahe,[]),(mr,(s,Just f)))
          handle = Iso f g
          f ((lahe,idx1,idx2),sumti) = do
              name <- randName (showAtom sumti)
              let referent = cCN name noTv
              l <- apply _frames ((highTv,lahe),[(sumti,idx1),(referent,idx2)])
              pushAtom l
              pure referent
          g = error "Not implemente laheP.hangle.g"
          --FIXME find predicates for no'e/je'a
          ls = [insert (cPN "sinxa" highTv,"1","2") . ignore "la'e"
               ,insert (cPN "sinxa" highTv,"2","1") . ignore "lu'e"
               ,insert (cPN "cmima" highTv,"1","2") . ignore "lu'a"
               ,insert (cPN "cmima" highTv,"2","1") . ignore "lu'i"
               ,insert (cPN "gunma" highTv,"1","2") . ignore "lu'o"
               ,insert (cPN "porsi" highTv,"1","3") . ignore "vu'i"
               ,insert (cPN "drata" highTv,"1","2") . ignore "na'e"
               ,insert (cPN "dukti" highTv,"1","2") . ignore "to'e"
               ,insert (cPN "ckini" highTv,"1","2") . ignore "tu'a"
               ,insert (cPN "no'ebo" highTv,"1","2") . ignore "no'e"
               ,insert (cPN "je'abo" highTv,"1","2") . ignore "je'a"
               ]


--FIXME handle default assingment cll exampel 17.23
lerfuP :: Syntax Atom
lerfuP = (handleFREEs2 ||| id) . ifJustB
      <<< lerfu_string &&& optional (sepMorph "BOI" &&> frees)


--TODO: Check if "-" as seperator coudl cause issues
lerfu_string :: Syntax Atom
lerfu_string = instanceOf . concept . isoIntercalate "-" . cons
            <<< lerfu_word &&& many (morph "PA" <+> lerfu_word)

--FIXME handle ga'e to'a upper/lower caes
lerfu_word :: Syntax String
lerfu_word = morph "BY"
         <+> (tolist2 <<< consonant &&& token ((==) 'y'))
         <+> handle . anyWord
 -- <+> adtMorph "LAU" &+& lerfu_word
 -- <+> adtMorph "TEI" &+& lerfu_string &+& adtMorph "FOI"
    where handle = Iso f g
          f a = case (drop (length a - 2) a) of
                    "bu" -> pure a
                    _    -> lift $ Left "No BU"
          g abu = pure  abu

opt_relative_clauses :: Syntax [Atom]
opt_relative_clauses = handle <<< optional relative_clauses
    where handle = mkIso f g
          f Nothing  = []
          f (Just l) = l
          g [] = Nothing
          g l  = Just l

relative_clauses :: Syntax [Atom]
relative_clauses = cons <<< relative_clause &&& many (sepMorph "ZIhE" &&> relative_clause)

relative_clause :: Syntax Atom
relative_clause = (goi <&& optMorph "GEhU") <+> (noi <&& optMorph "KUhO")
    where goi :: Syntax Atom
          goi = ptp (morph "GOI") goiToNoi noi where
              goiToNoi = mkSynonymIso [("pe "  ,"poi ke'a srana ")
                                      ,("po "  ,"poi ke'a se steci srana ")
                                      ,("po'e ","poi jinzi ke se steci srana ")
                                      ,("po'u ","poi ke'a du ")
                                      ,("ne "  ,"noi ke'a srana ")
                                      ,("no'u ","noi ke'a du ")
                                      ,("goi " ,"poi ke'a du ")]

          noi :: Syntax Atom
          noi = sepMorph "NOI" &&> ((hasKEhA <<< relSentence)
                                      <+>
                                      ptp bridi_tail addKEhA relSentence)
              where hasKEhA = Iso f f
                    f a = if atomAny (\case { Link{} -> False
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
   <<< ((setFlagValueIso "LE_FLAG" . morph "LE")
       <+>
       (setFlagIso "LA_FLAG" . sepMorph "LA"))
   &&> sumti_tail
   &&& optional (sepMorph "KU" &&> frees)

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
             (
              setWithSize
              <<<
              quantifier --can't have optional quant conflict with "pa moi"
              &&&
              ((isoFoldl handleKEhA ||| id) . ifJustB
               <<< (handle <<< selbri)
               &&& optional relative_clauses
              )
             )
             <+>
             ((isoFoldl handleKEhA ||| id) . ifJustB
              <<< (handle <<< selbri)
              &&& optional relative_clauses
             )
            )
            <+>
            (setWithSize <<< quantifier &&& sumti)
    where --myselbri = (addfst Nothing . addfst noTv . meP ||| selbri) . switchOnFlag "moi"
          varNode = addsnd [(Node "VariableNode" "$var" noTv,Nothing)]
          handle = (handleLA ||| handleLE) . switchOnFlag "LA_FLAG"
          handleLE = choice leHandlers . first (getFlagValueIso "LE_FLAG")
                   . commute . unit . _ssl . handleBRIDI . varNode

          --handleLA = (second.second) (Iso f g) where
          handleLA = Iso f g . rmfstAny noTv . rmfstAny Nothing where
              f pmeaning = do
                  cmene <- apply instanceOf (cPN "cmene" lowTv)
                  sumni <- apply instanceOf (cPN "sumni" lowTv)
                  name  <- randName (nodeName pmeaning)
                  named <- randName name
                  let cname  = cCN name lowTv
                      cnamed = cCN named lowTv
                      cmenet = (highTv,cmene)
                      sumnit = (highTv,sumni)
                  l1 <- apply _frames (cmenet,[(cname,"1"),(cnamed,"2")])
                  l2 <- apply _frames (sumnit,[(pmeaning,"1"),(cname,"2")])
                  pushAtom l1
                  pushAtom l2
                  pure cnamed
              g _ = error "Not implemente sumti_tail_1.handleLA.g"

          leHandlers = [genInstance "IntensionalInheritanceLink" . rmfst "le"
                       ,genInstance "SubsetLink"                 . rmfst "lo"
                       ,genInstance "InheritanceLink"            . rmfst "la"
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

{-laP :: Syntax Atom
laP = handleName . wordNode
    <<< sepMorph "LA"
    &&> anyWord
    <&& optMorph "KU"
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
                  pure a-}

laP :: Syntax Atom
laP = handleRelClause . first handleFREEs2 . reorder
    <<< sepMorph "LA"
    &&> frees
    &&& optional relative_clauses
    &&& handleName . listl . some (wordNode . Morph.cmevla)
    &&& frees
    <&& optMorph "KU"
    where handleName :: SynIso Atom Atom
          handleName = Iso f g where
              f a@(LL (n1:_)) = do
                  p <- apply instanceOf (cPN "cmene" lowTv)
                  name <- randName (nodeName n1 ++ "___" ++ nodeName n1)
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
          reorder = mkIso f g where
              f (f1,(mr,(n,f2))) = ((n,f1++f2),mr)
              g ((n,f),mr) = ([],(mr,(n,f)))

liP :: Syntax Atom
liP = sepMorph "LI" &&> (xo <+> number) <&& optMorph "LOhO"

xo :: Syntax Atom
xo = randvarnode <<< word "xo"

quantifier :: Syntax Atom
quantifier = number <&& optMorph "BOI"
       --FIXME <+> sepMorph "VEI" &&& mex &&& optMorph "VEhO"

number :: Syntax Atom
number = handleFREEs2 <<< number' &&& frees

number' :: Syntax Atom
number' =  (    numberNode    |||    concept   )
    . (showReadIso . paToNum |^| isoIntercalate " ")
    . cons
    -- <<< some (morphN 2 "PA") <&& sepSpace
    <<< morph "PA" &&& many (morph "PA" <+> lerfu_word)
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
    where koha = concept . morph "KOhA"
          ma   = varnode . word "ma"
          da   = concept . oneOfS word ["da","de","di"]
          ko   = setFlagIso "ko" . concept . word "ko"
          keha = concept . word "ke'a"

luP :: Syntax Atom
luP = instanceOf <<< sepMorph "LU" &&> text <&& optMorph "LIhU"

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
    g (CTLeaf a) = pure a
    g _ = lift $ Left "Not a CTLeaf."

tagPat :: Syntax (Tagged SelbriNA) -> Syntax JJCTTS
tagPat syn = isoFoldl toTree <<< ctLeaf . syn
                             &&& many (joik_jek &&& syn)
    where toTree :: SynIso (JJCTTS,(JOIK_JEK,Tagged SelbriNA)) JJCTTS
          toTree = Iso f g
          f (a,(c,b)) = pure $ CTNode c (a,CTLeaf b)
          g (CTNode c (a,CTLeaf b)) = pure (a,(c,b))
          g _ = lift $ Left "toTree.g: Doesn't match expected pattern."

tag :: Syntax JJCTTS
tag = tagPat tense_modal

stag :: Syntax JJCTTS
stag = tagPat simple_tense_modal

tense_modal :: Syntax (Tagged SelbriNA)
tense_modal = simple_tense_modal
            <+> addsnd Nothing . (sepMorph "FIhO" &&> selbri <&& optMorph "FEhU")

--FIXME: Not complete handeling for space_time CAha
--missing NAhE and KI handeling
simple_tense_modal :: Syntax (Tagged SelbriNA)
simple_tense_modal = _bai
                  <+> _space_time
                  <+> _CAhA
                  <+> _CUhE

_bai :: Syntax (Tagged SelbriNA)
_bai = addsnd Nothing . commute <<< ((_NAhE <+> insert (stv 0.75 0.9))
                                &&& sebai)
                                &&& optional (morph "NAI" &&& indicatorsNA)
                             -- &&& optional (morph "KI")
    where sebai = (handleSE ||| id) . ifJustA <<< optional (morph "SE") &&& bai

--FIXME: Should use DefineLink instead of EquivalenceLink but that doesnt' accept a
--PredicateNode only DefinedPredicateNode which messus patter matching in the rest
-- of the code
handleSE :: SynIso (String,Atom) Atom
handleSE = Iso f g where
    f (se,t@(PN name)) = let dpred = cPN (name ++ "_"++ se) noTv
                             dsch  = cDSN se
                             defl  = cEquivL noTv
                                        dpred
                                        (cEXOL noTv
                                            [dsch
                                            ,t
                                            ])
                         in do
                             pushAtom defl
                             pure dpred
    --FIXME: all popAtom's should be findAtom's
    g dpred = do
        (EquivL _ (EXOL [dsch,t])) <- popAtom
        let (DSN se) = dsch
        pure (se,t)


bai :: Syntax Atom
bai = ptp (morph "BAI") iso brivla
    where iso = Iso f g where
            f a = do
                btf <- asks wBai
                apply btf a
            g b = do
                btf <- asks wBai
                unapply btf b


_space_time :: Syntax (Tagged SelbriNA)
_space_time = addsnd (Just "space_time") . second (addfstAny noTv)
                                         . commute
                                         . space_time

_CAhA :: Syntax (Tagged SelbriNA)
_CAhA = addsndAny Nothing . addfst Nothing . addfstAny noTv
      . implicationOf . predicate . morph "CAhA"

_CUhE :: Syntax (Tagged SelbriNA)
_CUhE = addsndAny Nothing . addfst Nothing . addfstAny noTv
      . (randvarnode |||  implicationOf . predicate)
      . switchOnValue "cu'e" . morph "CUhE"

--Fails when the Syntax Succeds and the other way arround
--Either the syn succeds then we fail with the zeroArrow
--Or the right . insertc succeds because syn failed then we do nothing
notsyn :: Syntax a -> Syntax ()
notsyn x = (zeroArrow ||| id) . ((left <<< lookahead x) <+> (right . insert ()))

-------------------------------------------------------------------------------
--Selbri
-------------------------------------------------------------------------------

handleTanru :: SynIso (Selbri,Selbri) Selbri
handleTanru = sndToState 1 . second sel_iimpl . reorder
    where reorder = mkIso f g where
            f (g,t) = (t,(t,g))
            g(t,(_,g)) = (g,t)
          sel_iimpl = Iso f g where
            --FIXME do something sensible with the tvs
            f ((tv1,s1),(tv2,s2)) = apply (tolist1 . _iimpl) (s1,s2)
            g a = do
                (s1,s2) <- unapply (tolist1 . _iimpl) a
                pure ((noTv,s1),(noTv,s2))

selbri :: Syntax SelbriNA
selbri = (second.second) filterState . (handle ||| id) . ifJustA
      <<< optional tag &&& selbri_1
    where handle = Iso f g
          f (jjctts,(mna,(tv,s))) = do
              ns <- apply handleJJCTTS_Selbri (jjctts,s)
              pure (mna,(tv,ns))
          g (mna,(tv,ns)) = do
              (jjctts,s) <- unapply handleJJCTTS_Selbri ns
              pure (jjctts,(mna,(tv,s)))

--FIXME Can we just merge the uis???
selbri_1 :: Syntax SelbriNA
selbri_1 = (addfst Nothing . selbri_2)
       <+> (handle <<< morph "NA" &&& indicatorsNA &&& selbri)
    where handle = Iso f g --FIXME can be just combine it like that???
          f (na    ,(uis,(Nothing           ,s))) = pure (Just (na    ,uis)      ,s)
          f ("ja'a",(uis,(Just (na    ,uis2),s))) = pure (Just (na    ,uis++uis2),s)
          f ("na"  ,(uis,(Just ("ja'a",uis2),s))) = pure (Just ("na"  ,uis++uis2),s)
          f ("na"  ,(uis,(Just ("na"  ,uis2),s))) = pure (Just ("ja'a",uis++uis2),s)
          g (Nothing,s) = lift $ Left "Expecting NA"
          g (Just (na,uis),s) = pure (na,(uis,(Nothing,s)))

selbri_2 :: Syntax Selbri
selbri_2 = (handleTanru . commute ||| id) . ifJustB
        <<< selbri_3 &&& optional (sepMorph "CO" &&> selbri_2)

selbri_3 :: Syntax Selbri
selbri_3 = isoFoldl handleTanru . inverse cons <<< some selbri_4

selbri_4 :: Syntax Selbri
selbri_4 = ( handleSelbri4 ||| id) . ifJustB
        <<< selbri_5
        &&& optional (some (((just . joik_jek    &&& insert Nothing)
                              &&& selbriToEval . selbri_5)
                        <+> ((just . left . joik &&& optional stag)
                             &&& selbriToEval . tanruKE)))

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
                        <&& sepMorph "BO"
                      )
                     &&& selbriToEval . selbri_5)
    where handleSelbri5 = manageSelbriCon . handleCon2 .< selbriToEval


selbri_6 :: Syntax Selbri
selbri_6 = tanruBO <+> tanruGUHEK

tanruBO :: Syntax Selbri
tanruBO = (handleTanru ||| id) . ifJustB
       <<< tanru_unit &&& optional (sepMorph "BO" &&> selbri_6)

tanruGUHEK :: Syntax Selbri
tanruGUHEK = manageSelbriCon . handleCon . handleGIK
         <<< guhek &&& (selbriNAToEval . selbri) &&& gik &&& (selbriToEval . selbri_6)

tanru_unit :: Syntax Selbri
tanru_unit = isoFoldl handleCEI <<< tanru_unit_1
                                &&& many (sepMorph "CEI" &&> tanru_unit_1)
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
    <<< sepMorph "BE"
    &&> (cons <<<
              (handleBEFREEs <<< frees &&& tag2 . term)
              &&& (links <+> insert [])
        )
    &&& optional (sepMorph "BEhO" &&> frees)
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
              <<< sepMorph "BEI" &&> frees
                                   &&& term

--Also block selbri_4 when selbri_5+6 ???
tanru_unit_2 :: Syntax Selbri
tanru_unit_2 = addfst selbriDefaultTV
             . (brivla
                <+> nuP
                <+> meP
                <+> moiP
                <+> gohaP)
            <+> tanruSE
            <+> tanruKE
            <+> tanruNAhE
            <+> tanruJAI

tanruSE :: Syntax Selbri
tanruSE = second (handleFREEs2 . first handleSE) . reorder <<< morph "SE" &&& frees &&& tanru_unit_2
    where reorder = mkIso f g
          f (se,(frees,(tv,s))) = (tv,((se,s),frees))
          g (tv,((se,s),frees)) = (se,(frees,(tv,s)))

selbriDefaultTV = stv 0.75 0.9

selbriToEval :: SynIso Selbri Atom
selbriToEval = mkIso f g where
    f (tv,p) = cEvalL tv (cVN "$arg_place") (cLL [p,cVN "$arg"])
    g (EvalL tv _ (LL [p,_])) = (tv,p)

selbriNAToEval :: SynIso SelbriNA Atom
selbriNAToEval = mkIso f g where
    f (Nothing,(tv,p))
        = cEvalL tv (cVN "$arg_place") (cLL [p,cVN "$arg"])
    f (Just (na,[(CN "ge'e",natv)]),(ptv,p))
        = cEvalL natv
            (cGPN na noTv)
            (cLL [cEvalL ptv
                    (cVN "$arg_place")
                    (cLL [p,cVN "$arg"])
                 ]
            )
    f _ = error "Can't handle selbriNAToEval.f non ge'e UI"
    g (EvalL tv _ (LL [p,_]))
        = (Nothing,(tv,p))
    g (EvalL natv (GPN na) (LL [EvalL ptv _ (LL [p,_])]))
        = (Just (na,[(cCN "ge'e" lowTv,natv)]),(ptv,p))

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
tanruNAhE = handleNAhE <<< _NAhE &&& tanru_unit_2
    where handleNAhE = mkIso f g
          f (tv,(_,s)) = (tv,s)
          g (tv,s) = (tv,(noTv,s))

tanruJAI :: Syntax Selbri
tanruJAI = sepMorph "JAI"
         &&> setFlagIso "JAI" . jaiFlag . optional tag
         &&> tanru_unit_2
    where jaiFlag :: SynIso (Maybe JJCTTS) ()
          jaiFlag = Iso f g where
              f (Just a) = setJai a
              f (Nothing) = pure ()
              g () = gets sJAI

_NAhE :: Syntax TruthVal
_NAhE = naheToTV <<< morph "NAhE"
    where naheToTV = mkSynonymIso [("je'a",stv 1    0.9)
                                  ,("no'e",stv 0.5  0.9)
                                  ,("na'e",stv 0.25 0.9)
                                  ,("to'e",stv 0    0.9)]

brivla :: Syntax Atom
brivla = handleFREEs2 <<< brivla' &&& frees

brivla' :: Syntax Atom
brivla' = implicationOf . predicate <<< gismu <+> Morph.brivla

meP :: Syntax Atom
meP = (handleFREEs2 ||| id) . handle
     <<< sepMorph "ME" &&> frees
                       &&& me
                       &&& optional (morph "MEhU" &&& frees)
                       <&& (ignoreAny Nothing ||| failIfFound)
                           . switchOnFlag "moi"
                           . optional (lookahead (morph "MOI"))
    where me = iunit
             . second (toState 1 . tolist1 . _frame
                                 . (addfst noTv   *** addsnd "MEPlace")
               )
             <<< reorder . first implicationOf . addfst (cPN "me" noTv) . sumti
          reorder = mkIso f g where
              f (s,a)     = (s,(s,a))
              g (s,(_,a)) = (s,a)
          failIfFound = Iso f g where
              f (Nothing) = pure ()
              f (Just _ ) = lift $ Left "moi in me fail"
              g () = pure Nothing
          handle = mkIso f g
          f ([],(selb,Nothing)) = Right selb
          f (lsf1,(a,Nothing)) = Left (a,lsf1)
          f (lsf1,(a,Just (_,lsf2))) = Left (a,lsf1++lsf2)
          g (Left (a,lsf)) = ([],(a,Just ("me'u",lsf)))
          g (Right a)      = ([],(a,Just ("me'u",[])))

_MO :: Syntax Atom
_MO = randvarnode . word "mo"

--Could replace some words of class MOI
--for exampel moi with momkai
_MOI :: Syntax Atom
_MOI = implicationOf . predicate . morph "MOI"

moiP :: Syntax Atom
moiP = handleFREEs2 <<< moiP' &&& frees

moiP' :: Syntax Atom
moiP' = iunit
      . second (toState 1 . tolist1 . _frame
                          . (addfst noTv *** addsnd "MOIPlace")
               )
      . handleMOI
     <<< ((number <+> lerfu_string <+> withFlag "moi" me) &&& _MOI)
    where handleMOI = mkIso f g
          f (a,s) = (s,(s,a))
          g (s,(_,a)) = (a,s)
          me = reparse le . handleMEMOI . onlyText meP
          handleMEMOI = mkIso f g where
              f s = "le " ++ s ++ "ku "
              g s = drop 3 (take (length s - 3) s)



--FIXME handle specifc goha words
_GOhA :: Syntax Atom
_GOhA = implicationOf . predicate <<< morph "GOhA"

gohaP :: Syntax Atom
gohaP = handleFREEs2 <<< (_MO <+> _GOhA) &&& frees

tanruKE :: Syntax Selbri
tanruKE = (second handleFREEs2 ||| id) . handle
    <<< sepMorph "KE" &&> frees
                        &&& selbri_3
                        &&& optional (morph "KEhE" &&& frees)
    where handle = mkIso f g
          f ([],(selb,Nothing)) = Right selb
          f (lsf1,((tv,a),Nothing)) = Left (tv,(a,lsf1))
          f (lsf1,((tv,a),Just (_,lsf2))) = Left (tv,(a,lsf1++lsf2))
          g (Left (tv,(a,lsf))) = ([],((tv,a),Just ("ke'e",lsf)))
          g (Right selb) = ([],(selb,Just ("ke'e",[])))


nuP :: Syntax Atom
nuP = maybeImpl . isoFoldl handleCon2 . manage
    <<< (morph "NU" &&& many ((just.joik_jek &&& insert Nothing)
                                &&& morph "NU"))
    &&& subsentence
    <&& optMorph "KEI"
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
                  (s,_) <- unapply handleNU newa
                  pure (con,s)

          handleNU = withEmptyState $ choice nuHandlers

          maybeImpl :: SynIso Atom Atom
          maybeImpl = Iso f g where
             f l@Link{} = do
                 name <- randName (show l)
                 let pred = cPN name noTv
                 pushAtom $ cImpL noTv pred l
                 pure pred
             f a@Node{} = pure a
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

       nuImps = cImpL highTv nuPred (cPN nuType highTv)
            :(case nts of
                [] -> []
                [nts] -> let nuSec = cPN (nts ++ "_" ++ name) highTv
                         in [cIImpL highTv nuPred nuSec
                            ,cImpL highTv nuSec (cPN nts highTv)])

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
    rname <- randName $ show atom
    let name = rname ++ "___" ++ abstractor
        pred = cPN name highTv
    link <- apply (mkLink pred name nuTypeMarker) atom
    pushAtom link
    pure pred

  g (pred@(PN name)) = do
    state <- gets sAtoms
    atom <- case find (atomElem pred) state of -- remove "is_event" atoms
        Just l  -> unapply (mkLink pred name nuTypeMarker) l
        Nothing -> lift $ Left $ show pred ++ " can't be found in state."
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
        predicateVars = map (cVN.("$"++).show) [3 .. length predicateNodes + 2]
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

  mapf nvm pn@(PN _) = fromMaybe pn (lookup pn nvm)
  mapf _ a = a

-- (pred, (typedPredicateVars, eventAtom:state')
mkLink' :: SynIso (Atom, ([Atom], [Atom])) Atom
mkLink' = _equivl
         . first  (_evalTv . addfst highTv  . addsnd [cVN "$1"])
         . second
             (_meml . addfst (cVN "$1") . ssl . tolist2 . addfst (cVN "$2")
             . _exl . first (varll . mapIso (_typedvarl . addsnd (cTN "PredicateNode")))
             . second andl)

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


--Use if free belongs to the scope of the sentence
handleFREEs2 :: SynIso (Atom,[Free]) Atom
handleFREEs2 = iunit . second anyToState
                     . isoFoldl handleFREE
                     . first (addsnd [])
    where anyToState = Iso f g
          f = pushAtoms
          g () = error "Not Implement anyToState g."


--Use if free belongs outside the scope of the sentence
handleFREEs :: SynIso (Atom,[Free]) Atom
handleFREEs = (handleEmpty <+>
              listl . cons . addfst (cAN "frees")
                    . handleState
                    . isoFoldl handleFREE
                    . first (addsnd [])
              )
    where handleEmpty = Iso f g where
              f (a,[]) = pure a
              f _ = lift $ Left "Not empty Frees List f"
              g (LL _) = lift $ Left "Not empty Frees List g"
              g a = pure (a,[])
          handleState = Iso f g where
              f (a,as) = do
                    atoms <- gets sAtoms
                    pure (a:(as++atoms))
              g _ = error "Not Implement handleFREEs.handleState.g"


handleFREE :: SynIso ((Atom,[Atom]),Free) (Atom,[Atom])
handleFREE = (iunit            . second (inverse fNull))
         <+> (handle handleUIs . second (inverse fUI))
    where handle syn = Iso f g where
            f ((a,ls),f) = do
                res <- apply syn (a,f)
                pure (a,res:ls)
            g (a,res:ls) = do
                (_,f) <- unapply syn res
                pure ((a,ls),f)

frees :: Syntax [Free]
frees = many free

free :: Syntax Free
free = atomToFNull . sei
     <+> atomToFNull . maiP
  -- <+> soi
  -- <+> vocative &&& optional relative_clauses
  --              &&& selbri
  --              &&& optional relative_clauses
  --              <&& optMorph "DOhU"

  -- <+> vocative &&& optional relative_clauses
  --              &&& some cmene
  --              &&& optional relative_clauses
  --              <&& optMorph "DOhU"

   <+> atomToFNull . voc2
   <+> atomToFNull . voc1
   <+> fUI . indicators
   <+> atomToFNull . toTOI
    where atomToFNull = fNull . toState 1 . tolist1

maiP :: Syntax Atom
maiP = handle <<< number {-<+> lerfu_string )-} &&& morphPred "MAI"
    where handle = mkIso f g
          f (num,pred) = cEvalL noTv pred (cLL [this,num])
          g (EvalL _ pred (LL [_,num])) = (num,pred)
          this = cCN "This" noTv --FIXME get a node that refers to the text

voc1 :: Syntax Atom
voc1 = (handleVocatives . commute ||| listl) . ifJustB
       <<< vocatives &&& optional sumti <&& optMorph "DOhU"
    where handleVOC1 = _eval . second tolist1 . commute

voc2 :: Syntax Atom
voc2 = handleRelClause . first (handleFREEs2 . first handleVocatives)
     . reorder
     <<< vocatives
     &&& opt_relative_clauses
     &&& setl . mapIso concept . some Morph.cmevla
     &&& frees
     &&& opt_relative_clauses
     <&& optMorph "DOhU"
    where reorder = mkIso f g
          f (voc,(r1,(ws,(f,r2)))) = (((ws,voc),f),Just (r1++r2))
          g (((ws,voc),f),Just r)  = (voc,([],(ws,(f,r))))
          g (((ws,voc),f),Nothing) = (voc,([],(ws,(f,[]))))

handleVocatives :: SynIso (Atom,[Atom]) Atom
handleVocatives = listl . cons . addfst (cAN "vocative")
                . mapIso handleVocative . isoDistribute
    where handleVocative = _eval . second tolist1 . commute

vocatives :: Syntax [Atom]
vocatives = mapIso (implicationOf . predicate) . merge
    <<< oooob (some (handle <<< morph "COI" &&& optional (morph "NAI"))) []
              (tolist1 . morph "DOI")                                      []
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
            g _ = error "Not implemented vocative merge g."

toTOI :: Syntax Atom
toTOI = sepMorph "TO" &&> text <&& optMorph "TOI"

--Vice Versa
--soi :: Syntax Atom
--soi = sepMorph "soi" &&> sumti &&& optional sumti <&& optMorph "SEhU"

{-
free :: SyntaxState s => Syntax s [ADT]
free = adtSyntax "free" <<<
    <+> adtMorph "XI" &+& listoptional (concatSome free)
                        &+& (number <+> lerfu_string)
                        &+& listoptional (adtMorph "BOI")
    <+> adtMorph "XI" &+& listoptional (concatSome free)
                        &+& adtMorph "VEI"
                        &+& listoptional (concatSome free)
                        &+& mex
                        &+& listoptional (adtMorph "VEhO")
-}

------------------------------------
--Second Order Statments
-----------------------------------

type SEI = Atom

--SEIs are second order Statments that can appear almost anywhere
sei :: Syntax Atom
sei = handleBRIDI . commute
   <<< sepMorph "SEI" &&> ((terms <&& optMorph "CU") <+> insert [])
                        &&& selbri
                        <&& optMorph "SEhU"

indicatorsNA :: Syntax [UI]
--FIXME
--indicatorsNA = indicators <+> insert [(cCN "ge'e" lowTv,selbriDefaultTV)]
indicatorsNA = insert [(cCN "ge'e" lowTv,selbriDefaultTV)]

indicators :: Syntax [UI]
indicators = some indicator
    -- sepMorph "FUhE" &&> some indicator

indicator :: Syntax UI
indicator = uiP
 -- <+> adtMorph "Y" FIXME??? Not relevant for text
 -- <+> adtMorph "DAhO" FIXME resets various things to default
 -- <+> adtMorph "FUhO"

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
_UI = concept . handleXU <<< morph "UI"
    where handleXU = (setFlagIso "xu" ||| id) . mkIso f g where
              f "xu" = Left "xu"
              f a = Right a
              g (Left a) = a
              g (Right a) = a

naiP :: Syntax Double
naiP = handleNAI <<< (morph "NAI" <+> insert "")
    where handleNAI = mkSynonymIso [("nai"  , -1)
                                   ,(""     , 1)
                                   ,("ja'ai", 1)
                                   ]

--FIXME Recheck pei handeling
caiP :: Syntax Double
caiP = handleCAI . getFlagValueIso "CAI" . setFlagValueIso "CAI"
     <<< morph "CAI"
    where handleCAI :: SynIso String Double
          handleCAI = mkSynonymIso [("cai"    ,0.99)
                                   ,("sai"    ,0.75)
                                   ,(""       ,0.5 )
                                   ,("pei"    ,0.5 )
                                   ,("ru'e"   ,0.25)
                                   ,("cu'i"   ,0.01)
                                   ]

handleUIs :: SynIso (Atom,[UI]) Atom
handleUIs = setl . mapIso handleUI . isoDistribute

handleUI :: SynIso (Atom,(Atom,TruthVal)) Atom
handleUI = (handleXU ||| handleUI') . switchOnFlag "xu"
    where handleXU = rmFlagIso "xu" . addXUIso . rmsndAny (xu,tv)
          xu = Node "ConceptNode" "xu" noTv
          tv = stv 0.75  0.9

handleUI' :: SynIso (Atom,(Atom,TruthVal)) Atom
handleUI' = handlePEI . _frames . (first.second) instanceOf
          . manage
    where manage = mkIso f g where
              f (a,(ui,tv))     = ((tv,ui),[(getPred a,"1")])
              g ((tv,ui),[(ap,_)]) = error "handleUI' need inverse getPred."

          handlePEI = satl . tolist1 . ifFlagVlaue "CAI" "pei" <+> id

          --TODO Should get expanded to NU. Either when expanding UI or here???
          getPred (LL [_,CtxL _ (SL (EvalL _ _ (LL [pred,_]):_)) ]) = pred
          getPred a = a

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
          g _ = error "mergeMaybe g using wrong but fast implementeation"

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
    g pred = error "Not Implemented g mergePredicates"

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
    g pred = error "Not Implemented g mergePredicatesSeq"

mergePredicatesT :: Atom -> Atom -> Atom
                 -> SynIso ((Atom,Maybe NA),(Atom,Maybe NA)) (Atom,Maybe NA)
mergePredicatesT l1 l2 l3 = Iso f g where
    f ((p1,mna1),(p2,mna2)) = do
        let p1name = nodeName p1
            p2name = nodeName p2
            p1pred = drop 20 p1name
            p2pred = drop 20 p2name
            p1Eval' = cEvalL noTv p1 l1
            p2Eval' = cEvalL noTv p2 l2
            p1Eval = case mna1 of
                        Just (na,[(CN "ge'e",tv)]) -> cEvalL tv (cGPN na lowTv)
                                                              (cLL [p1Eval'])
                        Nothing -> p1Eval'
                        _ -> error "mergePredicatesT.f can't handle non ge'e ui"
            p2Eval = case mna2 of
                        Just (na,[(CN "ge'e",tv)]) -> cEvalL tv (cGPN na lowTv)
                                                              (cLL [p2Eval'])
                        Nothing -> p2Eval'
                        _ -> error "mergePredicatesT.f can't handle non ge'e ui"

        name <- randName (p1name ++ p2name)
        name2 <- randName (p1pred ++ p2pred)
        let pred = cPN (name ++ "___" ++ name2) noTv
        pushAtom $ cEquivL noTv (cEvalL noTv pred l3)
                                (cAL noTv [ p1Eval
                                          , p2Eval
                                          ])
        pure (pred,Nothing)
    g pred = error "Not Implemented g mergePredicatesSeq"

mergePredicatesSeqNA = mergePredicatesT (cLL [cVN "$1",cVN "$2"])
                                        (cLL [cVN "$2",cVN "$3"])
                                        (cLL [cVN "$1",cVN "$3"])

mergePredicatesNA = mergePredicatesT (cLL [cVN "$1",cVN "$2"])
                                     (cLL [cVN "$1",cVN "$2"])
                                     (cLL [cVN "$1",cVN "$2"])

imply :: String -> SynIso Atom Atom
imply string = Iso f g where
     f a = pushAtom (cImpL noTv a (cPN string noTv)) >> pure a
     g a = popAtom >> pure a

morphPred :: String -> Syntax Atom
morphPred s = implicationOf . imply s . predicate . morph s


-------------------------------------------------------------------------------
--SpaceTime
-------------------------------------------------------------------------------

space_time :: Syntax (Atom,Maybe NA)
space_time = mergeMaybe mergefun ((just . space &&& optional time)
                                  <+>
                                  (optional space &&& just . time))
    where mergefun = mergePredicatesNA

time :: Syntax (Atom,Maybe NA)
time = mergeMaybe mergefun (oooobm time_offset time_interval)
    where mergefun = mergePredicatesSeqNA

time_offset :: Syntax (Atom,Maybe NA)
time_offset = general_offset "PU" "ZI"

time_interval :: Syntax (Atom,Maybe NA)
time_interval = mergeMaybe mergefun (oooobm time_interval' interval_property)
    where mergefun = mergePredicatesNA

time_interval' :: Syntax (Atom,Maybe NA)
time_interval' = addsnd Nothing . handle_interval
                 <<< morphPred "ZEhA" &&& optional (morphPred "PU")
    where handle_interval = Iso f g
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

space :: Syntax (Atom,Maybe NA)
space = mergeMaybe mergefun (oooobm space_offset space_interval)
    where mergefun = mergePredicatesSeqNA

space_offset :: Syntax (Atom,Maybe NA)
space_offset = general_offset "FAhA" "VA"

general_offset :: String -> String -> Syntax (Atom,Maybe NA)
general_offset dir mag = isoFoldl mergePredicatesSeqNA . inverse cons . mapIso (handle_offset dir mag)
            <<< some (oooobm (morphPred dir &&& optional _NAI)
                             (morphPred mag)
                     )
            <+> (insert [(Nothing,Nothing)] . ifFlag "WithDefaultTenses")

handle_offset :: String -> String
              -> SynIso (Maybe (Atom,Maybe NA),Maybe Atom) (Atom,Maybe NA)
handle_offset dirC magC = Iso f g
    where f (mdir,mmag) = do
              let dirname = maybe "" (nodeName.fst) mdir
                  dirneg  = maybe Nothing snd mdir
                  magname = maybe "" nodeName mmag
              name <- randName (dirname ++('_':magname))
              let pred = cPN (name ++ "___offset") noTv
              case mmag of
                  Just mag -> pushAtom $ cImpL noTv pred mag
                  _ -> pushAtom $ cImpL noTv pred (cPN magC noTv)
              case mdir of
                  Just (dir,_) -> pushAtom $ cImpL noTv pred dir
                  _ -> pushAtom $ cImpL noTv pred (cPN dirC noTv)
              pure (pred,dirneg)
          g pred = error "Not Implemented g handle_offset"

space_interval :: Syntax (Atom,Maybe NA)
space_interval = mergeMaybe mergefun (oooobm space_interval' space_int_prop)
    where mergefun = mergePredicatesNA

space_interval' :: Syntax (Atom,Maybe NA)
space_interval' = addsnd Nothing . handle_interval
              <<< mergeMaybe mergePredicates (oooobm (morphPred "VEhA")
                                                     (morphPred "VIhA"))
              &&& optional (morphPred "FAhA")
    where handle_interval = Iso f g where
            f (pred,mfaha) = do
                case mfaha of
                    Just faha -> pushAtom $ cImpL noTv pred faha
                    Nothing -> pushAtom $ cImpL noTv pred (cPN "FAhA" noTv)
                pure pred
            g _ = error "Not Implemented"

space_int_prop :: Syntax (Atom,Maybe NA)
space_int_prop = (setFlagIso "FEhE" . sepMorph "FEhE") &&> interval_property

_NAI :: Syntax NA
_NAI = morph "NAI" &&& indicatorsNA

naiToNA :: SynIso String String
naiToNA = mkSynonymIso [("nai","na"),("ja'ai","ja'a")]

interval_property :: Syntax (Atom,Maybe NA)
interval_property = first handle
                    <<< (handleROI . (number &&& morphPred "ROI")
                         <+> morphPred "TAhE"
                         <+> morphPred "ZAhO"
                        )
                    &&& optional (first naiToNA . _NAI)
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

--type EK = (Bool,(Bool,(String,Bool)))

ek :: Syntax EK
ek = ekPat (morph "A")

jek :: Syntax EK
jek = ekPat (_JAtoA . morph "JA")

gihek :: Syntax EK
gihek = ekPat (_GIhAtoA . morph "GIhA")

--data JOIK = JOI (Bool,(String,Bool))
--          | INT (Bool,(String,Bool))
--          | INTGAhO (String,((Bool,(String,Bool)),String))
--          deriving (Show,Eq)

gek :: Syntax (Bool,(String,Bool))
gek = optBool "se" &&& (_GAtoA . morph "GA") &&& optBool "nai"
--  <+> joik &&& morph "GI"
--  <+> stag &&& gik

guhek :: Syntax (Bool,(String,Bool))
guhek = optBool "se" &&& (_GUhAtoA . morph "GUhA") &&& optBool "nai"

gik :: Syntax Bool
gik = sepMorph"GI" &&> optBool "nai"

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
    g (JOI a) = pure a
    g _ = lift $ Left "Not a JOI."

joik_INT :: SynIso (Bool,(String,Bool)) JOIK
joik_INT = Iso f g where
    f a = pure $ INT a
    g (INT a) = pure a
    g _ = lift $ Left "Not a INT."

joik_INTGAhO :: SynIso (String,((Bool,(String,Bool)),String)) JOIK
joik_INTGAhO = Iso f g where
    f a = pure $ INTGAhO a
    g (INTGAhO a) = pure a
    g _ = lift $ Left "Not a INTGAhO."

joik :: Syntax JOIK
joik = joik_JOI . (optBool "se" &&& morph "JOI" &&& optBool "nai")
   <+> joik_INT . interval
   <+> joik_INTGAhO . (morph "GAhO" &&& interval &&& morph "GAhO")
    where interval = optBool "se"
                   &&& morph "BIhI"
                   &&& optBool "nai"

joik_jek :: Syntax JOIK_JEK
joik_jek = left . joik <+> right . jek

joik_ek :: Syntax JOIK_EK
joik_ek = left . joik <+> right . ek

handleJJCTTS :: SynIso (JJCTTS,[Atom]) Atom
handleJJCTTS = Iso f g where
    f (CTLeaf ((mna,pred),Nothing),as) = do
        frames <- apply (_frames . second (handleTAG . toSumti)) (pred,as)
        case mna of
            Nothing -> pure frames
            Just (na,[(CN "ge'e",tv)]) -> pure $ cEvalL tv (cGPN na noTv)
                                                           (cLL [frames])
            _ -> error "handleJJCTTS.f only ge'e uis allowed"

    f (CTLeaf (pred,Just "space_time"),as) = apply handleSpaceTime (pred,as)

    f (CTNode joik_jek (x1,x2),as) = do
        a1 <- f (x1,as)
        a2 <- f (x2,as)
        case joik_jek of
            Right jek -> apply conLink (jek,(a1,a2))
            Left joik -> apply handleJOIK (joik,(a1,a2))
    g _ = error "handleJJCTTS g: not implemented."

    toSumti :: SynIso [Atom] [Sumti]
    toSumti = mkIso f g where
        f = map (\x -> (x,Nothing))
        g = map fst

    handleSpaceTime :: SynIso (SelbriNA,[Atom]) Atom
    handleSpaceTime = Iso f g where
        f ((mna,(tv,st)),as) = do
            eval <- case as of
                [s] -> do
                    nctx <- (`cCN` noTv) <$> randName (show s)
                    addCtx nctx
                    pure $ cEvalL tv st (cLL [nctx,s])
                _ -> pure $ cEvalL tv st (cLL as)
            case mna of
                Nothing      -> pure eval
                Just (na,[(CN "ge'e",tv)]) -> pure $ cEvalL tv (cGPN na noTv)
                                                               (cLL [eval])
                _ -> error "handleJJCTTS.handleSpaceTime.f only ge'e uis allowed"
        g _ = error "Not Implemented g handleSpaceTime"

handleJJCTTS_Selbri :: SynIso (JJCTTS,Atom) Atom
handleJJCTTS_Selbri = Iso f g where
    f (CTLeaf (pred,Nothing),selb) = do
        seval <- apply selbriToEval (noTv,selb)
        peval <- apply selbriNAToEval pred
        pushAtom $ cIImpL noTv seval peval
        pure selb

    f (CTLeaf (pred,Just "space_time"),selb) = do
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
    g _ = error "handleJJCTTS_Selbri g: not implemented."

    toSumti :: SynIso [Atom] [Sumti]
    toSumti = mkIso f g where
        f = map (\x -> (x,Nothing))
        g = map fst

    handleSpaceTime :: SynIso SelbriNA Atom
    handleSpaceTime = Iso f g where
        f (mna,(tv,pred)) = do
            ctx <- gets (head.sCtx)
            nctx <- (`cCN` noTv) <$> randName (show ctx)
            setPrimaryCtx nctx
            let eval = cEvalL tv pred (cLL [nctx,ctx])
            case mna of
                Nothing      -> pure eval
                Just (na,[(CN "ge'e",tv)]) -> pure $ cEvalL tv (cGPN na noTv)
                                                               (cLL [eval])
                _ -> error "handleJJCTTS_Selbri.handleSpaceTime.f only ge'e uis allowed"
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














