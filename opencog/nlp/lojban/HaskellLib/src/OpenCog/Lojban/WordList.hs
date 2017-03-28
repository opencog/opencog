{-# LANGUAGE ScopedTypeVariables #-}
module OpenCog.Lojban.WordList where

import Prelude hiding (map,(.))
import Control.Category ((.))
import Control.Monad

import System.Random
import System.Directory

import Data.List
import Data.Serialize

import OpenCog.Lojban.Syntax.Util
import OpenCog.Lojban.Syntax.Types

import Data.ListTrie.Patricia.Set.Ord as TS

import qualified Data.Map as M
import qualified Data.Csv as CSV
import qualified Data.Vector as V
import qualified Data.ByteString as BS
import qualified Data.ByteString.Char8 as C
import qualified Data.ByteString.Lazy as LBS
import qualified Data.HashMap.Strict as H
----------------------------------------------
----------------------------------------------
----------------------------------------------

newtype Gismu = Gismu {gName :: String}

instance CSV.FromNamedRecord Gismu where
    parseNamedRecord r = Gismu <$> r CSV..: C.pack "Verb"

newgetGismu :: String -> IO StringSet
newgetGismu csv = do
    csvdata <- LBS.readFile csv
    case CSV.decodeByName csvdata of
        Left s -> error s
        Right (_,v) -> pure (TS.fromList $ gName <$> V.toList v)

data Cmavo = Cmavo {cName :: !String,cClass :: !String}

instance CSV.FromNamedRecord Cmavo where
    parseNamedRecord r = Cmavo <$> r CSV..: C.pack "particle"
                               <*> r CSV..: C.pack "S"

newgetCmavo :: String -> IO [(String,String)]
newgetCmavo csv = do
    csvdata <- LBS.readFile csv
    case CSV.decodeByName csvdata of
        Left s -> error s
        Right (_,v) -> pure $ ((,) <$> cClass <*> cName) <$> V.toList v


handleCmavo :: [(String,String)] -> M.Map String [String]
handleCmavo ls = M.fromListWith (++) cmavo
    where cmavo = fmap f ls
          f (s,c) = (takeWhile p s,[c])
          p = (`notElem` "1234567890*")

data BAI = BAI {bName :: !String, bDef :: !String}

instance CSV.FromNamedRecord BAI where
    parseNamedRecord r =
        case H.lookup (C.pack "S") r of
            Just _ -> BAI <$> r CSV..: C.pack "particle"
                          <*> r CSV..: C.pack "definition"
            Nothing -> mzero

newgetBAI :: String -> IO [(String,String)]
newgetBAI csv = do
    csvdata <- LBS.readFile csv
    case CSV.decodeByName csvdata of
        Left s -> error s
        Right (_,v) -> pure $ handleBAIprefix . ((,) <$> bName <*> word.bDef) <$> V.toList v
  where
    word = takeWhile (' ' /=)

handleBAI :: [(String,String)] -> SynIso String String
handleBAI bai = mkSynonymIso bai . stripSpace

handleBAIprefix :: (String,String) -> (String,String)
handleBAIprefix (b,d) = if t2b `elem` se then (b,t2b ++ ' ' : nd) else (b,nd)
    where se  = ["se","te","ve","xe"]
          t2b = take 2 b
          nd  = d ++ " " --Parser Expects a space behind every word

----------------------------------------------
----------------------------------------------
----------------------------------------------

loadWordLists :: String -> String -> IO WordList
loadWordLists cmavoSrc gismuSrc  = do
    let ser = "wordlist.cache"
    dfe <- doesFileExist ser
    if dfe
        then (do
            bs <- BS.readFile ser
            (seed :: Int) <- randomIO
            let (Right (selmahos,gismu,bai)) = decode bs
                baiIso = handleBAI bai
            return WordList {cmavos = fmap TS.fromList selmahos
                            ,gismus = TS.fromList gismu
                            ,bai = baiIso
                            ,seed = seed
                            }
            )
        else (do
            (seed :: Int) <- randomIO
            gismu <- newgetGismu gismuSrc
            cmavo <- newgetCmavo cmavoSrc
            bai   <- newgetBAI   cmavoSrc
            let selmahos = handleCmavo cmavo
                baiIso = handleBAI bai
                res = WordList {cmavos = fmap fromList selmahos
                               ,gismus = gismu
                               ,bai = baiIso
                               ,seed = seed
                               }
                bs = encode (selmahos,TS.toList gismu,bai)
            BS.writeFile ser bs
            return res)
