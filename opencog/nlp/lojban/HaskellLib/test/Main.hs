{-# LANGUAGE ScopedTypeVariables#-}

module Main where

import OpenCog.AtomSpace
import OpenCog.Lojban

import Control.Exception
import Control.Monad

import Control.Parallel.Strategies

import System.Exit (exitFailure,exitSuccess)

main :: IO ()
main = do
    putStrLn "Starting Test"
    (parser,printer) <- initParserPrinter "cmavo.csv" "gismu.csv"
    sentences <- loadData
    let parsed = parMap rpar (ptest parser) sentences
    testRes <- sequence parsed
    let testResF  = filter id testRes
    putStrLn $
        "Of " ++ show (length sentences) ++ " sentences " ++
        show (length testResF) ++ " have been parsed/printed successfully."
    if length testResF == length sentences
        then exitSuccess
        else exitFailure

ptest :: (String -> Either String Atom) -> String -> IO Bool
ptest parser text = do
    case parser text of
        Left e  -> print text >> putStrLn e >> return False
        Right _ -> return True

pptest :: (String -> Either String Atom) -> (Atom -> Either String String) -> String -> IO Bool
pptest parser printer text =
    case parser text of
        Left e    -> print False >> return False
        Right res ->
            case printer res of
                Left e      -> print False >> return False
                Right ptext ->
                    if ptext == text
                        then return True
                        else print text >> print ptext >> return False

loadData :: IO [String]
loadData = do
    file <- readFile "data.txt"
    return (lines file)

isRight :: Either a b -> Bool
isRight (Left  _) = False
isRight (Right _) = True
