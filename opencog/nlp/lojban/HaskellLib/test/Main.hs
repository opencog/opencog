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
    (parser,printer) <- initParserPrinter
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

ptest :: (String -> Maybe Atom) -> String -> IO Bool
ptest parser text = do
    case parser text of
        Nothing -> print text >> print False >> return False
        Just _  -> return True

pptest :: (String -> Maybe Atom) -> (Atom -> Maybe String) -> String -> IO Bool
pptest parser printer text =
    case parser text of
        Nothing  -> print False >> return False
        Just res ->
            case printer res of
                Nothing    -> print False >> return False
                Just ptext ->
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
