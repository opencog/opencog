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
    putStrLn "Please specify which data file to use: "
    input <- getLine
    putStrLn "Starting Test"
    (parser,printer) <- initParserPrinter
    putStrLn "Loading Data"
    sentences <- loadData input
    putStrLn "Testing: "
    let parsed = parMap rpar (ptest parser) sentences
    testRes <- sequence parsed
 -- testRes <- mapM (ptest parser) sentences
    let testResF  = filter id testRes
    putStrLn $
        "Of " ++ show (length sentences) ++ " sentences " ++
        show (length testResF) ++ " have been parsed/printed successfully."
    if length testResF == length sentences
        then exitSuccess
        else exitFailure

ptest :: (String -> Either String (Maybe Atom)) -> String -> IO Bool
ptest parser text = do
    print text
    case parser text of
        Left e  -> putStrLn e >> return False
        Right _ -> return True

pptest :: (String -> Either String (Maybe Atom)) -> (Atom -> Either String String) -> String -> IO Bool
pptest parser printer text =
    case parser text of
        Left e    -> print False >> return False
        Right Nothing -> putStrLn "Empty parse no Error" >> return True
        Right (Just res) ->
            case printer res of
                Left e      -> print False >> return False
                Right ptext ->
                    if ptext == text
                        then return True
                        else print text >> print ptext >> return False

loadData :: String -> IO [String]
loadData num = do
    file <- readFile $ "data" ++ num ++ ".txt"
    return (lines file)

isRight :: Either a b -> Bool
isRight (Left  _) = False
isRight (Right _) = True
