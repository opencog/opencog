{-# LANGUAGE DataKinds          #-}
{-# LANGUAGE GADTs              #-}
{-# LANGUAGE TypeOperators      #-}
{-# LANGUAGE Rank2Types         #-}
{-# LANGUAGE ImpredicativeTypes #-}
{-# LANGUAGE ScopedTypeVariables#-}
module Main where

import OpenCog.AtomSpace
import OpenCog.Lojban

import Control.Concurrent
import Control.Monad
import Control.Monad.IO.Class
import Control.Exception

import System.Process

main :: IO ()
main = do
    (parser,printer) <- initParserPrinter "lojban.xml"
    mainloop parser printer

--camxesPath = "/home/roman/OpenCog/Lojban/ilmentufa"

mainloop parser printer = do
    putStrLn "Please Input some Lojban to Translate"
    input <- getLine

    {-let args = "run_camxes.js -std -m N \"" ++ input ++ "\""
    camxesres <-
        readCreateProcess (shell $ "node " ++ args){cwd = Just camxesPath
        ,std_out = CreatePipe} ""
    putStrLn camxesres-}

    let res = parser input

    case res of
        Just x -> printAtom x
        Nothing -> putStrLn "Parseing Failed."

    mainloop parser printer
    {-case res of
        Left _ -> mainloop parser printer
        Right atom -> do
            (res2 :: Either SomeException String) <- try $ printer atom
            print res2
            mainloop parser printer
-}
