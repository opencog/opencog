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
    (parser,printer) <- initParserPrinter
    mainloop parser printer

mainloop parser printer = do
    putStrLn "Please Input some Lojban to Translate"
    input <- getLine

    let res = parser input

    case res of
        Right (Just x)  -> printAtom x
        Right (Nothing) -> putStrLn "Empty parse no Error"
        Left e  -> putStrLn e

    putStrLn ""

    mainloop parser printer
