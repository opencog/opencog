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
    (parser,printer) <- initParserPrinter "cmavo.csv" "gismu.csv"
    mainloop parser printer

mainloop parser printer = do
    putStrLn "Please Input some Lojban to Translate"
    input <- getLine

    let res = parser input

    case res of
        Right x -> printAtom x
        Left e  -> putStrLn e

    putStrLn ""

    mainloop parser printer
