module Wrapper where

import qualified Lojban as L
import OpenCog.Lojban
import OpenCog.AtomSpace
import System.Random

import Foreign.C
import Foreign.Ptr
import Foreign.StablePtr

type WordList = L.WordList State

foreign export ccall "lojban_parse" c_parse :: Ptr AtomSpaceRef
                                     -> StablePtr WordList
                                     -> CString
                                     -> IO Handle

foreign export ccall "lojban_print" c_print :: Ptr AtomSpaceRef
                                     -> StablePtr WordList
                                     -> Handle
                                     -> IO CString

foreign export ccall "lojban_init" c_init :: IO (StablePtr WordList)

foreign export ccall "lojban_exit" c_exit :: StablePtr WordList -> IO ()

c_init :: IO (StablePtr WordList)
c_init = do
    wl <- loadWordLists
    newStablePtr wl

c_exit :: StablePtr WordList -> IO ()
c_exit = freeStablePtr

c_parse :: Ptr AtomSpaceRef -> StablePtr WordList -> CString -> IO Handle
c_parse asRef swl ctext = do
    as <- refToObj asRef
    wl <- deRefStablePtr swl
    seed <- randomIO
    text <- peekCString ctext
    print "Parsing: "
    print text
    case lojbanToAtomese wl seed text of
        Left m  -> pure nullPtr
        Right Nothing -> pure nullPtr
        Right (Just r) -> do
            print r
            mres <- as <: insertAndGetHandle r
            case mres of
                Nothing  -> pure nullPtr
                Just res -> pure res

c_print :: Ptr AtomSpaceRef -> StablePtr WordList -> Handle -> IO CString
c_print asRef swl h = do
    as <- refToObj asRef
    wl <- deRefStablePtr swl
    ma <- as <: getByHandle h
    case ma of
        Nothing -> pure nullPtr
        Just a -> do
            seed <- randomIO
            case atomeseToLojban wl seed a of
                Left m  -> pure nullPtr
                Right r -> newCString r

{-# ANN module "HLint: ignore Use camelCase" #-}
