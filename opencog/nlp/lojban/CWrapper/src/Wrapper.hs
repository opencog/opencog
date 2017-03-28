module Wrapper where

import OpenCog.Lojban
import OpenCog.AtomSpace

import Foreign.C
import Foreign.Ptr
import Foreign.StablePtr

foreign export ccall "lojban_parse" c_parse :: Ptr AtomSpaceRef
                                     -> StablePtr WordList
                                     -> CString
                                     -> IO Handle

foreign export ccall "lojban_print" c_print :: Ptr AtomSpaceRef
                                     -> StablePtr WordList
                                     -> Handle
                                     -> IO CString

foreign export ccall "lojban_init" c_init :: CString -> CString -> IO (StablePtr WordList)

foreign export ccall "lojban_exit" c_exit :: StablePtr WordList -> IO ()

c_init :: CString -> CString -> IO (StablePtr WordList)
c_init cCmavoSrc cGismuSrc = do
    cmavoSrc <- peekCString cCmavoSrc
    gismuSrc <- peekCString cGismuSrc
    wl <- loadWordLists cmavoSrc gismuSrc
    newStablePtr wl

c_exit :: StablePtr WordList -> IO ()
c_exit = freeStablePtr

c_parse :: Ptr AtomSpaceRef -> StablePtr WordList -> CString -> IO Handle
c_parse asRef swl ctext = do
    as <- refToObj asRef
    wl <- deRefStablePtr swl
    text <- peekCString ctext
    print "Parsing: "
    print text
    case lojbanToAtomese wl text of
        Left m  -> pure nullPtr
        Right r -> do
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
        Just a ->
            case atomeseToLojban wl a of
                Left m  -> pure nullPtr
                Right r -> newCString r

{-# ANN module "HLint: ignore Use camelCase" #-}
