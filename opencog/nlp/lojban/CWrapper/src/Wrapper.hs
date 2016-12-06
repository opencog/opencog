{-# LANGUAGE LambdaCase                 #-}
module Wrapper where


import OpenCog.Lojban
import OpenCog.AtomSpace

import Foreign.C
import Foreign.Ptr
import Foreign.StablePtr

foreign export ccall "lojban_parse" c_parse :: Ptr AtomSpaceRef
                                     -> StablePtr WordList
                                     -> CString
                                     -> IO (Handle)

foreign export ccall "lojban_print" c_print :: Ptr AtomSpaceRef
                                     -> StablePtr WordList
                                     -> Handle
                                     -> IO (CString)

foreign export ccall "lojban_init" c_init :: CString -> IO (StablePtr WordList)

foreign export ccall "lojban_exit" c_exit :: StablePtr WordList -> IO ()

c_init :: CString -> IO (StablePtr WordList)
c_init cpath = do
    path <- peekCString cpath
    wl <- loadWordLists path
    newStablePtr wl

c_exit :: StablePtr WordList -> IO ()
c_exit ptr = do
    freeStablePtr ptr

c_parse :: Ptr AtomSpaceRef -> StablePtr WordList -> CString -> IO (Handle)
c_parse asRef swl ctext = do
    as <- refToObj asRef
    wl <- deRefStablePtr swl
    text <- peekCString ctext
    print "Parsing: "
    print text
    case lojbanToAtomeseRaw wl text of
        Nothing -> pure nullPtr
        Just r@(res,_) -> do
            print r
            mres <- as <: insertAndGetHandle res
            case mres of
                Nothing  -> pure nullPtr
                Just res -> pure res

c_print :: Ptr AtomSpaceRef -> StablePtr WordList -> Handle -> IO (CString)
c_print asRef swl h = do
    as <- refToObj asRef
    wl <- deRefStablePtr swl
    ma <- as <: getByHandle h
    case ma of
        Nothing -> pure nullPtr
        Just a  ->
            case atomeseToLojban wl a of
                Nothing -> pure nullPtr
                Just r  -> newCString r
