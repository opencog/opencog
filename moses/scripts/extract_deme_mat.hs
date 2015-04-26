#!/usr/bin/runhaskell
import Data.List
import Data.List.Split
import Data.String.Utils
import System.IO (isEOF)

-- Parse MOSES log file and generate a CSV file describing for each
-- deme expansion the selected features.

-- Main function
main :: IO ()
main = do
  -- Write header
  putStrLn "parent_demeID,xmplr_rank,visit,new_demeID,xmplr_features,new_features"
  -- Then parse the file
  parseLines []

-- Parse each line, turning the content into some portion of the CSV
-- row, it's all order properly with the order of the information in
-- the log file so that it does generate the CSV rows properly
parseLines :: [String] -> IO ()
parseLines l = do
  isEOFRet <- isEOF
  if isEOFRet then return ()
  else do
    line <- getLine
    case line of
      [] -> return()
      _ | hasDemeParentID line  -> parseLines [parseDemeParentID line,
                                               parseXmplrRank line,
                                               parseVisit line]
        | hasDemeID line        -> parseLines (l ++ [parseDemeID line])
        | hasXmplrFeatures line -> parseLines (l ++ [join " " (parseXmplrFeatures line)])
        | hasNewFeatures line   -> let content = l ++ [join " " (parseNewFeatures line)]
                                   in do putStrLn (join "," content)
                                         parseLines (take 3 l)
        | otherwise             -> return ()
    parseLines l

-- Extract the deme's parent ID of the given log line
hasDemeParentID :: String -> Bool
hasDemeParentID = isInfixOf "Selected the"
parseDemeParentID :: String -> String
parseDemeParentID l = init (words l !! 9)
-- From same line extract visit
parseVisit :: String -> String
parseVisit l = mc 2 init (words l !! 12)
-- From same line extract the exemplar rank
parseXmplrRank :: String -> String
parseXmplrRank l = mc 2 init (words l !! 5)

-- Assume the (n+1)th word is a comma separated list and return that
-- list
csl :: Int -> String -> [String]
csl i l = let wl = words l in
  if length wl > i then
    splitOn "," (wl !! i)
  else []

-- Turn a log line containing the exemplar features into list of features
hasXmplrFeatures :: String -> Bool
hasXmplrFeatures = isInfixOf "Selected features which are in the exemplar:"
parseXmplrFeatures :: String -> [String]
parseXmplrFeatures = csl 10

-- Turn a log line containing the new features into list of features
hasNewFeatures :: String -> Bool
hasNewFeatures = isInfixOf "Selected features which are new:"
parseNewFeatures :: String -> [String]
parseNewFeatures = csl 8

-- Extract the deme ID of the given log line
hasDemeID :: String -> Bool
hasDemeID = isInfixOf "Breadth-first expansion for deme"
parseDemeID :: String -> String
parseDemeID l = words l !! 8

-- multiple composition, for example mc 3 f = f . (f . f)
mc :: Int -> (a -> a) -> (a -> a)
mc 0 _ = id
mc n f = f . (mc (n-1) f)
