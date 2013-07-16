#!/usr/bin/runhaskell
import Data.List
import Data.List.Split

-- Parse MOSES log file and generate a CSV file describing for each
-- deme expansion the selected features.

-- Main function
main :: IO ()
main = do
  -- Write header
  putStrLn "parent_demeID,xmplr_rank,visit,xmplr_features,new_features,prune_features,spawn_demeID"
  -- Then parse the file
  parseLines

-- Parse each line, turning the content into some portion of the CSV
-- row, it's all order properly with the order of the information in
-- the log file so that it does generate the CSV rows properly
parseLines :: IO ()
parseLines = do
  line <- getLine
  case line of
    [] -> return()
    _ | hasDemeParentID line  -> putStr (parseDemeParentID line ++ ","
                                         ++ (parseXmplrRank line) ++ ","
                                         ++ (parseVisit line) ++ ",")
      | hasXmplrFeatures line -> putStr (unwords (parseXmplrFeatures line) ++ ",")
      | hasNewFeatures line   -> putStr (unwords (parseNewFeatures line) ++ ",")
      | hasPruneFeatures line -> putStr (unwords (parsePruneFeatures line) ++ ",")
      | hasDemeID line        -> putStrLn (parseDemeID line)
      | otherwise             -> parseLines -- parse next lines
  parseLines

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
parseXmplrRank l = words l !! 5

-- Assume the (n+1)th word is a comma separated list and return that
-- list
csl :: Int -> String -> [String]
csl i l = let wl = words l in
  if length wl > i then
    splitOn "," (wl !! i)
  else []

-- Turn a log line containing the exemplar features into list of features
hasXmplrFeatures :: String -> Bool
hasXmplrFeatures = isInfixOf "From which were in the exemplar"
parseXmplrFeatures :: String -> [String]
parseXmplrFeatures = csl 9

-- Turn a log line containing the new features into list of features
hasNewFeatures :: String -> Bool
hasNewFeatures = isInfixOf "From which are new"
parseNewFeatures :: String -> [String]
parseNewFeatures = csl 7

hasPruneFeatures :: String -> Bool
hasPruneFeatures = isInfixOf "Prune the exemplar from non-selected features"
parsePruneFeatures :: String -> [String]
parsePruneFeatures = csl 9

-- Extract the deme ID of the given log line
hasDemeID :: String -> Bool
hasDemeID = isInfixOf "Expansion"
parseDemeID :: String -> String
parseDemeID l = words l !! 4

-- multiple composition, for example mc 3 f = f . (f . f)
mc :: Int -> (a -> a) -> (a -> a)
mc 0 _ = id
mc n f = f . (mc (n-1) f)
