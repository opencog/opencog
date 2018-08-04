-- GSoC 2015 - Haskell bindings for OpenCog.

-- | Simple example on using Haskell bindings and the Pattern Matcher.
-- (Following this example: https://github.com/ngeiswei/moses-pln-example)
import OpenCog.AtomSpace            (Atom(..),AtomGen(..),AtomSpace,appGen,cogBind,
                                     runOnNewAtomSpace,get,insert,remove,debug,
                                     printAtom,AtomType(..),noTv,stv,(|>),(\>))
import Control.Monad.IO.Class       (liftIO)
import BackgroundKnowledge          (bkn)
import PlnRules                     (plnRuleForAllHack
                                    ,plnRuleEliminateNeutralElementHack
                                    ,plnRuleEliminateDanglingJunctorHack
                                    ,plnRuleEquivalenceHack
                                    ,plnRuleAverageHack)
import MosesModel                   (mosesModel)

main :: IO ()
main = let insertGen :: AtomGen -> AtomSpace ()
           insertGen = appGen insert
           printGen :: AtomGen -> IO ()
           printGen = appGen printAtom
           showRes :: Maybe AtomGen -> AtomSpace ()
           showRes at = liftIO $ do
               putStrLn "Res:"
               case at of
                   Nothing -> print at
                   Just at -> printGen at
        in runOnNewAtomSpace $ do
                mapM_ insertGen bkn
                insert mosesModel
                insert plnRuleForAllHack
                insert plnRuleEliminateNeutralElementHack
                insert plnRuleEliminateDanglingJunctorHack
                insert plnRuleEquivalenceHack
                insert plnRuleAverageHack

                res1 <- cogBind plnRuleForAllHack
                res2 <- cogBind plnRuleEliminateNeutralElementHack
                res3 <- cogBind plnRuleEliminateDanglingJunctorHack
                res4 <- cogBind plnRuleEquivalenceHack
                res5 <- cogBind plnRuleAverageHack

                mapM_ showRes [res1,res2,res3,res4,res5]

