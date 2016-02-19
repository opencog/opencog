-- Module containing types and functions to experiment with TruthValue
-- types.

module TVToolBox (-- Types
                  MyFloat,
                  MultiDist,
                  Dist,
                  -- Contants
                  defaultK,
                  defaultResolution,
                  -- Plotting
                  showDist,
                  plotDists,
                  -- Simple TV
                  confidenceToCount,
                  strengthToCount,
                  -- Distributional TV
                  genDist,
                  genDist_beta,
                  genDist_beta_contin,
                  prob,
                  prob_beta,
                  prob_only_beta,
                  pdf_beta,
                  pdf_only_beta,
                  toDist,
                  discretize,
                  trim,
                  normalize,
                  mean,
                  mode,
                  stdDev,
                  sqrtJsd,
                  -- Indefinite TV
                  toUp,
                  indefiniteInterval,
                  indefiniteIntervalWidth,
                  -- General/Math
                  integerMiddle,
                  optimize,
                  optimizeDbg
                 ) where
                  
import Math.Gamma (gamma, lnGamma)
import Data.Maybe (fromJust)
import Data.Ratio ((%))
import Data.Number.BigFloat (BigFloat, Prec10, Prec50)
import Data.Function.Memoize (memoize)
import Math.Combinatorics.Exact.Binomial (choose)
import System.Environment (getArgs)
import Text.Format (format)
import Data.MultiMap (MultiMap, fromList, toMap, fromMap, mapKeys)
import Data.Map (Map, map, fromList, toList, foldr, size, lookup, unionWith,
                 foldrWithKey, foldlWithKey, empty, insertWith, filter)
-- import Data.List (length)
import Debug.Trace (trace)
import Graphics.Gnuplot.Simple (plotPathStyle, plotPathsStyle,
                                Attribute(Title, XLabel, YLabel, XRange, PNG),
                                PlotStyle, defaultStyle,
                                lineSpec, LineSpec(CustomStyle),
                                LineAttr(LineTitle))

-----------
-- Types --
-----------

-- type MyFloat = Float
type MyFloat = Double
-- type MyFloat = BigFloat Prec10
-- type MyFloat = BigFloat Prec50

-- Distribution type, maps a first order probability to its second
-- order probability (or strength to probability).
type MultiDist = MultiMap MyFloat MyFloat

-- Like MultiDist but each strength is unique
type Dist = Map MyFloat MyFloat

-- -- Define the TruthValue class
-- class TruthValue tv where
--   getTVMean :: tv -> MyFloat
--   getTVConfidence :: tv -> MyFloat

-- data STV = STV { mean :: MyFloat, confidence :: MyFloat }
--          deriving (TruthValue, Show, Eq)

-- data ITV = ITV { l :: MyFloat, u :: MyFloat, b :: MyFloat, k :: Integer }
--          deriving (TruthValue, Show, Eq)

-- data DTV = DTV { dist :: Map MyFloat MyFloat, k :: Integer }
--          deriving (TruthValue, Show, Eq)

---------------
-- Constants --
---------------

defaultK :: Integer
defaultK = 5000

-- Used to convert count from/to confidence
defaultSTVK :: Integer
defaultSTVK = defaultK          -- Equal to the lookahead for now

defaultResolution :: Integer
defaultResolution = 100

-- If n is below that threshold, then choose_beta, and prob_beta is
-- used instead of choose or prob.
defaultBetaThreshold :: Integer
defaultBetaThreshold = 140

--------------
-- Plotting --
--------------

-- Plot distributions, specifying whether to enable zoom or not.
plotDists :: [(String, Dist)] -> String -> Bool -> Bool -> IO ()
plotDists nhs title zoom save = plotPathsStyle attributes (Prelude.map fmt nhs)
  where attributes = [Title title, XLabel "Strength", YLabel "Probability"]
                     ++ (if zoom then [] else [XRange (0.0, 1.0)])
                     ++ (if save then [PNG (title ++ ".png")] else [])
        fmt (n, h) = (defaultStyle {lineSpec = CustomStyle [LineTitle n]},
                      toPath h)

-- Turn a distribution into a plotable path
toPath :: Dist -> [(Double, Double)]
toPath h = [(realToFrac s, realToFrac p) | (s, p) <- (Data.Map.toList h)]

showDist :: Dist -> String
showDist h = format "size = {0}, total = {1}, data = {2}"
             [show (size h), show (distSum h), show (Data.Map.toList h)]

defaultTitle :: Attribute
defaultTitle = Title (format "Simple TV distribution (k={0})" [show defaultK])

---------------
-- Simple TV --
---------------

-- Using the fact that c = n / (n+k) we infer that n = c*k / (1-c)
confidenceToCount :: MyFloat -> Integer -> Integer
confidenceToCount c k = round (c*(fromInteger k) / (1 - c))

-- x = s*n
strengthToCount :: MyFloat -> Integer -> Integer
strengthToCount s n = round (s * (fromInteger n))

-----------------------
-- Distributional TV --
-----------------------

-- Return the sum of the probabilities of the distribution. It should
-- normally be equal to 1.0
distSum :: Dist -> MyFloat
distSum d = Data.Map.foldr (+) 0 d

-- Given a simple TV <s, n> and a lookahead k, generate the
-- corresponding (multi)-distribution.
genDist :: MyFloat -> Integer -> Integer -> Dist
genDist s n k =
  Data.Map.fromList [(p, prob n x k cx) | cx <- [0..k], let p = cx2p n s k cx]
  where x = strengthToCount s n

-- Like genMultiDist but uses prob_beta, which might be more accurate
-- when n is low.
genDist_beta :: MyFloat -> Integer -> Integer -> Dist
genDist_beta s n k =
  Data.Map.fromList
  -- (trace (format "genDist_beta {0} {1} {2}"
  --                           [show s, show n, show k])
                      [(p, prob_beta n s k p)
                     | cx <- [0..k], let p = cx2p n s k cx]
  -- )

-- Like genMultiDist_beta but uses directly p instead of running cx
-- from 0 to k. This is convenient to get an approximation of a
-- continuous distribution, even when k is low. step is the difference
-- in probability between each p.
genDist_beta_contin :: MyFloat -> Integer -> Integer -> MyFloat -> Dist
genDist_beta_contin s n k step =
  Data.Map.fromList [(p, prob_beta n s k p) | p <- [0.0,step..1.0]]

-- Multiply the probabilities of a distribution by a given value
scale :: MyFloat -> Dist -> Dist
scale c h = Data.Map.map ((*) c) h

-- Normalize a distribution so that it sums up to 1
normalize :: Dist -> Dist
normalize h = scale (1.0 / (distSum h)) h

-- Add 2 distributions
add :: Dist -> Dist -> Dist
add = unionWith (+)

-- Compute the average distribution of 2 distributions, (hP + hQ) / 2
average :: Dist -> Dist -> Dist
average hP hQ = scale 0.5 (add hP hQ)

-- Return the nearest (lower) bin corresponding to a strength
bin :: Integer -> MyFloat -> MyFloat
bin n s = fromRational ((round (s * (fromInteger n))) % n)

-- Turn a multi-distribution into a distribution (sum up the
-- duplicated probabilities).
toDist :: MultiDist -> Dist
toDist d = Data.Map.map sum (toMap d)

-- Turn a distribution into a multi-distribution
toMultiDist :: Dist -> MultiDist
toMultiDist = Data.MultiMap.fromList . Data.Map.toList

-- Discretize a distribution in n bins
discretize :: Integer -> Dist -> Dist
discretize n h = toDist (mapKeys_fix (bin n) (toMultiDist h))

-- Discard probabilities under a given value
trim :: Double -> Dist -> Dist
trim e h = Data.Map.filter ((<=) e) h

-- P(x+X successes in n+k trials | x successes in n trials)
prob :: Integer -> Integer -> Integer -> Integer -> MyFloat
prob n x k cx = fromRational (num % den)
  where num = (n+1)*(choose k cx)*(choose n x)
        den = (k+n+1)*(choose (k+n) (cx+x))

-- Version of choose that takes fractional values as second
-- argument. Can be more accurate for lower n.
choose_beta :: Integer -> MyFloat -> MyFloat
choose_beta n k = 1.0 / ((nreal+1.0) * beta (nreal-k+1) (k+1))
  where nreal = fromInteger n

-- P((p*100)% success in n+k trials | (s*100)% success in n trials)
--
-- It uses the beta function instead of choose when the size is lower
-- than 1000 in order to calculate fractional probabilities that are
-- finer than n or k. This may be a way to work around the noise
-- created by JSD sqrt distances.
--
-- Note that it not be strictly speaking a probability because it may
-- not sum up to one. In order to obtain a probability you may
-- normalize the distribution obtained from it. Or use pdf_beta
-- instead, which is a pdf instead of a probability, that would sum up
-- to one if integrated.
prob_beta :: Integer -> MyFloat -> Integer -> MyFloat -> MyFloat
prob_beta n s k p | cxreal < 0 || kreal < cxreal = 0
                  | n+k < defaultBetaThreshold = num / den
                  | otherwise = prob n x k cx
  where num = (nreal+1)*(choose_beta k cxreal)*(choose_beta n xreal)
        den = (nkreal+1)*(choose_beta (k+n) cxxreal)
        nreal = fromInteger n
        kreal = fromInteger k
        nkreal = nreal + kreal
        xreal = s*nreal
        cxxreal = p*nkreal
        cxreal = cxxreal - xreal
        x = round xreal
        cx = round cxreal

-- Like prob_beta but using only the beta function. The formula has
-- been obtained with Maxima (it has automatically simplified it by
-- provided the unconditional version of prob_beta.
prob_only_beta :: Integer -> MyFloat -> Integer -> MyFloat -> MyFloat
prob_only_beta n s k p | cxreal < 0 || kreal < cxreal = 0
                   | otherwise = num / den
  where num = beta (nkreal*(1 - p) + 1) (nkreal*p + 1)
        den1 = kreal + 1
        den2 = beta (nreal*(1-s) + 1) (nreal*s + 1)
        den3 = beta (nkreal*p - nreal*s + 1) (nreal*s - nkreal*p + kreal + 1)
        den = den1 * den2 * den3
        nreal = fromInteger n
        kreal = fromInteger k
        nkreal = nreal + kreal
        xreal = s*nreal
        cxxreal = p*nkreal
        cxreal = cxxreal - xreal

-- Like prob_beta, except it is a pdf
pdf_beta :: Integer -> MyFloat -> Integer -> MyFloat -> MyFloat
pdf_beta n s k p = (nreal + kreal) * prob_beta n s k p
  where nreal = fromInteger n
        kreal = fromInteger k

-- Like prob_only_beta, except it is a pdf
pdf_only_beta :: Integer -> MyFloat -> Integer -> MyFloat -> MyFloat
pdf_only_beta n s k p = (nreal + kreal) * prob_only_beta n s k p
  where nreal = fromInteger n
        kreal = fromInteger k

-- Little function to compute the probability corresponding to cx
cx2p :: Integer -> MyFloat -> Integer -> Integer -> MyFloat
cx2p n s k cx = (s*nreal + cxreal) / (nreal+kreal)
    where [nreal, kreal, cxreal] = Prelude.map fromInteger [n, k, cx]

-- Compute the pdf of Chapter 6, page 136
--
--             x^(k*s)*(1-x)^(k*(1-s))*x^(n*p)*(1-x)^(n*(1-p))
-- f(x) = ----------------------------------------------------------
--        Int_0^1 x^(k*s)*(1-x)^(k*(1-s))*x^(n*p)*(1-x)^(n*(1-p)) dx
--
--
--             x^[(k*s)+(n*p)]*(1-x)^[(k*(1-s))+(n*(1-p))]
-- f(x) = ----------------------------------------------------------
--        Int_0^1 x^(k*s)*(1-x)^(k*(1-s))*x^(n*p)*(1-x)^(n*(1-p)) dx
--
-- TODO: that doesn't make sense
pdf_chapter_6 = undefined

-- Compute the Kullback-Leibler divergence from hP to hQ. It assumes
-- that hQ has the same support as hP or greater, and that their
-- probabilities are non-null.
kld :: Dist -> Dist -> MyFloat
kld hP hQ = sum [p * (log2 (p / (hQp s))) | (s, p) <- toList hP]
    where hQp s = fromJust (Data.Map.lookup s hQ)

-- Compute the Jensen-Shannon divergence between hP and hQ. hM must be
-- the average distribution between hP and hQ.
jsd :: Dist -> Dist -> Dist -> MyFloat
jsd hP hQ hM = ((kld hP hM) + (kld hQ hM)) / 2.0

-- Compute the square root of the Jensen-Shannon divergence between hP
-- and hQ (to be a true metric). Generate hM on the fly as well.
sqrtJsd :: Dist -> Dist -> MyFloat
sqrtJsd hP hQ = sqrt (jsd hP hQ (average hP hQ))

-- This function takes a function: strength x probability -> value,
-- and distribution, and accumulate the result of this function over a
-- distribution.
accumulateWith :: (MyFloat -> MyFloat -> MyFloat) -> Dist -> MyFloat
accumulateWith f = foldrWithKey (\s p r -> r + (f s p)) 0.0

-- Compute the mean of a distribution.
mean :: Dist -> MyFloat
mean = accumulateWith (*)

-- Compute the mode of a distribution (the strength with the highest
-- probability). -1.0 if the distribution is empty.
mode :: Dist -> MyFloat
mode h = fst (foldrWithKey max_p (-1.0, -1.0) h)
    where max_p s p (s_max_p, p_max_p) = if p > p_max_p then (s, p)
                                         else (s_max_p, p_max_p)

-- Compute the variance of a distribution.
variance :: Dist -> MyFloat
variance h = accumulateWith (\s p -> p*(s - m)**2.0) h
    where m = mean h

-- Compute the standard deviation of a distribution.
stdDev :: Dist -> MyFloat
stdDev = sqrt . variance

-------------------
-- Indefinite TV --
-------------------

-- Given a distribution, a confidence interval and L, compute the
-- corresponding U.
toUp :: Dist -> MyFloat -> MyFloat -> MyFloat
toUp h b l = fst (foldlWithKey f (0.0, 0.0) h)
  where
    -- Replace f by fDbg in the upper expression to debug
    fDbg (u, a) s p = trace (format "toUp f {0} {1} {2} = {3}"
                             [show (u, a), show s, show p, show res])
                      res where res = f (u, a) s p
    f (u, a) s p | s < l || b <= a = (u, a)
                 | otherwise = (s, a + p)

-- Given a distribution, a confidence interval and U, compute the
-- corresponding L.
toLow :: Dist -> MyFloat -> MyFloat -> MyFloat
toLow h b u = fst (foldrWithKey f (0.0, 0.0) h)
  where
    -- Replace f by fDbg in the upper expression to debug
    fDbg s p (l, a) = trace (format "toLow f {0} {1} {2} = {3}"
                             [show s, show p, show (l, a), show res])
                      res where res = f s p (l, a)
    f s p (l, a) | u < s || b <= a = (l, a)
                 | otherwise = (s, a + p)

-- Compute the interval [L, U] of a distribution as to minimize U-L
-- and such that (b*100)% of it is in this interval.
indefiniteInterval :: MyFloat -> Dist -> (MyFloat, MyFloat)
indefiniteInterval b h = (low, up)
    where min_low = 0
          max_low = toLow h b 1.0
          guess = (min_low + max_low) / 2
          step = 1.0 / (fromInteger defaultResolution)
          width l = (toUp h b l) - l
          jump = floatMiddle
          low = optimize width jump step min_low max_low guess
          up = toUp h b low

-- Return the width of a distribution
indefiniteIntervalWidth :: MyFloat -> Dist -> MyFloat
indefiniteIntervalWidth b h = up - low
    where (low, up) = indefiniteInterval b h

------------------
-- General/Math --
------------------

-- Base 2 log
log2 :: MyFloat -> MyFloat
log2 = logBase 2

-- Beta function
beta :: MyFloat -> MyFloat -> MyFloat
beta z w = (gamma z) * (gamma w) / gamma (z+w)

-- Find the middle between 2 integers using the division function d
-- (I'm sure I can do better by defining a division that works for
-- both Integral and Fractional types).
middle :: Num a => (a -> a -> a) -> a -> a -> a
middle d l u = d (l + u) 2

integerMiddle = middle div
floatMiddle = middle (/)

-- Find the value x that minimzes fun x, assuming x is in [low, up],
-- and given an initial guess. It is strongly adviced to memoize fun
-- beforehand.
--
-- jump is a function that take the new interval and returns a new
-- guess (it would typically be the middle of the new interval).
--
-- step is the smaller meaningful change in x. If the function is
-- noisy setting a larger step can be useful.
optimize :: (Num a, Ord a, Show a) =>
            (a -> MyFloat) -> (a -> a -> a) -> a -> a -> a -> a -> a
optimize fun jump step low up guess
    | width < step = guess
    | up < rs = -- trace (format "optimize up={0} < rs={1}, fgu={2}, fls={3}"
                --       [show up, show rs, show fgu, show fls]) $
                if fgu <= fls then guess else rec_optimize low ls lj
    | ls < low = -- trace (format "optimize ls={0} < low={1}, fgu={2}, frs={3}"
                 --       [show ls, show low, show fgu, show frs]) $
                 if fgu <= frs then guess else rec_optimize rs up rj
    | otherwise = -- trace (format "optimize fgu={0}, fls={1}, frs={2}"
                  --       [show fgu, show fls, show frs]) $
                  if fgu <= fls && fgu <= frs then guess
                  else if frs - fls >= 0
                       then rec_optimize low ls lj
                       else rec_optimize rs up rj
    where width = up - low
          ls = guess - step  -- step to the left
          rs = guess + step  -- step to the right
          lj = jump low ls  -- jump to the left
          rj = jump rs up   -- jump to the right
          fgu = fun guess
          fls = fun ls
          frs = fun rs
          rec_optimize = optimize fun jump step -- Simplified
                                                -- recursive call of
                                                -- optimize

optimizeDbg fun jump step low up guess =
    trace (format "optimize fun {0} {1} {2} = {3}"
           (Prelude.map show [low, up, guess, result]))
    result where result = optimize fun jump step low up guess

-- MultiMap mapKeys fix
mapKeys_fix f m = fromMap (foldrWithKey f' empty m')
    where m' = toMap m
          f' k a b = insertWith (++) (f k) a b
