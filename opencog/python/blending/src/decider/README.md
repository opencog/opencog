# decider
* Directory to save the available blending valuation decider.

## Files
* base_decider.py: Abstract class to provide 'blending_decide()' interface.
* decide_null.py: Not decide, just return chosen atoms.
* decide_random.py: It estimates that the chosen atoms are always worth.
* decide_best_sti.py: It estimates that the chosen atoms are worth when they 
 have higher than some threshold value.
* decider_finder.py: Provide connector instance for user.
