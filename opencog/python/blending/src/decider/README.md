# decider
* Directory to save the available blending valuation decider.

## Files
* base_decider.py: Abstract class to provide 'blending_decide()' interface.
* decide_null.py: Not decide, just returns given focus_atoms.
* decide_random.py: Decide to blend or not by randomly choosing the
 given focus_atoms.
* decide_best_sti.py: Decide to blend or not by checking the existence of atoms
 within proper STI range.
* decider_finder.py: Provider class to make blending decider instance.
