# chooser
* Directory to save the available Atom Choosers.

## Files
* base_chooser.py: Abstract class to provide 'atom_choose()' interface.
* choose_null.py: Not choose, just returns given focus_atoms.
* choose_all.py: Choose every atom that has given type,
 and check the number of atoms.
* choose_in_sti_range.py: Choose every atom that has given type,
 within proper STI range, and check the number of atoms.
* chooser_finder.py: Provider class to make atoms chooser instance.
