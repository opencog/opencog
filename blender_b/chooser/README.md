# chooser
* Directory to save the available Atom choosers.

## Files
* base_chooser.py: Abstract class to provide 'atom_choose()' interface.
* random_all.py: Choose atoms by random selecting.
* random_in_blend_target.py: Choose atoms by random selecting which 
  are connected with 'BlendTarget' ConceptNode. 
* random_in_sti_range.py: Choose atoms by random selecting which 
  have proper STI value. 

* chooser_factory.py: Provide chooser instance for user.