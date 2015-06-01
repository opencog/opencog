# Default Note
loadpy /home/kdm/opencog-python-blending/blending_agent
agents-step /home/kdm/opencog-python-blending/blending_agent.BlendingAgent

loadpy blending_agent
agents-step blending_agent.BlendingAgent


at.set_av(a1.h, 10000)
at.get_av(a1.h)
at.get_atoms_by_av(6000)
at.get_atoms_in_attentional_focus()

a1.av={'sti':100}
