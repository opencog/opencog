# Default Note
Shift+Ctrl+Alt+X

restapi.Start
agents-stop-loop

loadpy /home/kdm/opencog-python-blending/blending_agent
agents-start /home/kdm/opencog-python-blending/blending_agent.BlendingAgent
agents-step opencog::PyMindAgent(/home/kdm/opencog-python-blending/blending_agent.BlendingAgent)


agents-stop-loop
agents-start opencog::HebbianUpdatingAgent
agents-step opencog::HebbianUpdatingAgent

// agents-stop-loop
// loadpy blending_agent
// agents-step opencog::PyMindAgent(blending_agent.BlendingAgent)

at.set_av(a1.h, 10000)
at.get_av(a1.h)
at.get_atoms_by_av(6000)
at.get_atoms_in_attentional_focus()

a1.av={'sti':100}


opencog::ForgettingAgent
opencog::HebbianUpdatingAgent
opencog::ImportanceSpreadingAgent
opencog::ImportanceUpdatingAgent
opencog::SimpleHebbianUpdatingAgent
opencog::SimpleImportanceDiffusionAgent

loadmodule opencog/dynamics/attention/libhebbiancreation.so

