# Default Note
Shift+Ctrl+Alt+X

restapi.Start
agents-stop-loop

loadpy /home/kdm/opencog-python-blending/blending_agent
agents-start /home/kdm/opencog-python-blending/blending_agent.BlendingAgent
agents-step opencog::PyMindAgent(/home/kdm/opencog-python-blending/blending_agent.BlendingAgent)


opencog::HebbianUpdatingAgent
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

python agent가 매번 새로 초기화되는데 어쩔..?
-> 구조 고친다 -> 또 오래 걸림
-> 꼼수 쓴다
ocviewer STI 밸류 안 되는거 고침
config 파일 변경 안되게 바꿈
STI 설정에서 불러오기

새로운 노드의 STI는?
