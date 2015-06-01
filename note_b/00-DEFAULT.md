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

새로운 노드의 STI는?
다른 뽑는 방법 테스트
다른 예제도 바꾸기
