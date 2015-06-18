# OpenPsi

Work is progressing on this, so is not fully functional. The main task that is
being performed on it is refactoring, so as to make it run independent of the
embodiment code. When the refactoring is finalized it would be possible to use
OpenPsi to drive the goal driven dynamics of mind-agents/process, embodiment
agents as well as other opencog modules/components.

### Resources
* http://wiki.opencog.org/w/OpenPsi_%28Embodiment%29
* http://wiki.hansonrobotics.com/w/Emotion_modeling
* [MicroPsi publications](http://micropsi.com/publications/publications.html)
* [MicroPsi source code]()
* [Principles of Synthetic Intelligence](http://wiki.humanobs.org/_media/public:events:agi-summerschool-2012:psi-oup-version-draft-jan-08.pdf)


### Usage
After starting the cogserver
1. loadmodule opencog/dynamics/openpsi/libOpenPsi.so

2. Start any of the agents
    * agents-start opencog::PsiActionSelectionAgent
    * agents-start opencog::PsiModulatorUpdaterAgent
    * agents-start opencog::PsiDemandUpdaterAgent

3. Stop any of the started agents
    * agents-stop opencog::PsiActionSelectionAgent
    * agents-stop opencog::PsiModulatorUpdaterAgent
    * agents-stop opencog::PsiDemandUpdaterAgent

OpenPsi Rules aren't defined yet so nothing interesting occurs.
