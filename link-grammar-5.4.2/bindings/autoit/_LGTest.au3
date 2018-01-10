#include "_LinkGrammar.au3"

$Test = "My dog likes dog food."

$options = _LG_ParseOptionsCreate()
$dict = _LG_DictionaryCreateLang("en")
$Sentence = _LG_SentenceCreate($Test, $dict)
_LG_SentenceSplit($Sentence, $options)


$num_linkages = _LG_SentenceParse($Sentence, $options)

If $num_linkages > 0 Then
	$linkage = _LG_LinkageCreate(0, $Sentence, $options)
	$diagram = _LG_LinkagePrintDiagram($linkage)
	$diagram2 = _LG_LinkagePrintConstituentTree($linkage, 3)
	$diagram3 = _LG_LinkagePrintDisjuncts($linkage)

	ConsoleWrite($diagram & @CRLF)
	ConsoleWrite($diagram2 & @CRLF)
	ConsoleWrite($diagram3 & @CRLF)
EndIf
