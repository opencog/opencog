Global $_LG_DLL = DllOpen("link-grammar.dll")

Func _LG_GetVersion()
	;const char * linkgrammar_get_version(void);
	$result = DllCall($_LG_DLL, "str:cdecl", "linkgrammar_get_version")
	Return $result[0]
EndFunc   ;==>_LG_GetVersion

Func _LG_DictionaryCreateDefault()
	;Dictionary dictionary_create_default_lang(void);
	$result = DllCall($_LG_DLL, "ptr:cdecl", "dictionary_create_default_lang")
	Return $result[0]
EndFunc   ;==>_LG_DictionaryCreateDefault

Func _LG_DictionaryCreateLang($sLanguage)
	;Dictionary dictionary_create_lang(const char * lang);
	$result = DllCall($_LG_DLL, "ptr:cdecl", "dictionary_create_lang", "str", $sLanguage)
	Return $result[0]
EndFunc   ;==>_LG_DictionaryCreateLang

;Func _LG_Dictionary_Create($sDictName, $sPostProcessFileName, $sConstituentKnowledgeName, $sAffixName)
;Dictionary dictionary_create(const char *dict_name,
;                             const char *post_process_file_name,
;                             const char *constituent_knowledge_name,
;                             const char *affix_name);EndFunc
;EndFunc

Func _LG_DictionaryDelete($hDictionary)
	;int dictionary_delete(Dictionary dict);
	DllCall($_LG_DLL, "none:cdecl", "dictionary_delete", "ptr", $hDictionary)
EndFunc   ;==>_LG_DictionaryDelete

Func _LG_DictionarySetData($sPath)
	;void dictionary_set_data_dir(const char * path);
	DllCall($_LG_DLL, "none:cdecl", "dictionary_set_data_dir", "str", $sPath)
EndFunc   ;==>_LG_DictionarySetData

Func _LG_DictionaryGetDataDir()
	;const char * dictionary_get_data_dir(void);
	$result = DllCall($_LG_DLL, "str:cdecl", "dictionary_get_data_dir")
	Return $result[0]
EndFunc   ;==>_LG_DictionaryGetDataDir

Func _LG_DictionaryGetMaxCost($hDictionary)
	;int dictionary_get_max_cost(Dictionary dict);
	$result = DllCall($_LG_DLL, "int:cdecl", "dictionary_get_max_cost", "ptr", $hDictionary)
	Return $result[0]
EndFunc   ;==>_LG_DictionaryGetMaxCost

Func _LG_ParseOptionsCreate()
	;Parse_Options  parse_options_create();
	$result = DllCall($_LG_DLL, "ptr:cdecl", "parse_options_create")
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsCreate

Func _LG_ParseOptionsDelete($hOptions)
	;int parse_options_delete(Parse_Options opts);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_delete", "ptr", $hOptions)
EndFunc   ;==>_LG_ParseOptionsDelete

Func _LG_ParseOptionsSetVerbosity($hOptions, $iVerbosity)
	;void parse_options_set_verbosity(Parse_Options opts, int verbosity);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_verbosity", "ptr", $hOptions, "int", $iVerbosity)
EndFunc   ;==>_LG_ParseOptionsSetVerbosity

Func _LG_ParseOptionsGetVerbosity($hOptions)
	;int  parse_options_get_verbosity(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_verbosity", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetVerbosity

Func _LG_ParseOptionsSetLinkageLimit($hOptions, $iLinkageLimit)
	;void parse_options_set_linkage_limit(Parse_Options opts, int linkage_limit);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_linkage_limit", "ptr", $hOptions, "int", $iLinkageLimit)
EndFunc   ;==>_LG_ParseOptionsSetLinkageLimit

Func _LG_ParseOptionsGetLinkageLimit($hOptions)
	;int  parse_options_get_linkage_limit(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_linkage_limit", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetLinkageLimit

Func _LG_ParseOptionsSetDisjunctCost($hOptions, $iDisjunctCost)
	;void parse_options_set_disjunct_cost(Parse_Options opts, int disjunct_cost);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_disjunct_cost", "ptr", $hOptions, "int", $iDisjunctCost)
EndFunc   ;==>_LG_ParseOptionsSetDisjunctCost

Func _LG_ParseOptionsGetDisjunctCost($hOptions)
	;int  parse_options_get_disjunct_cost(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_disjunct_cost", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetDisjunctCost

Func _LG_ParseOptionsSetMinNullCount($hOptions, $iNullCount)
	;void parse_options_set_min_null_count(Parse_Options opts, int null_count);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_min_null_count", "ptr", $hOptions, "int", $iNullCount)
EndFunc   ;==>_LG_ParseOptionsSetMinNullCount

Func _LG_ParseOptionsGetMinNullCount($hOptions)
	;int  parse_options_get_min_null_count(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_min_null_count", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetMinNullCount

Func _LG_ParseOptionsSetMaxNullCount($hOptions, $iNullCount)
	;void parse_options_set_max_null_count(Parse_Options opts, int null_count);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_max_null_count", "ptr", $hOptions, "int", $iNullCount)
EndFunc   ;==>_LG_ParseOptionsSetMaxNullCount

Func _LG_ParseOptionsGetMaxNullCount($hOptions)
	;int  parse_options_get_max_null_count(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_max_null_count", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetMaxNullCount

Func _LG_ParseOptionsSetNullBlock($hOptions, $iNullBlock)
	;void parse_options_set_null_block(Parse_Options opts, int null_block);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_null_block", "ptr", $hOptions, "int", $iNullBlock)
EndFunc   ;==>_LG_ParseOptionsSetNullBlock

Func _LG_ParseOptionsGetNullBlock($hOptions)
	;int  parse_options_get_null_block(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_null_block", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetNullBlock

Func _LG_ParseOptionsSetShortLength($hOptions, $iShortLength)
	;void parse_options_set_short_length(Parse_Options opts, int short_length);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_short_length", "ptr", $hOptions, "int", $iShortLength)
EndFunc   ;==>_LG_ParseOptionsSetShortLength

Func _LG_ParseOptionsGetShortLength($hOptions)
	;int  parse_options_get_short_length(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_short_length", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetShortLength

Func _LG_ParseOptionsSetIslandsOk($hOptions, $iIslandsOk)
	;void parse_options_set_islands_ok(Parse_Options opts, int islands_ok);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_islands_ok", "ptr", $hOptions, "int", $iIslandsOk)
EndFunc   ;==>_LG_ParseOptionsSetIslandsOk

Func _LG_ParseOptionsGetIslandsOk($hOptions)
	;int  parse_options_get_islands_ok(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_islands_ok", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetIslandsOk

Func _LG_ParseOptionsSetMaxParseTime($hOptions, $iSeconds)
	;void parse_options_set_max_parse_time(Parse_Options  opts, int secs);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_max_parse_time", "ptr", $hOptions, "int", $iSeconds)
EndFunc   ;==>_LG_ParseOptionsSetMaxParseTime

Func _LG_ParseOptionsGetMaxParseTime($hOptions)
	;int  parse_options_get_max_parse_time(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_max_parse_time", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetMaxParseTime

Func _LG_ParseOptionsSetMaxMemory($hOptions, $iMemory)
	;void parse_options_set_max_memory(Parse_Options  opts, int mem);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_max_memory", "ptr", $hOptions, "int", $iMemory)
EndFunc   ;==>_LG_ParseOptionsSetMaxMemory

Func _LG_ParseOptionsGetMaxMemory($hOptions)
	;int  parse_options_get_max_memory(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_max_memory", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetMaxMemory

Func _LG_ParseOptionsTimerExpired($hOptions)
	;int  parse_options_timer_expired(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_timer_expired", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsTimerExpired

Func _LG_ParseOptionsMemoryExhausted($hOptions)
	;int  parse_options_memory_exhausted(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_memory_exhausted", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsMemoryExhausted

Func _LG_ParseOptionsResourcesExhausted($hOptions)
	;int  parse_options_resources_exhausted(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_resources_exhausted", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsResourcesExhausted

Func _LG_ParseOptionsResetResources($hOptions)
	;void parse_options_reset_resources(Parse_Options opts);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_reset_resources", "ptr", $hOptions)
EndFunc   ;==>_LG_ParseOptionsResetResources

Func _LG_ParseOptionsSetCostModelType($hOptions, $iCostModel)
	;void parse_options_set_cost_model_type(Parse_Options opts, int cm);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_cost_model_type", "ptr", $hOptions, "int", $iCostModel)
EndFunc   ;==>_LG_ParseOptionsSetCostModelType

Func _LG_ParseOptionsGetCostModelType($hOptions)
	;int  parse_options_get_cost_model_type(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_cost_model_type", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetCostModelType

Func _LG_ParseOptionsSetScreenWidth($hOptions, $iWidth)
	;void parse_options_set_screen_width(Parse_Options opts, int val);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_screen_width", "ptr", $hOptions, "int", $iWidth)
EndFunc   ;==>_LG_ParseOptionsSetScreenWidth

Func _LG_ParseOptionsGetScreenWidth($hOptions)
	;int  parse_options_get_screen_width(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_screen_width", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetScreenWidth

Func _LG_ParseOptionsSetAllowNull($hOptions, $iAllowNull)
	;void parse_options_set_allow_null(Parse_Options opts, int val);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_allow_null", "ptr", $hOptions, "int", $iAllowNull)
EndFunc   ;==>_LG_ParseOptionsSetAllowNull

Func _LG_ParseOptionsGetAllowNull($hOptions)
	;int  parse_options_get_allow_null(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_allow_null", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetAllowNull

Func _LG_ParseOptionsSetDisplayWalls($hOptions, $iDisplayWalls)
	;void parse_options_set_display_walls(Parse_Options opts, int val);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_display_walls", "ptr", $hOptions, "int", $iDisplayWalls)
EndFunc   ;==>_LG_ParseOptionsSetDisplayWalls

Func _LG_ParseOptionsGetDisplayWalls($hOptions)
	;int  parse_options_get_display_walls(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_display_walls", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetDisplayWalls

Func _LG_ParseOptionsSetAllShortConnectors($hOptions, $iShortConnectors)
	;void parse_options_set_all_short_connectors(Parse_Options opts, int val);
	DllCall($_LG_DLL, "none:cdecl", "parse_options_set_all_short_connectors", "ptr", $hOptions, "int", $iShortConnectors)
EndFunc   ;==>_LG_ParseOptionsSetAllShortConnectors

Func _LG_ParseOptionsGetAllShortConnectors($hOptions)
	;int  parse_options_get_all_short_connectors(Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "parse_options_get_all_short_connectors", "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_ParseOptionsGetAllShortConnectors

Func _LG_SentenceCreate($sInputString, $hDictionary)
	;Sentence sentence_create(const char *input_string, Dictionary dict);
	$result = DllCall($_LG_DLL, "ptr:cdecl", "sentence_create", "str", $sInputString, "ptr", $hDictionary)
	Return $result[0]
EndFunc   ;==>_LG_SentenceCreate

Func _LG_SentenceDelete($hSentence)
	;void sentence_delete(Sentence sent);
	DllCall($_LG_DLL, "none:cdecl", "sentence_delete", "ptr", $hSentence)
EndFunc   ;==>_LG_SentenceDelete

Func _LG_SentenceSplit($hSentence, $hOptions)
	;int sentence_split(Sentence sent, Parse_Options opts);
	DllCall($_LG_DLL, "none:cdecl", "sentence_split", "ptr", $hSentence, "ptr", $hOptions)
EndFunc   ;==>_LG_SentenceSplit

Func _LG_SentenceParse($hSentence, $hOptions)
	;int sentence_parse(Sentence sent, Parse_Options opts);
	$result = DllCall($_LG_DLL, "int:cdecl", "sentence_parse", "ptr", $hSentence, "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_SentenceParse

Func _LG_SentenceLength($hSentence)
	;int sentence_length(Sentence sent);
	$result = DllCall($_LG_DLL, "int:cdecl", "sentence_length", "ptr", $hSentence)
	Return $result[0]
EndFunc   ;==>_LG_SentenceLength

Func _LG_SentenceNullCount($hSentence)
	;int sentence_null_count(Sentence sent);
	$result = DllCall($_LG_DLL, "int:cdecl", "sentence_null_count", "ptr", $hSentence)
	Return $result[0]
EndFunc   ;==>_LG_SentenceNullCount

Func _LG_SentenceNumLinkagesFound($hSentence)
	;int sentence_num_linkages_found(Sentence sent);
	$result = DllCall($_LG_DLL, "int:cdecl", "sentence_num_linkages_found", "ptr", $hSentence)
	Return $result[0]
EndFunc   ;==>_LG_SentenceNumLinkagesFound

Func _LG_SentenceNumValidLinkages($hSentence)
	;int sentence_num_valid_linkages(Sentence sent);
	$result = DllCall($_LG_DLL, "int:cdecl", "sentence_num_valid_linkages", "ptr", $hSentence)
	Return $result[0]
EndFunc   ;==>_LG_SentenceNumValidLinkages

Func _LG_SentenceNumLinkagesPostProcessed($hSentence)
	;int sentence_num_linkages_post_processed(Sentence sent);
	$result = DllCall($_LG_DLL, "int:cdecl", "sentence_num_linkages_post_processed", "ptr", $hSentence)
	Return $result[0]
EndFunc   ;==>_LG_SentenceNumLinkagesPostProcessed

Func _LG_SentenceNumViolations($hSentence, $iNumber)
	;int sentence_num_violations(Sentence sent, int i);
	$result = DllCall($_LG_DLL, "int:cdecl", "sentence_num_violations", "ptr", $hSentence, "int", $iNumber)
	Return $result[0]
EndFunc   ;==>_LG_SentenceNumViolations

Func _LG_SentenceDisjunctCost($hSentence, $iNumber)
	;int sentence_disjunct_cost(Sentence sent, int i);
	$result = DllCall($_LG_DLL, "int:cdecl", "sentence_disjunct_cost", "ptr", $hSentence, "int", $iNumber)
	Return $result[0]
EndFunc   ;==>_LG_SentenceDisjunctCost

Func _LG_SentenceLinkCost($hSentence, $iNumber)
	;int sentence_link_cost(Sentence sent, int i);
	$result = DllCall($_LG_DLL, "int:cdecl", "sentence_link_cost", "ptr", $hSentence, "int", $iNumber)
	Return $result[0]
EndFunc   ;==>_LG_SentenceLinkCost

Func _LG_LinkageCreate($iIndex, $hSentence, $hOptions)
	;Linkage  linkage_create(int index, Sentence sent, Parse_Options opts);
	$result = DllCall($_LG_DLL, "ptr:cdecl", "linkage_create", "int", $iIndex, "ptr", $hSentence, "ptr", $hOptions)
	Return $result[0]
EndFunc   ;==>_LG_LinkageCreate

Func _LG_LinkageGetNumWords($hLinkage)
	;int linkage_get_num_words(Linkage linkage);
	$result = DllCall($_LG_DLL, "int:cdecl", "linkage_get_num_words", "ptr", $hLinkage)
	Return $result[0]
EndFunc   ;==>_LG_LinkageGetNumWords

Func _LG_LinkageGetNumLinks($hLinkage)
	;int  linkage_get_num_links(Linkage linkage);
	$result = DllCall($_LG_DLL, "int:cdecl", "linkage_get_num_links", "ptr", $hLinkage)
	Return $result[0]
EndFunc   ;==>_LG_LinkageGetNumLinks

Func _LG_LinkageGetLinkLength($hLinkage, $iIndex)
	;int linkage_get_link_length(Linkage linkage, int index);
	$result = DllCall($_LG_DLL, "int:cdecl", "linkage_get_link_length", "ptr", $hLinkage, "int", $iIndex)
	Return $result[0]
EndFunc   ;==>_LG_LinkageGetLinkLength

Func _LG_LinkageGetLinkLWord($hLinkage, $iIndex)
	;int linkage_get_link_lword(Linkage linkage, int index);
	$result = DllCall($_LG_DLL, "int:cdecl", "linkage_get_link_lword", "ptr", $hLinkage, "int", $iIndex)
	Return $result[0]
EndFunc   ;==>_LG_LinkageGetLinkLWord

Func _LG_LinkageGetLinkRWord($hLinkage, $iIndex)
	;int linkage_get_link_rword(Linkage linkage, int index);
	$result = DllCall($_LG_DLL, "int:cdecl", "linkage_get_link_rword", "ptr", $hLinkage, "int", $iIndex)
	Return $result[0]
EndFunc   ;==>_LG_LinkageGetLinkRWord

Func _LG_LinkagePrintDiagram($hLinkage)
	;char * linkage_print_diagram(Linkage linkage);
	$result = DllCall($_LG_DLL, "str:cdecl", "linkage_print_diagram", "ptr", $hLinkage)
	Return $result[0]
EndFunc   ;==>_LG_LinkagePrintDiagram

Func _LG_LinkageFreeDiagram($hDiagram)
	;void   linkage_free_diagram(char * str);
	DllCall($_LG_DLL, "none:cdecl", "linkage_free_diagram", "ptr", $hDiagram)
EndFunc   ;==>_LG_LinkageFreeDiagram

Func _LG_LinkagePrintPostscript($hLinkage, $iMode)
	;char * linkage_print_postscript(Linkage linkage, int mode);
	$result = DllCall($_LG_DLL, "str:cdecl", "linkage_print_postscript", "ptr", $hLinkage, "int", $iMode)
	Return $result[0]
EndFunc   ;==>_LG_LinkagePrintPostscript

Func _LG_LinkageFreePostscript($hPostscript)
	;void   linkage_free_postscript(char * str);
	DllCall($_LG_DLL, "none:cdecl", "linkage_free_postscript", "ptr", $hPostscript)
EndFunc   ;==>_LG_LinkageFreePostscript

Func _LG_LinkagePrintLinksAndDomains($hLinkage)
	;char * linkage_print_links_and_domains(Linkage linkage);
	$result = DllCall($_LG_DLL, "str:cdecl", "linkage_print_links_and_domains", "ptr", $hLinkage)
	Return $result[0]
EndFunc   ;==>_LG_LinkagePrintLinksAndDomains

Func _LG_LinkageFreeLinksAndDomains($hLinksAndDomains)
	;void   linkage_free_links_and_domains(char *str);
	DllCall($_LG_DLL, "none:cdecl", "linkage_free_links_and_domains", "ptr", $hLinksAndDomains)
EndFunc   ;==>_LG_LinkageFreeLinksAndDomains

Func _LG_LinkagePrintConstituentTree($hLinkage, $iOpt)
	;char * linkage_print_constituent_tree(Linkage linkage int iOpt);
	$result = DllCall($_LG_DLL, "str:cdecl", "linkage_print_constituent_tree", "ptr", $hLinkage, "int", $iOpt)
	Return $result[0]
EndFunc   ;==>_LG_LinkagePrintConstituentTree

Func _LG_LinkageGetLinkLabel($hLinkage, $iIndex)
	;const char * linkage_get_link_label(Linkage linkage, int index);
	$result = DllCall($_LG_DLL, "str:cdecl", "linkage_get_link_label", "ptr", $hLinkage, "int", $iIndex)
	Return $result[0]
EndFunc   ;==>_LG_LinkageGetLinkLabel

Func _LG_LinkageGetLinkLLabel($hLinkage, $iIndex)
	;const char * linkage_get_link_llabel(Linkage linkage, int index);
	$result = DllCall($_LG_DLL, "str:cdecl", "linkage_get_link_llabel", "ptr", $hLinkage, "int", $iIndex)
	Return $result[0]
EndFunc   ;==>_LG_LinkageGetLinkLLabel

Func _LG_LinkageGetLinkRLabel($hLinkage, $iIndex)
	;const char * linkage_get_link_rlabel(Linkage linkage, int index);
	$result = DllCall($_LG_DLL, "str:cdecl", "linkage_get_link_rlabel", "ptr", $hLinkage, "int", $iIndex)
	Return $result[0]
EndFunc   ;==>_LG_LinkageGetLinkRLabel

Func _LG_LinkageGetLinkNumDomains($hLinkage, $iIndex)
	;int     linkage_get_link_num_domains(Linkage linkage, int index);
	$result = DllCall($_LG_DLL, "int:cdecl", "linkage_get_link_num_domains", "ptr", $hLinkage, "int", $iIndex)
	Return $result[0]
EndFunc   ;==>_LG_LinkageGetLinkNumDomains

Func _LG_LinkageGetLinkDomainNames($hLinkage, $iIndex)
	;const char ** linkage_get_link_domain_names(Linkage linkage, int index);
	$result = DllCall($_LG_DLL, "str:cdecl", "linkage_get_link_domain_names", "ptr", $hLinkage, "int", $iIndex)
	Return $result[0]
EndFunc   ;==>_LG_LinkageGetLinkDomainNames

Func _LG_LinkageGetViolationName($hLinkage)
	;const char *  linkage_get_violation_name(Linkage linkage);
	$result = DllCall($_LG_DLL, "str:cdecl", "linkage_get_violation_name", "ptr", $hLinkage)
	Return $result[0]
EndFunc   ;==>_LG_LinkageGetViolationName

;Func _LG_LinkageGetWords($hLinkage)
;	;const char ** linkage_get_words(Linkage linkage);
;			$result = DllCall($_LG_DLL, "int:cdecl", "linkage_get_words", "ptr", $hLinkage)
;	Return $result[0]
;EndFunc

Func _LG_LinkageGetWord($hLinkage, $iWord)
	;const char *  linkage_get_word(Linkage linkage, int w);
	$result = DllCall($_LG_DLL, "str:cdecl", "linkage_get_word", "ptr", $hLinkage, "int", $iWord)
	Return $result[0]
EndFunc   ;==>_LG_LinkageGetWord

Func _LG_LinkageGetDisjunct($hLinkage, $iWord)
	;const char *  linkage_get_disjunct(Linkage linkage, int w);
	$result = DllCall($_LG_DLL, "str:cdecl", "linkage_get_disjunct", "ptr", $hLinkage, "int", $iWord)
	Return $result[0]
EndFunc   ;==>_LG_LinkageGetDisjunct

Func _LG_LinkageUnusedWordCost($hLinkage)
	;int linkage_unused_word_cost(Linkage linkage);
	$result = DllCall($_LG_DLL, "int:cdecl", "linkage_unused_word_cost", "ptr", $hLinkage)
	Return $result[0]
EndFunc   ;==>_LG_LinkageUnusedWordCost

Func _LG_LinkageDisjunctCost($hLinkage)
	;int linkage_disjunct_cost(Linkage linkage);
	$result = DllCall($_LG_DLL, "int:cdecl", "linkage_disjunct_cost", "ptr", $hLinkage)
	Return $result[0]
EndFunc   ;==>_LG_LinkageDisjunctCost

Func _LG_LinkageLinkCost($hLinkage)
	;int linkage_link_cost(Linkage linkage);
	$result = DllCall($_LG_DLL, "int:cdecl", "linkage_link_cost", "ptr", $hLinkage)
	Return $result[0]
EndFunc   ;==>_LG_LinkageLinkCost

Func _LG_LinkageHasInconsistentDomains($hLinkage)
	;int linkage_has_inconsistent_domains(Linkage linkage);
	$result = DllCall($_LG_DLL, "int:cdecl", "linkage_has_inconsistent_domains", "ptr", $hLinkage)
	Return $result[0]
EndFunc   ;==>_LG_LinkageHasInconsistentDomains

Func _LG_LinkageDelete($hLinkage)
	;void linkage_delete(Linkage linkage);
	$result = DllCall($_LG_DLL, "none:cdecl", "linkage_delete", "ptr", $hLinkage)
EndFunc   ;==>_LG_LinkageDelete

Func _LG_PostProcessOpen($sName)
	;PostProcessor   post_process_open(const char * name);
	$result = DllCall($_LG_DLL, "ptr:cdecl", "post_process_open", "str", $sName)
	Return $result[0]
EndFunc   ;==>_LG_PostProcessOpen

Func _LG_PostProcessClose($hPostProcessor)
	;void            post_process_close(PostProcessor postprocessor);
	DllCall($_LG_DLL, "none:cdecl", "post_process_close", "ptr", $hPostProcessor)
EndFunc   ;==>_LG_PostProcessClose

Func _LG_LinkagePostProcess($hLinkage, $hPostProcessor)
	;void linkage_post_process(Linkage linkage, PostProcessor postprocessor);
	DllCall($_LG_DLL, "none:cdecl", "linkage_post_process", "ptr", $hLinkage, "ptr", $hPostProcessor)
EndFunc   ;==>_LG_LinkagePostProcess


Func _LG_LinkagePrintDisjuncts($hLinkage)
	$result = DllCall($_LG_DLL, "str:cdecl", "linkage_print_disjuncts", "ptr", $hLinkage)
	Return $result[0]
EndFunc
