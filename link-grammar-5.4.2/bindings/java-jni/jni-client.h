
#include <jni.h>
/* Header for class LinkGrammar */

#ifndef _LinkGrammar_H_
#define _LinkGrammar_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Class:     LinkGrammar
 * Method:    setMaxParseSeconds
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_org_linkgrammar_LinkGrammar_setMaxParseSeconds
	(JNIEnv *, jclass, jint);

/*
 * Class:     LinkGrammar
 * Method:    setMaxCost
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_org_linkgrammar_LinkGrammar_setMaxCost
	(JNIEnv *, jclass, jdouble);

/*
 * Class:     LinkGrammar
 * Method:    setDictionariesPath
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_org_linkgrammar_LinkGrammar_setDictionariesPath
  (JNIEnv *, jclass, jstring);

/*
 * Class:     LinkGrammar
 * Method:    setLanguage
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_org_linkgrammar_LinkGrammar_setLanguage
  (JNIEnv *, jclass, jstring);

/*
 * Class:     LinkGrammar
 * Method:    setMaxLinkages
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_org_linkgrammar_LinkGrammar_setMaxLinkages
	(JNIEnv *, jclass, jint);

JNIEXPORT jint JNICALL Java_org_linkgrammar_LinkGrammar_getMaxLinkages
	(JNIEnv *, jclass);

/*
 * Class:     LinkGrammar
 * Method:    init
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_org_linkgrammar_LinkGrammar_init
	(JNIEnv *, jclass);

/*
 * Class:     LinkGrammar
 * Method:    parse
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_org_linkgrammar_LinkGrammar_parse
	(JNIEnv *, jclass, jstring);

void unit_test_jparse(JNIEnv *, const char*);

/*
 * Class:     LinkGrammar
 * Method:    close
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_org_linkgrammar_LinkGrammar_close
	(JNIEnv *, jclass);

/*
 * Class:     LinkGrammar
 * Method:    finalize
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_org_linkgrammar_LinkGrammar_doFinalize
	(JNIEnv *, jclass);

/*
 * Class:     LinkGrammar
 * Method:    numWords
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_org_linkgrammar_LinkGrammar_getNumWords
	(JNIEnv *, jclass);

/*
 * Class:     LinkGrammar
 * Method:    getWord
 * Signature: (I)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_org_linkgrammar_LinkGrammar_getLinkageWord
	(JNIEnv *, jclass, jint);

JNIEXPORT jstring JNICALL Java_org_linkgrammar_LinkGrammar_getLinkageDisjunct
	(JNIEnv *, jclass, jint);

JNIEXPORT jstring JNICALL Java_org_linkgrammar_LinkGrammar_getLinkageSense
   (JNIEnv *, jclass, jint, jint);

JNIEXPORT jdouble JNICALL Java_org_linkgrammar_LinkGrammar_getLinkageSenseScore
   (JNIEnv *, jclass, jint, jint);

JNIEXPORT jstring JNICALL
Java_org_linkgrammar_LinkGrammar_getVersion(JNIEnv *, jclass);

JNIEXPORT jstring JNICALL
Java_org_linkgrammar_LinkGrammar_getDictVersion(JNIEnv *, jclass);

/*
 * Class:     LinkGrammar
 * Method:    numSkippedWords
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_org_linkgrammar_LinkGrammar_getNumSkippedWords
	(JNIEnv *, jclass);

/*
 * Class:     LinkGrammar
 * Method:    numLinkages
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_org_linkgrammar_LinkGrammar_getNumLinkages
	(JNIEnv *, jclass);

/*
 * Class:     LinkGrammar
 * Method:    makeLinkage
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_org_linkgrammar_LinkGrammar_makeLinkage
	(JNIEnv *, jclass, jint);

/*
 * Class:     LinkGrammar
 * Method:    linkageNumViolations
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_org_linkgrammar_LinkGrammar_getLinkageNumViolations
	(JNIEnv *, jclass);

/*
 * Class:     LinkGrammar
 * Method:    linkageDisjunctCost
 * Signature: ()I
 */
JNIEXPORT jdouble JNICALL Java_org_linkgrammar_LinkGrammar_getLinkageDisjunctCost
	(JNIEnv *, jclass);

/*
 * Class:     LinkGrammar
 * Method:    linkageLinkCost
 * Signature: ()I
 */
JNIEXPORT jdouble JNICALL Java_org_linkgrammar_LinkGrammar_getLinkageLinkCost
	(JNIEnv *, jclass);

/*
 * Class:     LinkGrammar
 * Method:    numLinks
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_org_linkgrammar_LinkGrammar_getNumLinks
	(JNIEnv *, jclass);

/*
 * Class:     LinkGrammar
 * Method:    getLinkLWord
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_org_linkgrammar_LinkGrammar_getLinkLWord
	(JNIEnv *, jclass, jint);

/*
 * Class:     LinkGrammar
 * Method:    getLinkRWord
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_org_linkgrammar_LinkGrammar_getLinkRWord
	(JNIEnv *, jclass, jint);

/*
 * Class:     LinkGrammar
 * Method:    getLinkLLabel
 * Signature: (I)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_org_linkgrammar_LinkGrammar_getLinkLLabel
	(JNIEnv *, jclass, jint);

/*
 * Class:     LinkGrammar
 * Method:    getLinkRLabel
 * Signature: (I)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_org_linkgrammar_LinkGrammar_getLinkRLabel
	(JNIEnv *, jclass, jint);

/*
 * Class:     LinkGrammar
 * Method:    getLinkLabel
 * Signature: (I)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_org_linkgrammar_LinkGrammar_getLinkLabel
	(JNIEnv *, jclass, jint);

/*
 * Class:     LinkGrammar
 * Method:    constituentString
 * Signature: ()Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_org_linkgrammar_LinkGrammar_getConstituentString
	(JNIEnv *, jclass);

/*
 * Class:     LinkGrammar
 * Method:    getLinkString
 * Signature: ()Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_org_linkgrammar_LinkGrammar_getLinkString
	(JNIEnv *, jclass);


#ifdef __cplusplus
}
#endif

#endif /*_LinkGrammar_H_ */
