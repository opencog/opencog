/*************************************************************************/
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

package org.linkgrammar;

/**
 * This class serves as a wrapper to the C Link Grammar Parser library.
 * It provides a simple public Java API to the equivalent public C API.
 *
 * Unfortunately, this class is not at all OOP in style; it operates on
 * the single, current sentence and linkage.  This could be improved.
 */

public class LinkGrammar
{
    static
    {
        // On a Linux system, the actual name of the library
        // is prefixed with "lib" and suffixed with ".so"
        // -- e.g. "liblink-grammar-java.so"
        // Windows looks for "link-grammar-java.dll"
        // MacOS looks for "liblink-grammar-java.dylib"
        //
        // On a Windows system, we also need to load the prerequisite
        // libraries first. (Linux loaders do this automatically).
        // Actually, I guess Windows does this too, unless the user
        // failed to add the working directory to %PATH
        //
        try
        {
            String osname = System.getProperty("os.name");
            if (osname.indexOf("win") > -1 || osname.indexOf("Win") > -1)
            {
                System.loadLibrary("link-grammar");
            }
            // if (osname.indexOf("Mac OS X") > -1) {}
            System.loadLibrary("link-grammar-java");
        }
        catch (Throwable ex)
        {
            // If we don't catch here, then the thread pool will
            // silently eat the exception and make us into fools.
            // https://www.securecoding.cert.org/confluence/display/java/TPS03-J.+Ensure+that+tasks+executing+in+a+thread+pool+do+not+fail+silently
            System.err.println(ex);
            System.exit(-1);
        }
    }

    //! Get the version string for the parser.
    public static native String getVersion();

    //! Get the version string for the dictionary.
    public static native String getDictVersion();

    // C functions for changing linkparser options
    public static native void setMaxParseSeconds(int maxParseSeconds);

    public static native void setMaxCost(double maxCost);

    public static native void setMaxLinkages(int maxLinkages);
    public static native int getMaxLinkages();

    // Defaults to /usr/local/share/link-grammar/
    public static native void setDictionariesPath(String path);

    public static native void setLanguage(String lang);

    // C functions in the linkparser API
    public static native void init();

    public static native void parse(String sent);

    public static native void close();
    public static native void doFinalize();

    // C sentence access functions
    public static native int getNumWords();

    // Get the subscripted form of the word.
    public static native String getLinkageWord(int i);

    // Get string representing the disjunct actually used.
    public static native String getLinkageDisjunct(int i);

    public static native int getNumSkippedWords();

    // C linkage access functions
    public static native int getNumLinkages();

    public static native void makeLinkage(int index);

    public static native int getLinkageNumViolations();

    public static native double getLinkageDisjunctCost();

    public static native double getLinkageLinkCost();

    public static native int getNumLinks();

    public static native int getLinkLWord(int link);

    public static native int getLinkRWord(int link);

    public static native String getLinkLLabel(int link);

    public static native String getLinkRLabel(int link);

    public static native String getLinkLabel(int link);

    public static native String getConstituentString();

    public static native String getLinkString();

    public static native String getLinkageSense(int word, int sense);

    public static native double getLinkageSenseScore(int word, int sense);
}

