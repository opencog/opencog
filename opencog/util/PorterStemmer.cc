#include "PorterStemmer.h"
#include <stdio.h>
#include <string.h>

using namespace opencog;

#define TRUE 1
#define FALSE 0

std::string PorterStemmer::getStem(std::string word)
{
    struct stemmer stemmer_z;

    if (word.length() <= 1)
        return word; /*-DEPARTURE-*/

    stemmer_z.b = new char[word.length()];
    strncpy(stemmer_z.b, word.data(), word.length());
    stemmer_z.k = word.length() - 1; /* copy the parameters into z */

    /* With this line, strings of length 1 or 2 don't go through the
       stemming process, although no mention is made of this in the
       published algorithm. Remove the line to match the published
       algorithm. */

    step1ab(&stemmer_z); step1c(&stemmer_z); step2(&stemmer_z); step3(&stemmer_z); step4(&stemmer_z); step5(&stemmer_z);

    stemmer_z.b[stemmer_z.k+1] = '\0';
    std::string stemmedWord(stemmer_z.b);
    delete [] stemmer_z.b;

    return stemmedWord;
}

int PorterStemmer::cons(struct stemmer * z, int i)
{
    switch (z->b[i]) {
    case 'a': case 'e': case 'i': case 'o': case 'u': return FALSE;
    case 'y': return (i == 0) ? TRUE : !cons(z, i - 1);
    default: return TRUE;
    }
}

int PorterStemmer::m(struct stemmer * z)
{
    int n = 0;
    int i = 0;
    int j = z->j;
    while (TRUE) {
        if (i > j) return n;
        if (! cons(z, i)) break; i++;
    }
    i++;
    while (TRUE) {
        while (TRUE) {
            if (i > j) return n;
            if (cons(z, i)) break;
            i++;
        }
        i++;
        n++;
        while (TRUE) {
            if (i > j) return n;
            if (! cons(z, i)) break;
            i++;
        }
        i++;
    }
}

int PorterStemmer::vowelinstem(struct stemmer * z)
{
    int j = z->j;
    int i; for (i = 0; i <= j; i++) if (! cons(z, i)) return TRUE;
    return FALSE;
}

int PorterStemmer::doublec(struct stemmer * z, int j)
{
    char * b = z->b;
    if (j < 1) return FALSE;
    if (b[j] != b[j - 1]) return FALSE;
    return cons(z, j);
}

int PorterStemmer::cvc(struct stemmer * z, int i)
{
    if (i < 2 || !cons(z, i) || cons(z, i - 1) || !cons(z, i - 2)) return FALSE;
    {
        int ch = z->b[i];
        if (ch  == 'w' || ch == 'x' || ch == 'y') return FALSE;
    }
    return TRUE;
}

int PorterStemmer::ends(struct stemmer * z, const char * s)
{
    int length = s[0];
    char * b = z->b;
    int k = z->k;
    if (s[length] != b[k]) return FALSE; /* tiny speed-up */
    if (length > k + 1) return FALSE;
    if (memcmp(b + k - length + 1, s + 1, length) != 0) return FALSE;
    z->j = k - length;
    return TRUE;
}

void PorterStemmer::setto(struct stemmer * z, const char * s)
{
    int length = s[0];
    int j = z->j;
    memmove(z->b + j + 1, s + 1, length);
    z->k = j + length;
}

void PorterStemmer::r(struct stemmer * z, const char * s)
{
    if (m(z) > 0) setto(z, s);
}

void PorterStemmer::step1ab(struct stemmer * z)
{
    char * b = z->b;
    if (b[z->k] == 's') {
        if (ends(z, "\04" "sses")) z->k -= 2; else
                if (ends(z, "\03" "ies")) setto(z, "\01" "i"); else
                if (b[z->k - 1] != 's') z->k--;
    }
    if (ends(z, "\03" "eed")) {
        if (m(z) > 0) z->k--;
    } else
        if ((ends(z, "\02" "ed") || ends(z, "\03" "ing")) && vowelinstem(z)) {
            z->k = z->j;
            if (ends(z, "\02" "at")) setto(z, "\03" "ate"); else
                    if (ends(z, "\02" "bl")) setto(z, "\03" "ble"); else
                            if (ends(z, "\02" "iz")) setto(z, "\03" "ize"); else
                        if (doublec(z, z->k)) {
                            z->k--;
                            {
                                int ch = b[z->k];
                                if (ch == 'l' || ch == 's' || ch == 'z') z->k++;
                            }
                        } else if (m(z) == 1 && cvc(z, z->k)) setto(z, "\01" "e");
        }
}

void PorterStemmer::step1c(struct stemmer * z)
{
    if (ends(z, "\01" "y") && vowelinstem(z)) z->b[z->k] = 'i';
}


void PorterStemmer::step2(struct stemmer * z)
{
    switch (z->b[z->k-1]) {
    case 'a': if (ends(z, "\07" "ational")) {
            r(z, "\03" "ate"); break;
        }
        if (ends(z, "\06" "tional")) {
            r(z, "\04" "tion"); break;
        }
        break;
    case 'c': if (ends(z, "\04" "enci")) {
            r(z, "\04" "ence"); break;
        }
        if (ends(z, "\04" "anci")) {
            r(z, "\04" "ance"); break;
        }
        break;
    case 'e': if (ends(z, "\04" "izer")) {
            r(z, "\03" "ize"); break;
        }
        break;
    case 'l': if (ends(z, "\03" "bli")) {
            r(z, "\03" "ble"); break;
        } /*-DEPARTURE-*/

        /* To match the published algorithm, replace this line with
        case 'l': if (ends(z, "\04" "abli")) { r(z, "\04" "able"); break; } */

        if (ends(z, "\04" "alli")) {
            r(z, "\02" "al"); break;
        }
        if (ends(z, "\05" "entli")) {
            r(z, "\03" "ent"); break;
        }
        if (ends(z, "\03" "eli")) {
            r(z, "\01" "e"); break;
        }
        if (ends(z, "\05" "ousli")) {
            r(z, "\03" "ous"); break;
        }
        break;
    case 'o': if (ends(z, "\07" "ization")) {
            r(z, "\03" "ize"); break;
        }
        if (ends(z, "\05" "ation")) {
            r(z, "\03" "ate"); break;
        }
        if (ends(z, "\04" "ator")) {
            r(z, "\03" "ate"); break;
        }
        break;
    case 's': if (ends(z, "\05" "alism")) {
            r(z, "\02" "al"); break;
        }
        if (ends(z, "\07" "iveness")) {
            r(z, "\03" "ive"); break;
        }
        if (ends(z, "\07" "fulness")) {
            r(z, "\03" "ful"); break;
        }
        if (ends(z, "\07" "ousness")) {
            r(z, "\03" "ous"); break;
        }
        break;
    case 't': if (ends(z, "\05" "aliti")) {
            r(z, "\02" "al"); break;
        }
        if (ends(z, "\05" "iviti")) {
            r(z, "\03" "ive"); break;
        }
        if (ends(z, "\06" "biliti")) {
            r(z, "\03" "ble"); break;
        }
        break;
    case 'g': if (ends(z, "\04" "logi")) {
            r(z, "\03" "log"); break;
        } /*-DEPARTURE-*/

        /* To match the published algorithm, delete this line */
    }
}

void PorterStemmer::step3(struct stemmer * z)
{
    switch (z->b[z->k]) {
    case 'e': if (ends(z, "\05" "icate")) {
            r(z, "\02" "ic"); break;
        }
        if (ends(z, "\05" "ative")) {
            r(z, "\00" ""); break;
        }
        if (ends(z, "\05" "alize")) {
            r(z, "\02" "al"); break;
        }
        break;
    case 'i': if (ends(z, "\05" "iciti")) {
            r(z, "\02" "ic"); break;
        }
        break;
    case 'l': if (ends(z, "\04" "ical")) {
            r(z, "\02" "ic"); break;
        }
        if (ends(z, "\03" "ful")) {
            r(z, "\00" ""); break;
        }
        break;
    case 's': if (ends(z, "\04" "ness")) {
            r(z, "\00" ""); break;
        }
        break;
    }
}

/* step4(z) takes off -ant, -ence etc., in context <c>vcvc<v>. */

void PorterStemmer::step4(struct stemmer * z)
{
    switch (z->b[z->k-1]) {
    case 'a': if (ends(z, "\02" "al")) break; return;
    case 'c': if (ends(z, "\04" "ance")) break;
        if (ends(z, "\04" "ence")) break; return;
    case 'e': if (ends(z, "\02" "er")) break; return;
    case 'i': if (ends(z, "\02" "ic")) break; return;
    case 'l': if (ends(z, "\04" "able")) break;
        if (ends(z, "\04" "ible")) break; return;
    case 'n': if (ends(z, "\03" "ant")) break;
        if (ends(z, "\05" "ement")) break;
        if (ends(z, "\04" "ment")) break;
        if (ends(z, "\03" "ent")) break; return;
    case 'o': if (ends(z, "\03" "ion") && (z->b[z->j] == 's' || z->b[z->j] == 't')) break;
        if (ends(z, "\02" "ou")) break; return;
        /* takes care of -ous */
    case 's': if (ends(z, "\03" "ism")) break; return;
    case 't': if (ends(z, "\03" "ate")) break;
        if (ends(z, "\03" "iti")) break; return;
    case 'u': if (ends(z, "\03" "ous")) break; return;
    case 'v': if (ends(z, "\03" "ive")) break; return;
    case 'z': if (ends(z, "\03" "ize")) break; return;
    default: return;
    }
    if (m(z) > 1) z->k = z->j;
}

void PorterStemmer::step5(struct stemmer * z)
{
    char * b = z->b;
    z->j = z->k;
    if (b[z->k] == 'e') {
        int a = m(z);
        if (a > 1 || (a == 1 && !cvc(z, z->k - 1))) z->k--;
    }
    if (b[z->k] == 'l' && doublec(z, z->k) && m(z) > 1) z->k--;
}
