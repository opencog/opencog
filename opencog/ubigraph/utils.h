
/** Converts a character to upper case (also some Scandinavian characters) 
 *  WTF?? XXX same as standard C99 toupper ... but different !? 
 * */
char opencog::Isox(char s)
{
    if (s > 96 && s < 123) s -= 32;
    return s;
}

