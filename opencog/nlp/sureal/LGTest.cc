#include <locale.h>
#include <langinfo.h>
#include <string.h>
#include <string>
#include <sstream>
#include <link-grammar/dict-api.h>

using namespace std;

std::string lg_exp_to_scm_string(Exp* exp)
{
    if (CONNECTOR_type == exp->type)
    {
        stringstream ss;
        ss << "(LgConnector (LgConnectorNode ";
        ss << "\"" << exp->u.string << "\")";
        ss << "(LgConnDirNode \"" << exp->dir << "\")";
        if (exp->multi) ss << "(LgConnMultiNode \"@\")";
        ss << ")\n";
        return ss.str();
    }

    // Whenever a null appears in an OR-list, it means the
    // entire OR-list is optional.  A null can never appear
    // in an AND-list.
    E_list* el = exp->u.l;
    if (NULL == el)
        return "(LgConnector (LgConnectorNode \"0\"))\n";

    // The C data structure that link-grammar uses for connector
    // expressions is totally insane, as witnessed by the loop below.
    // Anyway: operators are infixed, i.e. are always binary,
    // with exp->u.l->e being the left hand side, and
    //      exp->u.l->next->e being the right hand side.
    // This means that exp->u.l->next->next is always null.
    std::string alist;

    if (AND_type == exp->type)
        alist = "(LgAnd ";

    if (OR_type == exp->type)
        alist = "(LgOr ";

    alist += lg_exp_to_scm_string(el->e);
    el = el->next;

    while (el && exp->type == el->e->type)
    {
        el = el->e->u.l;
        alist += lg_exp_to_scm_string(el->e);
        el = el->next;
    }

    if (el)
        alist += lg_exp_to_scm_string(el->e);

    alist.append (")\n");
    return alist;
}

int main ()
{
    // Get the locale from the environment...
    setlocale(LC_ALL, "");

    /* Check to make sure the current locale is UTF8; if its not,
     * then force-set this to the english utf8 locale */
    const char* codeset = nl_langinfo(CODESET);
    if (!strstr(codeset, "UTF") && !strstr(codeset, "utf"))
    {
        fprintf(stderr, "Warning: locale %s was not UTF-8; force-setting to en_US.UTF-8\n", codeset);
        setlocale(LC_CTYPE, "en_US.UTF-8");
    }

    Dictionary dict = dictionary_create_default_lang();

    std::string word = "dog";
    Dict_node* dn_head = dictionary_lookup_list(dict, word.c_str());

    print_expression(dn_head->exp);

    std::string set = "(SetLink\n";

    for (Dict_node* dn = dn_head; dn; dn = dn->right)
    {
        Exp* exp = dn->exp;

        // First atom at the front of the outgoing set is the word itself.
        // Second atom is the first disjuct that must be fulfilled.
        std::string word_cset = "  (LgWordCset (WordNode \"";
        word_cset += word;
        word_cset += "\")\n";
        word_cset += lg_exp_to_scm_string(exp);
        word_cset += "  )\n";

        set += word_cset;
    }

    free_lookup_list(dict, dn_head);

    set += ")\n";

    //printf("%s", set.c_str());

    dictionary_delete(dict);


    return 0;
}


