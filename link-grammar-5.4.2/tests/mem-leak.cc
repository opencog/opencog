
#include <locale.h>
#include <stdio.h>
#include "link-grammar/link-includes.h"

int main()
{
    Dictionary    dict;
    Parse_Options opts;
    Sentence      sent;
    Linkage       linkage;
    char *        diagram;
    int           i, num_linkages;
    const char *  input_string[] = {"Perhaps it is and perhaps it isnt."};

    setlocale(LC_ALL, "en_US.UTF-8");
    opts = parse_options_create();
    parse_options_set_max_null_count(opts, 10);
    parse_options_set_display_morphology(opts, 1);
    parse_options_set_spell_guess(opts, 0);

    dictionary_set_data_dir(DICTIONARY_DIR "/data");
    dict = dictionary_create_lang("en");
    if (!dict) {
        printf ("Fatal error: Unable to open the dictionary\n");
        return 1;
    }
    int qq = 0;
    while (++qq < 100)
    for (i=0; i<1; ++i) {
        sent = sentence_create(input_string[i], dict);
        sentence_split(sent, opts);
        num_linkages = sentence_parse(sent, opts);
        if (num_linkages > 0) {
            linkage = linkage_create(0, sent, opts);
            diagram = linkage_print_diagram(linkage, true, 800);
            linkage_free_diagram(diagram);
            linkage_delete(linkage);
        }
        sentence_delete(sent);
    }

    dictionary_delete(dict);
    parse_options_delete(opts);
    return 0;
}
