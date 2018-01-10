#! /usr/bin/env perl
#
# Example usage of the low-level link-grammar perl bindings
# You should not really use this directly; a higher-level wrapper
# would be far more appropriate.
#
# Copyright (c) 2014, Linas Vepstas

use clinkgrammar;

my $ver = clinkgrammar::linkgrammar_get_version();

print "Version $ver\n";

sub prtdiag
{
    my $txt = $_[0];
    my $dict = $_[1];
    my $po = clinkgrammar::parse_options_create();

    my $sent = clinkgrammar::sentence_create($txt, $dict);
    my $num_parses = clinkgrammar::sentence_parse($sent, $po);
    $num_parses = clinkgrammar::sentence_num_valid_linkages($sent);
    print "Found $num_parses valid parses for \"$txt\":\n";

    for (my $i=0; $i<$num_parses; $i++) {
        my $linkage = clinkgrammar::linkage_create($i, $sent, $po);
        my $diagram = clinkgrammar::linkage_print_diagram($linkage);
        print "Parse $i:\n$diagram";
    }
}

# English, Russian and Turkish dictionaries
my $en_dict = clinkgrammar::dictionary_create_lang("en");
prtdiag("This is a test", $en_dict);

my $ru_dict = clinkgrammar::dictionary_create_lang("ru");
prtdiag("это большой тест", $ru_dict);

my $tr_dict = clinkgrammar::dictionary_create_lang("tr");
prtdiag("adam ve kadın geldi", $tr_dict);

