#!/usr/bin/perl

#in the indented format any line that has a # is considered a comment

$stuff=$ARGV[0];
open STUFF, $stuff or die "Could not open $stuff\n";
@lines = <STUFF>;
close STUFF;

$previous_ntabs = 0;
$current_ntabs = 0;
#TODO translate from inh to InheritanceLink etc
$link_types = "_ForAll_And_And_InheritanceLink_inh_SimultaneousAnd_SimAnd_EvaluationLink_eval_ImplicationLink_imp_ListLink_list_SatisfyingSet_SatSet_PredImp_PredictiveImplication_SeqAnd_SimulAnd_";
$node_types = "_ConceptNode_conc_PredicateNode_pred_VariableNode_var_WordNode_word_FreeVariableNode_";

@pile;

$line_number = 1; 
print "<ListLink>\n\n";



foreach $line (@lines) {
    
    $line_number++;

    
    chomp $line;   
    
    if(!($line =~ /#/)) {
        
        #remove extra tabs from end of line
        while($line =~ /\t$/) {
            $line =~ s/\t$//;
        }
        
        #TODO I assume there are NO TABS in the middle of a line! 
        #number of tabs in the begining of line
        $current_ntabs = ($line =~ tr/\t//);
        #TODO I assume that truth values are in the form <s,c> with no extra
        #whitespaces and no missing elements

        @words = split(/ /, $line);               
        

        $to_pop = $previous_ntabs - $current_ntabs;

        if($to_pop < -1) {
            die "double tabs on line $line_number\n";
        }
        

        #right now files must end with an "extra" empty line TODO 
        if($to_pop > 0) {
            
            for($i = 0; $i < $to_pop; $i++) { 
            
                $popped = pop @pile;
                $ntabs = ($popped =~ tr/\t//);       
                $popped =~ s/\t//g;
                if($link_types =~ /$popped/) {
                    for($j = 0; $j < $ntabs; $j++) {
                        print "\t";
                    }
                    print "<\/$popped>\n";
                }
            }
        }        
    

        $word = $words[0];
        $word =~ s/\t//g;
                    
        if($link_types =~ $word) {
            push (@pile, $words[0]);

        }
        
        $ntabs = ($line =~ tr/\t//);       
        for($i = 0; $i < $ntabs; $i++) {
            print "\t";
        }
    
        $qualifiers="";
        $nqualifiers = @words-1; 
        if($nqualifiers > 0) {
            if($link_types =~ $word) {
                #qualifier is a truth value
                $words[1] =~ s/<//;
                $words[1] =~ s/>//;
                @tvs = split (/,/, $words[1]);
                $qualifiers = " strength=\"$tvs[0]\" confidence=\"$tvs[1]\"";
            }    
            if($node_types =~ $word) {
                #first qualifier is the node name; second, optional qualifier is tv
                $qualifiers = " name=\"$words[1]\"";
                if($nqualifiers == 2) {
                    $words[2] =~ s/<//;
                    $words[2] =~ s/>//;
                    @tvs = split (/,/, $words[2]);
                    $qualifiers = " strength=\"$tvs[0]\" confidence=\"$tvs[1]\"";
                }
            }
        }
        
        if($word ne "") {
            if($link_types =~ $word) {print "<$word$qualifiers>\n";}
            if($node_types =~ $word) {print "<Element$qualifiers class=\"$word\"\/>\n";} 
        }
        else {print "\n";}

        $previous_ntabs = $current_ntabs;
    }
}

print "</ListLink>\n";   

