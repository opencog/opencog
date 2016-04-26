#!/usr/bin/ruby
require 'securerandom'
require 'gsl'

module RandData
  def create_random_friendship size
    friendships = ""
    while size != 0 do
      f1 = "RandFriend-" + SecureRandom.uuid;
      f2 = "RandFriend-" + SecureRandom.uuid;
      eval = "(EvaluationLink (stv 0.4 1.0) (PredicateNode \"friends\")
            (ListLink (ConceptNode \"#{f1}\")(ConceptNode \"#{f2}\")))"
    friendships = friendships + "\n" + eval
    size-=1
    end
    return friendships
  end

  def create_random_eval_links size, n = 5
      #Lets create a SIZE number smokers and fried em to N individuals		
      scm_str = "" 
      rand_gen = GSL::Rng.alloc(1) #default generator is mt19973
      i = 0
      j = 0
      
      while i < size do
        smoker ='(ConceptNode "Smoker-' + SecureRandom.uuid+'")'
        smokes = sprintf "(EvaluationLink (stv %s 0.8)\n\t(PredicateNode \"smokes\")\n\t\t(ListLink\n\t\t\t%s))", 0.2 , smoker
        #has_cancer = sprintf "(EvaluationLink (stv %s 0.8)\n\t(PredicateNode \"cancer\")\n\t\t(ListLink\n\t\t\t%s))", 0.1 , smoker
        scm_str = scm_str + smokes + "\n" # + has_cancer + "\n"

        while j < n do
          non_smoker = '(ConceptNode "NonSmoker-' + SecureRandom.uuid+'")'
          friends = sprintf "(EvaluationLink (stv 0.85 0.8)\n\t(PredicateNode \"friends\")\n\t\t(ListLink\n\t\t\t%s\n\t\t\t%s))", 
            #rand_gen.uniform.round(2).to_s, smoker, non_smoker
            smoker, non_smoker
          scm_str = scm_str + "\n" + friends+ "\n"
          j = j + 1
        end
        i = i + 1
      end
      scm_str
  end
end

  def write_to_file(fname,str)
    File.open(fname, 'w') { |file| file.write(str) }
  end

  include  RandData

  if __FILE__ == $0
    puts ARGV.to_s
    if ARGV.length < 2
      printf "usage %s <filename> <size> \n",$0
      exit      
    end

    printf "Writing %d random friendship relations to %s \n", ARGV[1].to_i, ARGV[0]
    #	write_to_file(ARGV[0],RandData::create_random_friendship(ARGV[1].to_i))
    write_to_file(ARGV[1]+"-smokers-"+(ARGV[1].to_i*5).to_s+"-friendship-"+ARGV[0],RandData::create_random_eval_links(ARGV[1].to_i))
    printf "Done\n"
  end
