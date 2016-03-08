#!/usr/bin/ruby
require 'securerandom'

def create_random_friendship(size)
	friendships = ""
	while size != 0 do
		f1 = "RandFriend-" + SecureRandom.uuid;
		f2 = "RandFriend-" + SecureRandom.uuid;
		eval = "(EvaluationLink (PredicateNode \"friends\") (ListLink (ConceptNode \"#{f1}\")(ConceptNode \"#{f2}\")))"
		friendships = friendships + "\n" + eval
		size-=1
	end

	return friendships
end

def write_to_file(fname,str)
	File.open(fname, 'w') { |file| file.write(str) }
end

if __FILE__ == $0
	puts ARGV.to_s
	if ARGV.length < 2
		printf "usage %s <filename> <size> \n",$0
		exit      
	end

	printf "Writing %d random friendship relations to %s \n", ARGV[1].to_i, ARGV[0]
	write_to_file(ARGV[0],create_random_friendship(ARGV[1].to_i))
	printf "Done\n"
end
