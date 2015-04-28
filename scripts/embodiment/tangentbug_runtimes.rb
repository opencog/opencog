#!/usr/bin/env ruby

# Sorry, this script is not currently cross-platform.

# I run this with
# ./tangentbug_runtimes.rb build/src/Spacial/tangentbug | tee runtimes.log
program = ARGV[0] || raise(ArgumentError, "Pass executable to test as first arg.")
for n in 0...5_000
  output=`bash -c "(time #{program} #{n} &>/dev/null) 2>&1"`.split "\n"
  usertime=output[2].gsub(/user\s+0m/, "").to_f
  systime=output[3].gsub(/sys\s+0m/, "").to_f
  break if $?.to_s == "2" # TangentBug was killed by interrupt

  time = usertime + systime
  puts "seed #{n}: exited with status #{$?.exitstatus}"
  printf("seed #{n}: took %0.3f seconds.\n", time)
  $stdout.flush
end
