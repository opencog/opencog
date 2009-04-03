#!/usr/bin/env ruby

# When you have a log created by tangentbug_runtimes.rb called tangentbug.log,
# run this program:
# ./tangentbug_analyze_runtimes.rb < tangentbug.log

frequency = Hash.new {|h,k| h[k] = 0}
times = Hash.new {|h,k| h[k] = []}

input = $stdin
while line = input.gets
  next unless line =~ /^seed (\d+):.*status (\d+)$/
  #raise unless line =~ /^seed (\d+):.*status (\d+)$/
  seed = $1
  exit_status = $2
  times_info = input.gets
  time = times_info.split(/\s+/)[3].to_f
  frequency[exit_status] += 1
  times[exit_status] << time
end

total = 0; frequency.each {|status,num| total += num}
frequency.sort.each do |status, number|
  percent = 100.0 * number / total
  printf("%2.2f percent exited with status %d\n", percent, status)
end

puts
times.sort.each do |status, runtimes|
  avg = runtimes.inject(0) {|time, mem| mem + time} / runtimes.length
  printf("exit status #{status} had average runtime %.3f\n", avg)
end
