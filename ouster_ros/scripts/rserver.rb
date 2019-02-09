#!/usr/bin/env ruby

require 'socket'

#host = '192.168.2.29'
# Some improvements  
# IP address / hostname
# IMU Port
# Lidar Port
# Timestamp referencing


host = '127.0.0.1'
port = 7503
s = UDPSocket.new
#s.bind(nil,port)
fp = File.open(ARGV[0],"r")
prev_port = nil

alter_timestamp = true


fp.each_line { |line|
  #byebug
  next if line =~ /^\#/
  next if line =~ /^\s*$/
  port,data = line.chop.split(",")
  if port.nil? || port == ""
    port = prev_port
  else
    prev_port = port
  end
  #byebug
  # First change the time stamp here
  if alter_timestamp && port == "7503"
    tdata = [data].pack("H*").unpack("C*")
    a = Time.now
    tmp = a.to_i * 1_000_000_000   + a.nsec
    tdata[16..16+7] = [tmp].pack("Q").unpack("C*")
    data = tdata.pack("C*").unpack("H*")[0]
  elsif alter_timestamp && port == "7502"
    #byebug
    tdata = [data].pack("H*").unpack("C*")
    a = Time.now
    tmp = a.to_i * 1_000_000_000   + a.nsec
    tdata[0..0+7] = [tmp].pack("Q").unpack("C*")
    data = tdata.pack("C*").unpack("H*")[0]
  else
    puts "PROBLEM"
  end

  s.send([data].pack("H*"),0,host,port)
  #sleep(0.05)
}
