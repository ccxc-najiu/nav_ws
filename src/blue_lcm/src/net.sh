export LCM_DEFAULT_URL="udpm://224.0.0.1:7667?ttl=1"
ifconfig wlp0s20f3 multicast
route add -net 224.0.0.0 netmask 240.0.0.0 dev wlp0s20f3
