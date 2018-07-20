#!/bin/bash

# we assume mfs has run here already

if [ -e "/config/network.cfg" ]
then
  NETWORK=$(cat /config/network.cfg | grep -v "^#" | sed -rn 's/NETWORK=(\w*)\s*$/\1/p')
  if [ "$NETWORK" == "STATIC" ]
  then
    STATICIP=$(cat /config/network.cfg | grep -v "^#" | sed -rn 's/STATIC_IP=([^\s]*)\s*$/\1/p')
    GATEWAY=$(cat /config/network.cfg | grep -v "^#" | sed -rn 's/GATEWAY=([^\s]*)\s*$/\1/p')
  fi
else
  NETWORK=DHCP
fi

if [ "$NETWORK" = "DHCP" ]
then
  dhclient eth0
  if [ $? -eq 0 ]
  then
    echo "Successfully completed DHCP"
  else
    echo "DHCP failed"
    sleep 300
    reboot
  fi
fi

if [ "$NETWORK" = "STATIC" ]
then
  ip addr add $STATICIP dev eth0
  ip link set up eth0
  ip route add default via $GATEWAY dev eth0
fi

# Set up NAT for outgoing TCP connections
echo 2 > /proc/sys/net/ipv6/conf/eth0/accept_ra
echo 1 > /proc/sys/net/ipv6/conf/all/forwarding
ip6tables -t nat -A POSTROUTING -o eth0 -s fdde:ad00:beef:0::/64 -j MASQUERADE
# For some reason, this additional rule is needed to make masquerading work
ip6tables -t nat -A PREROUTING -d 2001:470:4a71:f170::5/64 -i eth0 -j DNAT --to-destination fdde:ad00:beef::91f5:6dd4:e66f:cf5b

systemctl stop ntp
ntpd -gq
systemctl start ntp
