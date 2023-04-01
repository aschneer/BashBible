# Sending/Receiving TCP/UDP Messages in Ubuntu

The server listens for messages, and the client sends messages.

The server must be started before the client, otherwise the client will not connect.

## References

Good easy guide for simple TCP chat server. \
[How to send and receive messages with NC in Linux?](https://linuxhint.com/send-receive-messages-nc-linux/)

I didn't have to make any firewall changes to get this to work, so don't mess with `ufw`. \
`ufw` can mess up existing connections like `ssh`. \
[Ubuntu | Security - Firewall](https://ubuntu.com/server/docs/security-firewall)

[How To Use Netcat to Establish and Test TCP and UDP Connections](https://www.digitalocean.com/community/tutorials/how-to-use-netcat-to-establish-and-test-tcp-and-udp-connections)

[Netcat (nc) Command with Examples](https://linuxize.com/post/netcat-nc-command-with-examples/)

[How to check if port is in use on Linux or Unix](https://www.cyberciti.biz/faq/unix-linux-check-if-port-is-in-use-command/)

IANA port registration list: \
[Service Name and Transport Protocol Port Number Registry](https://www.iana.org/assignments/service-names-port-numbers/service-names-port-numbers.xhtml)

Wikipedia port registration list: \
[List of TCP and UDP port numbers](https://en.wikipedia.org/wiki/List_of_TCP_and_UDP_port_numbers)

## Bash Commands

### TCP

```bash
# Server (IP = 1.2.3.4):
# Listen for TCP messages on port 5169 (machine IP = 1.2.3.4)
netcat -l -p 5169
netcat -lt -p 5169  # Equivalent
nc -l -p 5169  # Equivalent
nc -lt -p 5169  # Equivalent

# Client (any IP on network):
# Send TCP messages to port 5169 of machine 1.2.3.4 (from any IP on same network)
netcat 1.2.3.4 5169
netcat -t 1.2.3.4 5169  # Equivalent
nc 1.2.3.4 5169  # Equivalent
nc -t 1.2.3.4 5169  # Equivalent
```

### UDP

```bash
# Server (IP = 1.2.3.4):
# Listen for UDP messages on port 5170 (machine IP = 1.2.3.4)
netcat -lu -p 5170
nc -lu -p 5170  # Equivalent

# Client (any IP on network):
# Send UDP messages to port 5170 of machine 1.2.3.4 (from any IP on same network)
netcat -u 1.2.3.4 5169
nc -u 1.2.3.4 5169  # Equivalent
```
