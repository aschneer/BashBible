# Networking

```bash
# See hostname and all IP addresses associated
# with the client.
hostname
hostname -I
```

```bash
# Get IP address and network info
ifconfig # (old, deprecated)
# or (following are all the same)
ip address
ip addr
ip add
ip a
```

```bash
# Show current default gateway.
ip route
ip route show # (equivalent)
```

```bash
# List all nodes on the network and their info.
nmap 10.133.0.*
	# Lists all nodes within this subnet.
```

```bash
# Set network configuration in Ubuntu.
# This includes setting static IP, gateway, netmask, etc.
# for each network adapter.
sudo vim /etc/network/interfaces

# Add any of the following lines under
# the relevant adapter(s) as needed.
address 192.168.1.58 # static client IP
netmask 255.255.255.0 # static netmask
gateway 192.168.1.1 # static gateway

# Apply changes.
sudo systemctl restart networking
```

```bash
# Restart networking service to
# apply any changes made to
# network configurations.
sudo systemctl restart networking
```

```bash
# Check internet connection.
ping 1.1.1.1 # (Cloudflare DNS server)
```

```bash
# Check DNS status (which DNS is active).
systemd-resolve --status
	# Active DNS IP should appear at the
	# top of the output if one is active.
```

```bash
# Set the DNS servers in Ubuntu.
sudo vim /etc/systemd/resolved.conf
	# Under section "[Resolve]"...
	# Edit the following lines:
		DNS=1.1.1.2
		FallbackDNS=1.0.0.2

# Apply DNS changes.
service systemd-resolved restart
```
