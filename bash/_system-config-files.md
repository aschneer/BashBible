# _System Config File Locations

## Networking

```bash
# Network interface configuration
# (static IP, default gateway, subnet mask)
/etc/network/interfaces
# DNS server configuration.
/etc/systemd/resolved.conf
```

## SSH

```bash
# ssh server-side config file
/etc/ssh/sshd_config.d
# System-wide ssh config file, overrides
# "config" file in .ssh folder in ~/ directory.
/etc/ssh/ssh_config
```

## File System

```bash
# File system table - specify disk mount locations and instructions
/etc/fstab
# RAID config file
/etc/mdadm/mdadm.conf
```