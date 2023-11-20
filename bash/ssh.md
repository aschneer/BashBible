# SSH

Make sure `ssh-agent` starts automatically and keys persist across reboots.
```bash
# Add these lines to ~/.bashrc
eval "$(ssh-agent -s)"
ssh-add -k ~/.ssh/*

# Check keys added to ssh-agent
ssh-add -l
ssh-add -L
```

SSH into a host machine:
```bash
# -i = pass in a private key (identity) file.
# IP = ip of the host machine
ssh -i ~/.ssh/id_rsa username@192.168.1.45
# Specify port
ssh <user>@<hostname> -p 8090
```

Kill a frozen SSH terminal:
```bash
# Key sequence = Enter, shift + ~, period
```

Copy file from remote to local machine desktop folder:
```bash
scp username@192.168.1.45:~/files/test.txt ./Desktop
```

Run bash commands on remote machine over SSH:
```bash
ssh username@192.168.1.45 "ls -la ~/files"
ssh username@192.168.1.45 "ls -la ~/files; cd /; ifconfig"	# Multiple commands.
```

Adding SSH keys:
```bash
# Start SSH agent. This wipes all existing keys from the agent.
eval "$(ssh-agent -s)"
# Add the key.
ssh-add ~/.ssh/id_ed25519
# List existing keys registered with the agent.
ssh-add -l
```

Generate Public SSH Key from Private Key:
```bash
ssh-keygen -y -f ~/.ssh/id_rsa > ~/.ssh/id_rsa.pub
```