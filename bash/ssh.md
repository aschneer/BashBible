# SSH

SSH into a host machine:
```bash
# -i = pass in a private key (identity) file.
# IP = ip of the host machine
ssh -i ~/.ssh/id_rsa username@192.168.1.45
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

