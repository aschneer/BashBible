# File System

## Mounting File Systems

- Usually you don't need to add lines for disks into /etc/fstab
- The OS will detect and mount the drives automatically, or let the user mount them
- You only need to try adding lines to /etc/fstab if the OS doesn't see the drive at all, but it does show up in "lsblk"
- **BE CAREFUL** - if you add a line to /etc/fstab and then change the disk partitioning or structure in any way, the system will not be able to mount it on startup and Linux could fail to boot.
	- In my case, I created a hardware RAID after adding the individual HDD's as separate lines to /etc/fstab, causing Ubuntu not to boot (or boot into emergency mode)
	-   You might see a message "booting into emergency mode" when it fails to boot.
- The solution is to delete any lines you added from /etc/fstab and it should boot again.

## RAID

- **UBUNTU DOES NOT LIKE RAID ARRAYS CREATED BY THE INTEL HARDWARE RAID BOARD**
	- Intel RST (Rapid Storage Technology) - hardware RAID driver
	- This is strange because I booted from a Lubuntu live USB and it saw the hardware RAID just fine
	- But for whatever reason, Ubuntu 22.04 didn't see the original hardware RAID that was there, and I can't get it to see a new one that I reformat and set up with the Intel hardware.
	- Creating the Intel hardware RAID causes corruption in the first HDD's backup GPT table, which throws a warning in "sudo fdisk -l"
	- When trying to add the hardware RAID into /etc/fstab and mounting it, it throws errors "wrong fs type, bad option, bad superblock, missing codepage or helper program, or other error." Or, "can't read superblock on /dev/…"
	- Clearly the hardware RAID board is modifying the first HDD's partition table in a way that Ubuntu doesn't like.
- Rather than use the hardware RAID, better to use `mdadm` to make a Linux software RAID.
- `man mdadm`
- Mdadm cheat sheet | [Mdadm Cheat Sheet](https://www.ducea.com/2009/03/08/mdadm-cheat-sheet/)
- Intel RST in Linux Guide | [Intel® Rapid Storage Technology (Intel® RST) in Linux*](https://www.intel.com/content/dam/www/public/us/en/documents/white-papers/rst-linux-paper.pdf)
	- Contains error in RAID container create command - list of devices must come immediately after "-n 3" option
	- There are numerous errors in the code snippets in this document
- mdadm guide | [How to Set Up RAID in Linux](https://www.maketecheasier.com/set-up-raid-linux/)
- RAID comparison table | [RAID level comparison table](https://www.qnap.com/en-us/how-to/faq/article/raid-level-comparison-table)

## Create RAID Array Using 'mdadm'

### 1. Create RAID Container

### 2. Create RAID Volume

### 3. Create 'mdadm.conf' File

```bash
# By default, root doesn't have a password in Ubuntu. Set one as follows.
sudo passwd root
# Must be root for this command - sudo is insufficient
su
mdadm --detail --scan >> /etc/mdadm/mdadm.conf

# Remove this line if it is in the file
name=[devicename]:[x]  # Line format
name=ubuntu-test:0     # Line example
```

### 4. Update 'mdadm' to use 'mdadm.conf' File

```bash
# Update mdadm to use new mdadm.conf file.
sudo update-initramfs -u
```

### 5. Create a Mount Point for the RAID Array and Save it in the '/etc/fstab' File

```bash
# Open fstab file.
sudo vim /etc/fstab
# Add the following line (example):
/dev/md0 [path_to_mount_point] ext4 defaults 1 2
# More example fstab file contents for regular individual drives:
/dev/sda1 /media/1TB_CH0 exfat defaults 1 2
/dev/sdb1 /media/1TB_CH1 exfat defaults 1 2
```

### 6. Update Permissions to be able to read and write from the RAID drive

```bash
# These are recommended but don't always work:
sudo chown [user] [mount path]
sudo chown oakdell /media/oakdell/nas
```

Great article | [fstab mount options for umask, fmask, dmask for ntfs with noexec](https://unix.stackexchange.com/questions/396904/fstab-mount-options-for-umask-fmask-dmask-for-ntfs-with-noexec)

List of options for fstab entries | [Fstab File ( /etc/fstab ) Entry Options in Linux](https://linoxide.com/understanding-each-entry-of-linux-fstab-etcfstab-file/)

```bash
# Add these options to /etc/fstab.
# There is no group "users" in Ubuntu by default,
# but one could be created for this purpose.
/dev/md0 /media/username/nas exfat defaults,uid=oakdell,gid=users,dmask=0007,fmask-0117 1 2

# These options also work (if current user is member of group,
# don't need uid). However, for some reason this results
# in deleted files not going to the trash, but being
# permanently deleted with a warning.
/dev/md0 /media/oakdell/nas exfat defaults,gid=oakdell,dmask=0007,fmask-0117 1 2
```

## Get Information About RAID Array

```bash
# Display information about the RAID metadata
# format used by mdadm on the current system platform.
mdadm -–detail-platform
# Display RAID metadata format for a specific RAID array.
mdadm --detail /dev/md0
```

Meaning of "major" and "minor" | [https://www.linuxquestions.org/questions/linux-general-1/detailed-information-about-mdadm-output-781033/](https://www.linuxquestions.org/questions/linux-general-1/detailed-information-about-mdadm-output-781033/)

```bash
# Get RAID config information for a single disk in the array.
sudo mdadm -E /dev/sda
# See which RAID arrays are currently running.
cat /proc/mdstat
# Show real-time updates of the mdstat file to track progress.
watch -d cat /proc/mdstat
```

## Remove RAIDs that were Previously Created

Guide | [Removal of mdadm RAID Devices – How to do it quickly?](https://bobcares.com/blog/removal-of-mdadm-raid-devices/)

```bash
cat /proc/mdstat
# Do this for each one listed
sudo mdadm --stop /dev/md126
# Do this for each one listed
sudo mdadm --remove /dev/md1

sudo mdadm --zero-superblock /dev/sda /dev/sdb
lsblk
cat /proc/mdstat
```

## Restart a RAID that was Previously Running

```bash
sudo mdadm --assembly /dev/md0
```