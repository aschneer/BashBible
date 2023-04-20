# File System

## Mount Hidden System Volume

### Mount Method 1 - 'mountvol' (recommended)

REF = [How do I mount the EFI partition on Windows 8.1 so that it is readable and writeable?](https://superuser.com/questions/662823/how-do-i-mount-the-efi-partition-on-windows-8-1-so-that-it-is-readable-and-write)

```powershell
# With mapping (not sure what this means)
mountvol b: /s
	# 'b' is drive letter of your choosing
# Without mapping (not sure what this means)
mountvol b: /d
```

### Mount Method 2 - 'diskpart'

REF = [Recover/Create EFI Partition Windows 11/10 When EFI Boot Partition Missing](https://www.easeus.com/partition-master/restore-repair-deleted-efi-boot-partition-in-windows-10-8-7.html)

Run command prompt as administrator.

```powershell
diskpart
list disk
select disk 0
list partition
select partition 1
assign letter=b # or just 'assign'
exit
```

### Access the Files

If it is a system partition, like the EFI partition, you need to kill explorer.exe and run it with administrator priviledges.

I've found that navigating to C:\Windows and opening explorer with "Run as administrator" does not work.

The following method works.

First, run command prompt as administrator. Then...

```powershell
taskkill /im explorer.exe /f
explorer.exe /nouaccheck
```

Then navigate to drive B:\ in explorer and you should be able to browse the files.