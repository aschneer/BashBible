# WSL

## Shrink Virtual Disk

### Option 1

Reclaims unused space. Does not change maximum virtual disk size. Disk can automatically expand again in the future, so you have to do this periodically when you delete large files.

[https://stephenreescarter.net/how-to-shrink-a-wsl2-virtual-disk/](https://stephenreescarter.net/how-to-shrink-a-wsl2-virtual-disk/)

```powershell
wsl --shutdown
wsl -l -v    # Make sure all VMs are stopped
diskpart
Select vdisk file="<pathToVHD>"
detail vdisk
compact vdisk    # Reclaims unused space
```

{% hint style="info" %}
**TIP:** If the `compact vdisk` command seems to be hung up for a long time with no progress (ex. at 12%), hit **Enter.**
{% endhint %}

### Option 2

```powershell
Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V-All
```

Let it reboot.

```powershell
Resize-VHD -Path "<path-to-vhd>" -ToMinimumSize
# or
Optimize-VHD -Path "<path-to-vhd>" -Mode Full
```

## Set Maximum Size of Virtual Disk

[https://learn.microsoft.com/en-us/windows/wsl/disk-space](https://learn.microsoft.com/en-us/windows/wsl/disk-space)

Default is 1TB (1024 GB), so this is only relevant if you want to set the MAX size larger than 1TB.

The MAX size is the "Virtual Size" when you run the following in `diskpart`:

```powershell
detail vdisk
```

