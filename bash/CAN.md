# CAN

CAN message structure:

```
# CAN Frame:
XXXXXXX  AABBBBCC  [D]  EE FF GG HH II JJ KK LL

# Meaning:
XXXXXXX = Name of connected can bus
ID (hex) = Identifier

    AA = Priority
    BBBB = PGN (message family)
    CC = Source address

[D] (dec?) = Size of payload  
CAN 2.0 = 8 bytes  
CAN FD = variable number of bytes

PAYLOAD (hex) = Data transmitted
    EE = Data byte 1 = command byte / message identifier
    FF = Data byte 2
    GG = Data byte 3
    HH = Data byte 4
    II = Data byte 5
    JJ = Data byte 6
    KK = Data byte 7
    LL = Data byte 8

Reading left to right:
    Little Endian = bytes go from least significant to most significant (LSB to MSB)
    Big Endian = bytes go from most significant to least significant (MSB to LSB)

Whether to use little or big endian is the choice of the person defining the CAN message.
```

candump:

```bash
# CAN Dump
candump canport1
```

```bash
# CAN dump and search for string "[8] C1",
# Only report results containing it.
candump canport1 | grep "\[8\] C1"
```

```bash
# Candump with absolute timestamps in date format.
candump -tA canport1 | grep -i "FFFEB1"
```

```bash
# Candump pulling 1 data frame only.
candump -n 1 canport1
```

```bash
# Save candump output for 2 CAN buses to a log file.
candump canport1 canport2 > ./log.txt
```

candump filtering:

- https://manpages.ubuntu.com/manpages/bionic/man1/candump.1.html
- https://stackoverflow.com/questions/32870891/heavily-confused-by-candump-socketcan-id-filtering-feature

```bash
# Candump filtering:
candump -r 1 -n 1 brtcan2,18FFF4B1:0FFFFFF0
	# 18FFF4B1 is the ID, 0FFFFFF0 is the mask.
	# Both are hexadecimal.
	# They are ANDed together, so this returns all frames with ID matching _8FFF4B_.
	# -r 1 = sets buffer to 1 (not sure the effect of this, still testing).
candump -r 1 -n 1 brtcan2,18FFF4B1~0FFFFFF0
	# The "~" will return all frames NOT matching that ID+mask.
```

