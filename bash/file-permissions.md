# File Permissions

chmod:

```bash
sudo chmod <who to assign><permissions> <file/folder names>
sudo chmod u+x ./file.txt  # assigns the user execute permissions
sudo chmod +x ./file.txt  # assigns all users execute permissions
	sudo chmod a+x ./file.txt  # equivalent
sudo chmod g-x ./file.txt  # removes execute permissions for the group
# 'who' codes
o  # owner
g  # group
u  # user
a  # all users
# Binary permission codes. Add together to assign.
r = 4 = 100b
w = 2 = 010b
x = 1 = 001b
```

| decimal code |     |     |     |     |     |     |     |     |     |     |     |
| ------------ | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
|              | o   | o   | o   |     | g   | g   | g   |     | u   | u   | u   |
|              | r   | w   | x   |     | r   | w   | x   |     | r   | w   | x   |
|              |     |     |     |     |     |     |     |     |     |     |     |
| 700          | 1   | 1   | 1   |     | 0   | 0   | 0   |     | 0   | 0   | 0   |
| 600          | 1   | 1   | 0   |     | 0   | 0   | 0   |     | 0   | 0   | 0   |
| 500          | 1   | 0   | 1   |     | 0   | 0   | 0   |     | 0   | 0   | 0   |
| 400          | 1   | 0   | 0   |     | 0   | 0   | 0   |     | 0   | 0   | 0   |
| 300          | 0   | 1   | 1   |     | 0   | 0   | 0   |     | 0   | 0   | 0   |
| 200          | 0   | 1   | 0   |     | 0   | 0   | 0   |     | 0   | 0   | 0   |
| 100          | 0   | 0   | 1   |     | 0   | 0   | 0   |     | 0   | 0   | 0   |
| 000          | 0   | 0   | 0   |     | 0   | 0   | 0   |     | 0   | 0   | 0   |

| binary code | decimal code | owner | group | user (all others) |
| ----------- | ------------ | ----- | ----- | ----------------- |
| ooo ggg uuu |              |       |       |                   |
| rwx rwx rwx |              |       |       |                   |
| 100 000 000 | 400          | r--   | ---   | ---               |
| 010 000 000 | 200          | -w-   | ---   | ---               |
| 001 000 000 | 100          | --x   | ---   | ---               |
| 000 100 000 | 040          | ---   | r--   | ---               |
| 000 010 000 | 020          | ---   | -w-   | ---               |
| 000 001 000 | 010          | ---   | --x   | ---               |
| 111 000 000 | 700          | rwx   | ---   | ---               |
| 110 000 000 | 600          | rw-   | ---   | ---               |
| 101 000 000 | 500          | r-x   | ---   | ---               |
| 011 000 000 | 300          | -wx   | ---   | ---               |
| 111 000 100 | 704          | rwx   | ---   | rw-               |
