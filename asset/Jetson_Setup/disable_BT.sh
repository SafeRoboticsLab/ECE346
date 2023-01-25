#!/bin/bash

sudo install -b -m 755 /dev/stdin /etc/rc.local << EOF
#!/bin/sh
rfkill block bluetooth
exit 0
EOF

sudo chmod +x /etc/rc.local
