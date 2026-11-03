#!/bin/sh
# deploy_ps.sh -- install the PS-side binaries and the command service.
#
# Run on the board. Expects the cross-compiled binaries in /tmp.

set -e

cp /tmp/ik_zdt_pl /usr/bin/ik_zdt_pl
cp /tmp/osd_text  /usr/bin/osd_text
chmod 755 /usr/bin/ik_zdt_pl /usr/bin/osd_text

cp /tmp/arm_net.sh /usr/bin/arm_net.sh
chmod 755 /usr/bin/arm_net.sh

# inetd listens on 5000 and executes arm_net.sh; make sure the entry exists
grep -q "^5000" /etc/inetd.conf 2>/dev/null || \
    echo "5000 stream tcp nowait root /usr/bin/arm_net.sh arm_net.sh" >> /etc/inetd.conf
/etc/init.d/inetd restart 2>/dev/null || true

echo "PS deploy done"
