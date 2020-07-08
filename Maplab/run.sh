#!/bin/bash
set -exo pipefail

source $CATKIN_WS/devel/setup.sh
source /opt/ros/kinetic/setup.sh
export LD_LIBRARY_PATH=/opt/ros/kinetic/lib:$LD_LIBRARY_PATH
ldconfig -v

export TERM=vt100

cat > /etc/supervisor/conf.d/supervisord.conf <<EOF
[unix_http_server]
file=/var/run/supervisor.sock   ; (the path to the socket file)
chmod=0700                  ; sockef file mode (default 0700)

[supervisord]
logfile=/var/log/supervisor/supervisord.log ; (main log file;default $CWD/supervisord.log)
pidfile=/var/run/supervisord.pid ; (supervisord pidfile;default supervisord.pid)
childlogdir=/var/log/supervisor            ; ('AUTO' child log dir, default $TEMP)
nodaemon = true
autostart = true
autorestart = true
user = root
; the below section must remain in the config file for RPC
; (supervisorctl/web interface) to work, additional interfaces may be
; added by defining them in separate rpcinterface: sections

[rpcinterface:supervisor]
supervisor.rpcinterface_factory = supervisor.rpcinterface:make_main_rpcinterface

[supervisorctl]
serverurl=unix:///var/run/supervisor.sock ; use a unix:// URL  for a unix socket

[include]
files = /etc/supervisor/conf.d/*.conf
EOF

env | grep -v 'HOME\|PWD\|PATH' | while read line; do
   key="$(echo $line | cut -d= -f1)"
   value="$(echo $line | cut -d= -f2-)"
   echo "export $key=\"$value\"" >> /.bashrc
done

cat <<EOF > /tmp/rosrun.sh
#!/bin/bash
cd /$CATKIN_WS
. /.bashrc
. $CATKIN_WS/devel/setup.sh
. /opt/ros/kinetic/setup.sh
. $CATKIN_WS/devel/setup.sh
export LD_LIBRARY_PATH=/opt/ros/kinetic/lib:$LD_LIBRARY_PATH
exec screen -D -m -S rosrun rosrun maplab_console maplab_console
EOF

chmod 755 /tmp/rosrun.sh

cat <<EOF > /tmp/gotty.sh
#!/bin/bash
source /.bashrc
cd $CATKIN_WS
exec gotty -w --port ${PORT:-8090} screen -r -d rosrun
EOF

chmod 755 /tmp/gotty.sh

# If there is a DISPLAY set, pass it along.
cat <<EOF > /tmp/roscore.sh
#!/bin/bash
. /.bashrc
cd $CATKIN_WS
exec roscore -p ${ROSCORE_PORT:-11311}
EOF
chmod 755 /tmp/roscore.sh

cat > /etc/supervisor/conf.d/roscore.conf <<EOF
[program:roscore]
command=/tmp/roscore.sh
priority=10
directory=$CATKIN_WS
process_name=%(program_name)s
autostart=true
autorestart=true
stopsignal=TERM
stopwaitsecs=1
stdout_logfile=/dev/stdout
stdout_logfile_maxbytes=0
stderr_logfile=/dev/stderr
stderr_logfile_maxbytes=0
EOF

cat > /etc/supervisor/conf.d/rosrun.conf <<EOF
[program:rosrun]
command=/tmp/rosrun.sh
priority=10
directory=$CATKIN_WS
process_name=%(program_name)s
autostart=true
autorestart=true
stopsignal=TERM
stopwaitsecs=1
stdout_logfile=/dev/stdout
stdout_logfile_maxbytes=0
stderr_logfile=/dev/stderr
stderr_logfile_maxbytes=0
EOF

cat > /etc/supervisor/conf.d/gotty.conf <<EOF
[program:gotty]
command=/tmp/gotty.sh
priority=10
directory=$CATKIN_WS
process_name=%(program_name)s
autostart=true
autorestart=true
stdout_events_enabled=true
stderr_events_enabled=true
stopsignal=TERM
stopwaitsecs=1
stdout_logfile=/dev/stdout
stdout_logfile_maxbytes=0
stderr_logfile=/dev/stderr
stderr_logfile_maxbytes=0
EOF

chown daemon:daemon /etc/supervisor/conf.d/ /var/run/ /var/log/supervisor/

# start supervisord
exec /usr/bin/supervisord -c /etc/supervisor/supervisord.conf
