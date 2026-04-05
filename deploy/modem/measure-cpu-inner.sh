#!/system/bin/sh
# Runs ON the modem (pushed via adb). No external tools — only POSIX sh builtins + /proc.
# Do not run directly; use measure-cpu.sh on the Pi.

INTERVAL=${1:-5}
SNAP1=/data/logs/cpu_snap1
SNAP2=/data/logs/cpu_snap2
OIFS=$IFS

echo "=== Modem CPU Measurement (${INTERVAL}s window) ==="
echo ""

# Snapshot /proc/stat and all per-process stats.
# /proc/PID/stat format: "pid (name) state ppid ... utime stime ..."
# We split on parens to avoid set -- misinterpreting (name) as a subshell.
# After the closing ')': fields are state(1) ppid(2) ... utime(12) stime(13).
take_snapshot() {
    dir=$1
    mkdir -p $dir
    cat /proc/stat > $dir/_stat
    for f in /proc/[0-9]*/stat; do
        pid=${f%/stat}; pid=${pid#/proc/}
        IFS='()'
        read _p name after < $f 2>/dev/null
        IFS=$OIFS
        [ -z "$after" ] && continue
        set -- $after
        echo "${12} ${13} $name" > $dir/$pid
    done
}

take_snapshot $SNAP1
sleep $INTERVAL
take_snapshot $SNAP2

# ---- total CPU delta ----
while read n a b c d e rest; do case $n in cpu) u1=$a s1=$c i1=$d w1=$e;; esac; done < $SNAP1/_stat
while read n a b c d e rest; do case $n in cpu) u2=$a s2=$c i2=$d w2=$e;; esac; done < $SNAP2/_stat

du=$((u2-u1)); ds=$((s2-s1)); di=$((i2-i1)); dw=$((w2-w1))
total=$((du+ds+di+dw)); [ $total -eq 0 ] && total=1
active=$((du+ds+dw))

echo "--- Total CPU ---"
echo "  User:    $((du*100/total))%"
echo "  System:  $((ds*100/total))%"
echo "  IOWait:  $((dw*100/total))%"
echo "  Active:  $((active*100/total))%"
echo "  Idle:    $((di*100/total))%"
echo ""

# ---- per-core delta ----
echo "--- Per-core ---"
while read cn cu nn cs ci cw rest; do
    case $cn in cpu[0-9]) eval "_u1_${cn#cpu}=$cu _s1_${cn#cpu}=$cs _i1_${cn#cpu}=$ci";; esac
done < $SNAP1/_stat
while read cn cu nn cs ci cw rest; do
    case $cn in cpu[0-9])
        core=${cn#cpu}
        eval "pu=\$_u1_$core ps=\$_s1_$core pi=\$_i1_$core"
        dcu=$((cu-pu)); dcs=$((cs-ps)); dci=$((ci-pi))
        ct=$((dcu+dcs+dci)); [ $ct -eq 0 ] && ct=1
        echo "  cpu${core}: $(((dcu+dcs)*100/ct))% active  (user=$dcu sys=$dcs idle=$dci jiffies)"
    esac
done < $SNAP2/_stat
echo ""

# ---- per-process delta ----
echo "--- Processes with >0% CPU ---"
echo "  TOTAL%  PID      USER%  SYS%   NAME"
echo "  ----------------------------------------"

for f1 in $SNAP1/[0-9]*; do
    pid=${f1##*/}
    f2=$SNAP2/$pid
    [ -f "$f2" ] || continue
    read ut1 st1 pname < $f1
    read ut2 st2 rest  < $f2
    case $ut1 in ''|*[!0-9]*) continue;; esac
    case $ut2 in ''|*[!0-9]*) continue;; esac
    dut=$((ut2-ut1)); dst=$((st2-st1))
    [ $((dut+dst)) -le 0 ] && continue
    tpct=$(((dut+dst)*100/total))
    [ $tpct -le 0 ] && continue
    echo "  ${tpct}%      $pid      $((dut*100/total))%    $((dst*100/total))%    $pname"
done
echo ""

# ---- cleanup ----
rm -rf $SNAP1 $SNAP2 /data/logs/_measure_cpu.sh
