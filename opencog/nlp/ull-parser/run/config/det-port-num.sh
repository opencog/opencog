#!/bin/bash
#
# det-port-num.sh <mode> <lang>
#
# Determines port given mode and language
#
# Example usage:
#    ./det-port-num.sh pair en

if [ $# -ne 2 ]
then 
  echo "Usage: ./det-port-num.sh <mode> <language>"
  exit 0
fi


# Gets processing mode for the cogserver
case $1 in
   pairs)
      port_ini=17000
      ;;
   cmi)
      port_ini=18000
      ;;
   mst)
      port_ini=19000
      ;;
   *)
      echo "Usage: ./det-port-num.sh <mode> <language>"
      echo "<mode> must be either pairs, cmi or mst"
      exit 0
esac

# Assign port value according to language
case $2 in
   en)
      port_end=5
      ;;
   fr)
      port_end=3
      ;;
   lt)
      port_end=2
      ;;
   pl)
      port_end=4
      ;;
   yue)
      port_end=6
      ;;
   zh)
      port_end=7
      ;;
   *)
      echo "Usage: ./det-port-num.sh <mode> <language>"
      echo "<language> must be one of: en, fr, lt, pl, yue, zh"
      exit 0
esac
export PORT=$(($port_ini + $port_end))