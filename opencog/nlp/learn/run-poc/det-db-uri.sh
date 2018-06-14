#!/bin/bash
#
# det-db-uri.sh <lang>
#
# Determines database port given mode and language
#
# Example usage:
#    ./det-db-uri.sh en

# Modify here with your postgress username and password
export db_user="opencog_user" # postgres username
export db_pswd="cheese"       # postgres password

# Gets database name according to language
case $1 in
   en)
      export db_name="en_pairs"    # name of your database
      ;;
   fr)
      export db_name="fr_pairs"    # name of your database
      ;;
   lt)
      export db_name="lt_pairs"    # name of your database
      ;;
   pl)
      export db_name="pl_pairs"    # name of your database
      ;;
   yue)
      export db_name="yue_pairs"    # name of your database
      ;;
   zh)
      export db_name="zh_pairs"    # name of your database
      ;;
   *)
      echo "Usage: ./det-db-uri.sh <language>"
      echo "<language> must be one of: en, fr, lt, pl, yue, zh"
      exit 0
esac