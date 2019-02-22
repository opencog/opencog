dropdb $1

createdb $1

cat /atomspace/opencog/persist/sql/multi-driver/atom.sql | psql $1
