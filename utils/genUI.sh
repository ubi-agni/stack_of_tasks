#!/bin/bash
unset IN
unset OUT
IN=$1
OUT=$2

shopt -s nullglob
echo "Generating from $IN to $OUT...."
for f in $IN/*.ui ; do
    FNAME=$(basename -s '.ui' $f)
    echo $FNAME
    python3 -m PyQt5.uic.pyuic $f -o "$OUT/$FNAME.py"
done
shopt -u nullglob
