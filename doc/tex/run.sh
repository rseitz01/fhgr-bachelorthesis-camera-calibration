#!/bin/bash

#### invoke with:
# find . -iname "*.tex" | entr ./run.sh

#set -e
xelatex doku.tex
biber doku
#bibtex toku

outname=FHGR-Bachelorarbeit-Seitz-Raphael.pdf

cp doku.pdf $outname

if [[ -z $(lsof +d "$PWD" | grep "$outname") ]]; then
    echo "OPENING DOCUMENT"
    mupdf "$outname" &
    sleep 1 # give time for mupf to open the file
fi

pkill -HUP mupdf

