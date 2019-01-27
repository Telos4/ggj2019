#!/bin/sh
for file in $(ls Erinnerung*)
do
	convert $file  -alpha set -virtual-pixel transparent -channel A -blur 0x60 -level 50%,100% +channel  faded/$file
done
