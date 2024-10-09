#!/bin/bash

# Dump page at given address
picotool=~/Projects/picotool/build/picotool
file=$2
x=$1
#let "p1 = $((2*x*4*1024+0x10080000))"
#let "p2 = $((2*x*4*1024+0x10080000+0x2000))"
let "p1 = $((x+0x10000000))"
let "p2 = $((x+0x10000000+0x2000))"

page1=$( printf "0x%x" $p1 )
page2=$( printf "0x%x" $p2 )

echo Dump $page1 -> $page2 to file $file
sudo $picotool save -r $page1 $page2 $file
