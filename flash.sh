#!/bin/bash

# insert module page (4-15)
picotool=~/Projects/picotool/build/picotool
mod=$1
x=$2
let "x = $((2*x*4*1024+0x10080000))"

page=$( printf "0x%x" $x )

sudo $picotool load -v $mod -t bin -o $page