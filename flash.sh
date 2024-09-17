#!/bin/bash

# Load a ROM module at offset 512KB in flash
# With 4MB flash this lead to 3584KB for images
# Every page takes 8KB -> 448 ROM images!

# insert module page (4-15)
picotool=~/Projects/picotool/build/picotool
mod=$1 # The ROM file to flash
x=$2 # The page to flash to (decimal: 0-15)

if ((x >= 3 && x <= 32)); then
  let "x = $((2*x*4*1024+0x10080000))"

  page=$( printf "0x%x" $x )

  echo Flash to $page
  sudo $picotool load -v $mod -t bin -o $page
else
  echo "Page out of range!"
fi

