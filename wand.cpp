#include <stdio.h>
#include <string.h>
#include "wand.h"

// Hold current barcode to scan
//CBarcode cBar;
CBarcode *pBar = NULL;

void init_wand(void)
{
  pBar = new CBarcode();
}



