#include <stdio.h>
#include <stdlib.h>

int main(void)
{
  system("run serialtranslator.exe");
  system("matlab -r poc_analysis");
  return 0;
}