#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "serialport.h"

FILE *csv;
FILE *fname;

int main(void)
{
	HANDLE port = openSerialPort("COM7", B230400, one, off);
  time_t t = time(NULL);
  struct tm *tm = localtime(&t);
  char filename[64];
  strftime(filename, sizeof(filename), "dataset_%Y_%B_%d_%H_%M_%S.txt", tm);
  
  csv = fopen(filename, "w+");
  // Store the file name for convenience for shell program.
  remove("filename.txt");
  fname = fopen("filename.txt", "w+");
  fprintf(fname, filename);
  fclose(fname);
  
  char lineBuffer[1024];
  fprintf(csv, "time,faccx,faccy,faccz,fgyrx,fgyry,fgyrz,saccx,saccy,saccz,sgyrx,sgyry,sgyrz,load\n");
  
  int measurementStarted = 0;
  
  while (1)
  {
    // Wait for new data.
    int i = 0;
    lineBuffer[0] = '\0';
    while (lineBuffer[i == 0 ? 0 : i - 1] != '\n') readFromSerialPort(port, &lineBuffer[i++], 1);
    // Null terminate to avoid write errors.
    lineBuffer[i] = '\0';
    // Wait for START line while measurement not started.
    if (measurementStarted == 0)
    {
      if (strcmp(lineBuffer, "START\n") == 0)
      {
        printf("Measurement started.\n");
        measurementStarted = 1;
        continue;
      }
    }
    else
    {
      // Check for measurement END.
      if (strcmp(lineBuffer, "END\n") == 0)
      {
        printf("Measurement completed.\n");
        break;
      }
      else
      {
        // Put data in file.
        fprintf(csv, &lineBuffer[0]);
      }
    }
  }
	closeSerialPort(port);
  fclose(csv);
  printf("filename: %s\n", filename);
  return 0;
}