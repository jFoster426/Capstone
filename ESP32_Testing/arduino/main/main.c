#include <stdio.h>
#include "Arduino.h"

void app_main(void)
{
    pinMode(9, OUTPUT);

    while (1)
    {
        digitalWrite(9, HIGH);
        delay(100);
        digitalWrite(9, LOW);
        delay(100);
    }
}
