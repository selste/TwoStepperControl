/* test program for the wiringPi-library ...              */
/* compile with gcc -Wall -o st4test st4test.c -lwiringPi */
/* wbirkfellner 2017.                                     */

#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>

int main (void) {
short dp, rm, dm, rp;
  
  setenv("WIRINGPI_GPIOMEM", "1", 1);
  wiringPiSetup ();
  pinMode (2, INPUT);
  pinMode (3, INPUT);
  pinMode (4, INPUT);
  pinMode (5, INPUT);
  pullUpDnControl(2,PUD_UP);
  pullUpDnControl(3,PUD_UP);
  pullUpDnControl(4,PUD_UP);
  pullUpDnControl(5,PUD_UP);

  for (;;) {
    dp=abs(1-digitalRead(2));
    rp=abs(1-digitalRead(3));
    dm=abs(1-digitalRead(4));
    rm=abs(1-digitalRead(5));
    printf("%d %d %d %d \n", dp, rm, dm, rp);
    delay(10);
  }
  return 0 ;
}