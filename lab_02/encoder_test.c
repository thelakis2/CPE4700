#include <stdio.h>
#include <unistd.h>

#include "encoder.h"

int main(void)
{
    if (enc_init() != 0)
    {
        printf("Encoder init failed\n");
        return -1;
    }

    for (int i = 0; i < 100; i++)
    {
        int c0 = enc_get_count(0);
        int c1 = enc_get_count(1);

        double s0 = enc_get_speed(0);
        double s1 = enc_get_speed(1);

        printf("ENC0: count=%d speed=%f\n",
               c0, s0);

        printf("ENC1: count=%d speed=%f\n",
               c1, s1);

        usleep(50000);
    }

    enc_cleanup();

    return 0;
}