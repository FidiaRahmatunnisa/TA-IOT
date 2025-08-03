#include <stdio.h>
extern void test_esp1();
extern void test_esp2();

void app_main(void) {
    // Pilih salah satu tes:
    // test_esp1();  // Untuk Mic1 + Mic2 + UART TX
    test_esp2();  // Untuk Mic3 + UART RX
}
