#include  "RP2040/startup_RP2040.h"

#include "app2/app2.h"

int main() {
    init_app2();
    while (1) {
        update_app2();
    }
}

