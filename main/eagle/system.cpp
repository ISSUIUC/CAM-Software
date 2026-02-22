#ifdef IS_EAGLE

#include <eagle/system.h>

USBCDC USBSerial;
EAGLESystems sys;

/* Begin all system data */
[[noreturn]] void sys_begin() {

    USB.begin();
    USBSerial.begin(115200);

    sys.serial = &USBSerial;

    while(true) {
        sys.serial->println("hi");
        delay(1000);
    }
}

#endif
