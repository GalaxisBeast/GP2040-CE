#include "host/usbh.h"
#include "class/hid/hid.h"
#include "class/hid/hid_host.h"
#include "drivers/xbone/XBOneAuth.h"
#include "drivers/xbone/XBOneAuthUSBListener.h"
#include "CRC32.h"
#include "peripheralmanager.h"
#include "usbhostmanager.h"

#include "drivers/xbone/XBOneDescriptors.h"
#include "drivers/shared/xgip_protocol.h"
#include "drivers/shared/xinput_host.h"

void XBOneAuth::initialize() {
    if (!available())
        return;

    listener = new XBOneAuthUSBListener();

    xboxOneAuthData.xboneState    = GPAuthState::auth_idle_state;
    xboxOneAuthData.authCompleted = false;

    XBOneAuthUSBListener *l = (XBOneAuthUSBListener *)listener;
    l->setup();
    l->setAuthData(&xboxOneAuthData);
}


bool XBOneAuth::available() {
    return PeripheralManager::getInstance().isUSBEnabled(0);
}


void XBOneAuth::process() {
    if (listener == nullptr)
        return;
    ((XBOneAuthUSBListener *)listener)->process();
}