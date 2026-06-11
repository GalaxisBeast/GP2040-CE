#include "drivers/xbone/XBOneDriver.h"
#include "drivers/shared/driverhelper.h"

#include "drivers/xbone/XBOneAuth.h"
#include "peripheralmanager.h"
#include "storagemanager.h"

// ---------------------------------------------------------------------------
// Timing constants
// ---------------------------------------------------------------------------

#define XBONE_KEEPALIVE_TIMER 15000

// ---------------------------------------------------------------------------
// USB control-request constants (unchanged from original)
// ---------------------------------------------------------------------------

#define USB_SETUP_DEVICE_TO_HOST            0x80
#define USB_SETUP_HOST_TO_DEVICE            0x00
#define USB_SETUP_TYPE_VENDOR               0x40
#define USB_SETUP_TYPE_CLASS                0x20
#define USB_SETUP_TYPE_STANDARD             0x00
#define USB_SETUP_RECIPIENT_INTERFACE       0x01
#define USB_SETUP_RECIPIENT_DEVICE          0x00
#define USB_SETUP_RECIPIENT_ENDPOINT        0x02
#define USB_SETUP_RECIPIENT_OTHER           0x03

#define REQ_GET_OS_FEATURE_DESCRIPTOR           0x20
#define DESC_EXTENDED_COMPATIBLE_ID_DESCRIPTOR  0x0004
#define DESC_EXTENDED_PROPERTIES_DESCRIPTOR     0x0005
#define REQ_GET_XGIP_HEADER                     0x90

// ---------------------------------------------------------------------------
// Report queue timing
// ---------------------------------------------------------------------------

static uint32_t lastReportQueue = 0;
#define REPORT_QUEUE_INTERVAL 35

// ---------------------------------------------------------------------------
// Driver state machine (unchanged from original)
// ---------------------------------------------------------------------------

typedef enum {
    READY_ANNOUNCE,
    WAIT_DESCRIPTOR_REQUEST,
    SEND_DESCRIPTOR,
    SETUP_AUTH,
    AUTH_DONE,
    NOT_READY
} XboxOneDriverState;

static XboxOneDriverState xboneDriverState = NOT_READY;

// ---------------------------------------------------------------------------
// Static payloads (unchanged from original)
// ---------------------------------------------------------------------------

static uint8_t xb1_guide_on[]  = { 0x01, 0x5b };
static uint8_t xb1_guide_off[] = { 0x00, 0x5b };

static uint8_t xboneIdle[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,
    0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t authReady[] = { 0x01, 0x00 };

static uint8_t announcePacket[] = {
    0x00, 0x2a, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00,
    0xdf, 0x33, 0x14, 0x00, 0x01, 0x00, 0x01, 0x00,
    0x17, 0x01, 0x02, 0x00, 0x01, 0x00, 0x01, 0x00,
    0x01, 0x00, 0x01, 0x00
};

const uint8_t xboxOneDescriptor[] = {
    0x10, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCA, 0x00,
    0x8B, 0x00, 0x16, 0x00, 0x1F, 0x00, 0x20, 0x00,
    0x27, 0x00, 0x2D, 0x00, 0x4A, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x01,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,
    0x06, 0x01, 0x02, 0x03, 0x04, 0x06, 0x07, 0x05,
    0x01, 0x04, 0x05, 0x06, 0x0A, 0x01, 0x1A, 0x00,
    0x57, 0x69, 0x6E, 0x64, 0x6F, 0x77, 0x73, 0x2E,
    0x58, 0x62, 0x6F, 0x78, 0x2E, 0x49, 0x6E, 0x70,
    0x75, 0x74, 0x2E, 0x47, 0x61, 0x6D, 0x65, 0x70,
    0x61, 0x64, 0x04, 0x56, 0xFF, 0x76, 0x97, 0xFD,
    0x9B, 0x81, 0x45, 0xAD, 0x45, 0xB6, 0x45, 0xBB,
    0xA5, 0x26, 0xD6, 0x2C, 0x40, 0x2E, 0x08, 0xDF,
    0x07, 0xE1, 0x45, 0xA5, 0xAB, 0xA3, 0x12, 0x7A,
    0xF1, 0x97, 0xB5, 0xE7, 0x1F, 0xF3, 0xB8, 0x86,
    0x73, 0xE9, 0x40, 0xA9, 0xF8, 0x2F, 0x21, 0x26,
    0x3A, 0xCF, 0xB7, 0xFE, 0xD2, 0xDD, 0xEC, 0x87,
    0xD3, 0x94, 0x42, 0xBD, 0x96, 0x1A, 0x71, 0x2E,
    0x3D, 0xC7, 0x7D, 0x02, 0x17, 0x00, 0x20, 0x20,
    0x00, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x17, 0x00, 0x09, 0x3C, 0x00,
    0x01, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00
};

// ---------------------------------------------------------------------------
// ACK-wait state
// ---------------------------------------------------------------------------

static bool     waiting_ack         = false;
static uint32_t waiting_ack_timeout = 0;
#define XGIP_ACK_WAIT_TIMEOUT 2000

// ---------------------------------------------------------------------------
// Misc driver state
// ---------------------------------------------------------------------------

static uint32_t timer_wait_for_announce;
static bool     xbox_one_powered_on;
static uint8_t  report_led_mode;
static uint8_t  report_led_brightness;

// ---------------------------------------------------------------------------
// Report queue (unchanged from original)
// ---------------------------------------------------------------------------

#include <queue>

typedef struct {
    uint8_t  report[XBONE_ENDPOINT_SIZE];
    uint16_t len;
} report_queue_t;

static std::queue<report_queue_t> report_queue;

// ---------------------------------------------------------------------------
// TinyUSB interface table (unchanged from original)
// ---------------------------------------------------------------------------

#define CFG_TUD_XBONE             8
#define CFG_TUD_XINPUT_TX_BUFSIZE 64
#define CFG_TUD_XINPUT_RX_BUFSIZE 64

typedef struct {
    uint8_t  itf_num;
    uint8_t  ep_in;
    uint8_t  ep_out;
    CFG_TUSB_MEM_ALIGN uint8_t epin_buf[CFG_TUD_XINPUT_TX_BUFSIZE];
    CFG_TUSB_MEM_ALIGN uint8_t epout_buf[CFG_TUD_XINPUT_RX_BUFSIZE];
} xboned_interface_t;

CFG_TUSB_MEM_SECTION static xboned_interface_t _xboned_itf[CFG_TUD_XBONE];

// ---------------------------------------------------------------------------
// File-scope XGIP objects and auth data pointer
// ---------------------------------------------------------------------------

static XGIPProtocol    *outgoingXGIP   = nullptr;
static XGIPProtocol    *incomingXGIP   = nullptr;
static XboxOneAuthData *xboxOneAuthData = nullptr;

// ---------------------------------------------------------------------------
// Windows OS-descriptor (unchanged from original)
// ---------------------------------------------------------------------------

typedef struct {
    uint32_t TotalLength;
    uint16_t Version;
    uint16_t Index;
    uint8_t  TotalSections;
    uint8_t  Reserved[7];
    uint8_t  FirstInterfaceNumber;
    uint8_t  Reserved2;
    uint8_t  CompatibleID[8];
    uint8_t  SubCompatibleID[8];
    uint8_t  Reserved3[6];
} __attribute__((packed)) OS_COMPATIBLE_ID_DESCRIPTOR_SINGLE;

const OS_COMPATIBLE_ID_DESCRIPTOR_SINGLE DevCompatIDsOne = {
    TotalLength     : sizeof(OS_COMPATIBLE_ID_DESCRIPTOR_SINGLE),
    Version         : 0x0100,
    Index           : DESC_EXTENDED_COMPATIBLE_ID_DESCRIPTOR,
    TotalSections   : 1,
    Reserved        : {0},
    Reserved2       : 0x01,
    CompatibleID    : {'X','G','I','P','1','0',0,0},
    SubCompatibleID : {0},
    Reserved3       : {0}
};

// ---------------------------------------------------------------------------
// TinyUSB class-driver callbacks
// ---------------------------------------------------------------------------

static void xbone_reset(uint8_t rhport) {
    (void)rhport;
    timer_wait_for_announce = to_ms_since_boot(get_absolute_time());
    xbox_one_powered_on     = false;
    report_led_mode         = 0;
    while (!report_queue.empty())
        report_queue.pop();
    tu_memclr(&_xboned_itf, sizeof(_xboned_itf));
}

static void xbone_init(void) {
    xbone_reset(TUD_OPT_RHPORT);
}

static uint16_t xbone_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc,
                            uint16_t max_len) {
    uint16_t drv_len = 0;
    if (TUSB_CLASS_VENDOR_SPECIFIC == itf_desc->bInterfaceClass) {
        TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == itf_desc->bInterfaceClass, 0);

        drv_len = sizeof(tusb_desc_interface_t) +
                  (itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t));
        TU_VERIFY(max_len >= drv_len, 0);

        xboned_interface_t *p_xbone = nullptr;
        for (uint8_t i = 0; i < CFG_TUD_XBONE; i++) {
            if (_xboned_itf[i].ep_in == 0 && _xboned_itf[i].ep_out == 0) {
                p_xbone = &_xboned_itf[i];
                break;
            }
        }
        TU_VERIFY(p_xbone, 0);

        uint8_t const *p_desc = (uint8_t const *)itf_desc;

        if (itf_desc->bInterfaceSubClass == 0x47 &&
            itf_desc->bInterfaceProtocol == 0xD0) {
            p_desc = tu_desc_next(p_desc);
            TU_ASSERT(usbd_open_edpt_pair(rhport, p_desc,
                                          itf_desc->bNumEndpoints,
                                          TUSB_XFER_INTERRUPT,
                                          &p_xbone->ep_out, &p_xbone->ep_in), 0);

            p_xbone->itf_num = itf_desc->bInterfaceNumber;

            if (p_xbone->ep_out) {
                if (!usbd_edpt_xfer(rhport, p_xbone->ep_out,
                                    p_xbone->epout_buf,
                                    sizeof(p_xbone->epout_buf))) {
                    TU_LOG_FAILED();
                    TU_BREAKPOINT();
                }
            }

            if (incomingXGIP != nullptr && outgoingXGIP != nullptr) {
                xboneDriverState = XboxOneDriverState::READY_ANNOUNCE;
                incomingXGIP->reset();
                outgoingXGIP->reset();
            }
        }
    }
    return drv_len;
}

static void queue_xbone_report(void *report, uint16_t report_size) {
    report_queue_t item;
    memcpy(item.report, report, report_size);
    item.len = report_size;
    report_queue.push(item);
}

bool xbone_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                   tusb_control_request_t const *request) {
    uint8_t buf[255];
    if (stage != CONTROL_STAGE_SETUP)
        return true;

    if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
        uint16_t len = request->wLength;
        if (request->bmRequestType == (USB_SETUP_DEVICE_TO_HOST |
                                        USB_SETUP_RECIPIENT_DEVICE |
                                        USB_SETUP_TYPE_VENDOR) &&
            request->bRequest == REQ_GET_OS_FEATURE_DESCRIPTOR &&
            request->wIndex == DESC_EXTENDED_COMPATIBLE_ID_DESCRIPTOR) {
            memcpy(buf, &DevCompatIDsOne, len);
        }
        tud_control_xfer(rhport, request, (void *)buf, len);
    } else {
        tud_control_xfer(rhport, request, (void *)buf, request->wLength);
    }
    return true;
}

bool xbone_xfer_cb(uint8_t rhport, uint8_t ep_addr,
                   xfer_result_t result, uint32_t xferred_bytes) {
    if (xboxOneAuthData == nullptr || incomingXGIP == nullptr ||
        outgoingXGIP == nullptr)
        return true;

    (void)result;
    uint8_t itf = 0;
    xboned_interface_t *p_xbone = _xboned_itf;

    for (;; itf++, p_xbone++) {
        if (itf >= TU_ARRAY_SIZE(_xboned_itf)) return false;
        if (ep_addr == p_xbone->ep_out || ep_addr == p_xbone->ep_in) break;
    }

    if (ep_addr == p_xbone->ep_out) {
        incomingXGIP->parse(p_xbone->epout_buf, xferred_bytes);

        // ----------------------------------------------------------------
        // FIX: queue the ack packet directly -- do NOT wrap it inside a
        // second outgoingXGIP frame.  generateAckPacket() already builds
        // the complete wire-level packet.  The original code called
        // outgoingXGIP->setData(incomingXGIP->generateAckPacket(), ...)
        // which double-wrapped the ack and produced a malformed frame.
        // ----------------------------------------------------------------
        if (incomingXGIP->ackRequired()) {
            queue_xbone_report(
                (uint8_t *)incomingXGIP->generateAckPacket(),
                incomingXGIP->getPacketLength());
        }

        uint8_t command = incomingXGIP->getCommand();

        if (command == GIP_ACK_RESPONSE) {
            waiting_ack = false;

        } else if (command == GIP_DEVICE_DESCRIPTOR) {
            outgoingXGIP->reset();
            outgoingXGIP->setAttributes(GIP_DEVICE_DESCRIPTOR,
                                        incomingXGIP->getSequence(), 1, 1, 0);
            outgoingXGIP->setData(xboxOneDescriptor, sizeof(xboxOneDescriptor));
            xboneDriverState = XboxOneDriverState::SEND_DESCRIPTOR;

        } else if (command == GIP_POWER_MODE_DEVICE_CONFIG) {
            xbox_one_powered_on = true;

        } else if (command == GIP_CMD_LED_ON) {
            report_led_mode       = incomingXGIP->getData()[1];
            report_led_brightness = incomingXGIP->getData()[2];

            if (xboneDriverState == XboxOneDriverState::WAIT_DESCRIPTOR_REQUEST) {
                outgoingXGIP->reset();
                outgoingXGIP->setAttributes(GIP_DEVICE_DESCRIPTOR,
                                            incomingXGIP->getSequence(), 1, 1, 0);
                outgoingXGIP->setData(xboxOneDescriptor, sizeof(xboxOneDescriptor));
                xboneDriverState = XboxOneDriverState::SEND_DESCRIPTOR;
            }

        } else if (command == GIP_CMD_RUMBLE) {
            // No-op; extend here for rumble passthrough if needed

        } else if (command == GIP_AUTH || command == GIP_FINAL_AUTH) {
            // ----------------------------------------------------------------
            // FIX: check for auth completion BEFORE touching any buffers.
            // The original code called dongleBuffer.reset() first, which
            // zeroed the type field so the completion check was always false.
            // ----------------------------------------------------------------
            bool isAuthComplete = (incomingXGIP->getDataLength() == 2 &&
                                   memcmp(incomingXGIP->getData(),
                                          authReady, sizeof(authReady)) == 0);
            if (isAuthComplete) {
                xboxOneAuthData->authCompleted = true;
                xboneDriverState = AUTH_DONE;
            }

            bool complete = (!incomingXGIP->getChunked()) ||
                            (incomingXGIP->getChunked() && incomingXGIP->endOfChunk());
            if (complete) {
                xboxOneAuthData->consoleBuffer.setBuffer(
                    incomingXGIP->getData(),
                    incomingXGIP->getDataLength(),
                    incomingXGIP->getSequence(),
                    incomingXGIP->getCommand());
                xboxOneAuthData->xboneState = GPAuthState::send_auth_console_to_dongle;
                incomingXGIP->reset();
            }
        }

        TU_ASSERT(usbd_edpt_xfer(rhport, p_xbone->ep_out,
                                  p_xbone->epout_buf,
                                  sizeof(p_xbone->epout_buf)));

    } else if (ep_addr == p_xbone->ep_in) {
        // IN transfer completed; nothing extra needed
    }

    return true;
}

// ---------------------------------------------------------------------------
// XBOneDriver::initialize()
// FIX: removed local shadow variables for incomingXGIP / outgoingXGIP.
// The originals declared new locals inside this function that immediately
// went out of scope, leaving the file-scope statics un-initialised.
// ---------------------------------------------------------------------------

void XBOneDriver::initialize() {
    xboneReport = {
        .sync            = 0,
        .guide           = 0,
        .start           = 0,
        .back            = 0,
        .a               = 0,
        .b               = 0,
        .x               = 0,
        .y               = 0,
        .dpadUp          = 0,
        .dpadDown        = 0,
        .dpadLeft        = 0,
        .dpadRight       = 0,
        .leftShoulder    = 0,
        .rightShoulder   = 0,
        .leftThumbClick  = 0,
        .rightThumbClick = 0,
        .leftTrigger     = 0,
        .rightTrigger    = 0,
        .leftStickX      = GAMEPAD_JOYSTICK_MID,
        .leftStickY      = GAMEPAD_JOYSTICK_MID,
        .rightStickX     = GAMEPAD_JOYSTICK_MID,
        .rightStickY     = GAMEPAD_JOYSTICK_MID,
        .reserved        = {}
    };

    class_driver = {
#if CFG_TUSB_DEBUG >= 2
        .name            = "XBONE",
#endif
        .init            = xbone_init,
        .reset           = xbone_reset,
        .open            = xbone_open,
        .control_xfer_cb = xbone_vendor_control_xfer_cb,
        .xfer_cb         = xbone_xfer_cb,
        .sof             = nullptr
    };

    keep_alive_timer         = to_ms_since_boot(get_absolute_time());
    keep_alive_sequence      = 1;
    virtual_keycode_sequence = 0;
    xb1_guide_pressed        = false;
    last_report_counter      = 0;

    // Allocate file-scope XGIP objects (not as local shadows)
    if (incomingXGIP == nullptr) incomingXGIP = new XGIPProtocol();
    if (outgoingXGIP == nullptr) outgoingXGIP = new XGIPProtocol();

    xboxOneAuthData = nullptr;
    xbone_led_mode  = 0;
}

// ---------------------------------------------------------------------------
// XBOneDriver::initializeAux()
// ---------------------------------------------------------------------------

void XBOneDriver::initializeAux() {
    authDriver = new XBOneAuth();
    if (authDriver->available()) {
        authDriver->initialize();
        // Share the exact same XboxOneAuthData instance between the driver
        // and the listener so console<->dongle state is never out of sync.
        xboxOneAuthData = ((XBOneAuth *)authDriver)->getAuthData();
    } else {
        xboxOneAuthData = nullptr;
    }
}

// ---------------------------------------------------------------------------
// XBOneDriver::get_usb_auth_listener()
// ---------------------------------------------------------------------------

USBListener *XBOneDriver::get_usb_auth_listener() {
    if (authDriver->available())
        return authDriver->getListener();
    return nullptr;
}

// ---------------------------------------------------------------------------
// XBOneDriver::set_ack_wait()
// ---------------------------------------------------------------------------

void XBOneDriver::set_ack_wait() {
    waiting_ack         = true;
    waiting_ack_timeout = to_ms_since_boot(get_absolute_time());
}

// ---------------------------------------------------------------------------
// XBOneDriver::update()
// FIX: the original had a `now` reference inside receive_report() / update()
// that was actually declared only in process().  All timing logic that uses
// `now` is now correctly scoped inside update(), which declares it locally.
// ---------------------------------------------------------------------------

void XBOneDriver::update() {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    process_report_queue(now);

    if (waiting_ack) {
        if ((now - waiting_ack_timeout) < XGIP_ACK_WAIT_TIMEOUT)
            return;
        waiting_ack = false;
    }

    switch (xboneDriverState) {

        case READY_ANNOUNCE:
            if ((now - timer_wait_for_announce) > 500) {
                memcpy((void *)&announcePacket[3], &now, 3);
                outgoingXGIP->reset();
                outgoingXGIP->setAttributes(GIP_ANNOUNCE, 1, 1, 0, 0);
                outgoingXGIP->setData(announcePacket, sizeof(announcePacket));
                queue_xbone_report(
                    outgoingXGIP->generatePacket(),
                    outgoingXGIP->getPacketLength());
                xboneDriverState = WAIT_DESCRIPTOR_REQUEST;
            }
            break;

        case SEND_DESCRIPTOR:
            queue_xbone_report(
                outgoingXGIP->generatePacket(),
                outgoingXGIP->getPacketLength());
            if (outgoingXGIP->endOfChunk()) {
                xboneDriverState = SETUP_AUTH;
            }
            if (outgoingXGIP->getPacketAck() == 1) {
                set_ack_wait();
            }
            break;

        case SETUP_AUTH:
            if (xboxOneAuthData->xboneState == GPAuthState::send_auth_dongle_to_console) {
                // ----------------------------------------------------------------
                // FIX: capture type BEFORE resetting the buffer.
                // The original reset() the buffer first then checked its type,
                // which was always zero after the reset so completion was never
                // detected and the state machine stalled permanently.
                // ----------------------------------------------------------------
                uint16_t len      = xboxOneAuthData->dongleBuffer.length;
                uint8_t  type     = xboxOneAuthData->dongleBuffer.type;
                uint8_t  sequence = xboxOneAuthData->dongleBuffer.sequence;
                uint8_t *buffer   = xboxOneAuthData->dongleBuffer.data;
                bool isChunked    = (len > GIP_MAX_CHUNK_SIZE);

                outgoingXGIP->reset();
                outgoingXGIP->setAttributes(type, sequence, 1, isChunked, 1);
                outgoingXGIP->setData(buffer, len);

                xboxOneAuthData->xboneState = GPAuthState::wait_auth_dongle_to_console;
                xboxOneAuthData->dongleBuffer.reset(); // safe to reset after capture
            }

            if (xboxOneAuthData->xboneState == GPAuthState::wait_auth_dongle_to_console) {
                queue_xbone_report(
                    outgoingXGIP->generatePacket(),
                    outgoingXGIP->getPacketLength());
                if (!outgoingXGIP->getChunked() || outgoingXGIP->endOfChunk()) {
                    xboxOneAuthData->xboneState = GPAuthState::auth_idle_state;
                }
                if (outgoingXGIP->getPacketAck() == 1) {
                    set_ack_wait();
                }
            }
            break;

        case AUTH_DONE:
        case NOT_READY:
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// XBOneDriver::process()
// ---------------------------------------------------------------------------

bool XBOneDriver::process(Gamepad *gamepad) {
    if (xboxOneAuthData == nullptr)
        return false;

    uint16_t xboneReportSize = 0;

    this->update();

    if (xbone_led_mode != report_led_mode) {
        Gamepad *processedGamepad = Storage::getInstance().GetProcessedGamepad();
        processedGamepad->auxState.playerID.active     = true;
        processedGamepad->auxState.playerID.ledValue   = report_led_mode;
        processedGamepad->auxState.playerID.ledBlinkOn = report_led_brightness;
    }

    if (!xboxOneAuthData->authCompleted) {
        GIP_HEADER((&xboneReport), GIP_INPUT_REPORT, false, last_report_counter);
        memcpy((void *)&((uint8_t *)&xboneReport)[4], xboneIdle, sizeof(xboneIdle));
        xboneReportSize = sizeof(XboxOneGamepad_Data_t);
        send_xbone_usb((uint8_t *)&xboneReport, xboneReportSize);
        return true;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());

    if ((now - keep_alive_timer) > XBONE_KEEPALIVE_TIMER) {
        memset(&xboneReport.Header, 0, sizeof(GipHeader_t));
        GIP_HEADER((&xboneReport), GIP_KEEPALIVE, 1, keep_alive_sequence);
        xboneReport.Header.length = 4;
        static uint8_t keepAlive[] = { 0x80, 0x00, 0x00, 0x00 };
        memcpy(&((uint8_t *)&xboneReport)[4], &keepAlive, sizeof(keepAlive));
        xboneReportSize = sizeof(GipHeader_t) + sizeof(keepAlive);
        if (send_xbone_usb((uint8_t *)&xboneReport, xboneReportSize)) {
            keep_alive_timer = to_ms_since_boot(get_absolute_time());
            keep_alive_sequence++;
            if (keep_alive_sequence == 0)
                keep_alive_sequence = 1;
        }
        return true;
    }

    bool virtual_keycode_change =
        (xb1_guide_pressed == true  && !gamepad->pressedA1()) ||
        (xb1_guide_pressed == false &&  gamepad->pressedA1());

    if (virtual_keycode_change) {
        uint8_t new_sequence = virtual_keycode_sequence;
        new_sequence++;
        if (new_sequence == 0) new_sequence = 1;
        GIP_HEADER((&xboneReport), GIP_VIRTUAL_KEYCODE, 1, new_sequence);
        if (!xb1_guide_pressed) {
            xboneReport.Header.length = sizeof(xb1_guide_on);
            memcpy(&((uint8_t *)&xboneReport)[4], &xb1_guide_on, sizeof(xb1_guide_on));
            xboneReportSize = sizeof(GipHeader_t) + sizeof(xb1_guide_on);
        } else {
            xboneReport.Header.length = sizeof(xb1_guide_off);
            memcpy(&((uint8_t *)&xboneReport)[4], &xb1_guide_off, sizeof(xb1_guide_off));
            xboneReportSize = sizeof(GipHeader_t) + sizeof(xb1_guide_off);
        }
        if (send_xbone_usb((uint8_t *)&xboneReport, xboneReportSize)) {
            virtual_keycode_sequence = new_sequence;
            xb1_guide_pressed        = !xb1_guide_pressed;
        }
        return true;
    }

    XboxOneGamepad_Data_t newInputReport;
    memset(&newInputReport, 0, sizeof(XboxOneGamepad_Data_t));
    GIP_HEADER((&newInputReport), GIP_INPUT_REPORT, false, last_report_counter);

    newInputReport.a               = gamepad->pressedB1();
    newInputReport.b               = gamepad->pressedB2();
    newInputReport.x               = gamepad->pressedB3();
    newInputReport.y               = gamepad->pressedB4();
    newInputReport.leftShoulder    = gamepad->pressedL1();
    newInputReport.rightShoulder   = gamepad->pressedR1();
    newInputReport.leftThumbClick  = gamepad->pressedL3();
    newInputReport.rightThumbClick = gamepad->pressedR3();
    newInputReport.start           = gamepad->pressedS2();
    newInputReport.back            = gamepad->pressedS1();
    newInputReport.guide           = 0;
    newInputReport.sync            = 0;
    newInputReport.dpadUp          = gamepad->pressedUp();
    newInputReport.dpadDown        = gamepad->pressedDown();
    newInputReport.dpadLeft        = gamepad->pressedLeft();
    newInputReport.dpadRight       = gamepad->pressedRight();

    newInputReport.leftStickX  = static_cast<int16_t>(gamepad->state.lx)  + INT16_MIN;
    newInputReport.leftStickY  = static_cast<int16_t>(~gamepad->state.ly) + INT16_MIN;
    newInputReport.rightStickX = static_cast<int16_t>(gamepad->state.rx)  + INT16_MIN;
    newInputReport.rightStickY = static_cast<int16_t>(~gamepad->state.ry) + INT16_MIN;

    if (gamepad->hasAnalogTriggers) {
        newInputReport.leftTrigger  = gamepad->pressedL2() ? 0x03FF : gamepad->state.lt;
        newInputReport.rightTrigger = gamepad->pressedR2() ? 0x03FF : gamepad->state.rt;
    } else {
        newInputReport.leftTrigger  = gamepad->pressedL2() ? 0x03FF : 0;
        newInputReport.rightTrigger = gamepad->pressedR2() ? 0x03FF : 0;
    }

    if (memcmp(&last_report[4], &((uint8_t *)&newInputReport)[4],
               sizeof(XboxOneGamepad_Data_t) - 4) != 0) {
        xboneReportSize = sizeof(XboxOneGamepad_Data_t);
        memcpy(&xboneReport, &newInputReport, xboneReportSize);
        xboneReport.Header.sequence = last_report_counter + 1;
        if (xboneReport.Header.sequence == 0)
            xboneReport.Header.sequence = 1;

        if (send_xbone_usb((uint8_t *)&xboneReport, xboneReportSize)) {
            if (memcmp(&last_report[4], &((uint8_t *)&xboneReport)[4],
                       xboneReportSize - 4) != 0) {
                last_report_counter++;
                if (last_report_counter == 0)
                    last_report_counter = 1;
                memcpy(last_report, &xboneReport, xboneReportSize);
                return true;
            }
        }
    }

    return false;
}

// ---------------------------------------------------------------------------
// XBOneDriver::processAux()
// ---------------------------------------------------------------------------

void XBOneDriver::processAux() {
    if (authDriver != nullptr && authDriver->available()) {
        ((XBOneAuth *)authDriver)->process();
    }
}

// ---------------------------------------------------------------------------
// XBOneDriver::getAuthSent()
// ---------------------------------------------------------------------------

bool XBOneDriver::getAuthSent() {
    if (xboxOneAuthData == nullptr)
        return false;
    return xboxOneAuthData->authCompleted;
}

// ---------------------------------------------------------------------------
// XBOneDriver::send_xbone_usb()
// ---------------------------------------------------------------------------

bool XBOneDriver::send_xbone_usb(uint8_t const *report, uint16_t report_size) {
    uint8_t itf = 0;
    xboned_interface_t *p_xbone = _xboned_itf;
    for (;; itf++, p_xbone++) {
        if (itf >= TU_ARRAY_SIZE(_xboned_itf)) return false;
        if (p_xbone->ep_in) break;
    }
    if (tud_ready() &&
        (p_xbone->ep_in != 0) &&
        !usbd_edpt_busy(TUD_OPT_RHPORT, p_xbone->ep_in)) {
        usbd_edpt_claim(0, p_xbone->ep_in);
        usbd_edpt_xfer(0, p_xbone->ep_in, (uint8_t *)report, report_size);
        usbd_edpt_release(0, p_xbone->ep_in);
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// TinyUSB HID callbacks (unchanged from original)
// ---------------------------------------------------------------------------

uint16_t XBOneDriver::get_report(uint8_t report_id, hid_report_type_t report_type,
                                  uint8_t *buffer, uint16_t reqlen) {
    memcpy(buffer, &xboneReport, sizeof(xboneReport));
    return sizeof(xboneReport);
}

void XBOneDriver::set_report(uint8_t report_id, hid_report_type_t report_type,
                              uint8_t const *buffer, uint16_t bufsize) {}

bool XBOneDriver::vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                          tusb_control_request_t const *request) {
    uint8_t buf[255];
    if (stage != CONTROL_STAGE_SETUP)
        return true;

    if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
        uint16_t len = request->wLength;
        if (request->bmRequestType == (USB_SETUP_DEVICE_TO_HOST |
                                        USB_SETUP_RECIPIENT_DEVICE |
                                        USB_SETUP_TYPE_VENDOR) &&
            request->bRequest == REQ_GET_OS_FEATURE_DESCRIPTOR &&
            request->wIndex == DESC_EXTENDED_COMPATIBLE_ID_DESCRIPTOR) {
            memcpy(buf, &DevCompatIDsOne, len);
        }
        tud_control_xfer(rhport, request, (void *)buf, len);
    } else {
        tud_control_xfer(rhport, request, (void *)buf, request->wLength);
    }
    return true;
}

const uint16_t *XBOneDriver::get_descriptor_string_cb(uint8_t index, uint16_t langid) {
    const char *value = (const char *)xbone_get_string_descriptor(index);
    return getStringDescriptor(value, index);
}

const uint8_t *XBOneDriver::get_descriptor_device_cb() {
    return xbone_device_descriptor;
}

const uint8_t *XBOneDriver::get_hid_descriptor_report_cb(uint8_t itf) {
    return nullptr;
}

const uint8_t *XBOneDriver::get_descriptor_configuration_cb(uint8_t index) {
    return xbone_configuration_descriptor;
}

const uint8_t *XBOneDriver::get_descriptor_device_qualifier_cb() {
    return xbone_device_qualifier;
}

// ---------------------------------------------------------------------------
// XBOneDriver::process_report_queue()
// ---------------------------------------------------------------------------

void XBOneDriver::process_report_queue(uint32_t now) {
    if (!report_queue.empty() && (now - lastReportQueue) > REPORT_QUEUE_INTERVAL) {
        if (send_xbone_usb(report_queue.front().report, report_queue.front().len)) {
            memcpy(last_report, &report_queue.front().report,
                   report_queue.front().len);
            report_queue.pop();
            lastReportQueue = now;
        } else {
            sleep_ms(REPORT_QUEUE_INTERVAL);
        }
    }
}

// ---------------------------------------------------------------------------
// XBOneDriver::GetJoystickMidValue()
// ---------------------------------------------------------------------------

uint16_t XBOneDriver::GetJoystickMidValue() {
    return GAMEPAD_JOYSTICK_MID;
}
