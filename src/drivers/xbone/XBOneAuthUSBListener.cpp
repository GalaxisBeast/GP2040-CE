#include "host/usbh.h"
#include "class/hid/hid.h"
#include "class/hid/hid_host.h"
#include "drivers/xbone/XBOneAuthUSBListener.h"
#include "CRC32.h"
#include "peripheralmanager.h"
#include "usbhostmanager.h"

#include "drivers/xbone/XBOneDescriptors.h"
#include "drivers/shared/xgip_protocol.h"
#include "drivers/shared/xinput_host.h"

#include <queue>
#include <cstring>

// ---------------------------------------------------------------------------
// Internal queue type
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  report[XBONE_ENDPOINT_SIZE];
    uint16_t len;
} report_queue_t;

static std::queue<report_queue_t> report_queue;
static uint32_t lastReportQueue = 0;
#define REPORT_QUEUE_INTERVAL 15

static bool is_direct_controller = false;

static const char XBONE_GAMEPAD_TYPE_STR[] = "Windows.Xbox.Input.Gamepad";

// Simple byte-pattern search (memmem is not guaranteed on newlib)
static bool descriptor_contains_gamepad_string(const uint8_t *data, uint16_t len) {
    const uint16_t patLen = sizeof(XBONE_GAMEPAD_TYPE_STR) - 1; // exclude NUL
    if (data == nullptr || len < patLen)
        return false;
    for (uint16_t i = 0; i + patLen <= len; i++) {
        if (memcmp(&data[i], XBONE_GAMEPAD_TYPE_STR, patLen) == 0)
            return true;
    }
    return false;
}

void XBOneAuthUSBListener::setup() {
    xboxOneAuthData = nullptr;
    xbone_dev_addr  = 0;
    xbone_instance  = 0;
    mounted         = false;
    is_direct_controller = false;
    incomingXGIP.reset();
    outgoingXGIP.reset();
}

void XBOneAuthUSBListener::setAuthData(XboxOneAuthData *authData) {
    xboxOneAuthData = authData;
    xboxOneAuthData->dongle_ready = false;
}

// ---------------------------------------------------------------------------
// process() -- called every task tick from XBOneAuth::process()
// ---------------------------------------------------------------------------

void XBOneAuthUSBListener::process() {
    if (!mounted || xboxOneAuthData == nullptr)
        return;

    // Console -> controller/dongle: build the outgoing XGIP packet
    if (xboxOneAuthData->xboneState == GPAuthState::send_auth_console_to_dongle) {
        uint8_t isChunked = (xboxOneAuthData->consoleBuffer.length > GIP_MAX_CHUNK_SIZE);
        uint8_t needsAck  = (xboxOneAuthData->consoleBuffer.length > 2);
        outgoingXGIP.reset();
        outgoingXGIP.setAttributes(
            xboxOneAuthData->consoleBuffer.type,
            xboxOneAuthData->consoleBuffer.sequence,
            1, isChunked, needsAck);
        outgoingXGIP.setData(
            xboxOneAuthData->consoleBuffer.data,
            xboxOneAuthData->consoleBuffer.length);
        xboxOneAuthData->consoleBuffer.reset();
        xboxOneAuthData->xboneState = GPAuthState::wait_auth_console_to_dongle;
    }

    // Drain outgoing packet (may span multiple chunks)
    if (xboxOneAuthData->xboneState == GPAuthState::wait_auth_console_to_dongle) {
        queue_host_report(outgoingXGIP.generatePacket(), outgoingXGIP.getPacketLength());
        if (!outgoingXGIP.getChunked() || outgoingXGIP.endOfChunk()) {
            xboxOneAuthData->xboneState = GPAuthState::auth_idle_state;
        }
    }

    process_report_queue();
}

// ---------------------------------------------------------------------------
// xmount()
// ---------------------------------------------------------------------------

void XBOneAuthUSBListener::xmount(uint8_t dev_addr, uint8_t instance,
                                   uint8_t controllerType, uint8_t subtype) {
    if (controllerType == xinput_type_t::XBOXONE) {
        xbone_dev_addr = dev_addr;
        xbone_instance = instance;
        incomingXGIP.reset();
        outgoingXGIP.reset();
        mounted = true;
        // Detection of direct controller vs dongle happens later when the
        // device descriptor arrives; assume dongle until proven otherwise.
        is_direct_controller = false;
    }
}

// ---------------------------------------------------------------------------
// unmount()
// ---------------------------------------------------------------------------

void XBOneAuthUSBListener::unmount(uint8_t dev_addr) {
    if (dev_addr != xbone_dev_addr)
        return;

    mounted = false;
    is_direct_controller = false;
    while (!report_queue.empty())
        report_queue.pop();
    incomingXGIP.reset();
    outgoingXGIP.reset();

    if (xboxOneAuthData != nullptr)
        xboxOneAuthData->dongle_ready = false;
}

// ---------------------------------------------------------------------------
// report_received()
// ---------------------------------------------------------------------------

void XBOneAuthUSBListener::report_received(uint8_t dev_addr, uint8_t instance,
                                            uint8_t const *report, uint16_t len) {
    if (!mounted || xboxOneAuthData == nullptr ||
        dev_addr != xbone_dev_addr || instance != xbone_instance)
        return;

    incomingXGIP.parse(report, len);
    if (!incomingXGIP.validate()) {
        sleep_ms(50); // first packet from a fresh device may be garbage
        incomingXGIP.reset();
        return;
    }

    // Ack before any state mutation so sequence numbers are intact.
    // generateAckPacket() builds the complete wire packet; queue it directly.
    if (incomingXGIP.ackRequired()) {
        queue_host_report(
            (uint8_t *)incomingXGIP.generateAckPacket(),
            incomingXGIP.getPacketLength());
    }

    switch (incomingXGIP.getCommand()) {

        // ----------------------------------------------------------------
        // GIP_ANNOUNCE (0x02) is the device-arrival command.  Both dongles
        // and regular controllers send this when they connect.  Santroller
        // names this same command GIP_ARRIVAL; it is NOT a different
        // command, just a different name in that codebase.
        // ----------------------------------------------------------------
        case GIP_ANNOUNCE:
            outgoingXGIP.reset();
            outgoingXGIP.setAttributes(GIP_DEVICE_DESCRIPTOR, 1, 1, false, 0);
            queue_host_report(
                (uint8_t *)outgoingXGIP.generatePacket(),
                outgoingXGIP.getPacketLength());
            break;

        case GIP_DEVICE_DESCRIPTOR:
            if (incomingXGIP.endOfChunk() && !xboxOneAuthData->dongle_ready) {

                // Identify the device type from the descriptor content,
                // exactly like Santroller does: a real controller's
                // descriptor advertises "Windows.Xbox.Input.Gamepad".
                is_direct_controller = descriptor_contains_gamepad_string(
                    incomingXGIP.getData(), incomingXGIP.getDataLength());

                // Power-on packets are identical for both device types
                outgoingXGIP.reset();
                outgoingXGIP.setAttributes(GIP_POWER_MODE_DEVICE_CONFIG, 2, 1, false, 0);
                outgoingXGIP.setData(XBOXONE_POWER_ON, sizeof(XBOXONE_POWER_ON));
                queue_host_report(
                    (uint8_t *)outgoingXGIP.generatePacket(),
                    outgoingXGIP.getPacketLength());

                outgoingXGIP.reset();
                outgoingXGIP.setAttributes(GIP_POWER_MODE_DEVICE_CONFIG, 3, 1, false, 0);
                outgoingXGIP.setData(XBOXONE_POWER_ON_SINGLE, sizeof(XBOXONE_POWER_ON_SINGLE));
                queue_host_report(
                    (uint8_t *)outgoingXGIP.generatePacket(),
                    outgoingXGIP.getPacketLength());

                if (is_direct_controller) {
                    // -----------------------------------------------------
                    // Direct controller path (matches Santroller xone_host):
                    // LED with internal=1, rumble enable sent as
                    // GIP_POWER_MODE_DEVICE_CONFIG sequence 1.
                    // -----------------------------------------------------
                    outgoingXGIP.reset();
                    outgoingXGIP.setAttributes(GIP_CMD_LED_ON, 1, 1, false, 0);
                    outgoingXGIP.setData(XBOXONE_LED_ON, sizeof(XBOXONE_LED_ON));
                    queue_host_report(
                        (uint8_t *)outgoingXGIP.generatePacket(),
                        outgoingXGIP.getPacketLength());

                    outgoingXGIP.reset();
                    outgoingXGIP.setAttributes(GIP_POWER_MODE_DEVICE_CONFIG, 1, 1, false, 0);
                    outgoingXGIP.setData(XBOXONE_RUMBLE_ON, sizeof(XBOXONE_RUMBLE_ON));
                    queue_host_report(
                        (uint8_t *)outgoingXGIP.generatePacket(),
                        outgoingXGIP.getPacketLength());
                } else {
                    // -----------------------------------------------------
                    // Dongle path: byte-for-byte the original GP2040-CE
                    // sequence.  LED with internal=0, rumble as standalone
                    // GIP_CMD_RUMBLE.  Magic-X and similar dongles depend
                    // on this exact sequence -- do not change it.
                    // -----------------------------------------------------
                    outgoingXGIP.reset();
                    outgoingXGIP.setAttributes(GIP_CMD_LED_ON, 1, 0, false, 0); // not internal function
                    outgoingXGIP.setData(XBOXONE_LED_ON, sizeof(XBOXONE_LED_ON));
                    queue_host_report(
                        (uint8_t *)outgoingXGIP.generatePacket(),
                        outgoingXGIP.getPacketLength());

                    outgoingXGIP.reset();
                    outgoingXGIP.setAttributes(GIP_CMD_RUMBLE, 1, 0, false, 0); // not internal function
                    outgoingXGIP.setData(XBOXONE_RUMBLE_ON, sizeof(XBOXONE_RUMBLE_ON));
                    queue_host_report(
                        (uint8_t *)outgoingXGIP.generatePacket(),
                        outgoingXGIP.getPacketLength());
                }

                xboxOneAuthData->dongle_ready = true;
            }
            break;

        case GIP_AUTH:
        case GIP_FINAL_AUTH:
            if (!incomingXGIP.getChunked() ||
                (incomingXGIP.getChunked() && incomingXGIP.endOfChunk())) {
                xboxOneAuthData->dongleBuffer.setBuffer(
                    incomingXGIP.getData(),
                    incomingXGIP.getDataLength(),
                    incomingXGIP.getSequence(),
                    incomingXGIP.getCommand());
                xboxOneAuthData->xboneState = GPAuthState::send_auth_dongle_to_console;
                incomingXGIP.reset();
            }
            break;

        case GIP_ACK_RESPONSE:
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

void XBOneAuthUSBListener::queue_host_report(void *report, uint16_t len) {
    report_queue_t item;
    memcpy(item.report, report, len);
    item.len = len;
    report_queue.push(item);
}

void XBOneAuthUSBListener::process_report_queue() {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (mounted && !report_queue.empty() &&
        (now - lastReportQueue) > REPORT_QUEUE_INTERVAL) {
        if (tuh_xinput_send_report(xbone_dev_addr, xbone_instance,
                                   report_queue.front().report,
                                   report_queue.front().len)) {
            report_queue.pop();
            lastReportQueue = now;
        } else {
            sleep_ms(REPORT_QUEUE_INTERVAL);
        }
    }
}