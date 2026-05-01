/*************************************************************************************************
    OBD-II_PIDs - Updated
    Supports all service modes via serial command: MMPP\n
      MM = mode (hex), PP = param/PID (hex)
    Modes with no param (03, 04, 07, 0A): PP is ignored, send "MM00\n"
    VIN shortcut: send "v\n"
    Multi-frame ISO 15765-2 handled automatically.
***************************************************************************************************/
#include <SPI.h>

#define CAN_2515
// #define CAN_2518FD

#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
const int SPI_CS_PIN  = BCM8;
const int CAN_INT_PIN = BCM25;
#else
const int SPI_CS_PIN  = 9;
const int CAN_INT_PIN = 2;
#endif

#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN);
#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN);
#endif

#define CAN_ID_PID  0x7DF
#define CAN_ID_ECU  0x7E0

// Pending command
uint8_t pendingMode   = 0;
uint8_t pendingParam  = 0;
uint8_t sendPending   = 0;

// Serial line buffer
char    cmdBuf[16];
uint8_t cmdLen = 0;

// Multi-frame (ISO 15765-2) state
bool          waitingForCF = false;
int           multiTotalLen = 0;
int           multiBytesRecv = 0;
unsigned char multiBuffer[64];

// -----------------------------------------------------------------------

bool isNoParamMode(uint8_t mode) {
    return (mode == 0x03 || mode == 0x04 || mode == 0x07 || mode == 0x0A);
}

void sendOBDRequest(uint8_t mode, uint8_t param) {
    unsigned char tmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    if (isNoParamMode(mode)) {
        tmp[0] = 0x01;
        tmp[1] = mode;
    } else if (mode == 0x02) {
        // Freeze frame needs mode + PID + frame number (0x00)
        tmp[0] = 0x03;
        tmp[1] = 0x02;
        tmp[2] = param;
        tmp[3] = 0x00;
    } else {
        tmp[0] = 0x02;
        tmp[1] = mode;
        tmp[2] = param;
    }

    SERIAL_PORT_MONITOR.print("SEND: mode=0x");
    SERIAL_PORT_MONITOR.print(mode, HEX);
    if (!isNoParamMode(mode)) {
        SERIAL_PORT_MONITOR.print(" param=0x");
        SERIAL_PORT_MONITOR.print(param, HEX);
    }
    SERIAL_PORT_MONITOR.println();
    CAN.sendMsgBuf(CAN_ID_PID, 0, 8, tmp);
}

void sendFlowControl() {
    unsigned char fc[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    CAN.sendMsgBuf(CAN_ID_ECU, 0, 8, fc);
}

void printMultiFrame() {
    // Print raw bytes in the same tab-separated format as single frames
    SERIAL_PORT_MONITOR.println("\r\n------------------------------------------------------------------");
    SERIAL_PORT_MONITOR.println("Get Data From id: 0x7E8");
    for (int i = 0; i < multiTotalLen; i++) {
        SERIAL_PORT_MONITOR.print("0x");
        SERIAL_PORT_MONITOR.print(multiBuffer[i], HEX);
        SERIAL_PORT_MONITOR.print("\t");
    }
    SERIAL_PORT_MONITOR.println();

    // If VIN response (Mode 09, InfoType 02), also print decoded string
    if (multiTotalLen >= 20 && multiBuffer[0] == 0x49 && multiBuffer[1] == 0x02) {
        SERIAL_PORT_MONITOR.print("VIN: ");
        for (int i = 3; i < 20; i++) {
            SERIAL_PORT_MONITOR.print((char)multiBuffer[i]);
        }
        SERIAL_PORT_MONITOR.println();
    }
}

// -----------------------------------------------------------------------

void processCmd() {
    if (cmdLen == 0) return;

    // VIN shortcut: "v" or "V"
    if (cmdLen == 1 && (cmdBuf[0] == 'v' || cmdBuf[0] == 'V')) {
        pendingMode  = 0x09;
        pendingParam = 0x02;
        sendPending  = 1;
        return;
    }

    // All other commands must be exactly 4 hex chars: MMPP
    if (cmdLen != 4) return;
    for (int i = 0; i < 4; i++) {
        if (!isxdigit(cmdBuf[i])) return;
    }

    char modStr[3] = {cmdBuf[0], cmdBuf[1], '\0'};
    char parStr[3] = {cmdBuf[2], cmdBuf[3], '\0'};
    pendingMode  = (uint8_t)strtol(modStr, NULL, 16);
    pendingParam = (uint8_t)strtol(parStr, NULL, 16);
    sendPending  = 1;
}

void set_mask_filt() {
    CAN.init_Mask(0, 0, 0x7FC);
    CAN.init_Mask(1, 0, 0x7FC);
    CAN.init_Filt(0, 0, 0x7E8);
    CAN.init_Filt(1, 0, 0x7E8);
    CAN.init_Filt(2, 0, 0x7E8);
    CAN.init_Filt(3, 0, 0x7E8);
    CAN.init_Filt(4, 0, 0x7E8);
    CAN.init_Filt(5, 0, 0x7E8);
}

// -----------------------------------------------------------------------

void setup() {
    SERIAL_PORT_MONITOR.begin(115200);
    while (!Serial) {}

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");
    SERIAL_PORT_MONITOR.println("DEVICE:Seeed Studio CAN Bus Shield V2");
    set_mask_filt();
}

void loop() {
    taskCanRecv();
    taskDbg();

    if (sendPending) {
        sendPending = 0;
        sendOBDRequest(pendingMode, pendingParam);
    }
}

void taskCanRecv() {
    unsigned char len = 0;
    unsigned char buf[8];

    if (CAN_MSGAVAIL == CAN.checkReceive()) {
        CAN.readMsgBuf(&len, buf);

        SERIAL_PORT_MONITOR.println("\r\n------------------------------------------------------------------");
        SERIAL_PORT_MONITOR.print("Get Data From id: 0x");
        SERIAL_PORT_MONITOR.println(CAN.getCanId(), HEX);
        for (int i = 0; i < len; i++) {
            SERIAL_PORT_MONITOR.print("0x");
            SERIAL_PORT_MONITOR.print(buf[i], HEX);
            SERIAL_PORT_MONITOR.print("\t");
        }
        SERIAL_PORT_MONITOR.println();

        unsigned char frameType = (buf[0] & 0xF0) >> 4;

        if (frameType == 0x1) {
            // First Frame
            multiTotalLen  = ((buf[0] & 0x0F) << 8) | buf[1];
            multiBytesRecv = 6;
            memcpy(multiBuffer, &buf[2], 6);
            waitingForCF   = true;
            sendFlowControl();
        } else if (frameType == 0x2 && waitingForCF) {
            // Consecutive Frame
            int copyLen = min(7, multiTotalLen - multiBytesRecv);
            memcpy(&multiBuffer[multiBytesRecv], &buf[1], copyLen);
            multiBytesRecv += copyLen;
            if (multiBytesRecv >= multiTotalLen) {
                waitingForCF = false;
                printMultiFrame();
            }
        }
    }
}

void taskDbg() {
    while (SERIAL_PORT_MONITOR.available()) {
        char c = SERIAL_PORT_MONITOR.read();
        if (c == '\r') continue;
        if (c == '\n') {
            cmdBuf[cmdLen] = '\0';
            processCmd();
            cmdLen = 0;
        } else if (cmdLen < 15) {
            cmdBuf[cmdLen++] = c;
        }
    }
}
// END FILE
