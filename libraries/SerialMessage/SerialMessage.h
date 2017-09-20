#ifndef SerialMessage_h
#define SerialMessage_h

#include <inttypes.h>

#define PRINT_MAX_BUF       256
static char errMsgBuf[PRINT_MAX_BUF] = "";
static char printMsgBuf[PRINT_MAX_BUF] = "";

void printMessage(const char *msg);
uint8_t failMessage(const char *msg);

#endif