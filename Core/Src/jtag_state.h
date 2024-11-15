#pragma once
#include <stdint.h>

typedef enum {
  JTAG_STATE_UNKNOWN = 0,
  JTAG_STATE_TEST_LOGIC_RESET,
  JTAG_STATE_RUN_TEST_IDLE,
  JTAG_STATE_SELECT_DR_SCAN,
  JTAG_STATE_CAPTURE_DR,
  JTAG_STATE_SHIFT_DR,
  JTAG_STATE_EXIT1_DR,
  JTAG_STATE_PAUSE_DR,
  JTAG_STATE_EXIT2_DR,
  JTAG_STATE_UPDATE_DR,
  JTAG_STATE_SELECT_IR_SCAN,
  JTAG_STATE_CAPTURE_IR,
  JTAG_STATE_SHIFT_IR,
  JTAG_STATE_EXIT1_IR,
  JTAG_STATE_PAUSE_IR,
  JTAG_STATE_EXIT2_IR,
  JTAG_STATE_UPDATE_IR,
  JTAG_STATE_COUNT = JTAG_STATE_UPDATE_IR
} jtag_state_t;

struct jtag_state_path {
  uint8_t path;
  uint8_t n;
};

#define FROM_TEST_LOGIC_RESET                   \
  /* to TEST_LOGIC_RESET */ { .path = 0b00000001, .n = 1 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000000, .n = 1 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000010, .n = 2 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000010, .n = 3 },  \
  /* to SHIFT-DR         */ { .path = 0b00000010, .n = 4 },  \
  /* to EXIT1-DR         */ { .path = 0b00001010, .n = 4 },  \
  /* to PAUSE-DR         */ { .path = 0b00001010, .n = 5 },  \
  /* to EXIT2-DR         */ { .path = 0b00101010, .n = 6 },  \
  /* to UPDATE-DR        */ { .path = 0b00011010, .n = 5 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00000110, .n = 3 },  \
  /* to CAPTURE-IR       */ { .path = 0b00000110, .n = 4 },  \
  /* to SHIFT-IR         */ { .path = 0b00000110, .n = 5 },  \
  /* to EXIT1-IR         */ { .path = 0b00010110, .n = 5 },  \
  /* to PAUSE-IR         */ { .path = 0b00010110, .n = 6 },  \
  /* to EXIT2-IR         */ { .path = 0b01010110, .n = 7 },  \
  /* to UPDATE-IR        */ { .path = 0b00110110, .n = 6 }

#define FROM_RUN_TEST_IDLE                      \
  /* to TEST_LOGIC_RESET */ { .path = 0b00000111, .n = 3 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000000, .n = 1 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000001, .n = 1 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000001, .n = 2 },  \
  /* to SHIFT-DR         */ { .path = 0b00000001, .n = 3 },  \
  /* to EXIT1-DR         */ { .path = 0b00000101, .n = 3 },  \
  /* to PAUSE-DR         */ { .path = 0b00000101, .n = 4 },  \
  /* to EXIT2-DR         */ { .path = 0b00010101, .n = 5 },  \
  /* to UPDATE-DR        */ { .path = 0b00001101, .n = 4 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00000011, .n = 2 },  \
  /* to CAPTURE-IR       */ { .path = 0b00000011, .n = 3 },  \
  /* to SHIFT-IR         */ { .path = 0b00000011, .n = 4 },  \
  /* to EXIT1-IR         */ { .path = 0b00001011, .n = 4 },  \
  /* to PAUSE-IR         */ { .path = 0b00001011, .n = 5 },  \
  /* to EXIT2-IR         */ { .path = 0b00101011, .n = 6 },  \
  /* to UPDATE-IR        */ { .path = 0b00011011, .n = 5 }

#define FROM_SEL_DR_SCAN                        \
  /* to TEST_LOGIC_RESET */ { .path = 0b00000011, .n = 2 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000011, .n = 3 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00001011, .n = 4 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000000, .n = 1 },  \
  /* to SHIFT-DR         */ { .path = 0b00000000, .n = 2 },  \
  /* to EXIT1-DR         */ { .path = 0b00000010, .n = 2 },  \
  /* to PAUSE-DR         */ { .path = 0b00000010, .n = 3 },  \
  /* to EXIT2-DR         */ { .path = 0b00001010, .n = 4 },  \
  /* to UPDATE-DR        */ { .path = 0b00000110, .n = 3 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00000001, .n = 1 },  \
  /* to CAPTURE-IR       */ { .path = 0b00000001, .n = 2 },  \
  /* to SHIFT-IR         */ { .path = 0b00000001, .n = 3 },  \
  /* to EXIT1-IR         */ { .path = 0b00000101, .n = 3 },  \
  /* to PAUSE-IR         */ { .path = 0b00000101, .n = 4 },  \
  /* to EXIT2-IR         */ { .path = 0b00010101, .n = 5 },  \
  /* to UPDATE-IR        */ { .path = 0b00001101, .n = 4 }

#define FROM_CAPTURE_DR                         \
  /* to TEST_LOGIC_RESET */ { .path = 0b00011111, .n = 5 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000011, .n = 3 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000111, .n = 3 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000111, .n = 4 },  \
  /* to SHIFT-DR         */ { .path = 0b00000000, .n = 1 },  \
  /* to EXIT1-DR         */ { .path = 0b00000001, .n = 1 },  \
  /* to PAUSE-DR         */ { .path = 0b00000010, .n = 2 },  \
  /* to EXIT2-DR         */ { .path = 0b00000101, .n = 3 },  \
  /* to UPDATE-DR        */ { .path = 0b00000011, .n = 2 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00001111, .n = 4 },  \
  /* to CAPTURE-IR       */ { .path = 0b00001111, .n = 5 },  \
  /* to SHIFT-IR         */ { .path = 0b00001111, .n = 6 },  \
  /* to EXIT1-IR         */ { .path = 0b00101111, .n = 6 },  \
  /* to PAUSE-IR         */ { .path = 0b00101111, .n = 7 },  \
  /* to EXIT2-IR         */ { .path = 0b10101111, .n = 8 },  \
  /* to UPDATE-IR        */ { .path = 0b01101111, .n = 7 }

#define FROM_SHIFT_DR                           \
  /* to TEST_LOGIC_RESET */ { .path = 0b00011111, .n = 5 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000011, .n = 3 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000111, .n = 3 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000111, .n = 4 },  \
  /* to SHIFT-DR         */ { .path = 0b00000000, .n = 1 },  \
  /* to EXIT1-DR         */ { .path = 0b00000001, .n = 1 },  \
  /* to PAUSE-DR         */ { .path = 0b00000010, .n = 2 },  \
  /* to EXIT2-DR         */ { .path = 0b00000101, .n = 3 },  \
  /* to UPDATE-DR        */ { .path = 0b00000011, .n = 2 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00001111, .n = 4 },  \
  /* to CAPTURE-IR       */ { .path = 0b00001111, .n = 5 },  \
  /* to SHIFT-IR         */ { .path = 0b00001111, .n = 6 },  \
  /* to EXIT1-IR         */ { .path = 0b00101111, .n = 6 },  \
  /* to PAUSE-IR         */ { .path = 0b00101111, .n = 7 },  \
  /* to EXIT2-IR         */ { .path = 0b10101111, .n = 8 },  \
  /* to UPDATE-IR        */ { .path = 0b01101111, .n = 7 }

#define FROM_EXIT1_DR                           \
  /* to TEST_LOGIC_RESET */ { .path = 0b00001111, .n = 4 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000001, .n = 2 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000011, .n = 2 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000011, .n = 3 },  \
  /* to SHIFT-DR         */ { .path = 0b00000011, .n = 4 },  \
  /* to EXIT1-DR         */ { .path = 0b00001011, .n = 4 },  \
  /* to PAUSE-DR         */ { .path = 0b00000000, .n = 1 },  \
  /* to EXIT2-DR         */ { .path = 0b00000010, .n = 2 },  \
  /* to UPDATE-DR        */ { .path = 0b00000011, .n = 2 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00000111, .n = 3 },  \
  /* to CAPTURE-IR       */ { .path = 0b00000111, .n = 4 },  \
  /* to SHIFT-IR         */ { .path = 0b00000111, .n = 5 },  \
  /* to EXIT1-IR         */ { .path = 0b00010111, .n = 5 },  \
  /* to PAUSE-IR         */ { .path = 0b00010111, .n = 6 },  \
  /* to EXIT2-IR         */ { .path = 0b01010111, .n = 7 },  \
  /* to UPDATE-IR        */ { .path = 0b00110111, .n = 6 }

#define FROM_PAUSE_DR                           \
  /* to TEST_LOGIC_RESET */ { .path = 0b00011111, .n = 5 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000011, .n = 3 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000111, .n = 3 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000111, .n = 4 },  \
  /* to SHIFT-DR         */ { .path = 0b00000001, .n = 2 },  \
  /* to EXIT1-DR         */ { .path = 0b00000101, .n = 3 },  \
  /* to PAUSE-DR         */ { .path = 0b00000000, .n = 1 },  \
  /* to EXIT2-DR         */ { .path = 0b00000001, .n = 1 },  \
  /* to UPDATE-DR        */ { .path = 0b00000011, .n = 2 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00001111, .n = 4 },  \
  /* to CAPTURE-IR       */ { .path = 0b00001111, .n = 5 },  \
  /* to SHIFT-IR         */ { .path = 0b00001111, .n = 6 },  \
  /* to EXIT1-IR         */ { .path = 0b00101111, .n = 6 },  \
  /* to PAUSE-IR         */ { .path = 0b00101111, .n = 7 },  \
  /* to EXIT2-IR         */ { .path = 0b10101111, .n = 8 },  \
  /* to UPDATE-IR        */ { .path = 0b01101111, .n = 7 }

#define FROM_EXIT2_DR                           \
  /* to TEST_LOGIC_RESET */ { .path = 0b00001111, .n = 4 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000001, .n = 2 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000011, .n = 2 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000011, .n = 3 },  \
  /* to SHIFT-DR         */ { .path = 0b00000000, .n = 1 },  \
  /* to EXIT1-DR         */ { .path = 0b00000010, .n = 2 },  \
  /* to PAUSE-DR         */ { .path = 0b00000010, .n = 3 },  \
  /* to EXIT2-DR         */ { .path = 0b00001010, .n = 4 },  \
  /* to UPDATE-DR        */ { .path = 0b00000001, .n = 1 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00000111, .n = 3 },  \
  /* to CAPTURE-IR       */ { .path = 0b00000111, .n = 4 },  \
  /* to SHIFT-IR         */ { .path = 0b00000111, .n = 5 },  \
  /* to EXIT1-IR         */ { .path = 0b00010111, .n = 5 },  \
  /* to PAUSE-IR         */ { .path = 0b00010111, .n = 6 },  \
  /* to EXIT2-IR         */ { .path = 0b01010111, .n = 7 },  \
  /* to UPDATE-IR        */ { .path = 0b00110111, .n = 6 }

#define FROM_UPDATE_DR                          \
  /* to TEST_LOGIC_RESET */ { .path = 0b00000111, .n = 3 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000000, .n = 1 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000001, .n = 1 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000001, .n = 2 },  \
  /* to SHIFT-DR         */ { .path = 0b00000001, .n = 3 },  \
  /* to EXIT1-DR         */ { .path = 0b00000101, .n = 3 },  \
  /* to PAUSE-DR         */ { .path = 0b00000101, .n = 4 },  \
  /* to EXIT2-DR         */ { .path = 0b00010101, .n = 5 },  \
  /* to UPDATE-DR        */ { .path = 0b00001101, .n = 4 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00000011, .n = 2 },  \
  /* to CAPTURE-IR       */ { .path = 0b00000011, .n = 3 },  \
  /* to SHIFT-IR         */ { .path = 0b00000011, .n = 4 },  \
  /* to EXIT1-IR         */ { .path = 0b00001011, .n = 4 },  \
  /* to PAUSE-IR         */ { .path = 0b00001011, .n = 5 },  \
  /* to EXIT2-IR         */ { .path = 0b00101011, .n = 6 },  \
  /* to UPDATE-IR        */ { .path = 0b00011011, .n = 5 }

#define FROM_SEL_IR_SCAN                        \
  /* to TEST_LOGIC_RESET */ { .path = 0b00000001, .n = 1 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000001, .n = 2 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000101, .n = 3 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000101, .n = 4 },  \
  /* to SHIFT-DR         */ { .path = 0b00000101, .n = 5 },  \
  /* to EXIT1-DR         */ { .path = 0b00010101, .n = 5 },  \
  /* to PAUSE-DR         */ { .path = 0b00010101, .n = 6 },  \
  /* to EXIT2-DR         */ { .path = 0b01010101, .n = 7 },  \
  /* to UPDATE-DR        */ { .path = 0b00110101, .n = 6 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00001101, .n = 4 },  \
  /* to CAPTURE-IR       */ { .path = 0b00000000, .n = 1 },  \
  /* to SHIFT-IR         */ { .path = 0b00000000, .n = 2 },  \
  /* to EXIT1-IR         */ { .path = 0b00000010, .n = 2 },  \
  /* to PAUSE-IR         */ { .path = 0b00000010, .n = 3 },  \
  /* to EXIT2-IR         */ { .path = 0b00001010, .n = 4 },  \
  /* to UPDATE-IR        */ { .path = 0b00000110, .n = 3 }

#define FROM_CAPTURE_IR                         \
  /* to TEST_LOGIC_RESET */ { .path = 0b00011111, .n = 5 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000011, .n = 3 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000111, .n = 3 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000111, .n = 4 },  \
  /* to SHIFT-DR         */ { .path = 0b00000111, .n = 5 },  \
  /* to EXIT1-DR         */ { .path = 0b00010111, .n = 5 },  \
  /* to PAUSE-DR         */ { .path = 0b00010111, .n = 5 },  \
  /* to EXIT2-DR         */ { .path = 0b00010111, .n = 6 },  \
  /* to UPDATE-DR        */ { .path = 0b00110111, .n = 6 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00001111, .n = 4 },  \
  /* to CAPTURE-IR       */ { .path = 0b00001111, .n = 5 },  \
  /* to SHIFT-IR         */ { .path = 0b00000000, .n = 1 },  \
  /* to EXIT1-IR         */ { .path = 0b00000001, .n = 1 },  \
  /* to PAUSE-IR         */ { .path = 0b00000001, .n = 2 },  \
  /* to EXIT2-IR         */ { .path = 0b00000101, .n = 3 },  \
  /* to UPDATE-IR        */ { .path = 0b00000011, .n = 2 }

#define FROM_SHIFT_IR                           \
  /* to TEST_LOGIC_RESET */ { .path = 0b00011111, .n = 5 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000011, .n = 3 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000111, .n = 3 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000111, .n = 4 },  \
  /* to SHIFT-DR         */ { .path = 0b00000111, .n = 5 },  \
  /* to EXIT1-DR         */ { .path = 0b00010111, .n = 5 },  \
  /* to PAUSE-DR         */ { .path = 0b00010111, .n = 5 },  \
  /* to EXIT2-DR         */ { .path = 0b00010111, .n = 6 },  \
  /* to UPDATE-DR        */ { .path = 0b00110111, .n = 6 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00001111, .n = 4 },  \
  /* to CAPTURE-IR       */ { .path = 0b00001111, .n = 5 },  \
  /* to SHIFT-IR         */ { .path = 0b00000000, .n = 1 },  \
  /* to EXIT1-IR         */ { .path = 0b00000001, .n = 1 },  \
  /* to PAUSE-IR         */ { .path = 0b00000001, .n = 2 },  \
  /* to EXIT2-IR         */ { .path = 0b00000101, .n = 3 },  \
  /* to UPDATE-IR        */ { .path = 0b00000011, .n = 2 }

#define FROM_EXIT1_IR                           \
  /* to TEST_LOGIC_RESET */ { .path = 0b00001111, .n = 4 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000001, .n = 2 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000011, .n = 2 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000011, .n = 3 },  \
  /* to SHIFT-DR         */ { .path = 0b00000011, .n = 4 },  \
  /* to EXIT1-DR         */ { .path = 0b00001011, .n = 4 },  \
  /* to PAUSE-DR         */ { .path = 0b00001011, .n = 5 },  \
  /* to EXIT2-DR         */ { .path = 0b00101011, .n = 6 },  \
  /* to UPDATE-DR        */ { .path = 0b00011011, .n = 5 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00000111, .n = 3 },  \
  /* to CAPTURE-IR       */ { .path = 0b00000111, .n = 4 },  \
  /* to SHIFT-IR         */ { .path = 0b00000111, .n = 5 },  \
  /* to EXIT1-IR         */ { .path = 0b00010111, .n = 5 },  \
  /* to PAUSE-IR         */ { .path = 0b00000000, .n = 1 },  \
  /* to EXIT2-IR         */ { .path = 0b00000010, .n = 2 },  \
  /* to UPDATE-IR        */ { .path = 0b00000001, .n = 1 }

#define FROM_PAUSE_IR                           \
  /* to TEST_LOGIC_RESET */ { .path = 0b00011111, .n = 5 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000011, .n = 3 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000111, .n = 3 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000111, .n = 4 },  \
  /* to SHIFT-DR         */ { .path = 0b00000111, .n = 5 },  \
  /* to EXIT1-DR         */ { .path = 0b00010111, .n = 5 },  \
  /* to PAUSE-DR         */ { .path = 0b00010111, .n = 6 },  \
  /* to EXIT2-DR         */ { .path = 0b01010111, .n = 7 },  \
  /* to UPDATE-DR        */ { .path = 0b00110111, .n = 6 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00001111, .n = 4 },  \
  /* to CAPTURE-IR       */ { .path = 0b00001111, .n = 5 },  \
  /* to SHIFT-IR         */ { .path = 0b00000001, .n = 2 },  \
  /* to EXIT1-IR         */ { .path = 0b00000101, .n = 5 },  \
  /* to PAUSE-IR         */ { .path = 0b00000000, .n = 1 },  \
  /* to EXIT2-IR         */ { .path = 0b00000001, .n = 1 },  \
  /* to UPDATE-IR        */ { .path = 0b00000011, .n = 2 }

#define FROM_EXIT2_IR                           \
  /* to TEST_LOGIC_RESET */ { .path = 0b00001111, .n = 4 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000010, .n = 2 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000011, .n = 2 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000011, .n = 3 },  \
  /* to SHIFT-DR         */ { .path = 0b00000011, .n = 4 },  \
  /* to EXIT1-DR         */ { .path = 0b00001011, .n = 4 },  \
  /* to PAUSE-DR         */ { .path = 0b00001011, .n = 5 },  \
  /* to EXIT2-DR         */ { .path = 0b00101011, .n = 6 },  \
  /* to UPDATE-DR        */ { .path = 0b00011011, .n = 5 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00000111, .n = 3 },  \
  /* to CAPTURE-IR       */ { .path = 0b00000111, .n = 4 },  \
  /* to SHIFT-IR         */ { .path = 0b00000000, .n = 1 },  \
  /* to EXIT1-IR         */ { .path = 0b00000010, .n = 2 },  \
  /* to PAUSE-IR         */ { .path = 0b00000010, .n = 3 },  \
  /* to EXIT2-IR         */ { .path = 0b00001010, .n = 4 },  \
  /* to UPDATE-IR        */ { .path = 0b00000001, .n = 1 }

#define FROM_UPDATE_IR                          \
  /* to TEST_LOGIC_RESET */ { .path = 0b00000111, .n = 3 },  \
  /* to RUN_TEST_IDLE    */ { .path = 0b00000000, .n = 1 },  \
  /* to SEL-DR-SCAN      */ { .path = 0b00000001, .n = 1 },  \
  /* to CAPTURE-DR       */ { .path = 0b00000001, .n = 2 },  \
  /* to SHIFT-DR         */ { .path = 0b00000001, .n = 3 },  \
  /* to EXIT1-DR         */ { .path = 0b00000101, .n = 3 },  \
  /* to PAUSE-DR         */ { .path = 0b00000101, .n = 4 },  \
  /* to EXIT2-DR         */ { .path = 0b00010101, .n = 5 },  \
  /* to UPDATE-DR        */ { .path = 0b00001101, .n = 4 },  \
  /* to SEL-IR-SCAN      */ { .path = 0b00000011, .n = 2 },  \
  /* to CAPTURE-IR       */ { .path = 0b00000011, .n = 3 },  \
  /* to SHIFT-IR         */ { .path = 0b00000011, .n = 4 },  \
  /* to EXIT1-IR         */ { .path = 0b00001011, .n = 4 },  \
  /* to PAUSE-IR         */ { .path = 0b00001011, .n = 5 },  \
  /* to EXIT2-IR         */ { .path = 0b00101011, .n = 6 },  \
  /* to UPDATE-IR        */ { .path = 0b00011011, .n = 5 }


#if 1
struct jtag_state_path jtag_state_matrix[JTAG_STATE_COUNT][JTAG_STATE_COUNT] = {
  {FROM_TEST_LOGIC_RESET},
  {FROM_RUN_TEST_IDLE},
  {FROM_SEL_DR_SCAN},
  {FROM_CAPTURE_DR},
  {FROM_SHIFT_DR},
  {FROM_EXIT1_DR},
  {FROM_PAUSE_DR},
  {FROM_EXIT2_DR},
  {FROM_UPDATE_DR},
  {FROM_SEL_IR_SCAN},
  {FROM_CAPTURE_IR},
  {FROM_SHIFT_IR},
  {FROM_EXIT1_IR},
  {FROM_PAUSE_IR},
  {FROM_EXIT2_IR},
  {FROM_UPDATE_IR}
};
#endif

const struct jtag_state_path *jtag_get_state_path(jtag_state_t from,
  jtag_state_t to)
{
  if (from ==  JTAG_STATE_UNKNOWN || to == JTAG_STATE_UNKNOWN
      || from > JTAG_STATE_COUNT || to > JTAG_STATE_COUNT)
    return NULL;

  return &jtag_state_matrix[from - 1][to - 1];
}
