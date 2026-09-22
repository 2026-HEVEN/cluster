#include <unity.h>
#include "../../src/core/board_pins.h"

void setUp() {}
void tearDown() {}

void test_pcb_v3_cluster_pin_contract() {
    TEST_ASSERT_EQUAL_INT(23, board_pins::LCD_CS);
    TEST_ASSERT_EQUAL_INT(22, board_pins::LCD_RST);
    TEST_ASSERT_EQUAL_INT(21, board_pins::LCD_DC);
    TEST_ASSERT_EQUAL_INT(17, board_pins::SPI_MOSI);
    TEST_ASSERT_EQUAL_INT(18, board_pins::SPI_SCK);
    TEST_ASSERT_EQUAL_INT(35, board_pins::SPI_MISO);
    TEST_ASSERT_EQUAL_INT(16, board_pins::TOUCH_CS);
    TEST_ASSERT_EQUAL_INT(14, board_pins::CAN_RX);
    TEST_ASSERT_EQUAL_INT(13, board_pins::CAN_TX);
    TEST_ASSERT_EQUAL_INT(25, board_pins::GNSS_RX);
    TEST_ASSERT_EQUAL_INT(26, board_pins::GNSS_TX);
    TEST_ASSERT_EQUAL_INT(27, board_pins::GNSS_PPS);
    TEST_ASSERT_EQUAL_INT(4, board_pins::VESS_PWM);
    TEST_ASSERT_EQUAL_INT(32, board_pins::TV_SWITCH);
    TEST_ASSERT_EQUAL_INT(33, board_pins::PADDOCK_SWITCH);
    TEST_ASSERT_EQUAL_INT(19, board_pins::HOME_BUTTON);
    TEST_ASSERT_EQUAL_INT(5, board_pins::LAP_BUTTON);
    TEST_ASSERT_EQUAL_INT(36, board_pins::REGEN_BIT0);
    TEST_ASSERT_EQUAL_INT(39, board_pins::REGEN_BIT1);
    TEST_ASSERT_EQUAL_INT(34, board_pins::START_SENSE_ADC);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_pcb_v3_cluster_pin_contract);
    return UNITY_END();
}
