#include <unity.h>
#include "wheel_speed_logic.h"

namespace {
constexpr WheelSpeedConfig TEST_CONFIG { 0.4597f };

void put_u16le(uint8_t *data, uint16_t value) {
    data[0] = (uint8_t)(value & 0xFF);
    data[1] = (uint8_t)(value >> 8);
}

void make_frame(uint8_t data[8], uint16_t fl, uint16_t fr,
                uint16_t rl, uint16_t rr) {
    put_u16le(data + 0, fl);
    put_u16le(data + 2, fr);
    put_u16le(data + 4, rl);
    put_u16le(data + 6, rr);
}
}

void test_decodes_all_four_rpm_values(void) {
    uint8_t data[8];
    make_frame(data, 100, 200, 300, 400);

    const FourWheelSpeed result = decode_wheel_speed_frame(data, TEST_CONFIG);

    TEST_ASSERT_EQUAL_UINT16(100, result.rpm_fl);
    TEST_ASSERT_EQUAL_UINT16(200, result.rpm_fr);
    TEST_ASSERT_EQUAL_UINT16(300, result.rpm_rl);
    TEST_ASSERT_EQUAL_UINT16(400, result.rpm_rr);
}

void test_converts_each_rpm_to_kph(void) {
    uint8_t data[8];
    make_frame(data, 100, 200, 300, 400);

    const FourWheelSpeed result = decode_wheel_speed_frame(data, TEST_CONFIG);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 8.665f, result.kph_fl);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 17.330f, result.kph_fr);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.995f, result.kph_rl);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 34.660f, result.kph_rr);
}

void test_vehicle_speed_uses_middle_two_wheels(void) {
    uint8_t data[8];
    make_frame(data, 100, 200, 300, 1000);

    const FourWheelSpeed result = decode_wheel_speed_frame(data, TEST_CONFIG);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 21.6625f, result.vehicle_kph);
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_decodes_all_four_rpm_values);
    RUN_TEST(test_converts_each_rpm_to_kph);
    RUN_TEST(test_vehicle_speed_uses_middle_two_wheels);
    return UNITY_END();
}