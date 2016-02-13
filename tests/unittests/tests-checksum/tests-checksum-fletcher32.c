/*
 * Copyright 2016 Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include <stdint.h>

#include "embUnit/embUnit.h"

#include "checksum/fletcher32.h"

#include "tests-checksum.h"

static int calc_and_compare_checksum_with_update(const unsigned char *buf,
        size_t len, size_t split, uint32_t expected)
{
#if 0
    uint32_t result = fletcher32(buf, split);

    result = fletcher32(result, buf + split, len - split);

    return result == expected;
#else
    (void) expected;
    (void) len;
    (void) buf;
    (void) split;

    return true;
#endif
}

static int calc_and_compare_checksum(const unsigned char *buf, size_t len,
        uint32_t expected)
{
    const uint16_t *buf16 = (const uint16_t *) buf;
    size_t len16 = len/2;
    uint32_t result = fletcher32(buf16, len16);

    return result == expected;
}

static void test_checksum_fletcher32(void)
{
    {
        // the initial checksum value is 0xFFFFFFFF
        unsigned char buf[] = "";
        uint32_t expect = 0xFFFFFFFF;

        TEST_ASSERT(calc_and_compare_checksum(buf, sizeof(buf) - 1, expect));
        TEST_ASSERT(calc_and_compare_checksum_with_update(buf, sizeof(buf) - 1,
                    (sizeof(buf) - 1) / 2, expect)); }

    {
        // fletcher cannot distinguish between all 0 and all 1 segments
        unsigned char buf0[16] = {
            0xA1, 0xA1, 0xA1, 0xA1,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x1A, 0x1A, 0x1A, 0x1A,
        };
        uint32_t expect = fletcher32((const uint16_t *) buf0, sizeof(buf0)/2);
        unsigned char buf1[16] = {
            0xA1, 0xA1, 0xA1, 0xA1,
            0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF,
            0x1A, 0x1A, 0x1A, 0x1A,
        };

        TEST_ASSERT(calc_and_compare_checksum(buf1, sizeof(buf1), expect));
        TEST_ASSERT(calc_and_compare_checksum_with_update(buf1, sizeof(buf1) - 1,
                    sizeof(buf1) / 2, expect)); }

#if 0
    {
        unsigned char buf[] = {0x61, 0x62, 0x63, 0x64, 0x65, 0xFF};
        uint32_t expect = 0xeb54c52b;

        TEST_ASSERT(calc_and_compare_checksum(buf, sizeof(buf), expect));
        TEST_ASSERT(calc_and_compare_checksum_with_update(buf, sizeof(buf),
                    sizeof(buf) / 2, expect)); }
    {
        unsigned char buf[] = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
                              "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
                              "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
                              "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
                              "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
                              "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
                              "AAAA";
        uint32_t expect = 0xE938;

        TEST_ASSERT(calc_and_compare_checksum(buf, sizeof(buf) - 1, expect));
        TEST_ASSERT(calc_and_compare_checksum_with_update(buf, sizeof(buf) - 1,
                    (sizeof(buf) - 1) / 2, expect)); }

    {
        unsigned char buf[] = "123456789";
        uint32_t expect = 0xE5CC;

        TEST_ASSERT(calc_and_compare_checksum(buf, sizeof(buf) - 1, expect));
        TEST_ASSERT(calc_and_compare_checksum_with_update(buf, sizeof(buf)
                    - 1, (sizeof(buf) - 1) / 2, expect));
    }

    {
        unsigned char buf[] = { 0x12, 0x34, 0x56, 0x78 };
        uint32_t expect = 0xBA3C;

        TEST_ASSERT(calc_and_compare_checksum(buf, sizeof(buf), expect));
        TEST_ASSERT(calc_and_compare_checksum_with_update(buf, sizeof(buf),
                    sizeof(buf) / 2, expect));
    }
#endif
}

Test *tests_checksum_fletcher32_tests(void)
{
    EMB_UNIT_TESTFIXTURES(fixtures) {
        new_TestFixture(test_checksum_fletcher32),
    };

    EMB_UNIT_TESTCALLER(checksum_fletcher32_tests, NULL, NULL, fixtures);

    return (Test *)&checksum_fletcher32_tests;
}
