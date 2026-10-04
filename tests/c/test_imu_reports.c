/* Host-side test for firmware/reflex/Core/Src/imu_reports.c (pure logic).
 * Packets are built by hand from the layout in CEVA's SH-2 source (see the
 * header) — NOT captured from a real BNO086. Built and run by
 * tests/test_c_firmware_logic.py. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "imu_reports.h"

static int s_failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            s_failures++;                                                    \
        }                                                                    \
    } while (0)

#define NEAR(a, b, tol) (fabsf((float)(a) - (float)(b)) < (tol))

typedef struct {
    ImuReport reports[8];
    int count;
} Collected;

static void collect(const ImuReport *r, void *ctx)
{
    Collected *c = (Collected *)ctx;
    if (c->count < 8) {
        c->reports[c->count++] = *r;
    }
}

static int put_i16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v & 0xFFU);
    p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFFU);
    return 2;
}

static int put_base_timestamp(uint8_t *p, int32_t timebase)
{
    p[0] = 0xFB;
    uint32_t u = (uint32_t)timebase;
    p[1] = (uint8_t)(u & 0xFF);
    p[2] = (uint8_t)((u >> 8) & 0xFF);
    p[3] = (uint8_t)((u >> 16) & 0xFF);
    p[4] = (uint8_t)((u >> 24) & 0xFF);
    return 5;
}

/* [id][seq][status][delay] header, then caller appends data */
static int put_header(uint8_t *p, uint8_t id, uint8_t accuracy, uint16_t delay)
{
    p[0] = id;
    p[1] = 0;
    p[2] = (uint8_t)((accuracy & 0x03U) | ((delay >> 6) & 0xFCU));
    p[3] = (uint8_t)(delay & 0xFFU);
    return 4;
}

static int put_grv(uint8_t *p, int16_t i, int16_t j, int16_t k, int16_t real, uint8_t acc, uint16_t delay)
{
    int n = put_header(p, 0x08, acc, delay);
    n += put_i16(p + n, i);
    n += put_i16(p + n, j);
    n += put_i16(p + n, k);
    n += put_i16(p + n, real);
    return n;  /* 12 */
}

static int put_vec3(uint8_t *p, uint8_t id, int16_t x, int16_t y, int16_t z, uint8_t acc, uint16_t delay)
{
    int n = put_header(p, id, acc, delay);
    n += put_i16(p + n, x);
    n += put_i16(p + n, y);
    n += put_i16(p + n, z);
    return n;  /* 10 */
}

static void test_game_rotation_vector_alone(void)
{
    uint8_t pkt[32];
    int n = put_base_timestamp(pkt, 0);
    n += put_grv(pkt + n, 0, 0, 0, 16384, 3, 0);   /* identity: real = 1.0 in Q14 */
    CHECK(n == 17);

    Collected c = {0};
    CHECK(ImuReports_Parse(pkt, (uint16_t)n, collect, &c) == IMU_PARSE_OK);
    CHECK(c.count == 1);
    CHECK(c.reports[0].report_id == IMU_REPORT_GAME_ROTATION_VECTOR);
    CHECK(c.reports[0].accuracy == 3);
    CHECK(NEAR(c.reports[0].v[3], 1.0f, 1e-6f));
    CHECK(NEAR(c.reports[0].v[0], 0.0f, 1e-6f));
}

static void test_q14_scaling_and_sign(void)
{
    uint8_t pkt[32];
    int n = put_base_timestamp(pkt, 0);
    n += put_grv(pkt + n, -8192, 4096, 0, 11585, 2, 0);  /* -0.5, 0.25, 0, ~0.7071 */
    Collected c = {0};
    CHECK(ImuReports_Parse(pkt, (uint16_t)n, collect, &c) == IMU_PARSE_OK);
    CHECK(NEAR(c.reports[0].v[0], -0.5f, 1e-6f));
    CHECK(NEAR(c.reports[0].v[1], 0.25f, 1e-6f));
    CHECK(NEAR(c.reports[0].v[3], 0.70709f, 1e-4f));
}

static void test_several_reports_in_one_packet(void)
{
    /* the whole point: gyro + linear accel + rotation vector after ONE
     * base timestamp, as the sensor hub batches them. The old driver read
     * only the first report in a packet. */
    uint8_t pkt[64];
    int n = put_base_timestamp(pkt, 100);
    n += put_vec3(pkt + n, IMU_REPORT_GYROSCOPE, 512, -1024, 256, 3, 0);       /* 1, -2, 0.5 rad/s */
    n += put_vec3(pkt + n, IMU_REPORT_LINEAR_ACCELERATION, 256, -256, 2509, 2, 0);
    n += put_grv(pkt + n, 0, 0, 0, 16384, 3, 0);
    CHECK(n == 5 + 10 + 10 + 12);

    Collected c = {0};
    CHECK(ImuReports_Parse(pkt, (uint16_t)n, collect, &c) == IMU_PARSE_OK);
    CHECK(c.count == 3);
    CHECK(c.reports[0].report_id == IMU_REPORT_GYROSCOPE);
    CHECK(NEAR(c.reports[0].v[0], 1.0f, 1e-6f));    /* Q9: 512 -> 1 rad/s */
    CHECK(NEAR(c.reports[0].v[1], -2.0f, 1e-6f));
    CHECK(NEAR(c.reports[0].v[2], 0.5f, 1e-6f));
    CHECK(c.reports[1].report_id == IMU_REPORT_LINEAR_ACCELERATION);
    CHECK(NEAR(c.reports[1].v[0], 1.0f, 1e-6f));    /* Q8: 256 -> 1 m/s^2 */
    CHECK(NEAR(c.reports[1].v[2], 9.80078f, 1e-4f));
    CHECK(c.reports[2].report_id == IMU_REPORT_GAME_ROTATION_VECTOR);
}

static void test_delay_and_timebase_give_the_sample_time_offset(void)
{
    /* sample time = INT time + (-timebase + delay) * 100 us */
    uint8_t pkt[32];
    int n = put_base_timestamp(pkt, 250);              /* 25 ms */
    n += put_grv(pkt + n, 0, 0, 0, 16384, 3, 100);     /* delay 10 ms */
    Collected c = {0};
    ImuReports_Parse(pkt, (uint16_t)n, collect, &c);
    CHECK(c.reports[0].delay_100us == -250 + 100);

    /* the delay field is 14 bits wide: the upper six bits live in the
     * status byte */
    n = put_base_timestamp(pkt, 0);
    n += put_grv(pkt + n, 0, 0, 0, 16384, 1, 5000);
    c.count = 0;
    ImuReports_Parse(pkt, (uint16_t)n, collect, &c);
    CHECK(c.reports[0].delay_100us == 5000);
    CHECK(c.reports[0].accuracy == 1);                  /* accuracy not corrupted by the delay bits */
}

static void test_timestamp_rebase_adjusts_following_reports(void)
{
    uint8_t pkt[48];
    int n = put_base_timestamp(pkt, 100);
    n += put_vec3(pkt + n, IMU_REPORT_GYROSCOPE, 0, 0, 0, 3, 0);
    pkt[n] = 0xFA;                                      /* rebase: +40 */
    pkt[n + 1] = 40; pkt[n + 2] = 0; pkt[n + 3] = 0; pkt[n + 4] = 0;
    n += 5;
    n += put_vec3(pkt + n, IMU_REPORT_GYROSCOPE, 0, 0, 0, 3, 0);
    Collected c = {0};
    CHECK(ImuReports_Parse(pkt, (uint16_t)n, collect, &c) == IMU_PARSE_OK);
    CHECK(c.count == 2);
    CHECK(c.reports[0].delay_100us == -100);
    CHECK(c.reports[1].delay_100us == -100 + 40);
}

static void test_unwanted_known_reports_are_skipped_by_length(void)
{
    uint8_t pkt[64];
    int n = put_base_timestamp(pkt, 0);
    n += put_vec3(pkt + n, 0x03, 1, 2, 3, 3, 0);        /* magnetometer: skipped, 10 bytes */
    n += put_vec3(pkt + n, IMU_REPORT_GYROSCOPE, 512, 0, 0, 3, 0);
    Collected c = {0};
    CHECK(ImuReports_Parse(pkt, (uint16_t)n, collect, &c) == IMU_PARSE_OK);
    CHECK(c.count == 1);
    CHECK(c.reports[0].report_id == IMU_REPORT_GYROSCOPE);
    CHECK(NEAR(c.reports[0].v[0], 1.0f, 1e-6f));
}

static void test_truncated_packet_delivers_what_it_can(void)
{
    uint8_t pkt[64];
    int n = put_base_timestamp(pkt, 0);
    n += put_vec3(pkt + n, IMU_REPORT_GYROSCOPE, 512, 0, 0, 3, 0);
    n += put_grv(pkt + n, 0, 0, 0, 16384, 3, 0);
    Collected c = {0};
    CHECK(ImuReports_Parse(pkt, (uint16_t)(n - 3), collect, &c) == IMU_PARSE_TRUNCATED);
    CHECK(c.count == 1);                                 /* the gyro report before the cut survived */
}

static void test_unknown_report_id_stops_parsing(void)
{
    uint8_t pkt[32];
    int n = put_base_timestamp(pkt, 0);
    pkt[n++] = 0x7E;                                     /* not a report id we know */
    pkt[n++] = 0;
    n += put_vec3(pkt + n, IMU_REPORT_GYROSCOPE, 512, 0, 0, 3, 0);
    Collected c = {0};
    CHECK(ImuReports_Parse(pkt, (uint16_t)n, collect, &c) == IMU_PARSE_UNKNOWN_ID);
    CHECK(c.count == 0);                                 /* cannot know where the next report starts */
}

static void test_empty_and_tiny_payloads(void)
{
    Collected c = {0};
    CHECK(ImuReports_Parse((const uint8_t *)"", 0, collect, &c) == IMU_PARSE_OK);
    uint8_t one[1] = {0xFB};
    CHECK(ImuReports_Parse(one, 1, collect, &c) == IMU_PARSE_TRUNCATED);
    CHECK(c.count == 0);
    CHECK(ImuReports_Parse(one, 1, 0, 0) == IMU_PARSE_TRUNCATED);  /* null sink is fine */
}

int main(void)
{
    test_game_rotation_vector_alone();
    test_q14_scaling_and_sign();
    test_several_reports_in_one_packet();
    test_delay_and_timebase_give_the_sample_time_offset();
    test_timestamp_rebase_adjusts_following_reports();
    test_unwanted_known_reports_are_skipped_by_length();
    test_truncated_packet_delivers_what_it_can();
    test_unknown_report_id_stops_parsing();
    test_empty_and_tiny_payloads();

    if (s_failures == 0) {
        printf("imu_reports: all checks passed\n");
        return 0;
    }
    printf("imu_reports: %d check(s) FAILED\n", s_failures);
    return 1;
}
