/*
 * zdt_wait.c 鈥斺€?绛?4 涓?ZDT 鐢垫満鍏ㄩ儴鍋滅ǔ锛堝埌浣嶆爣蹇楋級
 *
 * 鐢ㄦ硶: zdt_wait [瓒呮椂绉抅
 * 杈撳嚭: ARRIVED  鎴? TIMEOUT
 *
 * 鍘熺悊: 鐩存帴鎿嶄綔 UART FIFO锛?x42C00000锛夛紝姣忚疆璇?J1~J4 鐨勫綋鍓嶄綅缃紝
 *       杩炵画涓よ疆瀹屽叏涓€鑷?=> 鐢垫満宸插仠绋炽€? *       鍏ㄧ▼鏃犺繘绋嬪垱寤猴紝鍗曡疆绾?5ms銆? */
#define _GNU_SOURCE
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <limits.h>
#include <time.h>
#include <unistd.h>

#define UL_BASE 0x42C00000UL
static volatile uint32_t *g_ul;

static long now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000L + t.tv_nsec / 1000000L;
}

static void rx_reset(void) { g_ul[3] = 0x02; }

/* 鍙戦€?3 瀛楄妭 */
static int ul_send(const unsigned char *b, int n)
{
    for (int i = 0; i < n; i++) {
        long guard = 500000;
        while ((g_ul[2] & 0x08) && --guard) { }
        if (!guard) return -1;
        g_ul[1] = b[i];
    }
    return 0;
}

/* 璇?n 瀛楄妭锛岃秴鏃?ms */
static int ul_recv(unsigned char *b, int n, int timeout_ms)
{
    long t0 = now_ms();
    int got = 0;
    while (got < n) {
        if (g_ul[2] & 0x01) {
            b[got++] = (unsigned char)(g_ul[0] & 0xFF);
            t0 = now_ms();
        } else if (now_ms() - t0 > timeout_ms) {
            break;
        }
    }
    return got;
}

/* 璇绘煇涓酱鐨勫綋鍓嶄綅缃紝澶辫触杩斿洖 LONG_MIN */
static long read_pos(int addr)
{
    unsigned char tx[3], rx[16];
    /* 娓呮帀娈嬬暀 */
    while ((g_ul[2] & 0x01)) { (void)g_ul[0]; }
    rx_reset();

    tx[0] = (unsigned char)addr;
    tx[1] = 0x36;
    tx[2] = 0x6B;
    if (ul_send(tx, 3) < 0) return LONG_MIN;

    int n = ul_recv(rx, 8, 150);
    if (n < 8 || rx[7] != 0x6B) return LONG_MIN;

    long mag = ((long)(rx[2] & 0x7F) << 24) | ((long)rx[3] << 16) |
               ((long)rx[4] << 8) | (long)rx[5];
    return (rx[2] & 0x80) ? -mag : mag;
}

int main(int argc, char **argv)
{
    int timeout_s = (argc > 1) ? atoi(argv[1]) : 25;
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("/dev/mem"); return 1; }
    void *p = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, UL_BASE);
    if (p == MAP_FAILED) { perror("mmap"); return 1; }
    g_ul = (volatile uint32_t *)p;

    long prev[4] = {LONG_MIN, LONG_MIN, LONG_MIN, LONG_MIN};
    int have_prev = 0;
    long t0 = now_ms();

    while (now_ms() - t0 < timeout_s * 1000L) {
        long cur[4];
        int ok = 1;
        for (int a = 0; a < 4; a++) {
            cur[a] = LONG_MIN;
            for (int try = 0; try < 3 && cur[a] == LONG_MIN; try++) {
                cur[a] = read_pos(a + 1);
                if (cur[a] == LONG_MIN) usleep(8000);
            }
            if (cur[a] == LONG_MIN) ok = 0;
            usleep(6000);            /* 关节之间留空隙，避免串口冲突 */
        }
        if (ok && have_prev) {
            int same = 1;
            for (int a = 0; a < 4; a++)
                if (cur[a] != prev[a]) { same = 0; break; }
            if (same) { printf("ARRIVED\n"); return 0; }
        }
        if (ok) {
            memcpy(prev, cur, sizeof(prev));
            have_prev = 1;
        }
        usleep(20000);   /* 20ms 閲囨牱闂撮殧 */
    }
    printf("TIMEOUT\n");
    return 0;
}
