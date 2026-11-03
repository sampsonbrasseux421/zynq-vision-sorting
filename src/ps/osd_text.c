/*
 * osd_text.c —— 把文字写进 PL 的硬件字符 OSD
 *
 * 用法:  osd_text "行1|行2|行3|行4|行5|行6"
 *        每行最多 32 字符，不足补空格，最多 6 行
 *        OSD 字符缓冲基址 0x40001000，6*32=192 字节
 */
#define _GNU_SOURCE
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>

#define OSD_BASE   0x40001000UL
#define LINE_CHARS 32
#define MAX_LINES  6
#define BUF_BYTES  (LINE_CHARS * MAX_LINES)

int main(int argc, char **argv)
{
    char text[MAX_LINES][LINE_CHARS];
    int  n = 0, i, j;

    /* 全部先填空格 */
    memset(text, ' ', sizeof(text));

    if (argc < 2) {
        fprintf(stderr, "usage: %s \"line1|line2|...\"\n", argv[0]);
        return 2;
    }

    /* 按 '|' 切分 */
    const char *p = argv[1];
    while (*p && n < MAX_LINES) {
        i = 0;
        while (*p && *p != '|' && i < LINE_CHARS) text[n][i++] = *p++;
        while (*p && *p != '|') p++;          /* 超长部分丢弃 */
        if (*p == '|') p++;
        n++;
    }

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("/dev/mem"); return 1; }
    volatile uint8_t *base = (volatile uint8_t *)mmap(
        NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, OSD_BASE);
    if (base == MAP_FAILED) { perror("mmap"); close(fd); return 1; }

    for (j = 0; j < MAX_LINES; j++)
        for (i = 0; i < LINE_CHARS; i++)
            base[j * LINE_CHARS + i] = (uint8_t)text[j][i];

    munmap((void *)base, 4096);
    close(fd);
    printf("OK OSD %d lines\n", n);
    return 0;
}