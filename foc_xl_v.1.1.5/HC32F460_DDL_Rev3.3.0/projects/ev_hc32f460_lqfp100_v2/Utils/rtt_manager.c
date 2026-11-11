/*=============================================================================
 * rtt_manager.c - RTT 调试宏统一管理
 *
 * 本文件提供 Keil MDK 工程文件引用。
 * 非阻塞限频打印功能由 rtt_manager.h 中的宏实现。
 *============================================================================*/
#include "rtt_manager.h"
#include <stdarg.h>

// 整数转字符串
static int intToStr(char* buf, int val, int base) {
    if (val == 0) {
        buf[0] = '0';
        buf[1] = '\0';
        return 1;
    }

    char tmp[16];
    int i = 0, j = 0;
    unsigned int u = (base == 10 && val < 0) ? (unsigned int)(-val) : (unsigned int)val;

    while (u) {
        tmp[i++] = "0123456789abcdef"[u % base];
        u /= base;
    }
    if (base == 10 && val < 0) {
        tmp[i++] = '-';
    }

    while (i--) {
        buf[j++] = tmp[i];
    }
    buf[j] = '\0';
    return j;
}

// 格式化并输出
static void _rtt_do_print(const char* format, va_list args) {
    char buf[256];
    const char* p = format;
    int len = 0;

    while (*p && len < 255) {
        if (*p == '%') {
            p++;
            if (*p == '%') {
                buf[len++] = '%';
                p++;
                continue;
            }
            if (*p == 's') {
                const char* s = va_arg(args, const char*);
                while (*s && len < 255) {
                    buf[len++] = *s++;
                }
            } else if (*p == 'c') {
                buf[len++] = (char)va_arg(args, int);
            } else if (*p == 'd' || *p == 'i') {
                len += intToStr(buf + len, va_arg(args, int), 10);
            } else if (*p == 'u') {
                len += intToStr(buf + len, (int)va_arg(args, unsigned int), 10);
            } else if (*p == 'x') {
                len += intToStr(buf + len, (int)va_arg(args, unsigned int), 16);
            } else if (*p == 'X') {
                int start = len;
                len += intToStr(buf + len, (int)va_arg(args, unsigned int), 16);
                for (int i = start; i < len; i++) {
                    if (buf[i] >= 'a' && buf[i] <= 'f') {
                        buf[i] -= 32;
                    }
                }
            } else {
                buf[len++] = '%';
                buf[len++] = *p;
            }
        } else {
            buf[len++] = *p;
        }
        p++;
    }

    buf[len] = '\0';

    // 把 buf 作为 %s 参数传给 MAIN_D
    MAIN_D("%s", buf);
}

// 带间隔控制的打印
void _rtt_interval_print(RTT_IntervalPrinter_t* printer, const char* format, va_list args) {
    if (printer == NULL) {
        return;
    }

    // 不限速，直接输出
    if (printer->intervalMs == 0) {
        _rtt_do_print(format, args);
        return;
    }

    // 限速检查
    if (!printer->delay.isRunning) {
        // 首次启动，放行本次打印
        nbDelay_Start(&printer->delay);
    } else if (!nbDelay_IsComplete(&printer->delay)) {
        // 还没到时间，丢弃
        printer->skippedCount++;
        return;
    } else {
        // 时间到了，重启延时器
        nbDelay_Start(&printer->delay);
    }

    // 输出
    _rtt_do_print(format, args);
}

// 获取丢弃次数
uint32_t RTT_GetSkippedCount(RTT_IntervalPrinter_t* printer) {
    if (printer == NULL) {
        return 0;
    }
    return printer->skippedCount;
}

// 清除丢弃计数
void RTT_ClearSkippedCount(RTT_IntervalPrinter_t* printer) {
    if (printer != NULL) {
        printer->skippedCount = 0;
    }
}
