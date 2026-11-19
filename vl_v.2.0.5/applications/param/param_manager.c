/**
 * @file    param_manager.c
 * @brief   参数存储引擎实现
 */

#include "param_manager.h"

#include <string.h>
#include <rtthread.h>

/* 扫描缓冲：必须是全局，不能放栈（记录最大 256B） */
static uint8_t s_scan_buf[PARAM_MAX_RECORD_SIZE];

/* ============================================================
 * 小工具
 * ============================================================ */
static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static uint32_t sector_end_addr(uint16_t sec)
{
    return PARAM_SEC_ADDR(sec) + PARAM_SECTOR_SIZE;
}

/* 记录完整性：head / tail / CRC32 */
static int record_is_valid(const param_config_t *cfg, const uint8_t *buf)
{
    uint32_t crc;

    if (rd32(buf + PARAM_REC_OFF_HEAD) != cfg->magic) {
        return 0;
    }
    if (rd32(buf + cfg->record_size - 4U) != PARAM_TAIL_OF(cfg->magic)) {
        return 0;
    }

    crc = Param_Crc32(buf, cfg->record_size - 8U);
    return (crc == rd32(buf + cfg->record_size - 8U)) ? 1 : 0;
}

/* ============================================================
 * 扫描
 * ============================================================ */
typedef struct {
    uint32_t addr;
    uint16_t sec;
    uint32_t seq;
    uint8_t  found;
} param_scan_t;

static void scan_sector(const param_config_t *cfg, uint16_t sec, param_scan_t *out)
{
    uint32_t addr = PARAM_SEC_ADDR(sec);
    uint32_t end  = sector_end_addr(sec);

    while ((addr + cfg->record_size) <= end) {
        uint32_t magic;

        Param_EfmRead(addr, (uint8_t *)&magic, 4U);
        if (magic == 0xFFFFFFFFU) {
            break;
        }

        if (magic == cfg->magic) {
            Param_EfmRead(addr, s_scan_buf, cfg->record_size);
            if (record_is_valid(cfg, s_scan_buf) != 0) {
                uint32_t seq = rd32(s_scan_buf + PARAM_REC_OFF_SEQ);

                if ((out->found == 0U) || (seq > out->seq)) {
                    out->found = 1U;
                    out->seq   = seq;
                    out->addr  = addr;
                    out->sec   = sec;
                }
            }
            addr += cfg->record_size;
        } else {
            /* 未命中 magic：按 4 字节滑动，容忍部分写入的残片 */
            addr += 4U;
        }
    }
}

/**
 * @brief 全扇区扫描，找到 seq 最大的有效记录
 */
static int scan_all(const param_config_t *cfg, param_runtime_t *rt, param_scan_t *out)
{
    uint16_t sec;

    memset(out, 0, sizeof(*out));

    /* 从最低扇区到最高扇区都扫；不假设写入顺序 */
    for (sec = cfg->sec_end; sec <= cfg->sec_start; sec++) {
        scan_sector(cfg, sec, out);
    }

    if (out->found == 0U) {
        return PARAM_ERR_NO_SPACE;
    }

    /* s_scan_buf 里是最后一条被扫描的候选，不一定是 seq 最大的那条，
     * 因此必须重新读一次最佳记录，再取它的 erase_count。 */
    Param_EfmRead(out->addr, s_scan_buf, cfg->record_size);

    rt->curr_sec   = out->sec;
    rt->curr_addr  = out->addr + cfg->record_size;
    rt->last_seq   = out->seq;
    rt->erase_count = rd32(s_scan_buf + PARAM_REC_OFF_ERASE);
    return PARAM_OK;
}

/* ============================================================
 * 初始化
 * ============================================================ */
int Param_Init(const param_config_t *cfg, param_runtime_t *rt,
               uint8_t *buf, param_defaults_fn pfn)
{
    param_scan_t found;
    int rc;

    if ((cfg == RT_NULL) || (rt == RT_NULL) || (buf == RT_NULL)) {
        return PARAM_ERR_PARAM;
    }
    if ((cfg->record_size == 0U) ||
        (cfg->record_size > PARAM_MAX_RECORD_SIZE) ||
        ((cfg->record_size % PARAM_RECORD_ALIGN) != 0U) ||
        (cfg->sec_end > cfg->sec_start)) {
        return PARAM_ERR_PARAM;
    }

    memset(rt, 0, sizeof(*rt));

    rc = scan_all(cfg, rt, &found);
    if (rc == PARAM_OK) {
        Param_EfmRead(found.addr, buf, cfg->record_size);
        rt->inited = 1U;
        rt->defaults_written = 0U;
        return PARAM_OK;
    }

    /* 无有效记录：填默认值并写入最高扇区 */
    memset(buf, 0, cfg->record_size);
    if (pfn != RT_NULL) {
        pfn(buf);
    }

    wr32(buf + PARAM_REC_OFF_HEAD, cfg->magic);
    wr32(buf + PARAM_REC_OFF_SEQ, 0U);
    wr32(buf + PARAM_REC_OFF_ERASE, 1U);
    wr32(buf + cfg->record_size - 8U, Param_Crc32(buf, cfg->record_size - 8U));
    wr32(buf + cfg->record_size - 4U, PARAM_TAIL_OF(cfg->magic));

    if (Param_EfmEraseSector(PARAM_SEC_ADDR(cfg->sec_start)) != PARAM_OK) {
        rt->inited = 1U;
        rt->defaults_written = 1U;
        return PARAM_ERR_IO;
    }
    if (Param_EfmWrite(PARAM_SEC_ADDR(cfg->sec_start), buf, cfg->record_size) != PARAM_OK) {
        rt->inited = 1U;
        rt->defaults_written = 1U;
        return PARAM_ERR_IO;
    }

    rt->curr_sec    = cfg->sec_start;
    rt->curr_addr   = PARAM_SEC_ADDR(cfg->sec_start) + cfg->record_size;
    rt->last_seq    = 0U;
    rt->erase_count = 1U;
    rt->write_count++;
    rt->inited = 1U;
    rt->defaults_written = 1U;
    return PARAM_OK;
}

int Param_Load(const param_config_t *cfg, param_runtime_t *rt, uint8_t *buf)
{
    param_scan_t found;
    int rc;

    if ((cfg == RT_NULL) || (rt == RT_NULL) || (buf == RT_NULL)) {
        return PARAM_ERR_PARAM;
    }

    rc = scan_all(cfg, rt, &found);
    if (rc != PARAM_OK) {
        return rc;
    }

    Param_EfmRead(found.addr, buf, cfg->record_size);
    rt->inited = 1U;
    rt->defaults_written = 0U;
    return PARAM_OK;
}

/* ============================================================
 * 写入
 * ============================================================ */
static int erase_current(const param_config_t *cfg, param_runtime_t *rt)
{
    (void)cfg;

    if (Param_EfmEraseSector(PARAM_SEC_ADDR(rt->curr_sec)) != PARAM_OK) {
        return PARAM_ERR_IO;
    }

    rt->curr_addr = PARAM_SEC_ADDR(rt->curr_sec);
    rt->erase_count++;
    return PARAM_OK;
}

static int rotate_sector(const param_config_t *cfg, param_runtime_t *rt)
{
    uint16_t next;

    if (cfg->sec_start == cfg->sec_end) {
        return erase_current(cfg, rt);
    }

    if (rt->curr_sec == cfg->sec_end) {
        next = cfg->sec_start;
    } else {
        next = (uint16_t)(rt->curr_sec - 1U);
    }

    /* 先擦成功再提交状态：擦除失败时保持原扇区/地址不变，
     * 后续重试仍指向旧扇区，不会写到未擦除的区域。 */
    if (Param_EfmEraseSector(PARAM_SEC_ADDR(next)) != PARAM_OK) {
        return PARAM_ERR_IO;
    }

    rt->curr_sec    = next;
    rt->curr_addr   = PARAM_SEC_ADDR(next);
    rt->erase_count = 1U;
    return PARAM_OK;
}

/** 保证当前扇区还有一条记录的空间；不够则换扇区/擦除 */
static int ensure_space(const param_config_t *cfg, param_runtime_t *rt)
{
    uint32_t end = sector_end_addr(rt->curr_sec);

    if ((rt->curr_addr + cfg->record_size) <= end) {
        return PARAM_OK;
    }

    /* 快区：擦除次数到限或写满都换扇区；慢区：原地擦除重写 */
    if (cfg->sec_start != cfg->sec_end) {
        return rotate_sector(cfg, rt);
    }
    return erase_current(cfg, rt);
}

int Param_Save(const param_config_t *cfg, param_runtime_t *rt, uint8_t *buf)
{
    int i;
    int rc;

    if ((cfg == RT_NULL) || (rt == RT_NULL) || (buf == RT_NULL) ||
        (rt->inited == 0U)) {
        return PARAM_ERR_PARAM;
    }

    /* 快区擦除次数到限时，即使还有空间也换扇区 */
    if ((cfg->max_erase_life != 0U) &&
        (rt->erase_count >= cfg->max_erase_life) &&
        (cfg->sec_start != cfg->sec_end)) {
        rc = rotate_sector(cfg, rt);
        if (rc != PARAM_OK) {
            return rc;
        }
    }

    rc = ensure_space(cfg, rt);
    if (rc != PARAM_OK) {
        return rc;
    }

    /* 头部信息 + 校验 */
    rt->last_seq++;
    wr32(buf + PARAM_REC_OFF_HEAD, cfg->magic);
    wr32(buf + PARAM_REC_OFF_SEQ, rt->last_seq);
    wr32(buf + PARAM_REC_OFF_ERASE, rt->erase_count);
    wr32(buf + cfg->record_size - 8U, Param_Crc32(buf, cfg->record_size - 8U));
    wr32(buf + cfg->record_size - 4U, PARAM_TAIL_OF(cfg->magic));

    /* 追加写；失败时顺延一个槽位重试（最多 8 次） */
    for (i = 0; i < 8; i++) {
        rc = Param_EfmWrite(rt->curr_addr, buf, cfg->record_size);
        if (rc == PARAM_OK) {
            rt->curr_addr += cfg->record_size;
            rt->write_count++;
            rt->defaults_written = 0U;    /* 已落盘真实记录 */
            return PARAM_OK;
        }

        rt->curr_addr += cfg->record_size;
        if ((rt->curr_addr + cfg->record_size) > sector_end_addr(rt->curr_sec)) {
            rc = ensure_space(cfg, rt);
            if (rc != PARAM_OK) {
                return rc;
            }
        }
    }

    return PARAM_ERR_VERIFY;
}

int Param_EraseAll(const param_config_t *cfg, param_runtime_t *rt, uint8_t *buf)
{
    uint16_t sec;

    if ((cfg == RT_NULL) || (rt == RT_NULL) || (buf == RT_NULL)) {
        return PARAM_ERR_PARAM;
    }

    for (sec = cfg->sec_end; sec <= cfg->sec_start; sec++) {
        if (Param_EfmEraseSector(PARAM_SEC_ADDR(sec)) != PARAM_OK) {
            return PARAM_ERR_IO;
        }
    }

    rt->curr_sec    = cfg->sec_start;
    rt->curr_addr   = PARAM_SEC_ADDR(cfg->sec_start);
    rt->last_seq    = 0U;
    rt->erase_count = 1U;
    rt->inited      = 1U;

    return Param_Save(cfg, rt, buf);
}

void Param_GetStats(const param_runtime_t *rt,
                    uint32_t *write_count, uint32_t *skip_count,
                    uint32_t *erase_count)
{
    if (rt == RT_NULL) {
        return;
    }
    if (write_count != RT_NULL) { *write_count = rt->write_count; }
    if (skip_count  != RT_NULL) { *skip_count  = rt->skip_count; }
    if (erase_count != RT_NULL) { *erase_count = rt->erase_count; }
}

void Param_GetStat(const param_config_t *cfg, const param_runtime_t *rt,
                   param_stat_t *out)
{
    uint32_t end;

    if ((cfg == RT_NULL) || (rt == RT_NULL) || (out == RT_NULL) ||
        (cfg->record_size == 0U)) {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->record_size      = cfg->record_size;
    out->sec_first        = cfg->sec_end;
    out->sec_last         = cfg->sec_start;
    out->slots_per_sec    = (uint16_t)(PARAM_SECTOR_SIZE / cfg->record_size);
    out->curr_sec         = rt->curr_sec;
    out->last_seq         = rt->last_seq;
    out->erase_count      = rt->erase_count;
    out->write_count      = rt->write_count;
    out->defaults_written = rt->defaults_written;

    end = PARAM_SEC_ADDR(rt->curr_sec) + PARAM_SECTOR_SIZE;
    if (rt->curr_addr < end) {
        out->slots_left = (uint16_t)((end - rt->curr_addr) / cfg->record_size);
    } else {
        out->slots_left = 0U;
    }
}
