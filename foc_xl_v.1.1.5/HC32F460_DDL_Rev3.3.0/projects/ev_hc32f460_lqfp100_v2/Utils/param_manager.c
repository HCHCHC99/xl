#include "param_manager.h"
#include <string.h>

#define SECTOR_SIZE         0x2000
#define SEC_START           62
#define SEC_END             56
#define MAX_ERASE_LIFE      10000
#define MAX_WRITE_RETRY     7

#define PARAM_ENTER_CRITICAL()    __disable_irq()
#define PARAM_EXIT_CRITICAL()     __enable_irq()

Param_Debug_t    g_ParamDebug;

static uint16_t s_curr_sec = SEC_START;
static uint32_t s_curr_addr = 0;

/*=============================================================================
 * 内部辅助函数
 *============================================================================*/

/**
 * @brief  从参数缓冲区中读取一个 uint32_t 字段
 */
static uint32_t GetField32(const void *pBuf, uint32_t offset)
{
    return *(const uint32_t*)((const uint8_t*)pBuf + offset);
}

/**
 * @brief  向参数缓冲区中写入一个 uint32_t 字段
 */
static void SetField32(void *pBuf, uint32_t offset, uint32_t value)
{
    *(uint32_t*)((uint8_t*)pBuf + offset) = value;
}

/**
 * @brief  计算参数结构体的 CRC32 校验值
 * @note   使用硬件 CRC32 计算
 *         校验范围：从结构体开头到 checksum 字段之前的所有数据
 */
static uint32_t CalcParamCRC(const void *pBuf, uint32_t paramSize, uint32_t checksumOffset)
{
    uint32_t crc_result = 0;
    uint32_t word_len;
    stc_crc_init_t stcCrcInit;

    if (pBuf == NULL)
    {
        return 0;
    }

    PARAM_ENTER_CRITICAL();

    /* 计算需要校验的字数（32-bit word）：从开头到 checksum 字段之前 */
    word_len = checksumOffset / 4;

    /* 配置 CRC：标准 CRC-32 */
    stcCrcInit.u32Protocol = CRC_CRC32;
    stcCrcInit.u32InitValue = 0xFFFFFFFFUL;
    stcCrcInit.u32RefIn = CRC_REFIN_ENABLE;
    stcCrcInit.u32RefOut = CRC_REFOUT_ENABLE;
    stcCrcInit.u32XorOut = CRC_XOROUT_ENABLE;

    CRC_Init(&stcCrcInit);
    CRC_CRC32_AccumulateData(CRC_DATA_WIDTH_32BIT, (const uint32_t*)pBuf, word_len, &crc_result);

    PARAM_EXIT_CRITICAL();

    PARAM_DEBUG("CalcParamCRC: word_len=%lu, result=0x%08lX\r\n", word_len, crc_result);

    return crc_result;
}

static void UpdateDebugInfo(int32_t res, uint32_t paramSize)
{
    PARAM_ENTER_CRITICAL();
    g_ParamDebug.curr_sec = s_curr_sec;
    g_ParamDebug.curr_addr = s_curr_addr;
    g_ParamDebug.next_addr = s_curr_addr + paramSize;
    g_ParamDebug.last_res = res;
    PARAM_EXIT_CRITICAL();
}

static int32_t Internal_Erase(uint32_t address)
{
    HC32FLASH_STATUS status;

    PARAM_ENTER_CRITICAL();
    status = HC32FLASH_EraseSector(address);
    PARAM_EXIT_CRITICAL();

    return (status == HC32FLASH_OK) ? PARAM_OK : PARAM_ERR;
}

static uint32_t Internal_ReadWord(uint32_t addr)
{
    uint32_t data;
    PARAM_ENTER_CRITICAL();
    data = HC32FLASH_ReaddWord(addr);
    PARAM_EXIT_CRITICAL();
    return data;
}

static int32_t Internal_WriteWord(uint32_t addr, uint32_t data)
{
    HC32FLASH_STATUS status;

    PARAM_ENTER_CRITICAL();
    status = HC32FLASH_WritedWord_Check(addr, data);
    PARAM_EXIT_CRITICAL();

    return (status == HC32FLASH_OK) ? PARAM_OK : PARAM_ERR;
}

static int32_t Internal_WriteBuffer(uint32_t addr, const uint32_t* buffer, uint32_t word_count)
{
    uint32_t i;
    for (i = 0; i < word_count; i++)
    {
        if (Internal_WriteWord(addr + i * 4, buffer[i]) != PARAM_OK)
        {
            return PARAM_ERR;
        }
    }
    return PARAM_OK;
}

static bool Internal_VerifyBuffer(uint32_t addr, const uint32_t* buffer, uint32_t word_count)
{
    uint32_t i;
    for (i = 0; i < word_count; i++)
    {
        if (Internal_ReadWord(addr + i * 4) != buffer[i])
        {
            return false;
        }
    }
    return true;
}

/*=============================================================================
 * 公开接口实现
 *============================================================================*/

int32_t Param_Init(const Param_Config_t *pConfig, void (*pSetDefaults)(void))
{
    uint32_t max_seq = 0;
    uint32_t best_addr = 0;
    uint16_t best_sec = SEC_START;
    bool found = false;
    int s;
    uint32_t paramWords;
    uint32_t headMagic;
    uint32_t tailMagic;
    uint32_t seq;
    uint32_t calcCrc;
    uint32_t storedCrc;

    if ((pConfig == NULL) || (pConfig->pParamBuf == NULL) || (pSetDefaults == NULL))
    {
        return PARAM_ERR_INVD_PARAM;
    }

    paramWords = pConfig->paramSize / 4;

    PARAM_DEBUG("Param init start, size=%lu\r\n", pConfig->paramSize);

    for (s = SEC_START; s >= SEC_END; s--)
    {
        uint32_t addr = s * SECTOR_SIZE;
        uint32_t sector_end = (s + 1) * SECTOR_SIZE;

        while (addr + pConfig->paramSize <= sector_end)
        {
            headMagic = Internal_ReadWord(addr);

            if (headMagic == pConfig->magicHead)
            {
                tailMagic = Internal_ReadWord(addr + pConfig->paramSize - 4);
                seq = Internal_ReadWord(addr + pConfig->seqOffset);

                PARAM_DEBUG("  Found block at 0x%08lX: head=0x%08lX, seq=%lu, tail=0x%08lX\r\n",
                           addr, headMagic, seq, tailMagic);

                if (tailMagic == pConfig->magicTail)
                {
                    /* 临时加载到内存进行 CRC 校验 */
                    uint8_t tempBuf[256];
                    uint32_t i;
                    uint32_t *pDest = (uint32_t*)tempBuf;

                    for (i = 0; i < paramWords; i++)
                    {
                        pDest[i] = Internal_ReadWord(addr + i * 4);
                    }

                    /* 计算 CRC 并验证 */
                    calcCrc = CalcParamCRC(tempBuf, pConfig->paramSize, pConfig->checksumOffset);
                    storedCrc = GetField32(tempBuf, pConfig->checksumOffset);

                    PARAM_DEBUG("    CRC: stored=0x%08lX, calculated=0x%08lX\r\n",
                               storedCrc, calcCrc);

                    if (calcCrc == storedCrc)
                    {
                        PARAM_DEBUG("    -> VALID block (CRC passed)\r\n");
                        if (!found || seq > max_seq)
                        {
                            max_seq = seq;
                            best_addr = addr;
                            best_sec = (uint16_t)s;
                            found = true;
                            PARAM_DEBUG("    -> New best: seq=%lu, addr=0x%08lX\r\n", max_seq, best_addr);
                        }
                    }
                    else
                    {
                        PARAM_DEBUG("    -> INVALID block (CRC mismatch), skipping\r\n");
                    }
                }
                else
                {
                    PARAM_DEBUG("    -> INVALID block (tail mismatch: 0x%08lX)\r\n", tailMagic);
                }
                addr += pConfig->paramSize;
            }
            else if (headMagic == 0xFFFFFFFF)
            {
                PARAM_DEBUG("  Hit empty area at 0x%08lX, stop scanning sector\r\n", addr);
                break;
            }
            else
            {
                addr += 4;
            }
        }
    }

    if (found)
    {
        /* 加载找到的最新的块到内存 */
        uint32_t *pDest = (uint32_t*)pConfig->pParamBuf;
        uint32_t i;
        for (i = 0; i < paramWords; i++)
        {
            pDest[i] = Internal_ReadWord(best_addr + i * 4);
        }

        s_curr_sec = best_sec;
        s_curr_addr = best_addr;

        PARAM_DEBUG("Param load SUCCESS: seq=%lu, addr=0x%08lX, sec=%lu\r\n",
                   max_seq, best_addr, best_sec);
    }
    else
    {
        PARAM_DEBUG("No valid param block found, using defaults and writing to Flash\r\n");
        pSetDefaults();
        Internal_Erase(SEC_START * SECTOR_SIZE);
        s_curr_sec = SEC_START;
        s_curr_addr = SEC_START * SECTOR_SIZE;
        SetField32(pConfig->pParamBuf, pConfig->seqOffset, 0);
        SetField32(pConfig->pParamBuf, pConfig->eraseCntOffset, 1);

        /* 立即保存默认参数到 Flash */
        Param_Save(pConfig);

        PARAM_DEBUG("Defaults saved to Flash at addr=0x%08lX\r\n", s_curr_addr);
    }

    UpdateDebugInfo(PARAM_OK, pConfig->paramSize);
    return PARAM_OK;
}

int32_t Param_Save(const Param_Config_t *pConfig)
{
    uint8_t retry = 0;
    uint32_t write_addr;
    uint32_t paramWords;
    uint32_t seq;

    if ((pConfig == NULL) || (pConfig->pParamBuf == NULL))
    {
        return PARAM_ERR_INVD_PARAM;
    }

    paramWords = pConfig->paramSize / 4;
    seq = GetField32(pConfig->pParamBuf, pConfig->seqOffset);

    PARAM_DEBUG("Param save start, old_seq=%lu\r\n", seq);

    while (retry < MAX_WRITE_RETRY)
    {
        write_addr = s_curr_addr;

        /* 检查写入后是否会超出扇区 */
        if (write_addr + pConfig->paramSize > (s_curr_sec + 1) * SECTOR_SIZE)
        {
            uint32_t eraseCnt = GetField32(pConfig->pParamBuf, pConfig->eraseCntOffset);

            if (eraseCnt < MAX_ERASE_LIFE)
            {
                if (Internal_Erase(s_curr_sec * SECTOR_SIZE) == PARAM_OK)
                {
                    write_addr = s_curr_sec * SECTOR_SIZE;
                    SetField32(pConfig->pParamBuf, pConfig->eraseCntOffset, eraseCnt + 1);
                }
                else
                {
                    retry++;
                    continue;
                }
            }
            else
            {
                s_curr_sec = (s_curr_sec <= SEC_END) ? SEC_START : (s_curr_sec - 1);
                write_addr = s_curr_sec * SECTOR_SIZE;
                Internal_Erase(s_curr_sec * SECTOR_SIZE);
                SetField32(pConfig->pParamBuf, pConfig->eraseCntOffset, 1);
            }
        }

        /* 更新序列号和 CRC */
        seq = GetField32(pConfig->pParamBuf, pConfig->seqOffset);
        SetField32(pConfig->pParamBuf, pConfig->seqOffset, seq + 1);
        SetField32(pConfig->pParamBuf, pConfig->checksumOffset,
                   CalcParamCRC(pConfig->pParamBuf, pConfig->paramSize, pConfig->checksumOffset));

        if (Internal_WriteBuffer(write_addr, (const uint32_t*)pConfig->pParamBuf, paramWords) == PARAM_OK)
        {
            if (Internal_VerifyBuffer(write_addr, (const uint32_t*)pConfig->pParamBuf, paramWords))
            {
                /* 保存成功后，更新 s_curr_addr 为下一个写入地址 */
                s_curr_addr = write_addr + pConfig->paramSize;
                g_ParamDebug.save_count++;
                UpdateDebugInfo(PARAM_OK, pConfig->paramSize);

                PARAM_DEBUG("Param save SUCCESS, new_seq=%lu, addr=0x%08lX\r\n",
                            GetField32(pConfig->pParamBuf, pConfig->seqOffset), write_addr);
                return PARAM_OK;
            }
        }

        retry++;
        /* 写入失败，尝试下一个地址 */
        s_curr_addr = write_addr + pConfig->paramSize;
    }

    UpdateDebugInfo(PARAM_ERR, pConfig->paramSize);
    PARAM_DEBUG("Param save FAILED after %d retries\r\n", MAX_WRITE_RETRY);
    return PARAM_ERR;
}

void Param_Debug_EraseAll(const Param_Config_t *pConfig, void (*pSetDefaults)(void))
{
    int s;
    PARAM_DEBUG("Erase all param sectors start\r\n");

    for (s = SEC_START; s >= SEC_END; s--)
    {
        Internal_Erase(s * SECTOR_SIZE);
    }

    Param_Init(pConfig, pSetDefaults);
    PARAM_DEBUG("Erase all param sectors done\r\n");
}

int32_t Param_EraseSector(uint32_t address)
{
    return Internal_Erase(address);
}
