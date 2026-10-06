//
// Created by TC63 on 2026/1/1.
//
#include "debug.h"
#include <string.h>

/* 内部变量定义 */
// 内部静态变量
static UART_HandleTypeDef* s_debug_huart = NULL;

void DebugUART_Init(UART_HandleTypeDef* huart) {
    s_debug_huart = huart;
}

/* 内部辅助函数 */
static void _AddFloatToBuffer(uint8_t* buffer, int* index, float value) {
    uint8_t* ptr = (uint8_t*)&value;
    for(int i = 0; i < 4; i++) {
        buffer[(*index)++] = ptr[i];
    }
}
UART_HandleTypeDef* DebugUART_GetHandle(void) {
    return s_debug_huart;
}
static void _AddInt32ToBuffer(uint8_t* buffer, int* index, int32_t value) {
    uint8_t* ptr = (uint8_t*)&value;
    for(int i = 0; i < 4; i++) {
        buffer[(*index)++] = ptr[i];
    }
}

static void _AddUint32ToBuffer(uint8_t* buffer, int* index, uint32_t value) {
    uint8_t* ptr = (uint8_t*)&value;
    for(int i = 0; i < 4; i++) {
        buffer[(*index)++] = ptr[i];
    }
}

static void _AddInt16ToBuffer(uint8_t* buffer, int* index, int16_t value) {
    uint8_t* ptr = (uint8_t*)&value;
    for(int i = 0; i < 2; i++) {
        buffer[(*index)++] = ptr[i];
    }
}

static void _AddUint16ToBuffer(uint8_t* buffer, int* index, uint16_t value) {
    uint8_t* ptr = (uint8_t*)&value;
    for(int i = 0; i < 2; i++) {
        buffer[(*index)++] = ptr[i];
    }
}

/* 公共函数实现 */
void SendDebugData(const char* format, ...) {
    if (s_debug_huart == NULL) {
        return;  // UART未初始化
    }

    uint8_t buffer[DEBUG_MAX_BUFFER_SIZE];
    int index = 0;

    // 固定头字节
    buffer[index++] = 0xAA;
    buffer[index++] = 0xBB;

    va_list args;
    va_start(args, format);

    // 解析格式字符串
    while (*format != '\0' && index < DEBUG_MAX_BUFFER_SIZE - 4) {
        switch (*format) {
            case 'f': { // float类型
                float value = (float)va_arg(args, double);
                _AddFloatToBuffer(buffer, &index, value);
                break;
            }
            case 'i': { // int32_t类型
                int32_t value = va_arg(args, int32_t);
                _AddInt32ToBuffer(buffer, &index, value);
                break;
            }
            case 'u': { // uint32_t类型
                uint32_t value = va_arg(args, uint32_t);
                _AddUint32ToBuffer(buffer, &index, value);
                break;
            }
            case 's': { // int16_t类型
                int16_t value = (int16_t)va_arg(args, int);
                _AddInt16ToBuffer(buffer, &index, value);
                break;
            }
            case 'h': { // uint16_t类型
                uint16_t value = (uint16_t)va_arg(args, int);
                _AddUint16ToBuffer(buffer, &index, value);
                break;
            }
            case 'c': { // uint8_t类型
                uint8_t value = (uint8_t)va_arg(args, int);
                buffer[index++] = value;
                break;
            }
            default:
                // 忽略未知格式字符
                break;
        }
        format++;
    }

    va_end(args);

    // 发送数据
    if (index > 0) {
        HAL_UART_Transmit(s_debug_huart, buffer, index, DEBUG_UART_TIMEOUT);
    }
}


// 使用示例：
// SendDebugData("ff", a, b);              // 发送2个float
// SendDebugData("fffi", a, b, c, intVal); // 发送3个float和1个int
// SendDebugData("fsci", fVal, sVal, cVal, iVal); // 混合类型