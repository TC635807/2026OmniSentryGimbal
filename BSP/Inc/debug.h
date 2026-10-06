//
// Created by TC63 on 2026/1/1.
//

#ifndef BALENCECAR_DEBUG_H
#define BALENCECAR_DEBUG_H
#include "main.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

    /* 头文件包含 */
#include <stdint.h>
#include <stdarg.h>

    /* 宏定义 */
#define DEBUG_UART_TIMEOUT   100     // 发送超时时间（ms）
#define DEBUG_MAX_BUFFER_SIZE 128    // 最大缓冲区大小

    /* 函数声明 */
    // 初始化函数，设置UART句柄
    void DebugUART_Init(UART_HandleTypeDef* huart);

    // 获取当前UART句柄（如果需要）
    UART_HandleTypeDef* DebugUART_GetHandle(void);

    /**
     * @brief 发送调试数据（支持可变参数和类型）
     * @param format 格式化字符串，指定数据类型：
     *               'f' - float (4字节)
     *               'i' - int32_t (4字节)
     *               's' - int16_t (2字节)
     *               'c' - uint8_t (1字节)
     *               'u' - uint32_t (4字节)
     *               'h' - uint16_t (2字节)
     * @param ... 对应格式的数据参数
     * @note 示例：SendDebugData("ffi", a, b, intVal) 发送2个float和1个int
     */
    void SendDebugData(const char* format, ...);

    static inline void SetDebugUART(UART_HandleTypeDef* huart) {
        extern UART_HandleTypeDef* debug_huart;
        debug_huart = huart;
    }

    /**
     * @brief 获取当前使用的UART句柄
     * @return 当前UART句柄指针
     */
    static inline UART_HandleTypeDef* GetDebugUART(void) {
        extern UART_HandleTypeDef* debug_huart;
        return debug_huart;
    }

#ifdef __cplusplus
}
#endif


#endif //BALENCECAR_DEBUG_H