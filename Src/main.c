#include <stdint.h>
#include <stdio.h>
#include "stm32h503xx.h"
#include "core_cm33.h"

// GPIO Mode definitions
#define GPIO_INPUT_MODE     0x00  // 00: Input mode
#define GPIO_OUTPUT_MODE    0x01  // 01: Output mode
#define GPIO_ALT_MODE       0x02  // 10: Alternate function mode
#define GPIO_ANALOG_MODE    0x03  // 11: Analog mode

// GPIO Output Type definitions
#define GPIO_OUTPUT_PP      0x00  // 0: Push-pull output
#define GPIO_OUTPUT_OD      0x01  // 1: Open-drain output

// GPIO Output Speed definitions
#define GPIO_OUTPUT_LS      0x00  // 00: Low speed
#define GPIO_OUTPUT_MS      0x01  // 01: Medium speed
#define GPIO_OUTPUT_HS      0x02  // 10: High speed
#define GPIO_OUTPUT_VHS     0x03  // 11: Very high speed

// GPIO Pull-up/Pull-down definitions
#define GPIO_NO_PUPD        0x00  // 00: No pull-up, pull-down
#define GPIO_PU             0x01  // 01: Pull-up
#define GPIO_PD             0x02  // 10: Pull-down

// General state definitions
#define LOW            0x00  // 0: Low level
#define HIGH           0x01  // 1: High level

// Enable/Disable definitions
#define DISABLE        0x00  // 0: Disable
#define ENABLE         0x01  // 1: Enable

// GPIO Alternate Function definitions
#define GPIO_AF0            0x00  // 0000: Alternate Function 0
#define GPIO_AF1            0x01  // 0001: Alternate Function 1
#define GPIO_AF2            0x02  // 0010: Alternate Function 2
#define GPIO_AF3            0x03  // 0011: Alternate Function 3
#define GPIO_AF4            0x04  // 0100: Alternate Function 4
#define GPIO_AF5            0x05  // 0101: Alternate Function 5
#define GPIO_AF6            0x06  // 0110: Alternate Function 6
#define GPIO_AF7            0x07  // 0111: Alternate Function 7
#define GPIO_AF8            0x08  // 1000: Alternate Function 8
#define GPIO_AF9            0x09  // 1001: Alternate Function 9
#define GPIO_AF10           0x0A  // 1010: Alternate Function 10
#define GPIO_AF11           0x0B  // 1011: Alternate Function 11
#define GPIO_AF12           0x0C  // 1100: Alternate Function 12
#define GPIO_AF13           0x0D  // 1101: Alternate Function 13
#define GPIO_AF14           0x0E  // 1110: Alternate Function 14
#define GPIO_AF15           0x0F  // 1111: Alternate Function 15
#define GPIO_AF_NONE       0xFF  // Special value indicating no AF is needed

// Bit masks for different field widths
#define BIT_MASK_1          0x01       // Mask for 1-bit field: 0000 0001
#define BIT_MASK_2          0x03       // Mask for 2-bit field: 0000 0011
#define BIT_MASK_3          0x07       // Mask for 3-bit field: 0000 0111
#define BIT_MASK_4          0x0F       // Mask for 4-bit field: 0000 1111
#define BIT_MASK_5          0x1F       // Mask for 5-bit field: 0001 1111
#define BIT_MASK_6          0x3F       // Mask for 6-bit field: 0011 1111
#define BIT_MASK_7          0x7F       // Mask for 7-bit field: 0111 1111
#define BIT_MASK_8          0xFF       // Mask for 8-bit field: 1111 1111
#define BIT_MASK_9          0x1FF      // Mask for 9-bit field
#define BIT_MASK_10         0x3FF      // Mask for 10-bit field
#define BIT_MASK_11         0x7FF      // Mask for 11-bit field
#define BIT_MASK_12         0xFFF      // Mask for 12-bit field
#define BIT_MASK_16         0xFFFF     // Mask for 16-bit field
#define BIT_MASK_32         0xFFFFFFFF // Mask for 32-bit field

// Helper macros for bit position calculations
#define PIN_MODE_POS(pin)   ((pin) * 2)      // Calculate bit position for MODE, OSPEED, PUPD registers
#define PIN_AFR_POS(pin)    (((pin) % 8) * 4) // Calculate bit position within AFRL/AFRH registers
#define PIN_AFR_REG(pin)    ((pin) / 8)      // Determine which AFR register to use (0=AFRL, 1=AFRH)

// GPIO Pin defines - simple numeric values
#define GPIO_PIN_0        0
#define GPIO_PIN_1        1
#define GPIO_PIN_2        2
#define GPIO_PIN_3        3
#define GPIO_PIN_4        4
#define GPIO_PIN_5        5
#define GPIO_PIN_6        6
#define GPIO_PIN_7        7
#define GPIO_PIN_8        8
#define GPIO_PIN_9        9
#define GPIO_PIN_10       10
#define GPIO_PIN_11       11
#define GPIO_PIN_12       12
#define GPIO_PIN_13       13
#define GPIO_PIN_14       14
#define GPIO_PIN_15       15

// FDCAN Mode Control Macros
#define FDCAN_MODE_NORMAL           0x00  // Normal operation mode
#define FDCAN_MODE_RESTRICTED       0x01  // Restricted operation mode
#define FDCAN_MODE_BUS_MONITORING   0x02  // Bus monitoring mode
#define FDCAN_MODE_INTERNAL_LOOPBACK 0x03  // Internal loopback mode
#define FDCAN_MODE_EXTERNAL_LOOPBACK 0x04  // External loopback mode

// FDCAN Control Register Bit Positions
#define FDCAN_CCCR_INIT_POS         0     // Initialization bit position
#define FDCAN_CCCR_CCE_POS          1     // Configuration Change Enable bit position
#define FDCAN_CCCR_ASM_POS          2     // Restricted Operation Mode bit position
#define FDCAN_CCCR_CSA_POS          3     // Clock Stop Acknowledge bit position
#define FDCAN_CCCR_CSR_POS          4     // Clock Stop Request bit position
#define FDCAN_CCCR_MON_POS          5     // Bus Monitoring Mode bit position
#define FDCAN_CCCR_DAR_POS          6     // Disable Automatic Retransmission bit position
#define FDCAN_CCCR_TEST_POS         7     // Test Mode Enable bit position
#define FDCAN_CCCR_FDOE_POS         8     // FD Operation Enable bit position
#define FDCAN_CCCR_BRSE_POS         9     // Bit Rate Switching Enable bit position
#define FDCAN_CCCR_PXHD_POS         12    // Protocol Exception Handling Disable bit position
#define FDCAN_CCCR_EFBI_POS         13    // Edge Filtering during Bus Integration bit position
#define FDCAN_CCCR_TXP_POS          14    // Transmit Pause bit position
#define FDCAN_CCCR_NISO_POS         15    // Non ISO Operation bit position

// FDCAN Test Register Bit Positions
#define FDCAN_TEST_LBCK_POS         4     // Loopback Mode bit position
#define FDCAN_TEST_TX_POS           5     // Control of Transmit Pin bit position
#define FDCAN_TEST_RX_POS           7     // Receive Pin bit position

// FDCAN Interrupt Register Bit Positions
#define FDCAN_IR_RF0N_POS           0     // Rx FIFO 0 New Message bit position
#define FDCAN_IR_RF0F_POS           1     // Rx FIFO 0 Full bit position
#define FDCAN_IR_RF0L_POS           2     // Rx FIFO 0 Message Lost bit position
#define FDCAN_IR_RF1N_POS           3     // Rx FIFO 1 New Message bit position
#define FDCAN_IR_RF1F_POS           4     // Rx FIFO 1 Full bit position
#define FDCAN_IR_RF1L_POS           5     // Rx FIFO 1 Message Lost bit position
#define FDCAN_IR_HPM_POS            6     // High Priority Message bit position
#define FDCAN_IR_TC_POS             7     // Transmission Completed bit position
#define FDCAN_IR_TCF_POS            8     // Transmission Cancellation Finished bit position
#define FDCAN_IR_TFE_POS            9     // Tx FIFO Empty bit position
#define FDCAN_IR_TEFN_POS           10    // Tx Event FIFO New Entry bit position
#define FDCAN_IR_TEFF_POS           11    // Tx Event FIFO Full bit position
#define FDCAN_IR_TEFL_POS           12    // Tx Event FIFO Element Lost bit position
#define FDCAN_IR_TSW_POS            13    // Timestamp Wraparound bit position
#define FDCAN_IR_MRAF_POS           14    // Message RAM Access Failure bit position
#define FDCAN_IR_TOO_POS            15    // Timeout Occurred bit position
#define FDCAN_IR_DRX_POS            16    // Message stored to Dedicated Rx Buffer bit position
#define FDCAN_IR_ELO_POS            22    // Error Logging Overflow bit position
#define FDCAN_IR_EP_POS             23    // Error Passive bit position
#define FDCAN_IR_EW_POS             24    // Warning Status bit position
#define FDCAN_IR_BO_POS             25    // Bus_Off Status bit position
#define FDCAN_IR_WDI_POS            26    // Watchdog Interrupt bit position
#define FDCAN_IR_PEA_POS            27    // Protocol Error in Arbitration Phase bit position
#define FDCAN_IR_PED_POS            28    // Protocol Error in Data Phase bit position
#define FDCAN_IR_ARA_POS            29    // Access to Reserved Address bit position

// FDCAN Frame Type Definitions
#define FDCAN_FRAME_CLASSIC         0     // Classic CAN frame
#define FDCAN_FRAME_FD_NO_BRS       1     // CAN FD frame without bit rate switching
#define FDCAN_FRAME_FD_BRS          2     // CAN FD frame with bit rate switching

// FDCAN ID Type Definitions
#define FDCAN_ID_STANDARD           0     // Standard ID (11-bit)
#define FDCAN_ID_EXTENDED           1     // Extended ID (29-bit)

// FDCAN Frame Type Definitions
#define FDCAN_DATA_FRAME            0     // Data frame
#define FDCAN_REMOTE_FRAME          1     // Remote frame

// FDCAN Data Length Code (DLC) to actual data byte conversion
#define FDCAN_DLC_BYTES_0           0x0   // 0 bytes
#define FDCAN_DLC_BYTES_1           0x1   // 1 byte
#define FDCAN_DLC_BYTES_2           0x2   // 2 bytes
#define FDCAN_DLC_BYTES_3           0x3   // 3 bytes
#define FDCAN_DLC_BYTES_4           0x4   // 4 bytes
#define FDCAN_DLC_BYTES_5           0x5   // 5 bytes
#define FDCAN_DLC_BYTES_6           0x6   // 6 bytes
#define FDCAN_DLC_BYTES_7           0x7   // 7 bytes
#define FDCAN_DLC_BYTES_8           0x8   // 8 bytes
#define FDCAN_DLC_BYTES_12          0x9   // 12 bytes (FD format only)
#define FDCAN_DLC_BYTES_16          0xA   // 16 bytes (FD format only)
#define FDCAN_DLC_BYTES_20          0xB   // 20 bytes (FD format only)
#define FDCAN_DLC_BYTES_24          0xC   // 24 bytes (FD format only)
#define FDCAN_DLC_BYTES_32          0xD   // 32 bytes (FD format only)
#define FDCAN_DLC_BYTES_48          0xE   // 48 bytes (FD format only)
#define FDCAN_DLC_BYTES_64          0xF   // 64 bytes (FD format only)

// FDCAN Filter Type Definitions
#define FDCAN_FILTER_RANGE          0     // Range filter from ID1 to ID2
#define FDCAN_FILTER_DUAL           1     // Dual ID filter for ID1 or ID2
#define FDCAN_FILTER_MASK           2     // Classic filter: ID1 = filter, ID2 = mask
#define FDCAN_FILTER_RANGE_NO_EIDM  3     // Range filter, EIDM not applied

// FDCAN Filter Configuration Definitions
#define FDCAN_FILTER_DISABLE        0     // Filter element disabled
#define FDCAN_FILTER_TO_RXFIFO0     1     // Store in Rx FIFO 0 if filter matches
#define FDCAN_FILTER_TO_RXFIFO1     2     // Store in Rx FIFO 1 if filter matches
#define FDCAN_FILTER_REJECT         3     // Reject ID if filter matches
#define FDCAN_FILTER_SET_PRIORITY   4     // Set priority if filter matches
#define FDCAN_FILTER_SET_PRIORITY_RXFIFO0 5  // Set priority and store in FIFO 0
#define FDCAN_FILTER_SET_PRIORITY_RXFIFO1 6  // Set priority and store in FIFO 1

// FDCAN Tx Buffer Operation Mode Definitions
#define FDCAN_TXBUFFER_FIFO         0     // Tx FIFO operation
#define FDCAN_TXBUFFER_QUEUE        1     // Tx Queue operation

/* Base address of FDCAN message RAM in SRAM */
#define SRAMCAN_BASE_ADDR (0x4000AC00UL)

/** @defgroup FDCAN_filter_type FDCAN Filter Type
 * @{
 */
#define FDCAN_FILTER_RANGE_t         ((uint32_t)0x00000000U) /*!< Range filter from FilterID1 to FilterID2                        */
#define FDCAN_FILTER_DUAL_t          ((uint32_t)0x00000001U) /*!< Dual ID filter for FilterID1 or FilterID2                       */
#define FDCAN_FILTER_MASK_t          ((uint32_t)0x00000002U) /*!< Classic filter: FilterID1 = filter, FilterID2 = mask            */
#define FDCAN_FILTER_RANGE_NO_EIDM_t ((uint32_t)0x00000003U) /*!< Range filter from FilterID1 to FilterID2, EIDM mask not applied */
/**
 * @}
 */

/** @defgroup FDCAN_Non_Matching_Frames FDCAN non-matching frames
 * @{
 */
#define FDCAN_ACCEPT_IN_RX_FIFO0_t ((uint32_t)0x00000000U) /*!< Accept in Rx FIFO 0 */
#define FDCAN_ACCEPT_IN_RX_FIFO1_t ((uint32_t)0x00000001U) /*!< Accept in Rx FIFO 1 */
#define FDCAN_REJECT_t             ((uint32_t)0x00000002U) /*!< Reject              */
/**
 * @}
 */

/** @defgroup FDCAN_Reject_Remote_Frames FDCAN reject remote frames
 * @{
 */
#define FDCAN_FILTER_REMOTE_t ((uint32_t)0x00000000U) /*!< Filter remote frames */
#define FDCAN_REJECT_REMOTE_t ((uint32_t)0x00000001U) /*!< Reject all remote frames */
/**
 * @}
 */

// FDCAN High-Level Function Macros
/*
 * Using do-while(0) to:
 * Create a macro that behaves properly in all contexts.
 *
 * Consider a macro without do-while(0):
 * #define BAD_MACRO(x) \
*     int temp = (x); \
*     temp = temp * 2; \
*     return temp
 *
 * When used in a conditional:
 * if (condition)
 *     BAD_MACRO(5);
 * else
 *     something_else();
 *
 * It would expand to:
 * if (condition)
 *     int temp = (5);     // Only this line is controlled by the if
 * temp = temp * 2;    // This will not in condition
 * return temp;        // This will not in condition
 * else
 *     something_else();   // ERROR: 'else' without matching 'if'
 *
 * Now with do-while(0):
 * #define GOOD_MACRO(x) do { \
*     int temp = (x); \
*     temp = temp * 2; \
*     return temp; \
* } while(0)
 *
 * When used in the same conditional:
 * if (condition)
 *     GOOD_MACRO(5);
 * else
 *     something_else();
 *
 * It expands to:
 * if (condition)
 *     do {
 *         int temp = (5);
 *         temp = temp * 2;
 *         return temp;
 *     } while(0);
 * else
 *     something_else();
 *
 * Now all statements are properly contained in the if-block,
 * and the else correctly attaches to the if. The entire do-while
 * block acts as a single statement.
 */

// Enter FDCAN initialization mode
#define FDCAN_ENTER_INIT_MODE(fdcan) do { \
    SET_BIT_FIELD((fdcan)->CCCR, FDCAN_CCCR_INIT_POS); \
    SET_BIT_FIELD((fdcan)->CCCR, FDCAN_CCCR_CCE_POS); \
    while(!(READ_BIT_FIELD((fdcan)->CCCR, FDCAN_CCCR_INIT_POS, 1))); \
} while(0)

// Exit FDCAN initialization mode
#define FDCAN_EXIT_INIT_MODE(fdcan) do { \
    CLEAR_BIT_FIELD((fdcan)->CCCR, FDCAN_CCCR_INIT_POS); \
    while(READ_BIT_FIELD((fdcan)->CCCR, FDCAN_CCCR_INIT_POS, 1)); \
} while(0)

// Enable FDCAN configuration change
#define FDCAN_ENABLE_CONFIG_CHANGE(fdcan) do { \
    SET_BIT_FIELD(*(fdcan)->CCCR, FDCAN_CCCR_CCE_POS); \
} while(0)

// Enable FDCAN External loopback mode
#define FDCAN_ENABLE_EXTERNAL_LOOPBACK(fdcan) do { \
    SET_BIT_FIELD((fdcan)->CCCR, FDCAN_CCCR_TEST_POS); \
    SET_BIT_FIELD((fdcan)->TEST, FDCAN_TEST_LBCK_POS); \
} while(0)

// Enable FDCAN Internal loopback mode
#define FDCAN_ENABLE_INTERNAL_LOOPBACK(fdcan) do { \
    SET_BIT_FIELD((fdcan)->CCCR, FDCAN_CCCR_TEST_POS); \
    SET_BIT_FIELD((fdcan)->TEST, FDCAN_TEST_LBCK_POS); \
    SET_BIT_FIELD((fdcan)->TEST, FDCAN_CCCR_MON_POS); \
} while(0)

// Enable FDCAN FD mode
#define FDCAN_ENABLE_FD_MODE(fdcan) do { \
    SET_BIT_FIELD((fdcan)->CCCR, FDCAN_CCCR_FDOE_POS); \
} while(0)

// Enable Classical CAN Mode
#define FDCAN_ENABLE_CLASSICAL_CAN_MODE(fdcan) do { \
    CLEAR_BIT_FIELD((fdcan)->CCCR, FDCAN_CCCR_FDOE_POS); \
} while(0)

// Enable FDCAN bit rate switching (for FD)
#define FDCAN_ENABLE_BRS(fdcan) do { \
    SET_BIT_FIELD((fdcan)->CCCR, FDCAN_CCCR_BRSE_POS); \
} while(0)

// Enable FDCAN TX FIFO operation
#define FDCAN_ENABLE_TX_FIFO(fdcan) do { \
    CLEAR_BIT_FIELD((fdcan)->TXBC, 24); \
} while(0)

// Enable FDCAN TX Queue operation
#define FDCAN_ENABLE_TX_QUEUE(fdcan) do { \
    SET_BIT_FIELD((fdcan)->TXBC, 24); \
} while(0)

// Enable FIFO0 Overwrite mode
#define FDCAN_ENABLE_FIFO0_OVERWRITE(fdcan) do { \
    SET_BIT_FIELD((fdcan)->RXGFC, 9); \
} while(0)

#define FDCAN_FIFO0_OVERWRITE_MODE 0x0
#define FDCAN_FIFO0_BLOCKING_MODE 0x1
#define FDCAN_FIFO1_OVERWRITE_MODE 0x2
#define FDCAN_FIFO1_BLOCKING_MODE 0x3

// FDCAN TX Buffer Element Offset macros
#define FDCAN_TX_ELEMENT_1_OFFSET   0x00  // First 32-bit word offset
#define FDCAN_TX_ELEMENT_2_OFFSET   0x04  // Second 32-bit word offset
#define FDCAN_TX_ELEMENT_DATA_OFFSET 0x08  // Data section starts at third 32-bit word

// FDCAN RX FIFO Element Offset macros
#define FDCAN_RX_ELEMENT_1_OFFSET   0x00  // First 32-bit word offset
#define FDCAN_RX_ELEMENT_2_OFFSET   0x04  // Second 32-bit word offset
#define FDCAN_RX_ELEMENT_DATA_OFFSET 0x08  // Data section starts at third 32-bit word

// FDCAN Message RAM Section Offsets (relative to SRAMCAN_BASE_ADDR)
#define FDCAN_STDID_FILTER_OFFSET   0x000 // Standard ID Filter elements
#define FDCAN_EXTID_FILTER_OFFSET   0x070 // Extended ID Filter elements
#define FDCAN_RX_FIFO0_OFFSET       0x0B0 // RX FIFO 0 elements
#define FDCAN_RX_FIFO1_OFFSET       0x188 // RX FIFO 1 elements
#define FDCAN_TX_EVENT_FIFO_OFFSET  0x260 // TX Event FIFO elements
#define FDCAN_TX_BUFFER_OFFSET      0x278 // TX Buffer elements

/** @defgroup FDCAN_filter_config FDCAN Filter Configuration
 * @{
 */
#define FDCAN_FILTER_DISABLE_t      ((uint32_t)0x00000000U) /*!< Disable filter element                                    */
#define FDCAN_FILTER_RXFIFO0    ((uint32_t)0x00000001U) /*!< Store in Rx FIFO 0 if filter matches                      */
#define FDCAN_FILTER_RXFIFO1    ((uint32_t)0x00000002U) /*!< Store in Rx FIFO 1 if filter matches                      */
#define FDCAN_FILTER_REJECT_t        ((uint32_t)0x00000003U) /*!< Reject ID if filter matches                               */
#define FDCAN_FILTER_HP_t            ((uint32_t)0x00000004U) /*!< Set high priority if filter matches                       */
#define FDCAN_FILTER_TO_RXFIFO0_HP_t ((uint32_t)0x00000005U) /*!< Set high priority and store in FIFO 0 if filter matches   */
#define FDCAN_FILTER_TO_RXFIFO1_HP_t ((uint32_t)0x00000006U) /*!< Set high priority and store in FIFO 1 if filter matches   */
/**
 * @}
 */

// *** RCC *** //
#define RCC_BASE_ADDR     (0x44020C00)

// *** PWR *** //
#define PWR_BASE_ADDR     (0x44020800)

// *** ICACHE *** //
#define ICACHE_BASE_ADDR (0x40030400)

// *** FLASH *** //
#define FLASH_BASE_ADDR (0x40022000)

// *** FD CAN 1 *** //
#define FDCAN1_BASE_ADDR (0x4000A400)

// *** I2C2 *** //
#define I2C2_BASE_ADDR (0x40005800)

// *** TIM2 *** //
#define TIM2_BASE_ADDR           (0x40000000UL)

// *** GPIO A FOR PA11 (FDCAN1_RX) & PA12 (FDCAN2_TX) *** //
#define GPIOA_BASE_ADDR (0x42020000)

// *** GPIO *** //
#define GPIOA_BASE_ADDR    (0x42020000)
#define GPIOB_BASE_ADDR    (0x42020400)
#define GPIOC_BASE_ADDR    (0x42020800)
#define GPIOD_BASE_ADDR    (0x42020C00)
#define GPIOH_BASE_ADDR    (0x42021C00)

// *** NVIC BASE ADDR *** //
#define NVIC_ISER0_ADDR  (0xE000E100)
#define NVIC_ISER1_ADDR  (NVIC_ISER0_ADDR + 0x04)
#define NVIC_ICER0_t (0XE000E180)
#define NVIC_ISPR0_t (0XE000E200)
#define NVIC_ICPR0_t (0XE000E280)
#define FDCAN1_IT0_IRQ_t 39
#define I2C2_EV_IRQ_t 53

volatile uint32_t *NVIC_ISER0_p = (volatile uint32_t*) NVIC_ISER0_ADDR;
volatile uint32_t *NVIC_ISER1_p = (volatile uint32_t*) NVIC_ISER1_ADDR;

volatile uint32_t *NVIC_ICER0_p = (uint32_t*) NVIC_ICER0_t;
volatile uint32_t *NVIC_ISPR0_p = (uint32_t*) NVIC_ISPR0_t;
volatile uint32_t *NVIC_ICPR0_p = (uint32_t*) NVIC_ICPR0_t;
volatile uint8_t receivedMessage = 0;
volatile uint8_t data_to_send;

/*****************************************************************************
 * STM32H5 FDCAN Driver Implementation
 *
 * This file implements a driver for the Flexible Data-rate Controller Area Network
 * (FDCAN) peripheral on STM32H5 series microcontrollers, providing functions for
 * CAN bus communication in both classic and FD formats.
 *****************************************************************************/

/***** Register Definition Macros *****/
#define COMP_CFGR1_PWRMODE_REG (0x40004000+0x0C)  // Comparator configuration register for power mode

/***** Bit Manipulation Macros *****/
/* Set a specific bit in a register */
#define SET_BIT_FIELD(reg, bit) ((reg) |= (1U << (bit)))

/* Write a value to a specific bit position */
#define SET_VAL_BIT(reg, val, bit) ((reg) |= ((val) << (bit)))

/* Clear specific bits using a mask at a specific position */
#define CLEAR_VAL_BIT(reg, mask, bit) ((reg) &= ~((mask) << (bit)))

/* Clear a specific bit in a register */
#define CLEAR_BIT_FIELD(reg, bit) ((reg) &= ~(1U << (bit)))

/* Write an entire register with a specified value */
#define WRITE_ALL_REG(reg, value) ((reg) = (value))

/* Write a value to a specific bit position, clearing all other bits */
#define WRITE_REG_BIT(reg, value, bit) ((reg) = (value << bit))

/* Read a specific bit field from a register using a mask */
#define READ_BIT_FIELD(reg, bit, mask) (((reg) >> (bit)) & (mask))

/***** FDCAN ID Type Definitions *****/
#define FDCAN_STANDARD_ID ((uint32_t)0x00000000U)  // 11-bit standard ID format
#define FDCAN_EXTENDED_ID ((uint32_t)0x40000000U)  // 29-bit extended ID format

/***** Comparator Power Mode Register *****/
volatile uint32_t *COMP_CFGR1_PWRMODE_p = (uint32_t*) COMP_CFGR1_PWRMODE_REG;

/****************************************************************************
 * Peripheral Register Structures
 *
 * These structures map the memory-mapped registers for each peripheral,
 * allowing typed access to specific registers through structure members.
 ****************************************************************************/

/***** RCC (Reset and Clock Control) Register Structure *****/
typedef struct {
	volatile uint32_t CR;              // 0x000: Clock Control Register
	uint32_t RESERVED0[3];             // 0x004-0x00C: Reserved
	volatile uint32_t HSICFGR;         // 0x010: HSI Configuration Register
	volatile uint32_t CRRCR;           // 0x014: Clock Recovery RC Register
	volatile uint32_t CSICFGR;         // 0x018: CSI Configuration Register
	volatile uint32_t CFGR1;           // 0x01C: Clock Configuration Register 1
	volatile uint32_t CFGR2;           // 0x020: Clock Configuration Register 2
	uint32_t RESERVED1[1];             // 0x024: Reserved
	volatile uint32_t PLL1CFGR;        // 0x028: PLL1 Configuration Register
	volatile uint32_t PLL2CFGR;        // 0x02C: PLL2 Configuration Register
	uint32_t RESERVED2[1];             // 0x030: Reserved
	volatile uint32_t PLL1DIVR;        // 0x034: PLL1 Divider Register
	volatile uint32_t PLL1FRACR;       // 0x038: PLL1 Fraction Register
	volatile uint32_t PLL2DIVR;        // 0x03C: PLL2 Divider Register
	volatile uint32_t PLL2FRACR;       // 0x040: PLL2 Fraction Register
	uint32_t RESERVED3[3];             // 0x044-0x04C: Reserved
	volatile uint32_t CIER;            // 0x050: Clock Interrupt Enable Register
	volatile uint32_t CIFR;            // 0x054: Clock Interrupt Flag Register
	volatile uint32_t CICR;            // 0x058: Clock Interrupt Clear Register
	uint32_t RESERVED4[1];             // 0x05C: Reserved
	volatile uint32_t AHB1RSTR;        // 0x060: AHB1 Peripheral Reset Register
	volatile uint32_t AHB2RSTR;        // 0x064: AHB2 Peripheral Reset Register
	uint32_t RESERVED5[3];             // 0x068-0x070: Reserved
	volatile uint32_t APB1LRSTR;       // 0x074: APB1L Peripheral Reset Register
	volatile uint32_t APB1HRSTR;       // 0x078: APB1H Peripheral Reset Register
	volatile uint32_t APB2RSTR;        // 0x07C: APB2 Peripheral Reset Register
	volatile uint32_t APB3RSTR;        // 0x080: APB3 Peripheral Reset Register
	uint32_t RESERVED6[1];             // 0x084: Reserved
	volatile uint32_t AHB1ENR;   // 0x088: AHB1 Peripheral Clock Enable Register
	volatile uint32_t AHB2ENR;   // 0x08C: AHB2 Peripheral Clock Enable Register
	uint32_t RESERVED7[3];             // 0x090-0x098: Reserved
	volatile uint32_t APB1LENR; // 0x09C: APB1L Peripheral Clock Enable Register
	volatile uint32_t APB1HENR; // 0x0A0: APB1H Peripheral Clock Enable Register
	volatile uint32_t APB2ENR;   // 0x0A4: APB2 Peripheral Clock Enable Register
	volatile uint32_t APB3ENR;   // 0x0A8: APB3 Peripheral Clock Enable Register
	uint32_t RESERVED8[1];             // 0x0AC: Reserved
	volatile uint32_t AHB1LPENR;  // 0x0B0: AHB1 Low Power Clock Enable Register
	volatile uint32_t AHB2LPENR;  // 0x0B4: AHB2 Low Power Clock Enable Register
	uint32_t RESERVED9[3];             // 0x0B8-0x0C0: Reserved
	volatile uint32_t APB1LLPENR; // 0x0C4: APB1L Low Power Clock Enable Register
	volatile uint32_t APB1HLPENR; // 0x0C8: APB1H Low Power Clock Enable Register
	volatile uint32_t APB2LPENR;  // 0x0CC: APB2 Low Power Clock Enable Register
	volatile uint32_t APB3LPENR;  // 0x0D0: APB3 Low Power Clock Enable Register
	uint32_t RESERVED10[1];            // 0x0D4: Reserved
	volatile uint32_t CCIPR1;          // 0x0D8: CCIPR1 Register
	volatile uint32_t CCIPR2;          // 0x0DC: CCIPR2 Register
	volatile uint32_t CCIPR3;          // 0x0E0: CCIPR3 Register
	volatile uint32_t CCIPR4;          // 0x0E4: CCIPR4 Register
	volatile uint32_t CCIPR5;          // 0x0E8: CCIPR5 Register
	uint32_t RESERVED11[1];            // 0x0EC: Reserved
	volatile uint32_t BDCR;            // 0x0F0: Backup Domain Control Register
	volatile uint32_t RSR;             // 0x0F4: Reset Status Register
	volatile uint32_t PRIVCFGR;       // 0x0F8: Privilege Configuration Register
} RCC_TypeDef_t;

/***** PWR (Power Control) Register Structure *****/
typedef struct {
	volatile uint32_t PMCR;            // Power Mode Control Register
	volatile uint32_t PMSR;            // Power Mode Status Register
	uint32_t RESERVED0[2];
	volatile uint32_t VOSCR;           // Voltage Scaling Control Register
	volatile uint32_t VOSSR;           // Voltage Scaling Status Register
	uint32_t RESERVED1[2];
	volatile uint32_t BDCR;            // Backup Domain Control Register
	volatile uint32_t DBPCR;           // Debug Power Control Register
	volatile uint32_t BDSR;            // Backup Domain Status Register
	uint32_t RESERVED2[1];
	volatile uint32_t SCCR;            // System Configuration Control Register
	volatile uint32_t VMCR;            // Voltage Monitoring Control Register
	volatile uint32_t WUSCR;           // Wakeup Source Control Register
	volatile uint32_t WUSR;            // Wakeup Source Status Register
	volatile uint32_t WUCR;            // Wakeup Source Clear Register
	uint32_t RESERVED3[1];
	volatile uint32_t IORETR;          // I/O Retention Register
	volatile uint32_t PRIVCFGR;        // Privilege Configuration Register
} PWR_TypeDef_t;

/***** GPIO (General Purpose Input/Output) Register Structure *****/
typedef struct {
	volatile uint32_t MODER;    // Mode Register (Input/Output/Alternate/Analog)
	volatile uint32_t OTYPER;     // Output Type Register (Push-Pull/Open-Drain)
	volatile uint32_t OSPEEDR;         // Output Speed Register
	volatile uint32_t PUPDR;           // Pull-up/Pull-down Register
	volatile uint32_t IDR;             // Input Data Register (Read pin status)
	volatile uint32_t ODR;           // Output Data Register (Set/clear outputs)
	volatile uint32_t BSRR;       // Bit Set/Reset Register (Atomic pin control)
	volatile uint32_t LCKR;            // Configuration Lock Register
	volatile uint32_t AFR[2];     // Alternate Function Register [0=Low, 1=High]
	volatile uint32_t BRR;             // Bit Reset Register
	volatile uint32_t HSLVR;           // High-Speed Low-Voltage Register
} GPIO_TypeDef_t;

/***** FDCAN (Flexible Data-rate Controller Area Network) Register Structure *****/
typedef struct {
	volatile uint32_t CREL;            // Core Release Register
	volatile uint32_t ENDN;            // Endian Register
	uint32_t RESERVED0[1];
	volatile uint32_t DBTP;            // Data Bit Timing & Prescaler Register
	volatile uint32_t TEST;            // Test Register
	volatile uint32_t RWD;             // RAM Watchdog Register
	volatile uint32_t CCCR;            // CC Control Register
	volatile uint32_t NBTP;           // Nominal Bit Timing & Prescaler Register
	volatile uint32_t TSCC;          // Timestamp Counter Configuration Register
	volatile uint32_t TSCV;            // Timestamp Counter Value Register
	volatile uint32_t TOCC;            // Timeout Counter Configuration Register
	volatile uint32_t TOCV;            // Timeout Counter Value Register
	uint32_t RESERVED1[4];
	volatile uint32_t ECR;             // Error Counter Register
	volatile uint32_t PSR;             // Protocol Status Register
	volatile uint32_t TDCR;           // Transmitter Delay Compensation Register
	uint32_t RESERVED2[1];
	volatile uint32_t IR;              // Interrupt Register
	volatile uint32_t IE;              // Interrupt Enable Register
	volatile uint32_t ILS;             // Interrupt Line Select Register
	volatile uint32_t ILE;             // Interrupt Line Enable Register
	uint32_t RESERVED3[8];
	volatile uint32_t RXGFC;           // RX FIFO Global Configuration Register
	volatile uint32_t XIDAM;           // Extended ID AND Mask Register
	volatile uint32_t HPMS;            // High Priority Message Status Register
	uint32_t RESERVED4[1];
	volatile uint32_t RXF0S;           // RX FIFO 0 Status Register
	volatile uint32_t RXF0A;           // RX FIFO 0 Acknowledge Register
	volatile uint32_t RXF1S;           // RX FIFO 1 Status Register
	volatile uint32_t RXF1A;           // RX FIFO 1 Acknowledge Register
	uint32_t RESERVED5[8];
	volatile uint32_t TXBC;            // TX Buffer Configuration Register
	volatile uint32_t TXFQS;           // TX FIFO/Queue Status Register
	volatile uint32_t TXBRP;           // TX Buffer Request Pending Register
	volatile uint32_t TXBAR;           // TX Buffer Add Request Register
	volatile uint32_t TXBCR;          // TX Buffer Cancellation Request Register
	volatile uint32_t TXBTO;         // TX Buffer Transmission Occurred Register
	volatile uint32_t TXBCF;         // TX Buffer Cancellation Finished Register
	volatile uint32_t TXBTIE; // TX Buffer Transmission Interrupt Enable Register
	volatile uint32_t TXBCIE; // TX Buffer Cancellation Finished Interrupt Enable Register
	volatile uint32_t TXEFS;           // TX Event FIFO Status Register
	volatile uint32_t TXEFA;           // TX Event FIFO Acknowledge Register
	uint32_t RESERVED6[5];
	volatile uint32_t CKDIV;           // Clock Divider Register
} FDCAN_TypeDef_t;

/***** FLASH Register Structure *****/
typedef struct {
	volatile uint32_t ACR;             // Access Control Register
// Add other FLASH registers as needed
} FLASH_TypeDef_t;

/***** ICACHE Register Structure *****/
typedef struct {
	volatile uint32_t CR;              // Control Register
// Add other ICACHE registers as needed
} ICACHE_TypeDef_t;

/**
 * @brief I2C Register Structure for STM32H503
 * @note All registers are volatile to prevent compiler optimization
 */
typedef struct {
	volatile uint32_t CR1; /*!< I2C Control register 1,           Address offset: 0x00 */
	volatile uint32_t CR2; /*!< I2C Control register 2,           Address offset: 0x04 */
	volatile uint32_t OAR1; /*!< I2C Own address 1 register,       Address offset: 0x08 */
	volatile uint32_t OAR2; /*!< I2C Own address 2 register,       Address offset: 0x0C */
	volatile uint32_t TIMINGR; /*!< I2C Timing register,              Address offset: 0x10 */
	volatile uint32_t TIMEOUTR; /*!< I2C Timeout register,             Address offset: 0x14 */
	volatile uint32_t ISR; /*!< I2C Interrupt and status register, Address offset: 0x18 */
	volatile uint32_t ICR; /*!< I2C Interrupt clear register,     Address offset: 0x1C */
	volatile uint32_t PECR; /*!< I2C PEC register,                 Address offset: 0x20 */
	volatile uint32_t RXDR; /*!< I2C Receive data register,        Address offset: 0x24 */
	volatile uint32_t TXDR; /*!< I2C Transmit data register,       Address offset: 0x28 */
} I2C_TypeDef_t;

/**
 * @brief TIM Register Structure for STM32H503 (TIM2/TIM3 - General Purpose Timers)
 * @note All registers are volatile to prevent compiler optimization
 */
typedef struct {
	volatile uint32_t CR1; /*!< TIM Control register 1,                     Address offset: 0x00 */
	volatile uint32_t CR2; /*!< TIM Control register 2,                     Address offset: 0x04 */
	volatile uint32_t SMCR; /*!< TIM Slave mode control register,            Address offset: 0x08 */
	volatile uint32_t DIER; /*!< TIM DMA/interrupt enable register,          Address offset: 0x0C */
	volatile uint32_t SR; /*!< TIM Status register,                        Address offset: 0x10 */
	volatile uint32_t EGR; /*!< TIM Event generation register,              Address offset: 0x14 */
	volatile uint32_t CCMR1; /*!< TIM Capture/compare mode register 1,        Address offset: 0x18 */
	volatile uint32_t CCMR2; /*!< TIM Capture/compare mode register 2,        Address offset: 0x1C */
	volatile uint32_t CCER; /*!< TIM Capture/compare enable register,        Address offset: 0x20 */
	volatile uint32_t CNT; /*!< TIM Counter register,                       Address offset: 0x24 */
	volatile uint32_t PSC; /*!< TIM Prescaler,                              Address offset: 0x28 */
	volatile uint32_t ARR; /*!< TIM Auto-reload register,                   Address offset: 0x2C */
	volatile uint32_t RESERVED1; /*!< Reserved,                                   Address offset: 0x30 */
	volatile uint32_t CCR1; /*!< TIM Capture/compare register 1,             Address offset: 0x34 */
	volatile uint32_t CCR2; /*!< TIM Capture/compare register 2,             Address offset: 0x38 */
	volatile uint32_t CCR3; /*!< TIM Capture/compare register 3,             Address offset: 0x3C */
	volatile uint32_t CCR4; /*!< TIM Capture/compare register 4,             Address offset: 0x40 */
	volatile uint32_t RESERVED2[5]; /*!< Reserved,                                 Address offset: 0x44-0x54 */
	volatile uint32_t ECR; /*!< TIM Encoder control register,               Address offset: 0x58 */
	volatile uint32_t TISEL; /*!< TIM Timer input selection register,         Address offset: 0x5C */
	volatile uint32_t AF1; /*!< TIM Alternate function register 1,          Address offset: 0x60 */
	volatile uint32_t AF2; /*!< TIM Alternate function register 2,          Address offset: 0x64 */
	volatile uint32_t RESERVED3[221]; /*!< Reserved,                               Address offset: 0x68-0x3D8 */
	volatile uint32_t DCR; /*!< TIM DMA control register,                   Address offset: 0x3DC */
	volatile uint32_t DMAR; /*!< TIM DMA address for full transfer,          Address offset: 0x3E0 */
} TIM_TypeDef_t;

/***** Base addresses as pointer instances for peripheral access *****/
#define RCC_t             ((RCC_TypeDef_t *) RCC_BASE_ADDR)
#define PWR_t             ((PWR_TypeDef_t *) PWR_BASE_ADDR)
#define GPIOA_t           ((GPIO_TypeDef_t *) GPIOA_BASE_ADDR)
#define GPIOB_t           ((GPIO_TypeDef_t *) GPIOB_BASE_ADDR)
#define GPIOC_t           ((GPIO_TypeDef_t *) GPIOC_BASE_ADDR)
#define FDCAN1_t          ((FDCAN_TypeDef_t *) FDCAN1_BASE_ADDR)
#define FLASH_t           ((FLASH_TypeDef_t *) FLASH_BASE_ADDR)
#define ICACHE_t          ((ICACHE_TypeDef_t *) ICACHE_BASE_ADDR)
#define I2C2_t              ((I2C_TypeDef_t *) I2C2_BASE_ADDR)
#define TIM2_t                  ((TIM_TypeDef_t *) TIM2_BASE)

/***** Clock Enable Macros *****/
#define GPIOA_CLK_EN()    (SET_BIT_FIELD(RCC->AHB2ENR, 0))   // Enable GPIOA clock
#define GPIOB_CLK_EN()    (SET_BIT_FIELD(RCC->AHB2ENR, 1))   // Enable GPIOB clock
#define GPIOC_CLK_EN()    (SET_BIT_FIELD(RCC->AHB2ENR, 2))   // Enable GPIOC clock
#define ICACHE_EN()       (SET_BIT_FIELD(ICACHE_t->CR, 0))   // Enable Instruction Cache
#define FDCAN1_CLK_EN()   (SET_BIT_FIELD(RCC_t->APB1HENR, 9)) // Enable FDCAN1 clock
#define I2C2_CLK_EN() (SET_BIT_FIELD(RCC_t->APB1LENR, 22)) // Enable I2C2 clock

/***** GPIO Pin Control Macros *****/
/* Create a pin mask for specified pin number */
#define GPIO_PIN_MASK(pin)  (1U << (pin))
/* Create bit pattern to set a pin (for BSRR register) */
#define GPIO_PIN_SET(pin)   GPIO_PIN_MASK(pin)
/* Create bit pattern to reset a pin (for BSRR register) */
#define GPIO_PIN_RESET(pin) (GPIO_PIN_MASK(pin) << 16)

/****************************************************************************
 * Handler Structures
 *
 * These structures bundle peripheral configurations into coherent objects
 * for easier initialization and management.
 ****************************************************************************/

/***** FDCAN Handler Structure *****/
typedef struct {
	FDCAN_TypeDef_t *Instace;          // FDCAN peripheral instance
	uint8_t mode;                     // Operating mode (normal, loopback, etc.)
	uint8_t ntseg2;                    // Time segment 2 (phase 2)
	uint8_t ntseg1;                    // Time segment 1 (prop seg + phase 1)
	uint8_t psc;                       // Prescaler - controls time quantum
	uint8_t tjw;                       // Time synchronization jump width
	uint8_t AutoRetransmission;        // Auto retransmission enable/disable
	uint8_t TxFifoQueueMode;           // TX FIFO or Queue mode
	uint8_t RxFifoOperationMode;  // RX FIFO operation mode (overwrite/blocking)
	uint32_t FrameFormat;              // Frame format (classic/FD)
	uint8_t StdFiltersNbr; // Specifies the number of standard Message ID filters
	uint8_t ExtFiltersNbr; // Specifies the number of Extended Message ID filters
} FDCAN_Handle_Typedef_t;

typedef struct {
	uint32_t IdType; /*!< Specifies the identifier type.
	 This parameter can be a value of @ref FDCAN_id_type       */

	uint32_t FilterIndex; /*!< Specifies the filter which will be initialized.
	 This parameter must be a number between:
	 - 0 and (SRAMCAN_FLS_NBR-1), if IdType is FDCAN_STANDARD_ID
	 - 0 and (SRAMCAN_FLE_NBR-1), if IdType is FDCAN_EXTENDED_ID */

	uint32_t FilterType; /*!< Specifies the filter type.
	 This parameter can be a value of @ref FDCAN_filter_type.
	 The value FDCAN_FILTER_RANGE_NO_EIDM is permitted
	 only when IdType is FDCAN_EXTENDED_ID.                    */

	uint32_t FilterConfig; /*!< Specifies the filter configuration.
	 This parameter can be a value of @ref FDCAN_filter_config */

	uint32_t FilterID1; /*!< Specifies the filter identification 1.
	 This parameter must be a number between:
	 - 0 and 0x7FF, if IdType is FDCAN_STANDARD_ID
	 - 0 and 0x1FFFFFFF, if IdType is FDCAN_EXTENDED_ID       */

	uint32_t FilterID2; /*!< Specifies the filter identification 2.
	 This parameter must be a number between:
	 - 0 and 0x7FF, if IdType is FDCAN_STANDARD_ID
	 - 0 and 0x1FFFFFFF, if IdType is FDCAN_EXTENDED_ID       */

} FDCAN_FilterTypeDef_t;

typedef struct {
	uint8_t ErrorStateIndicator;
	uint8_t DataLength;
	uint8_t BitRateSwitch;
	uint32_t Identifier;
	uint8_t IdType;
	uint8_t RxFrameType;
	uint8_t IsFilterMatchingFrame;
	uint32_t FilterIndex;
	uint32_t RxTimestamp;
	uint8_t FDFormat;
} FDCAN_RX_HEADER;

typedef struct {
	uint32_t Identifier; /*!< Specifies the identifier.
	 This parameter must be a number between:
	 - 0 and 0x7FF, if IdType is FDCAN_STANDARD_ID
	 - 0 and 0x1FFFFFFF, if IdType is FDCAN_EXTENDED_ID               */

	uint32_t IdType; /*!< Specifies the identifier type for the message that will be
	 transmitted.
	 This parameter can be a value of @ref FDCAN_id_type               */

	uint32_t TxFrameType; /*!< Specifies the frame type of the message that will be transmitted.
	 This parameter can be a value of @ref FDCAN_frame_type            */

	uint32_t DataLength; /*!< Specifies the length of the frame that will be transmitted.
	 This parameter can be a value of @ref FDCAN_data_length_code     */

	uint32_t ErrorStateIndicator; /*!< Specifies the error state indicator.
	 This parameter can be a value of @ref FDCAN_error_state_indicator */

	uint32_t BitRateSwitch; /*!< Specifies whether the Tx frame will be transmitted with or without
	 bit rate switching.
	 This parameter can be a value of @ref FDCAN_bit_rate_switching    */

	uint32_t FDFormat; /*!< Specifies whether the Tx frame will be transmitted in classic or
	 FD format.
	 This parameter can be a value of @ref FDCAN_format                */

	uint32_t TxEventFifoControl; /*!< Specifies the event FIFO control.
	 This parameter can be a value of @ref FDCAN_EFC                   */

	uint32_t MessageMarker; /*!< Specifies the message marker to be copied into Tx Event FIFO
	 element for identification of Tx message status.
	 This parameter must be a number between 0 and 0xFF                */

} FDCAN_TxHeaderTypeDef_t;

/***** GPIO Handler Structure *****/
typedef struct {
	GPIO_TypeDef_t *Instace;           // GPIO port instance
	uint8_t pin;                       // Pin number (0-15)
	uint8_t mode;                    // Pin mode (Input/Output/Alternate/Analog)
	uint8_t otype;                     // Output type (Push-Pull/Open-Drain)
	uint8_t ospeed;                    // Output speed
	uint8_t pupd;                      // Pull-up/Pull-down configuration
	uint8_t af;                        // Alternate function selection
} GPIO_Handle_Typedef_t;

/***** Function Prototypes *****/
void CAN1_Tx(FDCAN_Handle_Typedef_t *hFDCAN, FDCAN_TxHeaderTypeDef_t *hTXHeader,
		uint8_t *pTXData); // Transmit CAN message
void CAN1_Rx(FDCAN_Handle_Typedef_t *hFDCAN, FDCAN_RX_HEADER *hRXHeader,
		uint8_t *receivedData); // Receive CAN message
void SYSTEM_CLOCK_CONFIG(void);        // Configure system clock
void GPIO_INIT_t(GPIO_Handle_Typedef_t *hGPIOx); // Initialize GPIO pin
void GPIO_OUTPUT_t(GPIO_TypeDef_t *GPIOx, uint8_t pin, uint8_t val); // Set GPIO output
void USER_FDCAN_INIT(void);            // Initialize FDCAN with user settings
void USER_GPIOA_INIT(void);            // Initialize GPIOA pins for FDCAN
void USER_GPIOB_INIT(void);           // Initialize GPIOB pins for External LEDS
void USER_GPIOC_INIT(void);            // Initialize GPIOC pins for LED
void FDCAN_INIT(FDCAN_Handle_Typedef_t *hfdCAN1_Handle_t); // Initialize FDCAN peripheral
void USER_FDCAN_Config_Filter();
void FDCAN_FILTER_INIT(FDCAN_FilterTypeDef_t *hFilter);
void FDCAN_CONFIG_GLOBAL_FILTER(FDCAN_Handle_Typedef_t *hfdCan1,
		uint8_t rejectRemoteFrameExtended, uint8_t rejectRemoteFrameStandard,
		uint8_t AcceptNonMatchingFrameExtended,
		uint8_t AcceptNonMatchingFrameStandard);
void USER_CAN_RX();
void USER_CAN_TX();
uint8_t GPIO_INPUT_t(GPIO_TypeDef_t *GPIOx, uint8_t pin); // Read GPIO input
uint8_t FDCAN_GET_FREE_RXFIFO_LEVEL(FDCAN_Handle_Typedef_t *hFDCAN,
		uint32_t RxFifo);
void I2C_INIT();
void delayUS(uint32_t us);
void delayMS(uint32_t ms);
void I2C_WRITE(uint8_t addr, uint8_t data);
void lcd_write_4_bit(uint8_t addr, uint8_t nibble, uint8_t rs, uint8_t rw);
void lcd_send_cmd(uint8_t addr, uint8_t cmd);
void print_char(uint8_t addr, uint8_t data);
void lcd_set_cursor(uint8_t row, uint8_t column);
void print_string(uint8_t addr, char *data);
void lcd_init();
void lcd_clear();

#define DISPLAY_CLEAR 0x1
#define RETURN_HOME 0x2
#define DISPLAY_ON_CURSOR_OFF 0xC
#define FUNCTION_SET 0x28
#define ENTRY_MODE 0x6
#define NEW_LINE 0xC0
#define FIRST_ROW 0x80
#define SECOND_ROW 0xC0

#define LCD_I2C_ADDR    0x4E    // LCD I2C address (0x27 << 1)
#define LCD_BACKLIGHT   0x08    // Backlight bit
#define LCD_ENABLE      0x04    // Enable bit
#define LCD_RW          0x02    // Read/Write bit
#define LCD_RS          0x01    // Register Select bit

/***** Global Handler Instances *****/
FDCAN_Handle_Typedef_t hfdCan1;        // FDCAN1 handler
FDCAN_FilterTypeDef_t hFilter;
FDCAN_RX_HEADER hRXHeader;
FDCAN_TxHeaderTypeDef_t hTXHeader;
GPIO_Handle_Typedef_t hGPIOA;          // GPIOA handler
GPIO_Handle_Typedef_t hGPIOB;          // GPIOC handler
GPIO_Handle_Typedef_t hGPIOC;          // GPIOC handler

uint8_t receivedData[8];
uint8_t *send;
/****************************************************************************
 * Main Function
 *
 * Entry point that initializes the peripherals and contains the main loop
 * for the CAN communication demonstration.
 ****************************************************************************/
int main(void) {
	/* System initialization */
	SYSTEM_CLOCK_CONFIG();             // Configure system clock
	ICACHE_EN();                     // Enable instruction cache for performance

	/* Configure PA11 (FDCAN1_RX) and PA12 (FDCAN1_TX) */
	USER_GPIOA_INIT();                 // Initialize GPIOA pins for FDCAN

	/* Configure GPIOB pins for LED status indicators */
	USER_GPIOB_INIT();               // Initialize GPIOB pins for LED indicators

	// Config TIMER2
	// Enable TIM2
	SET_BIT_FIELD(RCC_t->APB1LENR, 0);

	// Prescaler for TIM2 --> 250Mhz to 1Mhz (each count will be 1us)
	WRITE_REG_BIT(TIM2_t->PSC, 249, 0);

	// ARR max value
	WRITE_REG_BIT(TIM2_t->ARR, 0xFFFF, 0);

	// ENABLE COUNTER
	SET_BIT_FIELD(TIM2_t->CR1, 0);

	// Update interrupt flag
	while (!READ_BIT_FIELD(TIM2_t->SR, 0, 0x1))
		;

	// I2C Init
	I2C_INIT();

	// NVIC I2C2 event interrupt at bit 53
	*NVIC_ISER1_p |= (1 << (I2C2_EV_IRQ_t % 32));

	// Set TXIE: TX interrupt enable
	SET_BIT_FIELD(I2C2_t->CR1, 1);

	// Set TCIE: Transfer complete interrupt enable
	SET_BIT_FIELD(I2C2_t->CR1, 6);

	// LCD Init
	lcd_init();

	lcd_clear();
	// Print char
	lcd_set_cursor(1, 5);
	print_string(0x4E, "LCD I2C");
	lcd_set_cursor(2, 3);
	print_string(0x4E, "Hello World");

	delayMS(1000);
	lcd_clear();

	/* Configure FDCAN peripheral */
	USER_FDCAN_INIT();                 // Setup FDCAN with specific parameters

	USER_FDCAN_Config_Filter();

	// Enable Interrupt for FDCAN at bit 39 (IRQ39)
	*NVIC_ISER1_p |= (1 << (FDCAN1_IT0_IRQ_t % 32));

	// Enable the Rx FIFO 0 new message interrupt
	SET_BIT_FIELD(hfdCan1.Instace->IE, 0);

	// FDCAN interrupt line select register (FDCAN_ILS)
	// BIT 0 --> LINE 0
	// BIT 1 --> LINE1
	CLEAR_BIT_FIELD(hfdCan1.Instace->ILS, 0);

	// FDCAN interrupt line enable register (FDCAN_ILE)
	SET_BIT_FIELD(hfdCan1.Instace->ILE, 0);

	/* Exit initialization mode to enter normal operation */
	FDCAN_EXIT_INIT_MODE(hfdCan1.Instace);

	/* Configure PC13 for LED blinking */
	USER_GPIOC_INIT();                 // Initialize GPIOC pin for status LED
	/* Main application loop */
	while (1) {
		// Do CAN operation first
		USER_CAN_TX();
//		delayMS(10); // Wait for I2C bus to be free
		// Then do LCD operations
		lcd_clear();
//		delayMS(5);
		lcd_set_cursor(1, 1);
//		delayMS(5);
		print_string(0x4E, "Sent: ");
//		delayMS(5);
		print_string(0x4E, (char*) send);
//		delayMS(5);

		lcd_set_cursor(2, 1);
//		delayMS(5);
		print_string(0x4E, "Received: ");
//		delayMS(5);
		print_string(0x4E, (char*) receivedData);

		// LED operations
		GPIO_OUTPUT_t(GPIOC_t, 13, LOW);
		delayMS(100);
		GPIO_OUTPUT_t(GPIOC_t, 13, HIGH);
		delayMS(100);

	}
}

/****************************************************************************
 * User Configuration Functions
 *
 * These functions set up the peripherals with user-specific configurations.
 ****************************************************************************/

/**
 * @brief  Configure FDCAN1 with user settings
 * @note   Sets up bit timing, mode, frame format and other parameters
 */
void USER_FDCAN_INIT() {
	hfdCan1.mode = FDCAN_MODE_NORMAL;        // Normal operating mode
	hfdCan1.AutoRetransmission = ENABLE;        // Enable auto retransmission
	hfdCan1.FrameFormat = FDCAN_FRAME_CLASSIC;  // Use classic CAN format
	hfdCan1.TxFifoQueueMode = FDCAN_TXBUFFER_FIFO; // Use FIFO for transmission
	hfdCan1.ntseg1 = 0x7;                      // Time segment 1 (8 time quanta)
	hfdCan1.ntseg2 = 0x2;                      // Time segment 2 (3 time quanta)
	hfdCan1.psc = 25;                           // Prescaler for bit timing
	hfdCan1.tjw = 1;                            // Resynchronization jump width
	hfdCan1.Instace = FDCAN1_t;                 // Use FDCAN1 peripheral
	hfdCan1.StdFiltersNbr = 1;
	hfdCan1.ExtFiltersNbr = 0;
	FDCAN_INIT(&hfdCan1);                       // Apply configuration
}

/**
 * @brief  Initialize FDCAN peripheral
 * @param  hfdCAN1_Handle_t: Pointer to FDCAN handler structure
 * @note   Configures the FDCAN peripheral based on handler parameters
 */
void FDCAN_INIT(FDCAN_Handle_Typedef_t *hfdCAN1_Handle_t) {
	/* Enable clock for FDCAN peripheral */
	if (hfdCAN1_Handle_t->Instace == FDCAN1_t) {
		FDCAN1_CLK_EN();
	}

	/* Enter initialization mode to modify configuration registers */
	FDCAN_ENTER_INIT_MODE(hfdCAN1_Handle_t->Instace);

	/* Configure test mode if specified */
	if (hfdCAN1_Handle_t->mode == FDCAN_MODE_EXTERNAL_LOOPBACK) {
		/* External loopback mode: Disconnected from CAN bus, TX connected to RX internally */
		FDCAN_ENABLE_EXTERNAL_LOOPBACK(hfdCAN1_Handle_t->Instace);
	} else if (hfdCAN1_Handle_t->mode == FDCAN_MODE_INTERNAL_LOOPBACK) {
		/* Internal loopback mode: Connected to CAN bus, messages looped back internally */
		FDCAN_ENABLE_INTERNAL_LOOPBACK(hfdCAN1_Handle_t->Instace);
	} else {
		/* Normal mode: Connected to CAN bus */
		/* No specific configuration needed for normal mode */
	}

	// Number of standard/extended filter elements in the list
	WRITE_REG_BIT(hfdCAN1_Handle_t->Instace->RXGFC, 0, 16);
	WRITE_REG_BIT(hfdCAN1_Handle_t->Instace->RXGFC, 0, 24);
	SET_VAL_BIT(hfdCAN1_Handle_t->Instace->RXGFC,
			hfdCAN1_Handle_t->StdFiltersNbr, 16);
	SET_VAL_BIT(hfdCAN1_Handle_t->Instace->RXGFC,
			hfdCAN1_Handle_t->ExtFiltersNbr, 24);

	/* Configure bit timing for classical CAN frame format */
	if (hfdCAN1_Handle_t->mode == FDCAN_FRAME_CLASSIC) {
		/* Reset the nominal bit timing register before configuring */
		WRITE_ALL_REG(hfdCAN1_Handle_t->Instace->NBTP, 0);

		/* Configure time segment 2 (phase2) [bits 0-7] */
		SET_VAL_BIT(hfdCAN1_Handle_t->Instace->NBTP,
				hfdCAN1_Handle_t->ntseg2 - 1, 0);

		/* Configure time segment 1 (prop_seg + phase1) [bits 8-15] */
		SET_VAL_BIT(hfdCAN1_Handle_t->Instace->NBTP,
				hfdCAN1_Handle_t->ntseg1 - 1, 8);

		/* Configure prescaler (controls time quantum length) [bits 16-24] */
		SET_VAL_BIT(hfdCAN1_Handle_t->Instace->NBTP, hfdCAN1_Handle_t->psc - 1,
				16);

		/* Configure sync jump width [bits 25-28] */
		if (hfdCAN1_Handle_t->tjw == 1) {
			/* Default TJW, no need to set */
			CLEAR_BIT_FIELD(hfdCAN1_Handle_t->Instace->NBTP, 25);
		} else {
			/* Custom TJW value */
			SET_VAL_BIT(hfdCAN1_Handle_t->Instace->NBTP,
					hfdCAN1_Handle_t->tjw - 1, 25);
		}

		/* Configure for classic CAN mode operation */
		FDCAN_ENABLE_CLASSICAL_CAN_MODE(hfdCAN1_Handle_t->Instace);
	}

	/* Configure TX buffer mode (FIFO or Queue) */
	if (hfdCAN1_Handle_t->TxFifoQueueMode == FDCAN_TXBUFFER_FIFO) {
		/* TX FIFO mode: messages transmitted in order they were put in */
		FDCAN_ENABLE_TX_FIFO(hfdCAN1_Handle_t->Instace);
	} else {
		/* TX Queue mode: messages transmitted based on identifier priority */
		FDCAN_ENABLE_TX_QUEUE(hfdCAN1_Handle_t->Instace);
	}

	/* Configure RX FIFO operation mode */
	if (hfdCAN1_Handle_t->RxFifoOperationMode == FDCAN_FIFO0_OVERWRITE_MODE) {
		/* Overwrite mode: New messages overwrite oldest when FIFO is full */
		FDCAN_ENABLE_FIFO0_OVERWRITE(hfdCAN1_Handle_t->Instace);
	}
	/* Else: blocking mode is the default (no overwrite) */
}

void USER_FDCAN_Config_Filter() {
	hFilter.IdType = FDCAN_STANDARD_ID;
	hFilter.FilterConfig = FDCAN_FILTER_RXFIFO0;
	hFilter.FilterIndex = 0;
	hFilter.FilterType = FDCAN_FILTER_MASK_t;
	hFilter.FilterID1 = 0x125;
	hFilter.FilterID2 = 0x7FF;

	FDCAN_FILTER_INIT(&hFilter);

	FDCAN_CONFIG_GLOBAL_FILTER(&hfdCan1, FDCAN_FILTER_REMOTE_t,
	FDCAN_FILTER_REMOTE_t, FDCAN_REJECT_t, FDCAN_REJECT_t);
}

#define SRAMCAN_FLS_SIZE (1*4)

void FDCAN_FILTER_INIT(FDCAN_FilterTypeDef_t *hFilter) {
	uint32_t *FilterAddress = (uint32_t*) (SRAMCAN_BASE_ADDR
			+ (hFilter->FilterIndex * SRAMCAN_FLS_SIZE));
// Build Word for Standard message ID filter element
	uint32_t firstElement = (hFilter->FilterType << 30
			| hFilter->FilterConfig << 27 | hFilter->FilterID1 << 16
			| hFilter->FilterID2 << 0);
	*FilterAddress = firstElement;
}

void FDCAN_CONFIG_GLOBAL_FILTER(FDCAN_Handle_Typedef_t *hfdCan1,
		uint8_t rejectRemoteFrameExtended, uint8_t rejectRemoteFrameStandard,
		uint8_t AcceptNonMatchingFrameExtended,
		uint8_t AcceptNonMatchingFrameStandard) {
	CLEAR_BIT_FIELD(hfdCan1->Instace->RXGFC, 0);
	SET_VAL_BIT(hfdCan1->Instace->RXGFC, rejectRemoteFrameExtended, 0);

	CLEAR_BIT_FIELD(hfdCan1->Instace->RXGFC, 1);
	SET_VAL_BIT(hfdCan1->Instace->RXGFC, rejectRemoteFrameStandard, 1);

	CLEAR_VAL_BIT(hfdCan1->Instace->RXGFC, 0x3, 2);
	SET_VAL_BIT(hfdCan1->Instace->RXGFC, AcceptNonMatchingFrameExtended, 2);

	CLEAR_VAL_BIT(hfdCan1->Instace->RXGFC, 0x3, 2);
	SET_VAL_BIT(hfdCan1->Instace->RXGFC, AcceptNonMatchingFrameStandard, 3);
}

/**
 * @brief  Initialize GPIOA pins for FDCAN
 * @note   Configures PA11 as FDCAN1_RX and PA12 as FDCAN1_TX
 */
void USER_GPIOA_INIT() {
	/* Configure PA11 for FDCAN1_RX */
	hGPIOA.Instace = GPIOA_t;
	hGPIOA.pin = GPIO_PIN_11;           // PA11 is FDCAN1_RX
	hGPIOA.af = GPIO_AF9;               // Alternate Function 9 for FDCAN
	hGPIOA.mode = GPIO_ALT_MODE;        // Alternate function mode
	hGPIOA.ospeed = GPIO_OUTPUT_VHS;    // Very high speed
	hGPIOA.otype = GPIO_OUTPUT_PP;      // Push-pull output
	hGPIOA.pupd = GPIO_NO_PUPD;         // No pull-up or pull-down
	GPIO_INIT_t(&hGPIOA);               // Apply configuration

	/* Configure PA12 for FDCAN1_TX */
	hGPIOA.pin = GPIO_PIN_12;           // PA12 is FDCAN1_TX
	GPIO_INIT_t(&hGPIOA);               // Apply same configuration to PA12
}

/**
 * @brief  Initialize GPIOB pin for LED
 * @note   Configures PB4, PB5, PB6 as output for LED control
 */
void USER_GPIOB_INIT(void) {
	/* Configure PC13 for LED output */
	hGPIOB.Instace = GPIOB_t;
	hGPIOB.pin = GPIO_PIN_0;           // PB0
	hGPIOB.af = GPIO_AF_NONE;           // No alternate function
	hGPIOB.mode = GPIO_OUTPUT_MODE;     // Output mode
	hGPIOB.ospeed = GPIO_OUTPUT_VHS;    // Very high speed
	hGPIOB.otype = GPIO_OUTPUT_PP;      // Push-pull output
	hGPIOB.pupd = GPIO_NO_PUPD;         // No pull-up or pull-down
	GPIO_INIT_t(&hGPIOB);               // Apply configuration

	/* Configure PB5 for LED control */
	hGPIOB.pin = GPIO_PIN_1;           // PB1
	GPIO_INIT_t(&hGPIOB);               // Apply same configuration to PB5

	/* Configure PB6 for LED control */
	hGPIOB.pin = GPIO_PIN_2;           // PB2
	GPIO_INIT_t(&hGPIOB);               // Apply same configuration to PB6

	hGPIOB.af = GPIO_AF8;
	hGPIOB.pin = GPIO_PIN_5; // PB5 - I2C2_SCL
	hGPIOB.mode = GPIO_ALT_MODE;
	hGPIOB.ospeed = GPIO_OUTPUT_VHS;
	hGPIOB.otype = GPIO_OUTPUT_OD;
	hGPIOB.pupd = GPIO_PU;

	GPIO_INIT_t(&hGPIOB);

	hGPIOB.pin = GPIO_PIN_4; // PB4 - I2C2_SDA
	GPIO_INIT_t(&hGPIOB);
}

/**
 * @brief  Initialize GPIOC pin for LED
 * @note   Configures PC13 as output for LED control
 */
void USER_GPIOC_INIT() {
	/* Configure PC13 for LED output */
	hGPIOC.Instace = GPIOC_t;
	hGPIOC.pin = GPIO_PIN_13;           // PC13 connected to onboard LED
	hGPIOC.af = GPIO_AF_NONE;           // No alternate function
	hGPIOC.mode = GPIO_OUTPUT_MODE;     // Output mode
	hGPIOC.ospeed = GPIO_OUTPUT_VHS;    // Very high speed
	hGPIOC.otype = GPIO_OUTPUT_PP;      // Push-pull output
	hGPIOC.pupd = GPIO_NO_PUPD;         // No pull-up or pull-down
	GPIO_INIT_t(&hGPIOC);               // Apply configuration
}

/**
 * @brief  Initialize GPIO pin
 * @param  hGPIOx: Pointer to GPIO handler structure
 * @note   Configures GPIO pin based on handler parameters
 */
void GPIO_INIT_t(GPIO_Handle_Typedef_t *hGPIOx) {
	/* Enable clock for the appropriate GPIO port */
	if (hGPIOx->Instace == GPIOA_t) {
		GPIOA_CLK_EN();                 // Enable GPIOA peripheral clock
	} else if (hGPIOx->Instace == GPIOB_t) {
		GPIOB_CLK_EN();                 // Enable GPIOB peripheral clock
	} else if (hGPIOx->Instace == GPIOC_t) {
		GPIOC_CLK_EN();                 // Enable GPIOC peripheral clock
	}

	/* Calculate pin position for 2-bit fields (each pin uses 2 bits in registers) */
	uint8_t pin_pos = PIN_MODE_POS(hGPIOx->pin);  // PIN_MODE_POS(pin) = pin * 2

	/* Configure pin mode (Input, Output, Alternate, Analog) */
	CLEAR_VAL_BIT(hGPIOx->Instace->MODER, BIT_MASK_2, pin_pos);
	SET_VAL_BIT(hGPIOx->Instace->MODER, hGPIOx->mode, pin_pos);

	/* Configure pull-up/pull-down resistors */
	CLEAR_VAL_BIT(hGPIOx->Instace->PUPDR, BIT_MASK_2, pin_pos);
	SET_VAL_BIT(hGPIOx->Instace->PUPDR, hGPIOx->pupd, pin_pos);

	/* Only configure output parameters for Output or Alternate Function modes */
	if (hGPIOx->mode == GPIO_OUTPUT_MODE || hGPIOx->mode == GPIO_ALT_MODE) {
		/* Configure output type (Push-Pull or Open-Drain) */
		if (hGPIOx->otype == GPIO_OUTPUT_PP) {
			CLEAR_BIT_FIELD(hGPIOx->Instace->OTYPER, hGPIOx->pin); // Push-pull (0)
		} else if (hGPIOx->otype == GPIO_OUTPUT_OD) {
			SET_BIT_FIELD(hGPIOx->Instace->OTYPER, hGPIOx->pin); // Open-drain (1)
		}

		/* Configure output speed (Low, Medium, High or Very High) */
		CLEAR_VAL_BIT(hGPIOx->Instace->OSPEEDR, BIT_MASK_2, pin_pos);
		SET_VAL_BIT(hGPIOx->Instace->OSPEEDR, hGPIOx->ospeed, pin_pos);

		/* Configure Alternate Function (only if mode is Alternate Function) */
		if (hGPIOx->mode == GPIO_ALT_MODE) {
			/* Determine which AFR register to use (AFRL for pins 0-7, AFRH for pins 8-15) */
			uint8_t afrIndex = PIN_AFR_REG(hGPIOx->pin);

			/* Calculate bit position within the AFR register (each pin uses 4 bits) */
			uint8_t afrPosition = PIN_AFR_POS(hGPIOx->pin);

			/* Clear and set alternate function value */
			CLEAR_VAL_BIT(hGPIOx->Instace->AFR[afrIndex], BIT_MASK_4,
					afrPosition);
			SET_VAL_BIT(hGPIOx->Instace->AFR[afrIndex], hGPIOx->af,
					afrPosition);
		}
	}
}

/**
 * @brief  Set GPIO output value
 * @param  GPIOx: GPIO port
 * @param  pin: Pin number (0-15)
 * @param  val: Output value (HIGH/LOW)
 * @note   Uses BSRR register for atomic bit set/reset
 */
void GPIO_OUTPUT_t(GPIO_TypeDef_t *GPIOx, uint8_t pin, uint8_t val) {
	if (val == HIGH) {
		GPIOx->BSRR = GPIO_PIN_SET(pin);    // Set pin high (atomic operation)
	} else {
		GPIOx->BSRR = GPIO_PIN_RESET(pin);  // Set pin low (atomic operation)
	}
}

/**
 * @brief  Read GPIO input value
 * @param  GPIOx: GPIO port
 * @param  pin: Pin number (0-15)
 * @retval Pin state (HIGH/LOW)
 */
uint8_t GPIO_INPUT_t(GPIO_TypeDef_t *GPIOx, uint8_t pin) {
	if (READ_BIT_FIELD(GPIOx->IDR, pin, BIT_MASK_1)) {
		return HIGH;    // Pin is high
	} else {
		return LOW;     // Pin is low
	}
}

void USER_CAN_RX() {
	/* Receive CAN message */
	CAN1_Rx(&hfdCan1, &hRXHeader, receivedData); // Receive CAN message
}

void USER_CAN_TX() {
	//		/* Transmit CAN message */
	send = (uint8_t*) "Hi";
	hTXHeader.BitRateSwitch = 0;
	hTXHeader.DataLength = FDCAN_DLC_BYTES_2;
	hTXHeader.ErrorStateIndicator = 0;
	hTXHeader.FDFormat = 0;
	hTXHeader.IdType = 0;
	hTXHeader.Identifier = 0x123;
	hTXHeader.MessageMarker = 0;
	hTXHeader.TxEventFifoControl = 0;
	hTXHeader.TxFrameType = 0;
	CAN1_Tx(&hfdCan1, &hTXHeader, (uint8_t*) send);
}

volatile uint8_t tc = 1;

void I2C2_EV_IRQHandler() {
	// If Transmit interrupt status is set by hardware when the I2C_TXDR register is empty
	if (READ_BIT_FIELD(I2C2_t->ISR, 1, 0x1)) {
		// Write to the data register
		// Write to the data register will clear this Transmit interrupt status bit
		WRITE_REG_BIT(I2C2_t->TXDR, data_to_send, 0);
	}

	// If transmit has completed
	if (READ_BIT_FIELD(I2C2_t->ISR, 6, 0x1)) {
		// Generate stop condition
		//  The bit TC: Transfer complete is cleared by software when START bit or STOP bit is set.
		SET_BIT_FIELD(I2C2_t->CR2, 14);

		// Wait for detect STOP bit
		while (!READ_BIT_FIELD(I2C2_t->ISR, 5, 0x1))
			;

		// Clear Flag STOP
		SET_BIT_FIELD(I2C2_t->ICR, 5);
		tc = 1;
	}

}

void FDCAN1_IT0_IRQHandler() {
	// If new message has come
	if (READ_BIT_FIELD(hfdCan1.Instace->IR, 0, 0x1)) {
		// A flag is cleared by writing 1 to the corresponding bit position.
		WRITE_REG_BIT(hfdCan1.Instace->IR, 1, 0);
		// Handling RX
		USER_CAN_RX();
	}
}

/****************************************************************************
 * System Clock Configuration
 *
 * Configures the system clock and peripheral clocks for maximum performance.
 * Sets up PLL to generate high-frequency system clock from HSI.
 ****************************************************************************/

/**
 * @brief  Configure system clock for optimal performance
 * @note   Sets up PLL with HSI as source, configures flash latency and bus prescalers
 */
void SYSTEM_CLOCK_CONFIG() {
	/* Configure power mode for high-speed operation */
	CLEAR_VAL_BIT(*COMP_CFGR1_PWRMODE_p, 0x3, 12);

	/* Set voltage scaling to highest performance level (VOS0) */
	SET_VAL_BIT(PWR_t->VOSCR, 3, 4);

	/* Reset Flash configuration before changing clock */
	WRITE_ALL_REG(FLASH_t->ACR, 0);

	/* Configure Flash latency and signal delay for high-frequency operation */
	SET_VAL_BIT(FLASH_t->ACR, 5, 0);   // Set latency to 5 wait states
	SET_VAL_BIT(FLASH_t->ACR, 2, 4);   // Set delay to 2 cycles

	/* Reset PLL1 configuration register */
	WRITE_ALL_REG(RCC_t->PLL1CFGR, 0);

	/* Select HSI as PLL clock source */
	SET_BIT_FIELD(RCC_t->PLL1CFGR, 0);

	/* Set PLL input division factor (PLLM = 4) */
	SET_VAL_BIT(RCC_t->PLL1CFGR, 4, 8);

	/* Configure PLL fractional divider */
	WRITE_REG_BIT(RCC_t->PLL1FRACR, 0, 3);     // Clear first
	WRITE_REG_BIT(RCC_t->PLL1FRACR, 2048, 3);  // Set value

	/* Set PLL1 input frequency range (range 3: 4-8 MHz) */
	SET_VAL_BIT(RCC_t->PLL1CFGR, 3, 2);

	/* Enable fractional divider */
	CLEAR_BIT_FIELD(RCC_t->PLL1CFGR, 4);   // Clear first
	SET_BIT_FIELD(RCC_t->PLL1CFGR, 4);     // Set the enable bit

	/* Set PLL1 VCO selection (wide range for input frequency > 2MHz) */
	CLEAR_BIT_FIELD(RCC_t->PLL1CFGR, 5);

	/* Enable PLL output clocks: PLL1P and PLL1Q (for FDCAN) */
	SET_BIT_FIELD(RCC_t->PLL1CFGR, 16);    // Enable PLL1P output
	SET_BIT_FIELD(RCC_t->PLL1CFGR, 17);    // Enable PLL1Q output
	CLEAR_BIT_FIELD(RCC_t->PLL1CFGR, 18);  // Disable PLL1R output

	/* Configure FDCAN clock source (PLL1Q) */
	SET_BIT_FIELD(RCC_t->CCIPR5, 8);

	/* Initialize PLL1 dividers */
	WRITE_ALL_REG(RCC_t->PLL1DIVR, 0);

	/* Configure PLL multiplication and division factors */
	SET_VAL_BIT(RCC_t->PLL1DIVR, 30, 0);   // PLL1N = 31 (multiplication factor)
	SET_VAL_BIT(RCC_t->PLL1DIVR, 1, 9);    // PLL1P = 2 (division factor)
	SET_VAL_BIT(RCC_t->PLL1DIVR, 1, 16);   // PLL1Q = 2 (division factor)
	SET_VAL_BIT(RCC_t->PLL1DIVR, 1, 24);   // PLL1R = 2 (division factor)

	/* Enable High-Speed Internal oscillator (HSI) */
	SET_BIT_FIELD(RCC_t->CR, 0);

	/* Configure HSI divider to 1 */
	CLEAR_VAL_BIT(RCC_t->CR, 0x3, 3);

	/* Wait for HSI to stabilize */
	while (!(READ_BIT_FIELD(RCC_t->CR, 1, 1)))
		;   // Wait until HSI ready flag is set

	/* Enable PLL1 */
	SET_BIT_FIELD(RCC_t->CR, 24);

	/* Wait for PLL1 to lock */
	while (!(READ_BIT_FIELD(RCC_t->CR, 25, 1)))
		;   // Wait until PLL1 ready flag is set

	/* Select PLL1 as system clock source */
	SET_VAL_BIT(RCC_t->CFGR1, 3, 0);

	/* Wait until PLL1 is selected as system clock */
	while (READ_BIT_FIELD(RCC_t->CFGR1, 3, 0x3) != 0x3)
		;   // Wait until PLL1 is used as system clock source

	/* Configure bus prescalers for optimal performance */
	CLEAR_BIT_FIELD(RCC_t->CFGR2, 0);    // AHB prescaler = 1 (SYSCLK = HCLK)
	CLEAR_BIT_FIELD(RCC_t->CFGR2, 4);    // APB1 prescaler = 1 (HCLK = PCLK1)
	CLEAR_BIT_FIELD(RCC_t->CFGR2, 8);    // APB2 prescaler = 1 (HCLK = PCLK2)
	CLEAR_BIT_FIELD(RCC_t->CFGR2, 12);   // APB3 prescaler = 1 (HCLK = PCLK3)

	/* Enable AHB and APB clocks */
	CLEAR_BIT_FIELD(RCC_t->CFGR2, 16);   // Enable AHB1 clock
	CLEAR_BIT_FIELD(RCC_t->CFGR2, 17);   // Enable AHB2 clock
	CLEAR_BIT_FIELD(RCC_t->CFGR2, 20);   // Enable APB1 clock
	CLEAR_BIT_FIELD(RCC_t->CFGR2, 21);   // Enable APB2 clock
	CLEAR_BIT_FIELD(RCC_t->CFGR2, 22);   // Enable APB3 clock
}

uint8_t FDCAN_GET_FREE_TXFIFO_LEVEL(FDCAN_Handle_Typedef_t *hFDCAN) {
	return READ_BIT_FIELD(hFDCAN->Instace->TXFQS, 0, 0x7);
}

/****************************************************************************
 * CAN Message Configuration and Transmission Parameters
 ****************************************************************************/

/* Error state indicator bit */
#define ESI_BIT 0   // 0: ESI bit depends only on error passive flag

/* ID configuration (standard or extended) */
#define ID_CONFIG 0 // 0: 11-bit standard identifier

/* Remote transmission request type */
#define TX_FRAME_TYPE 0     // 0: Transmit data frame, 1: Transmit remote frame

/* Message marker for tracking messages */
#define MESSAGE_MAKER 0     // Message identifier value

/* Event FIFO control */
#define EVENT_FIFO_CONTROL 0    // 0: Do not store TX events

/* CAN FD format selection */
#define FD_FORMAT 0         // 0: Classic CAN format

/* Bit rate switching for CAN FD */
#define BIT_RATE_SWITCHING 0    // 0: No bit rate switching

/* Data length code - number of data bytes (0-8 for classic CAN) */
#define DATA_LENGTH_CODE 0x5    // 5 bytes of data

/****************************************************************************
 * CAN Message RAM Configuration
 *
 * Defines the memory layout for the FDCAN message RAM areas in SRAM.
 ****************************************************************************/

/**
 * Message RAM Layout (based on Figure 509 in RM, Section 39.3.6):
 * - Standard filters section (11-bit): 28 words
 * - Extended filters section (29-bit): 16 words
 * - RX FIFO 0 section: 54 words
 * - RX FIFO 1 section: 54 words
 * - TX event FIFO section: 6 words
 * - TX FIFO/Queue section: follows all the above
 *
 * Total offset to TX FIFO/Queue: (28 + 16 + 54 + 54 + 6) × 4 bytes = 632 bytes = 0x278
 */
#define SRAMCAN_TFQSA (0x278)    // TX FIFO/Queue start address offset

/*
 * TX element size: 18 words per element (72 bytes)
 * Each element contains header (2 words) and data field (up to 16 words)
 */
#define SRAMCAN_TFQ_SIZE (18U * 4U)    // Size of each TX FIFO/Queue element

static const uint8_t DLCtoBytes[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24,
		32, 48, 64 };

/****************************************************************************
 * CAN Transmit Function
 *
 * Transmits a CAN message over the FDCAN peripheral.
 ****************************************************************************/

/**
 * @brief  Transmit a CAN message
 * @note   Sends a message with ID 0x123 containing "HELLO" text
 */
/**
 * @brief  Transmit a CAN message with GPIOB indicator
 * @note   Sends a message with ID 0x123 containing "HELLO" text and controls GPIOB4-6 based on put_index
 */
void CAN1_Tx(FDCAN_Handle_Typedef_t *hFDCAN, FDCAN_TxHeaderTypeDef_t *hTXHeader,
		uint8_t *pTxData) {
	/* 1. Check if TX FIFO has space available */
	uint8_t fifo_free_level = FDCAN_GET_FREE_TXFIFO_LEVEL(hFDCAN);
	printf("TX FIFO free level: %d\n", fifo_free_level);

	if (fifo_free_level == 0) {
		printf("TX FIFO is full\n");
		return;  // Cannot transmit if FIFO is full
	}

	/* 2. Get the buffer index where we can put our message */
	uint8_t put_index = READ_BIT_FIELD(hFDCAN->Instace->TXFQS, 16, 0x1F);
	printf("TX buffer index: %d\n", put_index);

	/* 3. Prepare TX header words for message RAM */
	uint32_t tx_element_w1, tx_element_w2;

	/* Configure first word of TX element (T0) - Header with ID and control bits */

	/* Build the T0 register (first word)
	 * Bit 31: ESI (Error State Indicator)
	 * Bit 30: XTD (Extended Identifier - 0 for standard ID)
	 * Bit 29: RTR (Remote Transmission Request)
	 * Bits 28-18: Standard Identifier (11 bits)
	 */
	uint8_t ESI = hTXHeader->ErrorStateIndicator;
	uint8_t idConfig = hTXHeader->IdType;
	uint8_t RTR = hTXHeader->TxFrameType;
	uint32_t identifier = (hTXHeader->Identifier << 18); // Standard ID placed at bit position 18

	/* Combine all fields into first word */
	tx_element_w1 = (ESI | idConfig | RTR | identifier);
	printf("TX header word 1: 0x%08lx\n", tx_element_w1);

	/* Configure second word of TX element (T1) - Contains DLC and other control bits */
	/* Build the T1 register (second word)
	 * Bits 31-24: Message Marker (used for TX event matching)
	 * Bit 23: Event FIFO Control (store TX events)
	 * Bit 21: FD Format (CAN FD vs Classic)
	 * Bit 20: Bit Rate Switching (for CAN FD)
	 * Bits 19-16: Data Length Code (0-8 bytes for Classic CAN)
	 */
	uint32_t messageMaker = (hTXHeader->MessageMarker << 24);
	uint32_t eventFifoControl = (hTXHeader->TxEventFifoControl << 23);
	uint32_t fdFormat = (hTXHeader->FDFormat << 21);
	uint32_t BRS = (hTXHeader->BitRateSwitch << 20);
	uint32_t DLC = (hTXHeader->DataLength << 16);

	/* Combine all fields into second word */
	tx_element_w2 = (messageMaker | eventFifoControl | fdFormat | BRS | DLC);
	printf("TX header word 2: 0x%08lx\n", tx_element_w2);

	/* 4. Calculate the memory address for this TX element */
	uint32_t TxFIFOQSA = SRAMCAN_BASE_ADDR + SRAMCAN_TFQSA;
	uint32_t *tx_address = (uint32_t*) (TxFIFOQSA
			+ (put_index * SRAMCAN_TFQ_SIZE));
	printf("TX buffer address: 0x%08lx\n", (uint32_t) tx_address);

	/* 5. Write the header words to the message RAM */
	*tx_address = tx_element_w1;  // Write T0 register
	tx_address++;                 // Move to next word
	*tx_address = tx_element_w2;  // Write T1 register
	tx_address++;                 // Move to data section

	/* 6. Prepare and write data bytes */
	uint32_t ByteCounter;

	/* Write Tx payload to the message RAM */
	for (ByteCounter = 0; ByteCounter < DLCtoBytes[hTXHeader->DataLength];
			ByteCounter += 4U) {
		*tx_address = (((uint32_t) pTxData[ByteCounter + 3U] << 24U)
				| ((uint32_t) pTxData[ByteCounter + 2U] << 16U)
				| ((uint32_t) pTxData[ByteCounter + 1U] << 8U)
				| (uint32_t) pTxData[ByteCounter]);
		tx_address++;
	}
	/* 7. Request transmission by setting the corresponding bit in TXBAR register */
	printf("Requesting transmission for buffer %d\n", put_index);
	SET_BIT_FIELD(hFDCAN->Instace->TXBAR, put_index);

	/* 8. Verify if the request was accepted (added to pending list) */

	if (READ_BIT_FIELD(hFDCAN->Instace->TXBRP, put_index, 1)) {
		printf("TX request accepted and pending\n");
		/* After successful transmission, the message will be processed
		 * and the TX FIFO put_index will be incremented automatically */
	} else {
		printf("TX request not accepted\n");
	}
}

/** @defgroup FDCAN_Rx_location FDCAN Rx Location
 * @{
 */
#define FDCAN_RX_FIFO0_t    (0x0) /*!< Get received message from Rx FIFO 0    */
#define FDCAN_RX_FIFO1_t    (0x1U) /*!< Get received message from Rx FIFO 1    */
/**
 * @}
 */

uint8_t FDCAN_GET_FREE_RXFIFO_LEVEL(FDCAN_Handle_Typedef_t *hFDCAN,
		uint32_t RxFifo) {
	if (RxFifo == FDCAN_RX_FIFO0_t) {
		return READ_BIT_FIELD(hFDCAN->Instace->RXF0S, 0, 0x7F);
	} else {
		return READ_BIT_FIELD(hFDCAN->Instace->RXF1S, 0, 0x7F);
	}
}

/****************************************************************************
 * CAN Receive Function
 *
 * Configures and handles reception of CAN messages.
 ****************************************************************************/

/* RX FIFO 0 start address in message RAM */
#define SRAMCAN_RFQSA 0x00B0

/* Size of each RX FIFO element (same structure as TX element) */
#define SRAMCAN_RFQ_SIZE (18*4)

/**
 * @brief  Configure and check for received CAN messages
 * @note   Reads any available messages from RX FIFO 0
 */
/**
 * @brief  Configure and check for received CAN messages with GPIOB indicator
 * @note   Reads any available messages from RX FIFO 0 and controls GPIOB4-6 based on get_index
 */
void CAN1_Rx(FDCAN_Handle_Typedef_t *hFDCAN, FDCAN_RX_HEADER *hRXHeader,
		uint8_t *receivedData) {
	/* 1. Check if there are any messages in RX FIFO 0 */
	uint8_t fifo_level = FDCAN_GET_FREE_RXFIFO_LEVEL(hFDCAN,
	FDCAN_RX_FIFO0_t); // F0FL field

	if (fifo_level == 0) {
		printf("RX FIFO is empty\n");
		return;  // No messages to process
	}

	printf("RX FIFO level: %d\n", fifo_level);

	/* 2. Handle overwrite mode condition if enabled */
	uint8_t get_index = 0;

	/* Check if FIFO is full and in overwrite mode */
	if ((READ_BIT_FIELD(hFDCAN->Instace->RXF0S, 24, 0x1) == 1) && // F0F bit (FIFO full)
			(READ_BIT_FIELD(hFDCAN->Instace->RXGFC, 4, 0x1) == 1)) { // F0OM bit (Overwrite mode)
		printf("FIFO full in overwrite mode - incrementing get_index\n");
		get_index = 1;  // Skip oldest message to avoid race condition
	}

	/* 3. Get current get index from the status register */
	get_index += READ_BIT_FIELD(hFDCAN->Instace->RXF0S, 8, 0x3);  // F0GI field
	printf("Get index: %d\n", get_index);

	/* Control GPIOB pins based on get_index value */
	if (get_index == 0) {
		/* For get_index = 0: Set B4 HIGH, B5 and B6 LOW */
		GPIO_OUTPUT_t(GPIOB_t, 0, HIGH);
		GPIO_OUTPUT_t(GPIOB_t, 1, LOW);
		GPIO_OUTPUT_t(GPIOB_t, 2, LOW);
	} else if (get_index == 1) {
		/* For get_index = 1: Set B5 HIGH, B4 and B6 LOW */
		GPIO_OUTPUT_t(GPIOB_t, 0, LOW);
		GPIO_OUTPUT_t(GPIOB_t, 1, HIGH);
		GPIO_OUTPUT_t(GPIOB_t, 2, LOW);
	} else if (get_index == 2) {
		/* For get_index = 2: Set B6 HIGH, B4 and B5 LOW */
		GPIO_OUTPUT_t(GPIOB_t, 0, LOW);
		GPIO_OUTPUT_t(GPIOB_t, 1, LOW);
		GPIO_OUTPUT_t(GPIOB_t, 2, HIGH);
	} else {
		/* For any other get_index: Set all pins LOW */
		GPIO_OUTPUT_t(GPIOB_t, 0, LOW);
		GPIO_OUTPUT_t(GPIOB_t, 1, LOW);
		GPIO_OUTPUT_t(GPIOB_t, 2, LOW);
	}

	/* 4. Calculate address of the RX element in message RAM */
	uint32_t RxFIFOQSA = SRAMCAN_BASE_ADDR + SRAMCAN_RFQSA;
	uint32_t *rx_address = (uint32_t*) (RxFIFOQSA
			+ (get_index * SRAMCAN_RFQ_SIZE));

	printf("RX address: 0x%08X\n", (unsigned int) rx_address);

	/* 5. Extract message information from the RX element */
	/* Read first word (R0) - Contains ID and frame information */
	uint32_t word1 = *rx_address;
	uint8_t ESI = ((word1 >> 31) & 0x1);       // Error state indicator
	hRXHeader->ErrorStateIndicator = ESI;
	uint8_t bit_identifier = ((word1 >> 30) & 0x1); // 0=standard, 1=extended
	hRXHeader->IdType = bit_identifier;
	uint8_t RTR = ((word1 >> 29) & 0x1);      // Remote transmission request
	hRXHeader->RxFrameType = RTR;
	uint32_t identifier;

	printf("Word1: 0x%08X\n", (unsigned int) word1);
	printf("ESI: %d, ID Type: %s, RTR: %d\n", hRXHeader->ErrorStateIndicator,
			(bit_identifier == 0) ? "Standard" : "Extended",
			hRXHeader->RxFrameType);

	/* Extract message ID based on format */
	if (bit_identifier == 0) {  // Standard ID (11 bits)
		identifier = (word1 >> 18) & 0x7FF;
		printf("Standard ID: 0x%03lX\n", identifier);
	} else {  // Extended ID (29 bits)
		identifier = word1 & 0x1FFFFFFF;
		printf("Extended ID: 0x%08lX\n", identifier);
	}
	hRXHeader->Identifier = identifier;

	/* Read second word (R1) - Contains DLC and additional flags */
	rx_address++;
	uint32_t word2 = *rx_address;

	uint8_t ANMF = ((word2 >> 31) & 0x1);     // Accepted non-matching frame
	hRXHeader->IsFilterMatchingFrame = ANMF;
	uint8_t FRAME_FORMAT = ((word2 >> 21) & 0x1); // CAN FD format
	hRXHeader->FDFormat = FRAME_FORMAT;
	uint8_t BRS = ((word2 >> 20) & 0x1);        // Bit rate switching
	hRXHeader->BitRateSwitch = BRS;
	uint8_t DLC = ((word2 >> 16) & 0xFF);       // Data length code
	hRXHeader->DataLength = DLC;

	printf("Word2: 0x%08X\n", (unsigned int) word2);
	printf("ANMF: %d, Frame Format: %d, BRS: %d\n", ANMF, FRAME_FORMAT, BRS);
	printf("DLC: %d\n", DLC);

	/* Move to data section */
	rx_address++;

	/* Extract and display data bytes */
	uint8_t *data_ptr = (uint8_t*) rx_address;  // Use a separate pointer

	/* Copy data to the receivedData array */
	for (int i = 0; i < DLC && i < 8; i++) {  // Limit to array size
		receivedData[i] = data_ptr[i];
	}

	/* Null-terminate if treating as string */
	if (DLC < 8) {
		receivedData[DLC] = '\0';
	}

	/* Display in hexadecimal format */
	printf("Data (hex): ");
	for (int i = 0; i < DLC; i++) {
		printf("%02X ", data_ptr[i]);  // Use data_ptr for printf
	}
	printf("\n");

	/* Display as ASCII characters if printable */
	printf("Data (char): ");
	for (int i = 0; i < DLC; i++) {
		if (data_ptr[i] >= 32 && data_ptr[i] <= 126) {
			printf("%c", data_ptr[i]);
		} else {
			printf(".");
		}
	}
	printf("\n");

	/* 6. Acknowledge reading the message to free the FIFO slot */
	/* Writing to RXF0A register acknowledges that the message has been read
	 * and the hardware will increment the get index */
	hFDCAN->Instace->RXF0A = get_index;

	/* Verify that get index has been updated */
	get_index = READ_BIT_FIELD(hFDCAN->Instace->RXF0S, 8, 0x3);  // F0GI field
	printf("Message received and acknowledged. Get index: %d\n", get_index);
}

void delayUS(uint32_t us) {
	TIM2_t->CNT = 0;
	while (TIM2_t->CNT < us)
		;
}
void delayMS(uint32_t ms) {
	for (int i = 0; i < ms; i++) {
		delayUS(1000);
	}
}

void I2C_INIT() {

	// Enable I2C Clock
	I2C2_CLK_EN();

	// CLEAR PE bit in I2C_CR1;
	CLEAR_BIT_FIELD(I2C2_t->CR1, 0);

	// Config ANFOFF and DNF[3:0] in I2C2_CR1
	// Off Analog noise
	CLEAR_BIT_FIELD(I2C2_t->CR1, 12);
	SET_BIT_FIELD(I2C2_t->CR1, 12);

	// Off Digital noise
	CLEAR_VAL_BIT(I2C2_t->CR1, 0xF, 8);

	// Reset all bit
	WRITE_ALL_REG(I2C2_t->TIMINGR, 0);

	// Prescale for I2C
	SET_VAL_BIT(I2C2_t->TIMINGR, 0x2, 28);

	// SCL HIGH
	SET_VAL_BIT(I2C2_t->TIMINGR, 99, 8);

	// SCL LOW
	SET_VAL_BIT(I2C2_t->TIMINGR, 107, 0);

	// Data hold time
	SET_VAL_BIT(I2C2_t->TIMINGR, 25, 16);

	// Data setup time
	SET_VAL_BIT(I2C2_t->TIMINGR, 25, 20);

	// I2C Peripheral EN
	SET_BIT_FIELD(I2C2_t->CR1, 0);
}

void I2C_WRITE(uint8_t addr, uint8_t data) {
	// Controller mode
	if (tc == 1) {
		// 7-bit addressing mode
		CLEAR_BIT_FIELD(I2C2_t->CR2, 11);

		// Target address to send
		uint8_t address = (addr >> 1);
		CLEAR_VAL_BIT(I2C2_t->CR2, 0x7F, 1);
		SET_VAL_BIT(I2C2_t->CR2, address, 1);

		// Transfer dirrection (Write)
		CLEAR_BIT_FIELD(I2C2_t->CR2, 10);

		// Number of bytes to transfer (1 bytes)
		SET_VAL_BIT(I2C2_t->CR2, 0x1, 16);

		// Clear auto end mode
		CLEAR_BIT_FIELD(I2C2_t->CR2, 25);

		data_to_send = data;
		tc = 0;
		// Set start bit
		SET_BIT_FIELD(I2C2_t->CR2, 13);
		while (tc == 0)
			;  // Don't proceed until ISR sets tc = 1
	}
//	// Wait for busy flag is set
//	while (READ_BIT_FIELD(I2C2_t->ISR, 15, 0x1))
//		;

	// Wait for I2C_TXDR register is empty
//	while (!(READ_BIT_FIELD(I2C2_t->ISR, 1, 0x1)))
//		;

	// Write to the data register
//	WRITE_REG_BIT(I2C2_t->TXDR, data, 0);

	// Wait for trasmission complete
//	while (!(READ_BIT_FIELD(I2C2_t->ISR, 6, 0x1)))
//		;

}

// LCD I2C 4-bit Mode Functions
// PCF8574 bit mapping:
// Bit 7-4: Data nibble (D7-D4)
// Bit 3: Backlight (BL)
// Bit 2: Enable (EN)
// Bit 1: Read/Write (RW)
// Bit 0: Register Select (RS)

void lcd_write_4_bit(uint8_t addr, uint8_t nibble, uint8_t rs, uint8_t rw) {
	uint8_t data = (nibble << 4) | LCD_BACKLIGHT | (rw ? LCD_RW : 0)
			| (rs ? LCD_RS : 0);

	// Pull high EN bit
	I2C_WRITE(addr, data | LCD_ENABLE);
	delayUS(50);

	// Pull low EN bit
	I2C_WRITE(addr, data & ~LCD_ENABLE);
	delayUS(50);
}

void lcd_send_cmd(uint8_t addr, uint8_t cmd) {
	lcd_write_4_bit(addr, (cmd >> 4), 0, 0);
	lcd_write_4_bit(addr, (cmd & 0xF), 0, 0);

	// Add proper delays based on command
	if (cmd == DISPLAY_CLEAR || cmd == RETURN_HOME) {
		delayMS(10);     // Clear and Home need more time
	} else {
		delayUS(100);    // Standard command delay
	}
}
void print_char(uint8_t addr, uint8_t data) {
	lcd_write_4_bit(addr, (data >> 4), 1, 0);
	lcd_write_4_bit(addr, (data & 0xF), 1, 0);
}

void print_string(uint8_t addr, char *data) {
	do {
		print_char(0x4E, (uint8_t) *data);
		data++;
	} while (*data != '\0');
}

void lcd_set_cursor(uint8_t row, uint8_t column) {
	column--;
	if (row == 1) {
		lcd_send_cmd(0x4E, (column |= FIRST_ROW));
	} else {
		lcd_send_cmd(0x4E, (column |= SECOND_ROW));
	}
}

void lcd_init() {
	// Bus recovery - ensure I2C bus is in clean state
	I2C_WRITE(0x4E, 0x00);  // Dummy transaction

	// Wait for > 15ms
	delayMS(50);

	// I2C WRITE
	lcd_write_4_bit(0x4E, 0x3, 0, 0);

	// Wait for > 4.1ms
	delayMS(5);

	// I2C WRITE
	lcd_write_4_bit(0x4E, 0x3, 0, 0);

	// Wait for > 100us
	delayUS(150);

	// I2C WRITE
	lcd_write_4_bit(0x4E, 0x3, 0, 0);

	// Function set  (Set interface to be 4 bits long.) Interface is 8 bits in length.
	lcd_write_4_bit(0x4E, 0x2, 0, 0);

	// Function set  (Interface is 4 bits long.  Specify the number of display lines and character font. The number of display lines and character font cannot be changed after this point.
	lcd_send_cmd(0x4E, FUNCTION_SET);

	// Display on Cursor off
	lcd_send_cmd(0x4E, DISPLAY_ON_CURSOR_OFF);

	// Display clear
	lcd_clear();

	// ENTRY MODE
	lcd_send_cmd(0x4E, ENTRY_MODE);

}

void lcd_clear() {
	// Display clear
	lcd_send_cmd(0x4E, DISPLAY_CLEAR);
}

/**
 * @brief  Redirects printf output to ITM for debugging
 * @param  file: File handle (unused)
 * @param  ptr: Pointer to data buffer
 * @param  len: Number of bytes to write
 * @retval Number of bytes written
 */
int _write(int file, char *ptr, int len) {
	(void) file;  // Unused parameter
	int DataIdx;

	/* Send each character to ITM port 0 */
	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}
