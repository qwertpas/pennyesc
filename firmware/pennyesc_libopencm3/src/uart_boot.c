#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <stdint.h>
#include <stdbool.h>

#define PENNYESC_FLASH_BASE 0x08000000u
#define PENNYESC_BOOT_BYTES 1408u
#define PENNYESC_APP_BASE (PENNYESC_FLASH_BASE + PENNYESC_BOOT_BYTES)
#define PENNYESC_APP_BYTES 14336u
#define PENNYESC_APP_LIMIT (PENNYESC_APP_BASE + PENNYESC_APP_BYTES)
#define PENNYESC_RAM_VECTOR_BASE 0x20000000u
#define PENNYESC_RAM_VECTOR_WORDS 64u
#define PENNYESC_PAGE_BYTES 128u
#define PENNYESC_HALF_PAGE_BYTES 64u
#define PENNYESC_BOOT_BAUD_DIV 139u
#define ROM_ACK 0x79u
#define ROM_NACK 0x1Fu
#define ROM_SYNC 0x7Fu
#define ROM_READ_MEMORY 0x11u
#define ROM_GO 0x21u
#define ROM_WRITE_MEMORY 0x31u
#define ROM_EXTENDED_ERASE 0x44u

#define BOOT_SYNC_SPINS 2000000u
#define BOOT_READ_SPINS 600000u

extern uint32_t _stack;
extern uint32_t _data_loadaddr;
extern uint32_t _data;
extern uint32_t _edata;
extern uint32_t _ebss;

static void blocking_handler(void) __attribute__((noreturn));
void reset_handler(void) __attribute__((noreturn));
int main(void);

__attribute__((section(".vectors")))
void *vector_table[] = {
    &_stack,
    reset_handler,
    blocking_handler,
    blocking_handler,
    0,
    0,
    0,
    0,
};

static uint8_t read_buf[260];

static void blocking_handler(void)
{
    while (1) {
    }
}

void reset_handler(void)
{
    uint32_t *src = &_data_loadaddr;
    uint32_t *dst = &_data;

    while (dst < &_edata) {
        *dst++ = *src++;
    }
    while (dst < &_ebss) {
        *dst++ = 0u;
    }

    main();
    while (1) {
    }
}

static void clock_setup(void)
{
    RCC_CR |= RCC_CR_HSI16ON;
    while ((RCC_CR & RCC_CR_HSI16RDY) == 0u) {
    }
    RCC_CFGR = (RCC_CFGR & ~(RCC_CFGR_SW_MASK << RCC_CFGR_SW_SHIFT)) |
               (RCC_CFGR_SW_HSI16 << RCC_CFGR_SW_SHIFT);
    while (((RCC_CFGR >> RCC_CFGR_SWS_SHIFT) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI16) {
    }

    RCC_IOPENR |= RCC_IOPENR_IOPAEN;
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;
}

static void usart2_setup(void)
{
    GPIO_AFRH(GPIOA) = (GPIO_AFRH(GPIOA) & ~((0xFu << 4) | (0xFu << 8))) | (4u << 4) | (4u << 8);
    GPIO_MODER(GPIOA) = (GPIO_MODER(GPIOA) & ~((3u << 18) | (3u << 20))) | (2u << 18) | (2u << 20);
    GPIO_OTYPER(GPIOA) |= (1u << 9) | (1u << 10);

    USART_CR1(USART2) = 0u;
    USART_CR2(USART2) = 0u;
    USART_CR3(USART2) = 0u;
    USART_BRR(USART2) = PENNYESC_BOOT_BAUD_DIV;
    USART_CR1(USART2) = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
}

static void send_byte(uint8_t value)
{
    while ((USART_ISR(USART2) & USART_ISR_TXE) == 0u) {
    }
    USART_TDR(USART2) = value;
}

static void send_ack(void)
{
    send_byte(ROM_ACK);
}

static void send_nack(void)
{
    send_byte(ROM_NACK);
}

static bool read_byte(uint32_t spins, uint8_t *value)
{
    while (spins-- > 0u) {
        if ((USART_ISR(USART2) & USART_ISR_RXNE) != 0u) {
            *value = (uint8_t)USART_RDR(USART2);
            return true;
        }
    }
    return false;
}

static bool read_exact(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        if (!read_byte(BOOT_READ_SPINS, &buf[i])) {
            return false;
        }
    }
    return true;
}

static uint8_t xor_bytes(const uint8_t *data, uint32_t len)
{
    uint8_t value = 0u;

    for (uint32_t i = 0; i < len; i++) {
        value ^= data[i];
    }
    return value;
}

static bool app_valid(void)
{
    uint32_t sp = MMIO32(PENNYESC_APP_BASE);
    uint32_t pc = MMIO32(PENNYESC_APP_BASE + 4u);

    if ((sp & 0x2FF80000u) != 0x20000000u) {
        return false;
    }
    if ((pc & 1u) == 0u) {
        return false;
    }
    if (pc < PENNYESC_APP_BASE || pc >= PENNYESC_APP_LIMIT) {
        return false;
    }
    return true;
}

static void jump_to_app(void) __attribute__((noreturn));

static void jump_to_app(void)
{
    uint32_t sp = MMIO32(PENNYESC_APP_BASE);
    uint32_t *src = (uint32_t *)PENNYESC_APP_BASE;
    uint32_t *dst = (uint32_t *)PENNYESC_RAM_VECTOR_BASE;

    for (uint32_t i = 0; i < PENNYESC_RAM_VECTOR_WORDS; i++) {
        dst[i] = src[i];
    }

    __asm__("cpsid i");
    USART_CR1(USART2) = 0u;
    SCB_VTOR = PENNYESC_RAM_VECTOR_BASE;
    __asm__ volatile("msr msp, %0" : : "r"(sp) :);
    ((void (*)(void))MMIO32(PENNYESC_APP_BASE + 4u))();
    while (1) {
    }
}

static void __attribute__((long_call, section(".ramtext"))) flash_wait(void)
{
    while ((FLASH_SR & FLASH_SR_BSY) != 0u) {
    }
}

static void flash_unlock_all(void)
{
    FLASH_PEKEYR = FLASH_PEKEYR_PEKEY1;
    FLASH_PEKEYR = FLASH_PEKEYR_PEKEY2;
    FLASH_PRGKEYR = FLASH_PRGKEYR_PRGKEY1;
    FLASH_PRGKEYR = FLASH_PRGKEYR_PRGKEY2;
}

static void flash_lock_all(void)
{
    FLASH_PECR |= FLASH_PECR_PRGLOCK | FLASH_PECR_PELOCK;
}

static void flash_erase_page_raw(uint32_t page_address)
{
    FLASH_PECR |= FLASH_PECR_ERASE | FLASH_PECR_PROG;
    MMIO32(page_address) = 0u;
    flash_wait();
    FLASH_PECR &= ~(FLASH_PECR_ERASE | FLASH_PECR_PROG);
}

static void __attribute__((long_call, noinline, section(".ramtext"))) flash_program_half_page_raw(volatile uint32_t *dst, const void *buf)
{
    const uint32_t *src = (const uint32_t *)buf;

    FLASH_PECR |= FLASH_PECR_FPRG | FLASH_PECR_PROG;
    flash_wait();
    for (uint32_t i = 0; i < (PENNYESC_HALF_PAGE_BYTES / sizeof(uint32_t)); i++) {
        __asm__ volatile("str %1, [%0]" : : "r"(dst), "r"(*src) : "memory");
        dst++;
        src++;
    }
    flash_wait();
    FLASH_PECR &= ~(FLASH_PECR_FPRG | FLASH_PECR_PROG);
}

static bool read_command(uint8_t *command)
{
    uint8_t complement;

    if (!read_byte(BOOT_READ_SPINS, command)) {
        return false;
    }
    if (!read_byte(BOOT_READ_SPINS, &complement)) {
        return false;
    }
    return (uint8_t)(*command ^ complement) == 0xFFu;
}

static bool read_address(uint32_t *address)
{
    uint8_t payload[5];

    if (!read_exact(payload, sizeof(payload))) {
        return false;
    }
    if (xor_bytes(payload, 4u) != payload[4]) {
        return false;
    }

    *address = ((uint32_t)payload[0] << 24) |
               ((uint32_t)payload[1] << 16) |
               ((uint32_t)payload[2] << 8) |
               (uint32_t)payload[3];
    return true;
}

static bool address_valid(uint32_t address, uint32_t len)
{
    if (address < PENNYESC_APP_BASE) {
        return false;
    }
    if (len > PENNYESC_APP_LIMIT - address) {
        return false;
    }
    return true;
}

static void handle_extended_erase(void)
{
    if (!read_exact(read_buf, 3u)) {
        return;
    }

    uint16_t count = ((uint16_t)read_buf[0] << 8) | (uint16_t)read_buf[1];
    uint32_t total = (uint32_t)count + 1u;
    uint32_t payload_len = (total * 2u) + 3u;
    uint32_t max_pages = PENNYESC_APP_BYTES / PENNYESC_PAGE_BYTES;

    if (total == 0u || total > max_pages || payload_len > sizeof(read_buf)) {
        send_nack();
        return;
    }
    if (!read_exact(&read_buf[3], payload_len - 3u)) {
        return;
    }
    if (xor_bytes(read_buf, payload_len - 1u) != read_buf[payload_len - 1u]) {
        send_nack();
        return;
    }

    flash_unlock_all();
    for (uint32_t i = 0; i < total; i++) {
        uint16_t page = ((uint16_t)read_buf[2u + (i * 2u)] << 8) | read_buf[3u + (i * 2u)];
        if (page >= max_pages) {
            flash_lock_all();
            send_nack();
            return;
        }
        flash_erase_page_raw(PENNYESC_APP_BASE + ((uint32_t)page * PENNYESC_PAGE_BYTES));
    }
    flash_lock_all();
    send_ack();
}

static void handle_write_memory(void)
{
    uint32_t address;
    uint8_t crc = 0u;
    uint8_t *data = &read_buf[4];

    if (!read_address(&address) || !address_valid(address, PENNYESC_HALF_PAGE_BYTES)) {
        send_nack();
        return;
    }
    send_ack();

    if (!read_exact(read_buf, 1u)) {
        return;
    }

    uint32_t len = (uint32_t)read_buf[0] + 1u;
    if (len == 0u || len > 256u || (len % PENNYESC_HALF_PAGE_BYTES) != 0u) {
        send_nack();
        return;
    }
    if ((address & (PENNYESC_HALF_PAGE_BYTES - 1u)) != 0u || !address_valid(address, len)) {
        send_nack();
        return;
    }
    if (!read_exact(data, len + 1u)) {
        return;
    }
    crc ^= read_buf[0];
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    if (crc != data[len]) {
        send_nack();
        return;
    }

    flash_unlock_all();
    for (uint32_t offset = 0; offset < len; offset += PENNYESC_HALF_PAGE_BYTES) {
        flash_program_half_page_raw((uint32_t *)(uintptr_t)(address + offset), &data[offset]);
    }
    flash_lock_all();
    send_ack();
}

static void handle_read_memory(void)
{
    uint32_t address;

    if (!read_address(&address) || !address_valid(address, 1u)) {
        send_nack();
        return;
    }
    send_ack();

    if (!read_exact(read_buf, 2u)) {
        return;
    }
    uint32_t len = (uint32_t)read_buf[0] + 1u;
    if ((uint8_t)(read_buf[0] ^ read_buf[1]) != 0xFFu || !address_valid(address, len)) {
        send_nack();
        return;
    }

    send_ack();
    for (uint32_t i = 0; i < len; i++) {
        send_byte(MMIO8(address + i));
    }
}

static void handle_go(void)
{
    uint32_t address;

    if (!read_address(&address) || address != PENNYESC_APP_BASE || !app_valid()) {
        send_nack();
        return;
    }
    send_ack();
    jump_to_app();
}

static void command_loop(void) __attribute__((noreturn));

static void command_loop(void)
{
    while (1) {
        uint8_t command;

        if (!read_command(&command)) {
            break;
        }

        switch (command) {
        case ROM_READ_MEMORY:
            send_ack();
            handle_read_memory();
            break;
        case ROM_GO:
            send_ack();
            handle_go();
            break;
        case ROM_WRITE_MEMORY:
            send_ack();
            handle_write_memory();
            break;
        case ROM_EXTENDED_ERASE:
            send_ack();
            handle_extended_erase();
            break;
        default:
            send_nack();
            break;
        }
    }

    while (1) {
        uint8_t byte;

        if (read_byte(BOOT_SYNC_SPINS, &byte) && byte == ROM_SYNC) {
            send_ack();
            continue;
        }
        if (app_valid()) {
            jump_to_app();
        }
    }
}

int main(void)
{
    clock_setup();
    usart2_setup();

    if (!app_valid()) {
        while (1) {
            uint8_t byte;

            if (read_byte(BOOT_SYNC_SPINS, &byte) && byte == ROM_SYNC) {
                send_ack();
                command_loop();
            }
        }
    }

    for (uint32_t spins = BOOT_SYNC_SPINS; spins > 0u; spins--) {
        if ((USART_ISR(USART2) & USART_ISR_RXNE) == 0u) {
            continue;
        }

        uint8_t byte = (uint8_t)USART_RDR(USART2);
        if (byte != ROM_SYNC) {
            continue;
        }

        send_ack();
        command_loop();
    }

    jump_to_app();
}
