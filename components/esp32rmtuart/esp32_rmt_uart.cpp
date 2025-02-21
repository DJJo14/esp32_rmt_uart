#include "esp32_rmt_uart.h"

RMTUARTComponent::RMTUARTComponent(UARTComponent *parent, int tx_pin, int rx_pin, int baud_rate)
    : UARTDevice(parent), tx_gpio_(tx_pin), rx_gpio_(rx_pin), baud_rate_(baud_rate),
      tx_head_(0), tx_tail_(0), rx_head_(0), rx_tail_(0) {}

void RMTUARTComponent::setup() {
    ESP_LOGI("esp32_rmt_uart", "Initializing RMT UART on TX: %d, RX: %d, Baud Rate: %d", tx_gpio_, rx_gpio_, baud_rate_);

    // TX Configuration
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = tx_gpio_,
        .clk_src = RMT_CLK_SRC_APB,
        .resolution_hz = CLOCK_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4
    };
    rmt_new_tx_channel(&tx_config, &rmt_tx_channel_);
    rmt_enable(rmt_tx_channel_);

    // RX Configuration
    rmt_rx_channel_config_t rx_config = {
        .gpio_num = rx_gpio_,
        .clk_src = RMT_CLK_SRC_APB,
        .resolution_hz = CLOCK_HZ,
        .mem_block_symbols = UART_RX_BUFFER_SIZE
    };
    rmt_new_rx_channel(&rx_config, &rmt_rx_channel_);
    rmt_enable(rmt_rx_channel_);
    rmt_receive(rmt_rx_channel_, rx_buffer_, UART_RX_BUFFER_SIZE, nullptr);
}

void RMTUARTComponent::write_byte(uint8_t byte) {
    tx_buffer_[tx_tail_] = byte;
    tx_tail_ = (tx_tail_ + 1) % UART_TX_BUFFER_SIZE;
    process_tx_queue();
}

bool RMTUARTComponent::read_byte(uint8_t *byte) {
    if (rx_head_ == rx_tail_) return false;
    *byte = rx_buffer_[rx_head_];
    rx_head_ = (rx_head_ + 1) % UART_RX_BUFFER_SIZE;
    return true;
}

void RMTUARTComponent::process_tx_queue() {
    if (tx_head_ == tx_tail_) return;

    int length = (tx_tail_ - tx_head_ + UART_TX_BUFFER_SIZE) % UART_TX_BUFFER_SIZE;
    rmt_symbol_word_t symbols[length * 10];

    for (int j = 0; j < length; j++) {
        uint8_t byte = tx_buffer_[tx_head_ + j];
        symbols[j * 10] = { .duration0 = CLOCK_HZ / baud_rate_, .level0 = 0 }; // Start bit
        for (int i = 0; i < 8; i++) {
            symbols[j * 10 + i + 1] = { .duration0 = CLOCK_HZ / baud_rate_, .level0 = (byte >> i) & 1 };
        }
        symbols[j * 10 + 9] = { .duration0 = CLOCK_HZ / baud_rate_, .level0 = 1 }; // Stop bit
    }

    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    rmt_transmit(rmt_tx_channel_, NULL, symbols, length * 10, &tx_config);

    tx_head_ = (tx_head_ + length) % UART_TX_BUFFER_SIZE;
}

void RMTUARTComponent::decode_rmt_rx_data(const rmt_symbol_word_t *symbols, int count) {
    uint8_t received_byte = 0;
    for (int i = 1; i <= 8; i++) {
        if (symbols[i].level0 == 1) {
            received_byte |= (1 << (i - 1));
        }
    }
    rx_buffer_[rx_tail_] = received_byte;
    rx_tail_ = (rx_tail_ + 1) % UART_RX_BUFFER_SIZE;
}
