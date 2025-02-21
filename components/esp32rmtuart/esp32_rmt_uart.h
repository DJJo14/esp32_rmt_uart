#pragma once

#include "esphome.h"
// #include "driver/rmt_tx.h"
// #include "driver/rmt_rx.h"

#define CLOCK_HZ 80000000
#define DEFAULT_BAUD_RATE 9600
#define UART_TX_BUFFER_SIZE 256
#define UART_RX_BUFFER_SIZE 256

class RMTUARTComponent : public Component, public UARTDevice {
 public:
    RMTUARTComponent(UARTComponent *parent, int tx_pin, int rx_pin, int baud_rate = DEFAULT_BAUD_RATE);
    void setup() override;
    void loop() override;
    void write_byte(uint8_t byte) override;
    bool read_byte(uint8_t *byte) override;
    void set_baud_rate(int baud_rate) { baud_rate_ = baud_rate; }

 private:
    int tx_gpio_;
    int rx_gpio_;
    int baud_rate_;
    rmt_channel_handle_t rmt_tx_channel_;
    rmt_channel_handle_t rmt_rx_channel_;
    uint8_t tx_buffer_[UART_TX_BUFFER_SIZE];
    uint8_t rx_buffer_[UART_RX_BUFFER_SIZE];
    int tx_head_, tx_tail_;
    int rx_head_, rx_tail_;
    void process_tx_queue();
    void decode_rmt_rx_data(const rmt_symbol_word_t *symbols, int count);
};
