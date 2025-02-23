#include "esp32_rmt_uart.h"

#ifdef USE_ESP32

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <esp_attr.h>

namespace esphome {
namespace esp32_rmt_uart {

static const char *const TAG = "esp32_rmt_uart";

#ifdef USE_ESP32_VARIANT_ESP32H2
static const uint32_t RMT_CLK_FREQ = 32000000;
static const uint8_t RMT_CLK_DIV = 1;

#else
static const uint32_t RMT_CLK_FREQ = 80000000;
static const uint8_t RMT_CLK_DIV = 2;
#endif
#define CLOCK_HZ (RMT_CLK_FREQ/RMT_CLK_DIV)

RMTUARTComponent::RMTUARTComponent(UARTComponent *parent, int tx_pin, int rx_pin, int baud_rate)
    : UARTDevice(parent), tx_gpio_(tx_pin), rx_gpio_(rx_pin), baud_rate_(baud_rate),
      tx_head_(0), tx_tail_(0), rx_head_(0), rx_tail_(0) {}

void RMTUARTComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up ESP32 Extra uarts on TX: %d, RX: %d, Baud Rate: %d", tx_gpio_, rx_gpio_, baud_rate_);


#if ESP_IDF_VERSION_MAJOR >= 5
    RAMAllocator<rmt_symbol_word_t> rmt_allocator(this->use_psram_ ? 0 : RAMAllocator<rmt_symbol_word_t>::ALLOC_INTERNAL);
  
    // 8 bits per byte, 1 rmt_symbol_word_t per bit + 1 rmt_symbol_word_t for reset
    this->rmt_buf_ = rmt_allocator.allocate(buffer_size * 8 + 1);
  
    rmt_tx_channel_config_t channel;
    memset(&channel, 0, sizeof(channel));
    channel.clk_src = RMT_CLK_SRC_DEFAULT;
    channel.resolution_hz = RMT_CLK_FREQ / RMT_CLK_DIV;
    channel.gpio_num = gpio_num_t(this->tx_gpio_);
    channel.mem_block_symbols = this->rmt_tx_symbols_;
    channel.trans_queue_depth = 1;
    channel.flags.io_loop_back = 0;
    channel.flags.io_od_mode = 0;
    channel.flags.invert_out = 0;
    channel.flags.with_dma = 0;
    channel.intr_priority = 0;
    if (rmt_new_tx_channel(&channel, &this->channel_) != ESP_OK) {
      ESP_LOGE(TAG, "Channel creation failed");
      this->mark_failed();
      return;
    }
  
    rmt_copy_encoder_config_t encoder;
    memset(&encoder, 0, sizeof(encoder));
    if (rmt_new_copy_encoder(&encoder, &this->encoder_) != ESP_OK) {
      ESP_LOGE(TAG, "Encoder creation failed");
      this->mark_failed();
      return;
    }
  
    if (rmt_enable(this->channel_) != ESP_OK) {
      ESP_LOGE(TAG, "Enabling channel failed");
      this->mark_failed();
      return;
    }
#else
    RAMAllocator<rmt_item32_t> rmt_allocator(this->use_psram_ ? 0 : RAMAllocator<rmt_item32_t>::ALLOC_INTERNAL);
  
    // 8 bits per byte, 1 rmt_item32_t per bit + 1 rmt_item32_t for reset
    this->rmt_buf_ = rmt_allocator.allocate(buffer_size * 8 + 1);
  
    rmt_config_t config;
    memset(&config, 0, sizeof(config));
    config.channel = this->tx_channel_;
    config.rmt_mode = RMT_MODE_TX;
    config.gpio_num = gpio_num_t(this->tx_gpio_);
    config.mem_block_num = 1;
    config.clk_div = RMT_CLK_DIV;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.tx_config.idle_output_en = true;
  
    if (rmt_config(&config) != ESP_OK) {
      ESP_LOGE(TAG, "Cannot initialize RMT!");
      this->mark_failed();
      return;
    }
    if (rmt_driver_install(config.channel, 0, 0) != ESP_OK) {
      ESP_LOGE(TAG, "Cannot install RMT driver!");
      this->mark_failed();
      return;
    }
#endif

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

}  // namespace esp32_rmt_uart
}  // namespace esphome


#endif  // USE_ESP32
