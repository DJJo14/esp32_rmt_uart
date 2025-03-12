#include "esp32_rmt_uart.h"
#include <cinttypes>

// #ifdef USE_ESP32

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
static const uint32_t RMT_CLK_DIV = 1;
#endif
#define CLOCK_HZ (RMT_CLK_FREQ/RMT_CLK_DIV)

RMTUARTComponent::RMTUARTComponent()
    : tx_head_(0), tx_tail_(0), rx_head_(0), rx_tail_(0), use_psram_(false) {}

void RMTUARTComponent::generate_baud_rate_timing_array() {
    uint32_t base_timing = CLOCK_HZ / this->baud_rate_;
    for (int i = 0; i < 10; i++) {
        this->baud_rate_timing_array_[i] = base_timing + ((i < (CLOCK_HZ % this->baud_rate_)) ? 1 : 0);
    }
}

void RMTUARTComponent::setup() {
    ESP_LOGCONFIG(TAG, "Extra uarts on TX: %d, RX: %d, Baud Rate: %d", tx_pin_, rx_pin_, baud_rate_);

    this->generate_baud_rate_timing_array();

#if ESP_IDF_VERSION_MAJOR >= 5
    RAMAllocator<rmt_symbol_word_t> rmt_allocator(this->use_psram_ ? 0 : RAMAllocator<rmt_symbol_word_t>::ALLOC_INTERNAL);
  
    // 8 bits per byte, 1 rmt_symbol_word_t per bit + 1 rmt_symbol_word_t for reset
    this->rmt_tx_buf_ = rmt_allocator.allocate(tx_buffer_size_ * 8 + 1);
  
    rmt_tx_channel_config_t channel;
    memset(&channel, 0, sizeof(channel));
    channel.clk_src = RMT_CLK_SRC_DEFAULT;
    channel.resolution_hz = RMT_CLK_FREQ / RMT_CLK_DIV;
    channel.gpio_num = gpio_num_t(this->tx_pin_);
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


    //we need to send a start bit, for the end of transmission level to be correct
    rmt_symbol_word_t *symbols = this->rmt_tx_buf_;
    symbols[0].val = 0;
    symbols[0].duration0 = (uint16_t) 1;
    symbols[0].level0 = 1; // Start bit
    symbols[0].duration1 = (uint16_t) 1;
    symbols[0].level1 = 1; // Start bit
    
    rmt_transmit_config_t config;
    memset(&config, 0, sizeof(config));
    config.loop_count = 0;
    config.flags.eot_level = 1;
    esp_err_t error = rmt_transmit(this->channel_, this->encoder_, symbols, 1 * sizeof(rmt_symbol_word_t), &config);

    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "RMT TX error");
        this->status_set_warning();
        return;
    }
#else
    RAMAllocator<rmt_item32_t> rmt_allocator(this->use_psram_ ? 0 : RAMAllocator<rmt_item32_t>::ALLOC_INTERNAL);
  
    // 8 bits per byte, 1 rmt_item32_t per bit + 1 rmt_item32_t for reset
    this->rmt_tx_buf_ = rmt_allocator.allocate(tx_buffer_size_ * 8 + 1);
  
    rmt_config_t config;
    memset(&config, 0, sizeof(config));
    config.channel = this->tx_channel_;
    config.rmt_mode = RMT_MODE_TX;
    config.gpio_num = gpio_num_t(this->tx_pin_);
    config.mem_block_num = 1;
    config.clk_div = RMT_CLK_DIV;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
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
    //TODO RX Configuration
}

void RMTUARTComponent::set_baud_rate(int baud_rate) {
    this->baud_rate_ = baud_rate;
    this->generate_baud_rate_timing_array();
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
    esp_err_t error = 0;
    if (tx_head_ == tx_tail_) return;

    int length = (tx_tail_ - tx_head_ + UART_TX_BUFFER_SIZE) % UART_TX_BUFFER_SIZE;
    if (this->rmt_tx_buf_ == nullptr) {
        ESP_LOGE(TAG, "RMT TX buffer is null");
        return;
    }
    
    if (rmt_tx_symbols_ < length * RMT_TX_SYMBOLS_PER_BYTE) {
        length = rmt_tx_symbols_ / RMT_TX_SYMBOLS_PER_BYTE;
    }

    rmt_symbol_word_t *symbols = this->rmt_tx_buf_;
    uint8_t byte;

    for (int j = 0; j < length; j++) {
        byte = tx_buffer_[(tx_head_ + j) % UART_TX_BUFFER_SIZE];
        symbols[j * RMT_TX_SYMBOLS_PER_BYTE].val = 0;
        symbols[j * RMT_TX_SYMBOLS_PER_BYTE].duration0 = this->baud_rate_timing_array_[0];
        symbols[j * RMT_TX_SYMBOLS_PER_BYTE].level0 = 0; // Start bit
        for (int i = 0; i < 4; i++) {
            symbols[j * RMT_TX_SYMBOLS_PER_BYTE + i].duration1 = this->baud_rate_timing_array_[i + 1];
            symbols[j * RMT_TX_SYMBOLS_PER_BYTE + i].level1 = (byte >> (i * 2)) & 1;
            symbols[j * RMT_TX_SYMBOLS_PER_BYTE + i + 1].val = 0;
            symbols[j * RMT_TX_SYMBOLS_PER_BYTE + i + 1].duration0 = this->baud_rate_timing_array_[i + 1];
            symbols[j * RMT_TX_SYMBOLS_PER_BYTE + i + 1].level0 = (byte >> (i * 2 + 1)) & 1;
        }
        symbols[j * RMT_TX_SYMBOLS_PER_BYTE + RMT_TX_SYMBOLS_PER_BYTE - 1].duration1 = this->baud_rate_timing_array_[9];
        symbols[j * RMT_TX_SYMBOLS_PER_BYTE + RMT_TX_SYMBOLS_PER_BYTE - 1].level1 = 1; // Stop bit
    }
    
#if ESP_IDF_VERSION_MAJOR >= 5
    rmt_transmit_config_t config;
    memset(&config, 0, sizeof(config));
    config.loop_count = 0;
    config.flags.eot_level = 1;
    error = rmt_transmit(this->channel_, this->encoder_, symbols, length * RMT_TX_SYMBOLS_PER_BYTE * sizeof(rmt_symbol_word_t), &config);

    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "RMT TX error");
        this->status_set_warning();
        return;
    }
    this->status_clear_warning();
#else
    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    rmt_transmit(rmt_tx_channel_, NULL, symbols, length * 10, &tx_config);
#endif

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

void RMTUARTComponent::write_array(const uint8_t *buffer, size_t length) {
    size_t available_space = UART_TX_BUFFER_SIZE - ((tx_tail_ - tx_head_ + UART_TX_BUFFER_SIZE) % UART_TX_BUFFER_SIZE);
    if (length > available_space) {
        ESP_LOGE(TAG, "Not enough space in TX buffer");
        return;
    }

    size_t first_chunk = (length < (UART_TX_BUFFER_SIZE - tx_tail_)) ? length : (UART_TX_BUFFER_SIZE - tx_tail_);
    memcpy(&tx_buffer_[tx_tail_], buffer, first_chunk);
    memcpy(&tx_buffer_[0], buffer + first_chunk, length - first_chunk);

    tx_tail_ = (tx_tail_ + length) % UART_TX_BUFFER_SIZE;
    process_tx_queue();
}

bool RMTUARTComponent::read_array(uint8_t *buffer, size_t length) {
    for (size_t i = 0; i < length; i++) {
        if (!read_byte(&buffer[i])) {
            return false;
        }
    }
    return true;
}

bool RMTUARTComponent::peek_byte(uint8_t *buffer) {
    if (rx_head_ == rx_tail_) return false;
    *buffer = rx_buffer_[rx_head_];
    return true;
}

int RMTUARTComponent::available() {
    return (rx_tail_ - rx_head_ + UART_RX_BUFFER_SIZE) % UART_RX_BUFFER_SIZE;
}

void RMTUARTComponent::flush() {
    while (tx_head_ != tx_tail_) {
        process_tx_queue();
    }
}

void RMTUARTComponent::loop()
{
    
}

void RMTUARTComponent::check_logger_conflict() {
  // Check if the logger is using the same UART pins
//   if (this->tx_pin_ == logger::global_logger->get_tx_pin() || this->rx_pin_ == logger::global_logger->get_rx_pin()) {
//     ESP_LOGW(TAG, "Logger is using the same UART pins. This may cause conflicts.");
//   }
}

}  // namespace esp32_rmt_uart
}  // namespace esphome

// #endif  // USE_ESP32
