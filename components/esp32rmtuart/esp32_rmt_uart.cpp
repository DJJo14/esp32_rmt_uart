#include "esp32_rmt_uart.h"
#include <cinttypes>

// #ifdef USE_ESP32

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <driver/gpio.h>

#include <algorithm>
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

#if ESP_IDF_VERSION_MAJOR >= 5
static bool IRAM_ATTR HOT rmt_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *event, void *arg) {
    ReceiverComponentStore *store = (ReceiverComponentStore *) arg;
  rmt_rx_done_event_data_t *event_buffer = (rmt_rx_done_event_data_t *) (store->buffer + store->buffer_write);
  uint32_t event_size = sizeof(rmt_rx_done_event_data_t);
  uint32_t next_write = store->buffer_write + event_size + event->num_symbols * sizeof(rmt_symbol_word_t);
  if (next_write + event_size + store->receive_size > store->buffer_size) {
    next_write = 0;
  }
  if (store->buffer_read - next_write < event_size + store->receive_size) {
    next_write = store->buffer_write;
    store->overflow = true;
  }
//   if (event->num_symbols <= store->filter_symbols) {
//     next_write = store->buffer_write;
//   }
  store->error =
      rmt_receive(channel, (uint8_t *) store->buffer + next_write + event_size, store->receive_size, &store->config);
  event_buffer->num_symbols = event->num_symbols;
  event_buffer->received_symbols = event->received_symbols;
  store->buffer_write = next_write;
  return false;
}
#endif

RMTUARTComponent::RMTUARTComponent()
    : tx_head_(0), tx_tail_(0), rx_head_(0), rx_tail_(0), use_psram_(false) {}

void RMTUARTComponent::load_settings() {
    //Generate baud rate timing array
    uint32_t base_timing = CLOCK_HZ / this->baud_rate_;
    uint32_t rest = (CLOCK_HZ % this->baud_rate_) / (this->baud_rate_/ 10);
    for (int i = 0; i < 10; i++) {
        this->baud_rate_timing_array_[i] = base_timing + ((i < rest) ? 1 : 0);
    }

    if (this->stop_bits_ == 2) {
        this->baud_rate_timing_array_[9] = this->baud_rate_timing_array_[9] * 2;
    }
}

void RMTUARTComponent::setup() {
    ESP_LOGCONFIG(TAG, "Extra uarts on TX: %d, RX: %d, Baud Rate: %d", tx_pin_, rx_pin_, baud_rate_);

#if ESP_IDF_VERSION_MAJOR >= 5
    RAMAllocator<rmt_symbol_word_t> rmt_allocator(this->use_psram_ ? 0 : RAMAllocator<rmt_symbol_word_t>::ALLOC_INTERNAL);
  
    // 8 bits per byte, 1 rmt_symbol_word_t per bit + 1 rmt_symbol_word_t for reset
    this->rmt_tx_buf_ = rmt_allocator.allocate(tx_buffer_size_ * 8 + 1);
  
    rmt_tx_channel_config_t tx_channel;
    memset(&tx_channel, 0, sizeof(tx_channel));
    tx_channel.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_channel.resolution_hz = RMT_CLK_FREQ / RMT_CLK_DIV;
    tx_channel.gpio_num = gpio_num_t(this->tx_pin_);
    tx_channel.mem_block_symbols = this->rmt_tx_symbols_;
    tx_channel.trans_queue_depth = 1;
    tx_channel.flags.io_loop_back = 0;
    tx_channel.flags.io_od_mode = 0;
    tx_channel.flags.invert_out = 0;
    tx_channel.flags.with_dma = 0;
    tx_channel.intr_priority = 0;
    if (rmt_new_tx_channel(&tx_channel, &this->tx_channel_) != ESP_OK) {
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
  
    if (rmt_enable(this->tx_channel_) != ESP_OK) {
      ESP_LOGE(TAG, "Enabling tx_channel_ failed");
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
    esp_err_t error = rmt_transmit(this->tx_channel_, this->encoder_, symbols, 1 * sizeof(rmt_symbol_word_t), &config);

    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "RMT TX error");
        this->status_set_warning();
        return;
    }
    
    // RX Configuration

    rmt_rx_channel_config_t rx_channel;
    memset(&rx_channel, 0, sizeof(rx_channel));
    rx_channel.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_channel.resolution_hz = RMT_CLK_FREQ / RMT_CLK_DIV;
    rx_channel.mem_block_symbols = this->rmt_rx_symbols_;
    rx_channel.gpio_num = gpio_num_t(this->rx_pin_);
    rx_channel.intr_priority = 0;
    rx_channel.flags.invert_in = 0;
    rx_channel.flags.with_dma = 0;
    rx_channel.flags.io_loop_back = 0;
    error = rmt_new_rx_channel(&rx_channel, &this->rx_channel_);
    if (error != ESP_OK) {
        this->error_code_ = error;
        if (error == ESP_ERR_NOT_FOUND) {
        this->error_string_ = "out of RMT symbol memory";
        } else {
        this->error_string_ = "in rmt_new_rx_channel";
        }
        this->mark_failed();
        return;
    }
    gpio_pullup_en(gpio_num_t(this->rx_pin_));

    //TODO PULLUP?
    // if (this->pin_->get_flags() & gpio::FLAG_PULLUP) {
    //     gpio_pullup_en(gpio_num_t(this->pin_->get_pin()));
    // } else {
    //     gpio_pullup_dis(gpio_num_t(this->pin_->get_pin()));
    // }
    error = rmt_enable(this->rx_channel_);
    if (error != ESP_OK) {
        this->error_code_ = error;
        this->error_string_ = "in rmt_enable";
        this->mark_failed();
        return;
    }

    rmt_rx_event_callbacks_t callbacks;
    memset(&callbacks, 0, sizeof(callbacks));
    callbacks.on_recv_done = rmt_callback;
    error = rmt_rx_register_event_callbacks(this->rx_channel_, &callbacks, &this->store_);
    if (error != ESP_OK) {
        this->error_code_ = error;
        this->error_string_ = "in rmt_rx_register_event_callbacks";
        this->mark_failed();
        return;
    }

    
    uint32_t event_size = sizeof(rmt_rx_done_event_data_t);
    uint32_t max_filter_ns = 255u * 1000 / (RMT_CLK_FREQ / 1000000);
    uint32_t max_idle_ns = 65535u * 1000;
    memset(&this->store_.config, 0, sizeof(this->store_.config));
    this->store_.config.signal_range_min_ns = std::min(((uint32_t)/* 90% of baud_rate*/9000000000u/(this->baud_rate_)), max_filter_ns);
    this->store_.config.signal_range_max_ns = std::min(1000000000u/(this->baud_rate_/10), max_idle_ns);
    this->store_.receive_size = this->rmt_rx_symbols_ * sizeof(rmt_symbol_word_t);
    this->store_.buffer_size = std::max((event_size + this->store_.receive_size) * 2, this->rx_buffer_size_);
    this->store_.buffer = new uint8_t[this->rx_buffer_size_];
    error = rmt_receive(this->rx_channel_, (uint8_t *) this->store_.buffer + event_size, this->store_.receive_size,
                        &this->store_.config);
    if (error != ESP_OK) {
        this->error_code_ = error;
        this->error_string_ = "in rmt_receive";
        this->mark_failed();
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

    this->load_settings();
}

void RMTUARTComponent::set_baud_rate(int baud_rate) {
    this->baud_rate_ = (uint32_t) baud_rate;
    this->load_settings();
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
    error = rmt_transmit(this->tx_channel_, this->encoder_, symbols, length * RMT_TX_SYMBOLS_PER_BYTE * sizeof(rmt_symbol_word_t), &config);

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

void RMTUARTComponent::put_rx_byte(uint8_t byte) {
    rx_buffer_[rx_tail_] = byte;
    rx_tail_ = (rx_tail_ + 1) % UART_RX_BUFFER_SIZE;
}

void RMTUARTComponent::decode_rmt_rx_data(const rmt_symbol_word_t *symbols, int count) {
    uint32_t min_time_bit = this->baud_rate_timing_array_[0] * 9 / 10;
    ESP_LOGD(TAG, "bit time %d", min_time_bit);
    uint8_t received_byte = 0;
    uint8_t recived_bits = 0;
    uint32_t total_recived_bits = 0;
    rmt_symbol_half_word_t *half_symbols = (rmt_symbol_half_word_t *) symbols;
    uint32_t count_half = count * 2;

    for (int i = 0; i < count_half; i++) {
        if (half_symbols[i].duration0 == 0) {
            //this is the last bit, before idle
            recived_bits = 1;
        }
        else
        {
            recived_bits = half_symbols[i].duration0 / min_time_bit;
        }
        
        ESP_LOGD(TAG, "reviced biit: %d, time %d, bits %d total %d", half_symbols[i].level0, half_symbols[i].duration0, recived_bits, total_recived_bits);

        if(total_recived_bits == 0 && (half_symbols[i].level0 != 0)) {
            ESP_LOGD(TAG, "Start bit not found");
            continue;
        } 

        // if(total_recived_bits == 0 && (half_symbols[i].duration0 < min_time_bit) ){
        //     ESP_LOGD(TAG, "Start bit not found");
        //     continue;
        // }
        //remove the start bit
        if(recived_bits > 0 && (total_recived_bits == 0 || (recived_bits + total_recived_bits) == 10))
        {
            recived_bits -= 1;
            total_recived_bits += 1;
        }

        for (uint8_t j = 0; j < recived_bits; j++) {
            received_byte |= (half_symbols[i].level0 << ((j + (total_recived_bits-1)) % 8 ));
        }
        total_recived_bits = total_recived_bits + recived_bits;

        if(total_recived_bits == 10) {
            ESP_LOGD(TAG, "reviced byte: %x, char %c", received_byte, received_byte);
            // put_rx_byte(received_byte);
            received_byte = 0;
            total_recived_bits = 0;
        }
        // if (symbols[i].level0 == 1) {
        //     received_byte |= (1 << (i % 8));
        // }
        // if (i % 8 == 7) {
        //     // put_rx_byte(received_byte);
        //     received_byte = 0;
        // }
    }


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
    #if ESP_IDF_VERSION_MAJOR >= 5
    if (this->store_.error != ESP_OK) {
        ESP_LOGE(TAG, "Receive error");
        this->error_code_ = this->store_.error;
        this->error_string_ = "in rmt_callback";
        this->mark_failed();
    }
    if (this->store_.overflow) {
        ESP_LOGW(TAG, "Buffer overflow");
        this->store_.overflow = false;
    }
    uint32_t buffer_write = this->store_.buffer_write;
    while (this->store_.buffer_read != buffer_write) {
        rmt_rx_done_event_data_t *event = (rmt_rx_done_event_data_t *) (this->store_.buffer + this->store_.buffer_read);
        uint32_t event_size = sizeof(rmt_rx_done_event_data_t);
        uint32_t next_read = this->store_.buffer_read + event_size + event->num_symbols * sizeof(rmt_symbol_word_t);
        if (next_read + event_size + this->store_.receive_size > this->store_.buffer_size) {
        next_read = 0;
        }
        this->decode_rmt_rx_data(event->received_symbols, event->num_symbols);
        this->store_.buffer_read = next_read;
    }
    #endif
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
