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
static const uint32_t RMT_CLK_DIV = 4;
#endif
#define CLOCK_HZ (RMT_CLK_FREQ/RMT_CLK_DIV)

#if ESP_IDF_VERSION_MAJOR >= 5
static bool IRAM_ATTR HOT rmt_rx_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *event, void *arg) {
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

static bool IRAM_ATTR HOT rmt_tx_callback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx) {
    bool *ptr_tx_done = (bool *) user_ctx;
    *ptr_tx_done = false;
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
    
    // always make the stop bit longer
    this->baud_rate_timing_array_[9] + 1;

    if (this->stop_bits_ == 2) {
        this->baud_rate_timing_array_[9] = this->baud_rate_timing_array_[9] * 2;
    }
}

void RMTUARTComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "  TX Pin: %d", this->tx_pin_);
    ESP_LOGCONFIG(TAG, "  TX Channel: %d", this->tx_channel_);
    ESP_LOGCONFIG(TAG, "  TX Symbols: %d symbols", this->rmt_tx_symbols_);
    ESP_LOGCONFIG(TAG, "  RX Pin: %d", this->rx_pin_);
    ESP_LOGCONFIG(TAG, "  RX Buffer Size: %u", this->rx_buffer_size_);
    ESP_LOGCONFIG(TAG, "  RX Channel: %d", this->rx_channel_);
    ESP_LOGCONFIG(TAG, "  RX Symbols: %d symbols", this->rmt_rx_symbols_);
    ESP_LOGCONFIG(TAG, "  Baud Rate: %" PRIu32 " baud", this->baud_rate_);
    ESP_LOGCONFIG(TAG, "  Data Bits: %u", this->data_bits_);
    ESP_LOGCONFIG(TAG, "  Parity: %s", LOG_STR_ARG(parity_to_str(this->parity_)));
    ESP_LOGCONFIG(TAG, "  Stop bits: %u", this->stop_bits_);
    ESP_LOGCONFIG(TAG, "  error: %s", this->error_string_.c_str());
    ESP_LOGCONFIG(TAG, "  error code: %d", this->error_code_);
    ESP_LOGCONFIG(TAG, "  filter min: %d", this->store_.config.signal_range_min_ns);
    ESP_LOGCONFIG(TAG, "  filter max: %d", this->store_.config.signal_range_max_ns);
    this->check_logger_conflict();
  }

  void RMTUARTComponent::setup() {
    ESP_LOGCONFIG(TAG, "Extra uarts on TX: %d, RX: %d, Baud Rate: %d", tx_pin_, rx_pin_, baud_rate_);

#if ESP_IDF_VERSION_MAJOR >= 5
    RAMAllocator<rmt_symbol_word_t> rmt_allocator(this->use_psram_ ? 0 : RAMAllocator<rmt_symbol_word_t>::ALLOC_INTERNAL);
  
    // 10 bits per byte, 1 rmt_symbol_word_t per bit + 1 rmt_symbol_word_t for reset
    this->rmt_tx_buf_ = rmt_allocator.allocate(tx_buffer_size_ * 10 + 1);
  
    rmt_tx_channel_config_t tx_channel;
    memset(&tx_channel, 0, sizeof(tx_channel));
    tx_channel.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_channel.resolution_hz = CLOCK_HZ;
    tx_channel.gpio_num = gpio_num_t(this->tx_pin_);
    tx_channel.mem_block_symbols = this->rmt_tx_symbols_;
    tx_channel.trans_queue_depth = 1;
    tx_channel.flags.io_loop_back = 0;
    tx_channel.flags.io_od_mode = 0;
    tx_channel.flags.invert_out = 0;
    tx_channel.flags.with_dma = 0;
    tx_channel.intr_priority = 0;
    if (rmt_new_tx_channel(&tx_channel, &this->tx_channel_) != ESP_OK) {
      ESP_LOGE(TAG, "Channel tx creation failed");
      this->mark_failed();
      return;
    }
  
    rmt_copy_encoder_config_t encoder;
    memset(&encoder, 0, sizeof(encoder));
    if (rmt_new_copy_encoder(&encoder, &this->encoder_) != ESP_OK) {
      ESP_LOGE(TAG, "Encoder tx creation failed");
      this->mark_failed();
      return;
    }
  
    if (rmt_enable(this->tx_channel_) != ESP_OK) {
      ESP_LOGE(TAG, "Enabling tx_channel_ failed");
      this->mark_failed();
      return;
    }

    //register tx callback
    rmt_tx_event_callbacks_t tx_callbacks;
    memset(&tx_callbacks, 0, sizeof(tx_callbacks));
    tx_callbacks.on_trans_done = rmt_tx_callback;
    esp_err_t error = rmt_tx_register_event_callbacks(this->tx_channel_, &tx_callbacks, &tx_is_sending_);
    if (error != ESP_OK) {
        this->error_code_ = error;
        this->error_string_ = "in rmt_tx_register_event_callbacks";
        this->mark_failed();
        return;
    }

    
    // RX Configuration

    this->rx_buffer_ =  new uint8_t[this->rx_buffer_size_];

    rmt_rx_channel_config_t rx_channel;
    memset(&rx_channel, 0, sizeof(rx_channel));
    rx_channel.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_channel.resolution_hz = CLOCK_HZ;
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
    callbacks.on_recv_done = rmt_rx_callback;
    error = rmt_rx_register_event_callbacks(this->rx_channel_, &callbacks, &this->store_);
    if (error != ESP_OK) {
        this->error_code_ = error;
        this->error_string_ = "in rmt_rx_register_event_callbacks";
        this->mark_failed();
        return;
    }

    
    uint32_t event_size = sizeof(rmt_rx_done_event_data_t);
    uint32_t max_filter_ns = 10 * (1000000000/(CLOCK_HZ));
    uint32_t max_idle_ns = 32768u * (1000000000/(CLOCK_HZ));
    uint32_t bits_per_byte = 1 /*startbit*/ + this->data_bits_ + (this->parity_ != esphome::uart::UART_CONFIG_PARITY_NONE ? 1 : 0) + this->stop_bits_;
    memset(&this->store_.config, 0, sizeof(this->store_.config));
    this->store_.config.signal_range_min_ns = std::min(((uint32_t)/* 80% of baud_rate*/8000000000u/(this->baud_rate_)), max_filter_ns);
    this->store_.config.signal_range_max_ns = std::min(1000000000u/(this->baud_rate_/bits_per_byte), max_idle_ns);
    ESP_LOGE(TAG, "signal range min: %d max: %d", this->store_.config.signal_range_min_ns, this->store_.config.signal_range_max_ns);
    this->store_.receive_size = this->rmt_rx_symbols_ * sizeof(rmt_symbol_word_t);
    this->store_.buffer_size = (event_size + this->store_.receive_size) * 3;
    this->store_.buffer = new uint8_t[this->store_.buffer_size];
    ESP_LOGE(TAG, "buffer size: %d", this->store_.buffer_size);
    error = rmt_receive(this->rx_channel_, (uint8_t *) this->store_.buffer + event_size, this->store_.receive_size,
                        &this->store_.config);
    if (error != ESP_OK) {
        this->error_code_ = error;
        this->error_string_ = "in rmt_receive";
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
    tx_is_sending_ = true;
    error = rmt_transmit(this->tx_channel_, this->encoder_, symbols, 1 * sizeof(rmt_symbol_word_t), &config);
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
    rx_head_ = (rx_head_ + 1) % this->rx_buffer_size_;
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

    if (tx_is_sending_) {
        return;
    }
    uint8_t rmt_bits_per_send_bytes = 1 /*start bit */ + this->data_bits_ + (this->parity_ != esphome::uart::UART_CONFIG_PARITY_NONE ? 1 : 0) + 1 /*stop bit always 1 sinds we make the time longer*/;

    if (rmt_tx_symbols_ <= (length * rmt_bits_per_send_bytes)/2 ) {
        length = (rmt_tx_symbols_ / rmt_bits_per_send_bytes) * 2;
    }

    rmt_symbol_half_word_t *half_symbols = (rmt_symbol_half_word_t *) this->rmt_tx_buf_;
    uint8_t byte;
    
    for (int j = 0; j < length; j++) {
        byte = tx_buffer_[(tx_head_ + j) % UART_TX_BUFFER_SIZE];
        bool parity_bit = (this->parity_ != esphome::uart::UART_CONFIG_PARITY_NONE) ? __builtin_parity(byte) : 0;
         // Start bit
        half_symbols[j * rmt_bits_per_send_bytes].duration0 = this->baud_rate_timing_array_[0];
        half_symbols[j * rmt_bits_per_send_bytes].level0 = 0;
        for (int i = 0; i < this->data_bits_; i++) {
            half_symbols[j * rmt_bits_per_send_bytes + i + 1].duration0 = this->baud_rate_timing_array_[i + 1];
            half_symbols[j * rmt_bits_per_send_bytes + i + 1].level0 = (byte >> (i)) & 1;
        }
        if(this->parity_ != esphome::uart::UART_CONFIG_PARITY_NONE)
        {
            // Parity bit
            half_symbols[j * rmt_bits_per_send_bytes + this->data_bits_ + 1].duration0 = this->baud_rate_timing_array_[9];
            half_symbols[j * rmt_bits_per_send_bytes + this->data_bits_ + 1].level0 = (this->parity_ != esphome::uart::UART_CONFIG_PARITY_ODD) ? parity_bit & 1 : (parity_bit ^ 1)  & 1; 
        }
        // stop bit
        half_symbols[j * rmt_bits_per_send_bytes + rmt_bits_per_send_bytes - 1].duration0 = this->stop_bits_ * this->baud_rate_timing_array_[9];
        half_symbols[j * rmt_bits_per_send_bytes + rmt_bits_per_send_bytes - 1].level0 = 1; 
    }
    uint8_t rmt_symbols_to_send = (length * rmt_bits_per_send_bytes)/2;
    if((length * rmt_bits_per_send_bytes) % 2 != 0)
    {
        half_symbols[length * rmt_bits_per_send_bytes].duration0 = 1;
        half_symbols[length * rmt_bits_per_send_bytes].level0 = 1;
        rmt_symbols_to_send++;
    }
    rmt_symbol_word_t *symbols = (rmt_symbol_word_t *) half_symbols;

    
#if ESP_IDF_VERSION_MAJOR >= 5
    rmt_transmit_config_t config;
    memset(&config, 0, sizeof(config));
    config.loop_count = 0;
    config.flags.eot_level = 1;
    tx_is_sending_ = true;
    error = rmt_transmit(this->tx_channel_, this->encoder_, half_symbols, rmt_symbols_to_send * sizeof(rmt_symbol_word_t), &config);

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
    rx_tail_ = (rx_tail_ + 1) % this->rx_buffer_size_;
}

void RMTUARTComponent::decode_rmt_rx_data(const rmt_symbol_word_t *symbols, int count) {
    uint32_t min_time_bit = this->baud_rate_timing_array_[0];
    uint32_t half_time_bit = (this->baud_rate_timing_array_[0] /2);
    uint16_t data_bytes_mask = (1 << this->data_bits_) - 1;
    uint16_t bits_per_byte = 1 /*startbit*/ + this->data_bits_ + (this->parity_ != esphome::uart::UART_CONFIG_PARITY_NONE ? 1 : 0) + this->stop_bits_;
    uint32_t total_byte_duration = 0;
    uint32_t received_byte = 0;
    uint8_t recived_bits = 0;
    uint32_t total_recived_bits = 0;
    rmt_symbol_half_word_t *half_symbols = (rmt_symbol_half_word_t *) symbols;
    uint32_t count_half = count * 2;
    bool parity_bit = false;

    for (int i = 0; i < count_half; i++)
    {
        if (total_recived_bits == 0 && (half_symbols[i].level0 != 0))
        {
            ESP_LOGD(TAG, "Start bit not found pin %d", this->rx_pin_);
            continue;
        }

        uint32_t duration = half_symbols[i].duration0;
        recived_bits = 0;

        if (half_symbols[i].duration0 == 0)
        {
            //this is the last bit, before idle
            recived_bits = bits_per_byte - total_recived_bits;
        }
        else if (half_symbols[i].duration0 <= half_time_bit)
        {
            //this is the last bit, before idle
            recived_bits = 1;
        }
        else
        {
            for (uint32_t bit_time = min_time_bit*total_recived_bits ; bit_time < (total_byte_duration + duration); bit_time += min_time_bit)
            {
                uint32_t mid_point = bit_time + half_time_bit;
                if (mid_point < (total_byte_duration + duration) )
                {
                    recived_bits++;
                }
            }
        }

        // ESP_LOGD(TAG, "%d l %d, d  %d, r_bits %d, t_r_bits %d t_time %d", i, half_symbols[i].level0, duration, recived_bits, total_recived_bits, total_byte_duration);

        uint8_t recived_bits_set = recived_bits;
        if (recived_bits_set + total_recived_bits > bits_per_byte)
        {
            recived_bits_set = bits_per_byte - total_recived_bits;
        }

        if (half_symbols[i].level0 == 1) {
            for (uint8_t j = 0; j < recived_bits_set; j++) {
                received_byte |= (1 << (j + total_recived_bits));
            }
        }

        total_recived_bits += recived_bits;
        total_byte_duration += duration;

        if (total_recived_bits >= bits_per_byte)
        {
            uint8_t data_byte = (uint8_t)((received_byte >> 1) & data_bytes_mask);
            if (this->parity_ != esphome::uart::UART_CONFIG_PARITY_NONE)
            {
                parity_bit = (received_byte >> (this->data_bits_ + 1)) & 1;
                bool calculated_parity = __builtin_parity(data_byte);
                if ((this->parity_ == esphome::uart::UART_CONFIG_PARITY_EVEN && calculated_parity != parity_bit) ||
                    (this->parity_ == esphome::uart::UART_CONFIG_PARITY_ODD && calculated_parity == parity_bit)) 
                {
                    ESP_LOGW(TAG, "Parity error detected  %02x t %03x b %d time %d", data_byte, received_byte, total_recived_bits, total_byte_duration);
                }
                else
                {
                    put_rx_byte(data_byte);
                    ESP_LOGD(TAG, "Rx byte %02x t %03x b %d time %d", data_byte, received_byte, total_recived_bits, total_byte_duration);
                }
            }
            else
            {
                put_rx_byte(data_byte);
                ESP_LOGD(TAG, "Rx byte %02x t %03x", data_byte, received_byte);
            }
            received_byte = 0;
            total_recived_bits = 0;
            total_byte_duration = 0;
        }
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
    return (rx_tail_ - rx_head_ + this->rx_buffer_size_) % this->rx_buffer_size_;
}

void RMTUARTComponent::flush() {
    while (tx_head_ != tx_tail_) {
        process_tx_queue();
        delay(1);
    }

    // Wait for all bytes to be sent
    while(tx_is_sending_) {
        delay(1);
    }
}

void RMTUARTComponent::loop()
{
    #if ESP_IDF_VERSION_MAJOR >= 5
    if (this->store_.error != ESP_OK) {
        ESP_LOGE(TAG, "Receive error");
        this->error_code_ = this->store_.error;
        this->error_string_ = "in rmt_rx_callback";
        this->mark_failed();
    }
    if (this->store_.overflow) {
        ESP_LOGW(TAG, "Buffer overflow read: %d write: %d", this->store_.buffer_read, this->store_.buffer_write);
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
        ESP_LOGCONFIG(TAG, "symbols used %d of %d", next_read, this->store_.buffer_size);
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
