#pragma once

// #include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

#include <esp_idf_version.h>

#if ESP_IDF_VERSION_MAJOR >= 5
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>
#else
#include <driver/rmt.h>
#endif

namespace esphome {
namespace esp32_rmt_uart {

#define DEFAULT_BAUD_RATE 9600
#define UART_TX_BUFFER_SIZE 256
#define UART_RX_BUFFER_SIZE 256

//class RMTUARTComponent : public uart::UARTComponent{

class RMTUARTComponent : public Component, public uart::UARTComponent {
 public:
    RMTUARTComponent(int tx_pin, int rx_pin, int baud_rate = DEFAULT_BAUD_RATE);
    void setup() override;
    void loop() override;
    void write_byte(uint8_t byte) override;
    bool read_byte(uint8_t *byte) override;
    void set_baud_rate(int baud_rate) { baud_rate_ = baud_rate; }

#if ESP_IDF_VERSION_MAJOR >= 5
    void set_tx_rmt_symbols(uint32_t rmt_symbols) { this->rmt_tx_symbols_ = rmt_symbols; }
    void set_rx_rmt_symbols(uint32_t rmt_symbols) { this->rmt_rx_symbols_ = rmt_symbols; }
#else
    void set_tx_rmt_channel(rmt_channel_t channel) { this->tx_channel_ = channel; }
    void set_rx_rmt_channel(rmt_channel_t channel) { this->rx_channel_ = channel; }
#endif
  //
  // we implements/overrides the virtual class from UARTComponent
  //

  /// @brief Writes a specified number of bytes to a serial port
  /// @param buffer pointer to the buffer
  /// @param length number of bytes to write
  /// @details This method sends 'length' characters from the buffer to the serial line. Unfortunately (unlike the
  /// Arduino equivalent) this method does not return any flag and therefore it is not possible to know if any/all bytes
  /// have been transmitted correctly. Another problem is that it is not possible to know ahead of time how many bytes
  /// we can safely send as there is no tx_available() method provided! To avoid overrun when using the write method you
  /// can use the flush() method to wait until the transmit fifo is empty.
  /// @n Typical usage could be:
  /// @code
  ///   // ...
  ///   uint8_t buffer[128];
  ///   // ...
  ///   write_array(&buffer, length);
  ///   flush();
  ///   // ...
  /// @endcode
  void write_array(const uint8_t *buffer, size_t length) override;

  /// @brief Reads a specified number of bytes from a serial port
  /// @param buffer buffer to store the bytes
  /// @param length number of bytes to read
  /// @return true if succeed, false otherwise
  /// @details Typical usage:
  /// @code
  ///   // ...
  ///   auto length = available();
  ///   uint8_t buffer[128];
  ///   if (length > 0) {
  ///     auto status = read_array(&buffer, length)
  ///     // test status ...
  ///   }
  /// @endcode
  bool read_array(uint8_t *buffer, size_t length) override;

  /// @brief Reads the first byte in FIFO without removing it
  /// @param buffer pointer to the byte
  /// @return true if succeed reading one byte, false if no character available
  /// @details This method returns the next byte from receiving buffer without removing it from the internal fifo. It
  /// returns true if a character is available and has been read, false otherwise.\n
  bool peek_byte(uint8_t *buffer) override;

  /// @brief Returns the number of bytes in the receive buffer
  /// @return the number of bytes available in the receiver fifo
  int available() override;

  /// @brief Flush the output fifo.
  /// @details If we refer to Serial.flush() in Arduino it says: ** Waits for the transmission of outgoing serial data
  /// to complete. (Prior to Arduino 1.0, this the method was removing any buffered incoming serial data.). ** Therefore
  /// we wait until all bytes are gone with a timeout of 100 ms
  void flush() override;

 private:
    int baud_rate_;


#if ESP_IDF_VERSION_MAJOR >= 5
    rmt_channel_handle_t channel_{nullptr};
    rmt_encoder_handle_t encoder_{nullptr};
    rmt_symbol_word_t *rmt_buf_{nullptr};
    uint32_t rmt_tx_symbols_;
    uint32_t rmt_rx_symbols_;
#else
    rmt_item32_t *rmt_buf_{nullptr};
    rmt_channel_t tx_channel_{RMT_CHANNEL_0};
    rmt_channel_t rx_channel_{RMT_CHANNEL_0};
#endif

    uint8_t tx_buffer_[UART_TX_BUFFER_SIZE];
    uint8_t rx_buffer_[UART_RX_BUFFER_SIZE];
    int tx_head_, tx_tail_;
    int rx_head_, rx_tail_;
    void process_tx_queue();
    void decode_rmt_rx_data(const rmt_symbol_word_t *symbols, int count);
};


}  // namespace esp32_rmt_uart
}  // namespace esphome
