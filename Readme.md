# ESP32 RMT uart

Bam i just created ~~3~~ 4 extra uarts on the esp32....
~~not jet it is just a thery that i have come up with chatgpt~~
~~i got the first part working, it still stays a prove of concept.~~
I got the transmit and the revice part working, and the output looks prommising.
I do not know if I can get it working with something like modbus. and get it as compatible as the normal uart, but i can use the fuctions get get_array and write_array, witch is normaly used.
Currently i used a lot of code from the weikai uart and the remote_receiver and the esp32_rmt_led_strip with already using the rmt perriferal.

feel free to help

Todo list:
- [x] The tx part works!, tested with the list of baudrates
- [x] The rx part 
- [x] make sure you can have multiple of then (make config array/list)
- [ ] turn one of the pins off
- [x] rx_buffer_size 
- [ ] make check on rx_buffer_size symbols (understand recive buffer!)
- [ ] data_bits 
- [ ] parity 
- [ ] psudo ram?
 -[ ] make it work for not ESP_IDF_VERSION_MAJOR >= 5?
- [ ] test with multiple at the same time
- [ ] use less symbols when sending (all bits after eath other that are the same can use the same symbol, with requesting data with modbus, you can use a lot less symbols)
- [ ] check if this works with modbus or other components
- [ ] create a correct flush function


the testing configuration
```yaml
external_components:
  - source:
      type: local
      path: "/config/components"

esp32:
  board: esp32dev
  framework:
    type: esp-idf


# Enable logging
logger:
  level: DEBUG 
  baud_rate: 0

# and the extra's for api and wifi

web_server:
  port: 80

esp32rmtuart:
  id: rmt1
  tx_pin: 1
  rx_pin: 3
  # tested baud_rate
  # baud_rate: 19200
  # baud_rate: 38400
  # baud_rate: 57600
  baud_rate: 115200
  # baud_rate: 230400
  # baud_rate: 460800
  # baud_rate: 921600
  # rmt_tx_channel: 1
  # rmt_rx_channel: 2

interval:
  - interval: 1s
    then:
      - logger.log: "sending A"
      - lambda: |-
          const uint8_t str1[] = "hello World \r\n";
          id(rmt1).write_array(str1, sizeof(str1));
      # - delay: 
      #     10s
      # - lambda: id(rmt1).write_byte('a');
      # - uart.write: 
      #     id: rmt1
      #     data: 'Hello World\r\n'
      # - lambda: id(rmt1).write_byte('b');
      # - lambda: id(rmt1).write_byte('c');
```