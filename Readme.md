# ESP32 RMT uart

Bam i just created ~~3~~ 4 extra uarts on the esp32....
~~not jet it is just a thery that i have come up with chatgpt~~
~~i got the first part working, it still stays a prove of concept.~~
I got the transmit and the revice part working, and the output looks prommising.
~~I do not know if I can get it working with something like modbus. and get it as compatible as the normal uart, but i can use the fuctions get get_array and write_array, witch is normaly used.~~
A test modbus client is working with the esp32_rmt_uart!! it is almost the same as the normal uart!
there are still some small bugs and features, but i am working on it.
Currently i used a lot of code from the weikai uart and the remote_receiver and the esp32_rmt_led_strip with already using the rmt perriferal.

feel free to help

Todo list:
- [x] The tx part works!, tested with the list of baudrates
- [x] The rx part 
- [x] make sure you can have multiple of then (make config array/list)
- [ ] turn one of the pins off
- [x] rx_buffer_size 
- [x] make check on rx_buffer_size symbols (understand recive buffer!)
- [x] data_bits 
- [x] stop bits
- [x] parity 
- [ ] psudo ram?
 -[ ] make it work for not ESP_IDF_VERSION_MAJOR >= 5?
- [ ] test with multiple uarts at the same time
- [ ] use less symbols when sending (all bits after eath other that are the same can use the same symbol, with requesting data with modbus, you can use a lot less symbols)
- [x] check if this works with modbus or other components
- [ ] create a correct flush function

# Examples

## uart echo
A uart test can you find in [esp32_rmt_uart_test.yaml](esp32_rmt_uart_test.yaml) you need to add the [base_example.yaml](base_example.yaml)
it recives characters and sends them back every 5 seconds. the pins are set to de default pins of the esp32. So you can just use a example board

## modbus
A modbus test can you find in [esp32_rmt_uart_modbus_test.yaml](esp32_rmt_uart_modbus_test.yaml) you need to add the [base_example.yaml](base_example.yaml)
Ony your pc you need to run [test_modbus_server.py](test_modbus_server.py). (requires python3 and pyserial, pymodbus==3.8.4)
the esp32 will request holding register every 0.5 sec and the python script will answer. and every read the value will be increased by 1.

