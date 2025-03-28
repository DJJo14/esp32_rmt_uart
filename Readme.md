# ESP32 RMT uart

Bam i just created 4 extra uarts on the esp32....
It started as a prove of concept, but it is working now. I needed it for reading of energy meter of different venders, where the baud rate and other settings could not be set to be compatible with each other. For this uart i used the RMT perriferal.

the RMT perriferal is meant to be used for receiving and sending ir and rf signals. It is already used in esphome for the IR remote and the led strip.
I used the RMT perriferal to send and recive uart signals, because it is non blocking even in the recive part.
This looks perfect but there is a but...

## limitations

This uart is meant to be used in combination with modbus, because the number of bytes that are send and recive are limited.
The RMT perriferal uses symbols. In a symbol you can set the level of the signal and the duration of the signal. To make it more complicated in one symbol you can set 2 levels and 2 durations.

The ESP32 (original, look at the esphome infrared receiver for more info) has a limit of 512 symbols. And it can be spread out over 8 channels. One channel is used for the tx and one for the rx.
Sinds you send and recive data per bit, you need 5 symbols for 1 byte. 1 start bit, 8 data bits, 1 stop bit. But keep in mind that if you add parity, you need 1 extra symbol.(or 7E1 is 10 bit again)
But there are more limitations the minimal symbols that you can use per channel is 64.

If the data is predictable, you can use less symbols. (Currently only for the rx part, see TODO list) For example if you receive a lot of 0's (bits) you can use 1 symbol, since the level does not change and only the duration takes longer.

This is why this is perfect for modbus, because you know (for the most part) what you are going to receive. If you request a holding register, you know that you are going to recive 2 bytes per holding register (+ adress + count + crc (1+1+2bytes)). And if you only send to modbus adress 0x01, you know that you are going to recive the same 0x01 byte back. So for the adress part it uses (1 (startbit) + 1 (adress high) + 1 (adress low) + 1 (stop bit))/2 = 2 symbols. 
But if you addres is 0x55 (binary 01010101) you need all 10/2 = 5 symbols. (always keep the 5 symbols in mind for the unknown data)

Here's an example of how the symbols work:

```wavedrom
{signal: [
    {name: 'bits', wave: 'x2434343432x', data: ['start', '1', '2', '3', '4', '5', '6', '7', '8', 'stop'], period: 1},
    {name: 'Data 0x55', wave: '1.0.1.0.1.0.1.0.1.0.1...', period: 0.5},
    {name: 'half symbols', wave: 'x2434343432x', data: ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10'], period: 1},
],
 head: {
     text: 'Symbols used Reciving 0x55 10/2=5 symbols',
 }
}
```

When sending address 0x01, fewer symbols are needed because the bit pattern is simpler:

```wavedrom
{signal: [
    {name: 'bits', wave: 'x2434343432x', data: ['start', '1', '2', '3', '4', '5', '6', '7', '8', 'stop'], period: 1},
    {name: 'Data 0x01', wave: '1.0.1.0.............1...', period: 0.5},
    {name: 'half symbols', wave: 'x243......2x', data: ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10'], period: 1},
],
 head: {
     text: 'Symbols used Reciving 0x01 4/2=2 symbols',
 }
}
```

The newer esp32's have dma on this periferal, so there is not the same limit as the older/clasic esp32's.(limit in dma channels?) But i did not tested this yet, i even got the hardware yet.

**If you think u can just spin up 3 (clasic) + 4 (rmt) = 7 uart, you are probably limited by some other limitation. the cpu usage or the memory usage. But i did not tested this yet. Take it to the limit and let me know**

# Finding so far
I tested a lot op baud rates, and my pc recived them perfectly a usb to uart reciver.
A test with 2 modbus client is working with the esp32_rmt_uart!! with a interval from 0.5 sec and 12 holding registers per uart. and it workd good. It is almost the same as the normal uart!

There are still some small bugs and features, but i am working on it.
Currently i used a lot of code from the weikai uart and the remote_receiver and the esp32_rmt_led_strip with already using the rmt perriferal.
thats why the folders are copied from the esphome project.

Feel free to help!

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
- [ ] DMA?
 -[ ] make it work for not ESP_IDF_VERSION_MAJOR >= 5?
- [X] test with multiple uarts at the same time
- [ ] use less symbols when sending (all bits after eath other that are the same can use the same symbol, with requesting data with modbus, you can use a lot less symbols)
- [x] check if this works with modbus or other components
- [ ] create a correct flush function
- [ ] check function if max symbols are reached.
- [ ] test if you can import it in you esphome yaml
- [ ] Test Baud rate lower than 9600

# Examples

## uart echo
A uart test can you find in [esp32_rmt_uart_test.yaml](esp32_rmt_uart_test.yaml) you need to add the [base_example.yaml](base_example.yaml)
it recives characters and sends them back every 5 seconds. the pins are set to de default pins of the esp32. So you can just use a example board

## modbus
A modbus test can you find in [esp32_rmt_uart_modbus_test.yaml](esp32_rmt_uart_modbus_test.yaml) you need to add the [base_example.yaml](base_example.yaml)
Ony your pc you need to run [test_modbus_server.py](test_modbus_server.py). (requires python3 and pyserial, pymodbus==3.8.4)
the esp32 will request holding register every 0.5 sec and the python script will answer. and every read the value will be increased by 1.

