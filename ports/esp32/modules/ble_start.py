#this will start ble advertising on boot up
try:
    import ble_uart_repl
    ble_uart_repl.start()
except:
    print('failed to start bluetooth')