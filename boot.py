from machine import Pin
BLE_NAME = None
if Pin("USER", Pin.IN).value():
    from atlas.ble import ble_repl
    BLE_NAME = "Atlas-1"
    ble_repl.start(BLE_NAME)
    
