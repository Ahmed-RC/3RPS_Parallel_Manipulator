import pywinusb.hid as hid

def handle_data(data):
    print("Raw Data:", data)

device_filter = hid.HidDeviceFilter(vendor_id=0x1234, product_id=0x5678)
devices = device_filter.get_devices()
if devices:
    device = devices[0]
    device.open()
    device.set_raw_data_handler(handle_data)
    input("Press Enter to quit...\n")
