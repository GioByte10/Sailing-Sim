import threading
import time
from evdev import InputDevice, categorize, ecodes, list_devices

# Shared variable
mode = "idle"
mode_lock = threading.Lock()  # For thread-safe access

def find_controller():
    devices = [InputDevice(path) for path in list_devices()]
    for device in devices:
        print(f"Found: {device.name} at {device.path}")
        if 'Xbox' in device.name or 'xbox' in device.name:
            print("Using:", device.path)
            return device
    return None


def read_controller_inputs(device):
    global mode
    print("Listening for input events...")
    for event in device.read_loop():
        print(f"Raw event: {event}")  # 🔍 See what events actually come i
        key_event = categorize(event)
        if event.type == ecodes.ABS_HAT0Y:
            print("CROSS")

        if event.type == ecodes.EV_KEY:
            key_event = categorize(event)
            print(f"Key event: {key_event}")  # 🔍 What key was pressed?

            if key_event.keystate == key_event.key_down:
                print(f"Keycode: {key_event.keycode}")
                if key_event.keycode == 'BTN_SOUTH':
                    print("A button pressed")
                    with mode_lock:
                        mode = "running"
                elif key_event.keycode == 'BTN_EAST':
                    print("B button pressed")
                    with mode_lock:
                        mode = "stopped"


def main_program_loop():
    global mode
    while True:
        with mode_lock:
            print(f"[MAIN] Current mode: {mode}")
        time.sleep(1)

if __name__ == "__main__":
    controller = find_controller()
    if not controller:
        print("Xbox controller not found.")
        exit(1)

    # Start input reading thread
    input_thread = threading.Thread(target=read_controller_inputs, args=(controller,), daemon=True)
    input_thread.start()

    # Run main logic
    main_program_loop()
