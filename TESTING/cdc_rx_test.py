import serial
import time

PORT = "/dev/ttyACM0"
BAUD = 115200
TIMEOUT = 1.0

def hexdump(data):
    return " ".join(f"{b:02X}" for b in data)

def main():
    print(f"Abriendo puerto {PORT}...")
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        timeout=TIMEOUT
    )

    # CDC necesita un pequeño delay al abrir
    time.sleep(2)

    print("Escuchando datos (Ctrl+C para salir)...\n")

    try:
        while True:
            data = ser.readline()  # lee hasta \n o timeout
            if data:
                print(f"RX ({len(data)} bytes)")
                print(" HEX :", hexdump(data))
                print(" ASCII:", data.decode(errors="ignore").strip())
                print("-" * 40)
            else:
                print(" (timeout)")
    except KeyboardInterrupt:
        print("\nSaliendo...")
    finally:
        ser.close()
        print("Puerto cerrado.")

if __name__ == "__main__":
    main()
