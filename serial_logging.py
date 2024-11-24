import serial
import argparse
import signal
import sys
from datetime import datetime

# Global variable to keep the file handle
logfile = None

def signal_handler(sig, frame):
    """Handle signal interrupts to safely close the file."""
    global logfile
    print("\nInterrupt received. Closing the file and exiting.")
    if logfile:
        logfile.close()
    sys.exit(0)

def main():
    global logfile

    # Set up argument parser
    parser = argparse.ArgumentParser(description="Capture Arduino serial output to a file.")
    parser.add_argument(
        "-d", "--device", required=True, help="Path to the serial device (e.g., /dev/ttyUSB0)"
    )
    parser.add_argument(
        "-b", "--baud", type=int, required=True, help="Baud rate for the serial communication"
    )
    parser.add_argument(
        "-o",
        "--output",
        default="arduino_log",
        help="Base name for the output file. A timestamp will be appended.",
    )
    args = parser.parse_args()

    # Create the filename with a timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"{args.output}_{timestamp}.log"

    try:
        # Open the serial connection
        print(f"Connecting to {args.device} at {args.baud} baud...")
        ser = serial.Serial(args.device, args.baud, timeout=1)
        print(f"Connected! Logging to file: {filename}")

        # Open the log file
        logfile = open(filename, "w")

        # Capture serial output
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                print(line)  # Also print to console
                logfile.write(line + "\n")
                logfile.flush()  # Ensure data is written immediately

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        # Clean up resources
        if logfile:
            logfile.close()
        print("Exiting.")

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    main()
