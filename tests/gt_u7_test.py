import serial
import pynmea2

def get_gps_location(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate)
        while True:
            data = ser.readline().decode('utf-8')
            if data.startswith('$GNGGA') or data.startswith('$GPGGA'):
                try:
                    msg = pynmea2.parse(data)
                    latitude = msg.latitude
                    longitude = msg.longitude
                    altitude = msg.altitude
                    latitude, longitude, altitude = mylist
                    return mylist
                except pynmea2.ParseError:
                    continue
    except serial.SerialException:
        print("Error: Serial port not available or in use.")

if __name__ == "__main__":
    latitude, longitude, altitude = get_gps_location("/dev/ttyAMC0/", "9600")
    print("Latitude:", latitude)
    print("Longitude:", longitude)
    print("Altitude:", altitude)
