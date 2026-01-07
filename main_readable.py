import time
import serial
from datetime import datetime

#serial port settings for synchronization with the board, change "COM3" to the port you're using
ser = serial.Serial("COM3", 115200, bytesize = 8, parity = 'N', stopbits = 1, timeout = 1)

#sends the current time and date to the board
def handle_time():
    now = datetime.now()
    formatted_time = f"{now.year} {now.month:02d} {now.day:02d} {now.hour:02d} {now.minute:02d} {now.second:02d}"
    print(f"invio al microcontrollore {formatted_time}")
    ser.write((formatted_time + "\n").encode("utf-8"))

#receives the log from the board and saves it on a txt file
def handle_log():
    print("ricevo il log")

    #sends ACK for handshake protocol
    ser.write(("ready\n").encode("utf-8"))
    
    last_data_time = time.time()

    with open("log.txt", "w", encoding = "utf-8") as file:
        while True:
            #if the communication takes too long it stops
            if time.time() - last_data_time > 10:
                print("TIMEOUT DI RICEZIONE: log terminato")
                break

            if ser.in_waiting >0:
                line = ser.readline().decode(errors = "replace").strip()
                last_data_time = time.time()

                #checks if the log is finished or the communication is aborted from the board
                if line == "log_end":
                    print("fine del log")
                    break
                if line == "log_abort":
                    print("ERRORE: log abortito")
                    break

                file.write(line + "\n")
                print(line)

            #checks the serial port every 0.01 seconds
            else:
                time.sleep(0.01)
               
#all the types of messages the board can send are stored with the corresponding function to call in response
message_handlers = {
    "time_sync" : handle_time,
    "log_start": handle_log
    }

print("in attesa di richiesta")

buffer = b""

#checks the serial port for a message every 0.01 seconds and if recognises a board request calls the corresponding function to handle it
while True:
    if ser.in_waiting > 0:
        buffer += ser.read(ser.in_waiting)

        while b'\n' in buffer:
            line, buffer = buffer.split(b'\n', 1)

            try:
                text = line.decode('ascii', errors='ignore').strip()
            except Exception:
                continue

            if text:
                print(f"RX: {text}")
                handler = message_handlers.get(text)
                if handler:
                    handler()
                    print("in attesa di richiesta")
                else:
                    print(f"messaggio sconosciuto: {text}")

    time.sleep(0.01)
