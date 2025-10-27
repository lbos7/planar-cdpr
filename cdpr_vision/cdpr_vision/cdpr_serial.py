import time

def serial_write(ser, data):
    try:
        ser.write((data.strip() + '\n').encode('utf-8'))
    except Exception as e:
        print(f"Serial write failed: {e}")

def serial_read(ser):
    try:
        line = ser.readline().decode('utf-8').strip()
        return line if line else None
    except Exception as e:
        print(f"Serial read failed: {e}")
        return None

def serial_read_thread(ser, queue, tracking_flag):
    while True:
        if tracking_flag[0]:
            line = serial_read(ser)
            if line:
                queue.append(line)
        else:
            time.sleep(0.001)