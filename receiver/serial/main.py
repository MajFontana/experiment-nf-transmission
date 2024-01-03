import serial
import pyaudio
import time
import math

ang = 0
freq = 440
def callback(in_data, frame_count, time_info, status):
    global ang
    if freq == -1:
        data = frame_count * b"\x00\x00"
    else:
        data = b""
        for i in range(frame_count):
            amp = int(math.sin(ang) * 32767)
            data += amp.to_bytes(2, "little", signed=True)
            ang += 2 * math.pi * freq / 44100
    return (data, pyaudio.paContinue)

pa = pyaudio.PyAudio()
stream = pa.open(format=pa.get_format_from_width(2),
                channels=1,
                rate=44100,
                output=True,
                frames_per_buffer=16,
                stream_callback=callback)
stream.start_stream()

serial = serial.Serial("COM9", 115200, timeout=1)
while 1:
    reads = [0, 0, 0]
    for i in range(1):
        d = serial.read(1)
        if d == b"2":
            reads[0] += 1
        elif d == b"0":
            reads[1] += 1
        elif d == b"1":
            reads[2] += 1
    if reads.index(max(reads)) == 0:
        freq = -1
    elif reads.index(max(reads)) == 1:
        freq = 440
    elif reads.index(max(reads)) == 2:
        freq = 880
