import smbus
import time


bus = smbus.SMBus(1)


nrem = 0
nprom = [0x1000, 0x7100, 0x6300, 0x4500, 0x4100, 0x6700, 0x6900, 0x0]

def startup():
    bus.write_byte(0x76, 0x1E)
    # rospy.sleep(.01)
    time.sleep(.01)
    print('ok')
    C = []
    for i in range(7):
        c = bus.read_word_data(0x76, 0xA0 + 2 * i)
        c = ((c & 0xFF) << 8) | (c >> 8)
        C.append(c)
    print([hex(data) for data in C])
    crc = (C[0] & 0xF000) >> 12
    print(crc)
    # if crc != _crc4(.C):
    #     return

def _crc4(n_prom):
    n_rem = 0

    n_prom[0] = ((n_prom[0]) & 0x0FFF)
    n_prom.append(0)

    for i in range(16):
        if i % 2 == 1:
            n_rem ^= ((n_prom[i >> 1]) & 0x00FF)
        else:
            n_rem ^= (n_prom[i >> 1] >> 8)

        for n_bit in range(8, 0, -1):
            if n_rem & 0x8000:
                n_rem = (n_rem << 1) ^ 0x3000
            else:
                n_rem = (n_rem << 1)

    n_rem = ((n_rem >> 12) & 0x000F)

    n_prom = n_prom
    n_rem = n_rem

    return n_rem ^ 0x00

# nrem = _crc4(nprom)
# print(nrem)

startup()