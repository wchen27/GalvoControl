# GalvoCam Target Sender — Linux Setup Guide

> **Note:** [orange](https://github.com/moments-behavior/orange) now implements the sender
> natively (`src/galvo_sender.cpp`, v2 packets with velocity + age, plus the GCC1 control
> channel) — if you're using orange, nothing here is needed. This guide remains for custom
> senders; it covers the **v1** (48-byte) packet, which the receiver still accepts. Prefer the
> v2 format in **PROTOCOL.md** for fast-target tracking (the receiver extrapolates with it).

For an agent on the **Linux triangulation machine** implementing the *sender* side of the
GalvoCam target-streaming protocol. You send 3D world coordinates; the Windows motor-control app
receives them and aims the mirrors. Wire spec: **PROTOCOL.md** (this guide implements it).

The receiver (Windows app) is already written — it listens on UDP, validates packets, and aims
the pan/tilt mirrors at the latest valid target. Your job is only the sender.

## 1. Physical link + addressing
A direct Ethernet cable connects this Linux box to the Windows box. No DHCP → set static IPs on
the direct-link NIC on both ends, same /24. Suggested:
- Windows (receiver): `192.168.50.1`
- Linux (sender):     `192.168.50.2`

Find the NIC and set the IP:
```bash
ip link                                   # identify the direct-link interface, e.g. enp3s0
sudo ip addr add 192.168.50.2/24 dev <iface>
sudo ip link set <iface> up
ping 192.168.50.1                         # verify you can reach the Windows box
```
(For persistence, use netplan/NetworkManager instead of the transient `ip addr add`.)

The app listens on UDP **5005**. Send datagrams to `192.168.50.1:5005`.

## 2. Packet — must match exactly (48 bytes, little-endian)
| off | size | type    | field        |
|-----|------|---------|--------------|
| 0   | 4    | char[4] | magic = `GCT1` |
| 4   | 2    | uint16  | version = 1  |
| 6   | 2    | uint16  | flags (bit0 = valid) |
| 8   | 4    | uint32  | seq          |
| 12  | 4    | uint32  | target_id (0) |
| 16  | 8    | uint64  | timestamp_ns |
| 24  | 8    | float64 | x (world mm) |
| 32  | 8    | float64 | y            |
| 40  | 8    | float64 | z            |

## 3. Python sender (reference)
```python
import socket, struct, time

WIN_IP, PORT = "192.168.50.1", 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

FMT = "<4sHHIIQ ddd".replace(" ", "")   # little-endian: 4s H H I I Q d d d
assert struct.calcsize(FMT) == 48

_seq = 0
def send_target(x, y, z, valid=True, target_id=0):
    global _seq
    pkt = struct.pack(FMT, b"GCT1", 1, (1 if valid else 0),
                      _seq & 0xFFFFFFFF, target_id, time.monotonic_ns(),
                      float(x), float(y), float(z))
    sock.sendto(pkt, (WIN_IP, PORT))
    _seq += 1

# stream loop — replace get_triangulated_point() with your triangulation output (world mm)
while True:
    pt = get_triangulated_point()          # -> (x, y, z) or None if tracking lost
    if pt is not None:
        send_target(pt[0], pt[1], pt[2], valid=True)
    else:
        send_target(0, 0, 0, valid=False)  # tell the receiver to hold
    time.sleep(0.005)                       # ~200 Hz; match your source rate
```

## 4. C sender (reference)
```c
#include <arpa/inet.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#pragma pack(push, 1)
typedef struct {
    char     magic[4];      // 'G','C','T','1'
    uint16_t version;       // 1
    uint16_t flags;         // bit0 = valid
    uint32_t seq;
    uint32_t target_id;
    uint64_t timestamp_ns;
    double   x, y, z;        // world mm
} GcTargetPacket;            // 48 bytes
#pragma pack(pop)

static int      sock_fd;
static struct sockaddr_in dst;
static uint32_t seq;

void sender_init(const char* win_ip, int port) {
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(port);
    inet_pton(AF_INET, win_ip, &dst.sin_addr);
}

void send_target(double x, double y, double z, int valid, uint32_t id) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    GcTargetPacket p;
    memcpy(p.magic, "GCT1", 4);
    p.version = 1;
    p.flags   = valid ? 1 : 0;
    p.seq     = seq++;
    p.target_id = id;
    p.timestamp_ns = (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
    p.x = x; p.y = y; p.z = z;
    sendto(sock_fd, &p, sizeof(p), 0, (struct sockaddr*)&dst, sizeof(dst));
}
// Both machines are x86-64 little-endian, so the native struct maps directly to the wire.
```

## 5. What to send
- **x, y, z in world millimeters** — the same frame the GalvoCam is calibrated to.
- Set **valid=1** only for a real triangulated point. When tracking is lost, send **valid=0**
  (or stop sending) so the mirrors hold instead of chasing a stale/garbage point.
- Increment **seq** every packet. Use **target_id = 0** (single GalvoCam).

## 6. Testing against the Windows app
1. Windows: launch the app, open **"Network target (UDP one-way)"**, set port 5005, tick
   **receive targets**. The `rx … pkts / seq / VALID / age` line updates as packets arrive.
2. Run your sender; watch `rx` climb and **last target** match what you send.
3. Tick **aim at network target** on Windows → the mirrors aim at your streamed point. Start with
   the motors free/clear and small coordinate changes.
4. Send `valid=0` → the app shows **no-target** and holds.

## 7. Notes / troubleshooting
- Fire-and-forget, no ACK. If the app's **age** keeps rising, packets aren't arriving — check IP,
  cable, port, and the **Windows firewall** (allow inbound UDP 5005, or the app, on the link).
- Keep rate ≤ ~1 kHz; the receiver aims at ~30 Hz from the latest packet.
- Endianness: both ends are x86-64 LE, so no byte swapping.
