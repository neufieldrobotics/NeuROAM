#!/usr/bin/env python3
import time, curses, sys, rclpy, datetime
from collections import deque
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

FIX_TOPIC = '/ublox_gps_node/fix'   # adjust if your topic differs
COUNT_WINDOW_SEC = 10               # window to compute message rate

BIG_FIX = [
    " ███████ ██ ██    ██ ",
    " ██      ██  ██  ██  ",
    " ███████ ██   ████   ",
    " ██      ██  ██  ██  ",
    " ██      ██ ██    ██ ",
]
BIG_NOFIX = [
    " ████  ██   ████    ███████ ██ ██    ██ ",
    " ██ ██ ██  ██  ██   ██      ██  ██  ██  ",
    " ██  █ ██  ██  ██   ███████ ██   ████   ",
    " ██   ███  ██  ██   ██      ██  ██  ██  ",
    " ██    ██   ████    ██      ██ ██    ██ ",
]

# === SMALL banners (fits ~40x18) ===
SMALL_FIX = ["+----------+",
             "|   FIX    |",
             "+----------+"]
SMALL_NOFIX = ["+------------+",
                     "|  NO  FIX   |",
                     "+------------+"]
def safe_addstr(stdscr, y, x, s):
    if s is None: return
    h, w = stdscr.getmaxyx()
    if not (0 <= y < h): return
    x = max(0, x)
    if x >= w: return
    maxlen = max(0, w - x - 1)
    try:
        stdscr.addnstr(y, x, s, maxlen)
    except curses.error:
        pass

def draw_center(stdscr, art, top=1):
    h, w = stdscr.getmaxyx()
    for i, row in enumerate(art):
        y = top + i
        x = max(0, (w - len(row)) // 2)
        safe_addstr(stdscr, y, x, row)

def ros_stamp_to_iso(stamp):
    try:
        epoch = stamp.sec + stamp.nanosec / 1e9
        return datetime.datetime.utcfromtimestamp(epoch).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] + 'Z'
    except Exception:
        return "n/a"

class CGPS(Node):
    def __init__(self, stdscr):
        super().__init__('cgps_like_tui')
        self.s = stdscr
        self.s.nodelay(True)
        curses.curs_set(0)
        self.msg = None
        self.last_rx = None
        self.fix_seen = False
        self.fix_start = None
        self.countdown = 15
        # self.shutdown = False
        self.msg_count = 0
        self.arrivals = deque()
        self.create_subscription(NavSatFix, FIX_TOPIC, self.on_fix, 100)
        self.create_timer(0.1, self.tick)

    def on_fix(self, msg: NavSatFix):
        self.msg = msg
        now = time.time()
        self.last_rx = now
        self.msg_count += 1
        self.arrivals.append(now)
        cutoff = now - COUNT_WINDOW_SEC
        while self.arrivals and self.arrivals[0] < cutoff:
            self.arrivals.popleft()
        if msg.status.status in (0, 1, 2) and not self.fix_seen:
            self.fix_seen = True
            self.fix_start = now

    def rate_hz(self):
        if len(self.arrivals) < 2: return 0.0
        span = self.arrivals[-1] - self.arrivals[0]
        return (len(self.arrivals) - 1) / span if span > 0 else 0.0

    # def quit_clean(self, code=0):
    #     self.shutdown = True 
        #try: rclpy.shutdown()
        #except Exception: pass
        #curses.endwin()
        #os._exit(code)

    def tick(self):
        try:
            ch = self.s.getch()
            if ch in (ord('q'), ord('Q')): raise SystemExit #self.quit_clean(0)
        except curses.error:
            pass

        self.s.erase()
        h, w = self.s.getmaxyx()

        # choose banner size based on window
        use_big = (w >= 64 and h >= 28)
        banner = BIG_FIX if (self.fix_seen and use_big) else \
                 BIG_NOFIX if use_big else \
                 SMALL_FIX if self.fix_seen else SMALL_NOFIX
        top = 1
        try: self.s.border()
        except curses.error: pass
        safe_addstr(self.s, 0, 2, " cgps(ROS) ")

        draw_center(self.s, banner, top)
        base = top + len(banner) + 1

        safe_addstr(self.s, base, 2, f"Topic: {FIX_TOPIC}")
        rate = self.rate_hz()
        sys_now = datetime.datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] + 'Z'
        safe_addstr(self.s, base+1, 2, f"Msgs: {self.msg_count}   Rate: {rate:4.1f} Hz")
        safe_addstr(self.s, base+2, 2, f"System UTC:  {sys_now}")
        if self.msg:
            safe_addstr(self.s, base+3, 2, f"ROS UTC:     {ros_stamp_to_iso(self.msg.header.stamp)}")
            st_map = {-1:"NO FIX", 0:"FIX", 1:"SBAS", 2:"GBAS"}
            st = st_map.get(self.msg.status.status, "n/a")
            safe_addstr(self.s, base+5, 2, f"Status: {st}")
            safe_addstr(self.s, base+6, 2, f"Lat : {self.msg.latitude: .8f}")
            safe_addstr(self.s, base+7, 2, f"Lon : {self.msg.longitude: .8f}")
            safe_addstr(self.s, base+8, 2, f"Alt : {self.msg.altitude: .3f} m")
            age = f"{(time.time()-self.last_rx):.1f}s" if self.last_rx else "n/a"
            safe_addstr(self.s, base+9, 2, f"Age : {age}")
        else:
            safe_addstr(self.s, base+5, 2, "Waiting for /fix …")

        # countdown after first FIX
        if self.fix_seen and self.fix_start:
            remaining = max(0, self.countdown - int(time.time() - self.fix_start))
            safe_addstr(self.s, base+11, 2, f"Auto-exit in: {remaining:2d}s")
            if remaining <= 0:
                raise SystemExit
                # self.quit_clean(0)

        safe_addstr(self.s, base+13, 2, "Press  Ctrl-C to quit")
        self.s.refresh()

def main(stdscr):
    rclpy.init()
    node = CGPS(stdscr)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass

if __name__ == "__main__":
    curses.wrapper(main)
