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
        # Diagnostics
        self.last_diagnostic_check = 0
        self.diagnostic_info = None
        self.diagnostic_interval = 2.0  # Check every 2 seconds

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

    def diagnose_no_messages(self):
        """Diagnose why we're not receiving messages"""
        try:
            # Get list of all topics
            topic_names_and_types = self.get_topic_names_and_types()
            all_topics = [name for name, _ in topic_names_and_types]
            
            # Check if our specific topic exists
            topic_exists = FIX_TOPIC in all_topics
            
            # Get list of all nodes
            node_names = self.get_node_names()
            
            # Check if ublox node is running
            ublox_nodes = [n for n in node_names if 'ublox' in n.lower()]
            
            # Find similar topics (NavSatFix type)
            navsat_topics = [name for name, types in topic_names_and_types 
                           if any('NavSatFix' in t for t in types)]
            
            return {
                'topic_exists': topic_exists,
                'all_topics_count': len(all_topics),
                'node_count': len(node_names),
                'ublox_nodes': ublox_nodes,
                'navsat_topics': navsat_topics,
            }
        except Exception as e:
            return {'error': str(e)}

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

        # ROS UTC
        current_line = base + 3
        if self.msg:
            safe_addstr(self.s, current_line, 2, f"ROS UTC:     {ros_stamp_to_iso(self.msg.header.stamp)}")
            current_line += 1
        else:
            # Run diagnostics periodically when no messages
            now = time.time()
            if now - self.last_diagnostic_check > self.diagnostic_interval:
                self.diagnostic_info = self.diagnose_no_messages()
                self.last_diagnostic_check = now
            
            safe_addstr(self.s, current_line, 2, f"ROS UTC:     Not available (no messages from {FIX_TOPIC})")
            current_line += 1
            
            # Display diagnostic information
            if self.diagnostic_info and 'error' not in self.diagnostic_info:
                diag = self.diagnostic_info
                
                if not diag['topic_exists']:
                    safe_addstr(self.s, current_line, 2, f"⚠ Topic '{FIX_TOPIC}' does not exist!")
                    current_line += 1
                    
                    if diag['navsat_topics']:
                        safe_addstr(self.s, current_line, 2, f"  Found NavSatFix topics: {', '.join(diag['navsat_topics'][:2])}")
                        current_line += 1
                    else:
                        safe_addstr(self.s, current_line, 2, f"  No NavSatFix topics found")
                        current_line += 1
                else:
                    safe_addstr(self.s, current_line, 2, f"✓ Topic exists but no data published")
                    current_line += 1
                
                # Node information
                if not diag['ublox_nodes']:
                    safe_addstr(self.s, current_line, 2, f"⚠ No ublox nodes detected (found {diag['node_count']} total nodes)")
                    current_line += 1
                    safe_addstr(self.s, current_line, 2, f"  → Check: Is ublox_gps_node running?")
                    current_line += 1
                else:
                    safe_addstr(self.s, current_line, 2, f"✓ Found ublox node(s): {', '.join(diag['ublox_nodes'])}")
                    current_line += 1
                
                # General diagnosis
                if diag['node_count'] == 1:  # Only this node
                    safe_addstr(self.s, current_line, 2, f"⚠ Only this node running - check ROS/Zenoh middleware")
                    current_line += 1
            elif self.diagnostic_info and 'error' in self.diagnostic_info:
                safe_addstr(self.s, current_line, 2, f"⚠ Diagnostic error: {self.diagnostic_info['error'][:50]}")
                current_line += 1
            else:
                safe_addstr(self.s, current_line, 2, f"  Likely: GPS node not running or topic mismatch")
                current_line += 1
        
        # Add spacing before GPS data
        current_line += 1

        # GPS Data
        if self.msg:
            st_map = {-1:"NO FIX", 0:"FIX", 1:"SBAS", 2:"GBAS"}
            st = st_map.get(self.msg.status.status, "n/a")
            safe_addstr(self.s, current_line, 2, f"Status: {st}")
            safe_addstr(self.s, current_line+1, 2, f"Lat : {self.msg.latitude: .8f}")
            safe_addstr(self.s, current_line+2, 2, f"Lon : {self.msg.longitude: .8f}")
            safe_addstr(self.s, current_line+3, 2, f"Alt : {self.msg.altitude: .3f} m")
            age = f"{(time.time()-self.last_rx):.1f}s" if self.last_rx else "n/a"
            safe_addstr(self.s, current_line+4, 2, f"Age : {age}")
            current_line += 5
        else:
            safe_addstr(self.s, current_line, 2, "Waiting for /fix …")
            current_line += 1
        
        # Add spacing before countdown
        current_line += 1

        # countdown after first FIX
        if self.fix_seen and self.fix_start:
            remaining = max(0, self.countdown - int(time.time() - self.fix_start))
            safe_addstr(self.s, current_line, 2, f"Auto-exit in: {remaining:2d}s")
            if remaining <= 0:
                raise SystemExit
                # self.quit_clean(0)
            current_line += 1
        
        # Add spacing before exit prompt
        current_line += 1
        safe_addstr(self.s, current_line, 2, "Press  Ctrl-C to quit")
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
