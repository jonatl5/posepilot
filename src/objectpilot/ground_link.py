"""Ground-only Tello SDK connection: video and range queries, no flight commands."""
import socket
import threading
import time
from .core import CustomRangeParser, RangeSample, parse_ext_tof


class GroundLink:
    # Read-only identity/status queries: any non-error reply is their answer.
    READ_ONLY = frozenset(("sdk?", "sn?", "hardware?", "battery?", "wifi?", "wifiversion?", "EXT version?"))
    ALLOWED = frozenset(("command", "streamon", "streamoff", "EXT tof?", "EXT oprange?")) | READ_ONLY

    def __init__(self, host="192.168.10.1", local_port=8889, config=None, remote_port=8889):
        self.host = socket.gethostbyname(host)
        self.remote_port = remote_port
        self.ignored_lines = 0
        self.last_flushed, self.last_unmatched = [], []
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind(("", local_port))
        except BaseException:
            self.socket.close()
            raise
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.parser = CustomRangeParser(config)
        self.range_sample = None
        self.worker = None
        self.streaming = False

    def request(self, command, timeout=.5):
        if command not in self.ALLOWED:
            raise ValueError("GroundLink permits only video setup and range/version queries")
        with self.lock:
            # Late replies to earlier queries are discarded here but kept for diagnosis.
            self.last_flushed, self.last_unmatched = [], []
            self.socket.settimeout(.001)
            try:
                while True:
                    raw, _ = self.socket.recvfrom(2048)
                    self.last_flushed.append(raw.decode("ascii", errors="replace").strip())
            except (socket.timeout, ConnectionResetError):
                pass
            start = time.monotonic()
            self.socket.sendto(command.encode("ascii"), (self.host, self.remote_port))
            while time.monotonic()-start < timeout:
                self.socket.settimeout(max(.001, timeout-(time.monotonic()-start)))
                try:
                    raw, peer = self.socket.recvfrom(2048)
                except socket.timeout:
                    break
                if peer[0] != self.host:
                    continue
                reply = raw.decode("ascii", errors="replace").strip()
                lower = reply.lower()
                if lower.startswith("unknown command"):
                    # The TT expansion controller forwards replies to its own traffic, e.g.
                    # "unknown command: keepalive"; only a rejection of this command is an error.
                    named = reply.split(":", 1)[1].strip() if ":" in reply else ""
                    if named and named != command:
                        self.ignored_lines += 1
                        continue
                    raise RuntimeError(reply)
                if lower.startswith("error"):
                    raise RuntimeError(reply)
                prefix = ("oprange " if command == "EXT oprange?" else
                          "tof " if command == "EXT tof?" else None)
                if (prefix and reply.startswith(prefix)) or (prefix is None and
                        (reply == "ok" or command in self.READ_ONLY)):
                    return reply, start, time.monotonic()
                self.last_unmatched.append(reply)
            raise TimeoutError(f"No matching reply to {command}")

    def listen(self, command, seconds):
        """Diagnostic: send one allowed command and return every line received for `seconds`."""
        if command not in self.ALLOWED:
            raise ValueError("GroundLink permits only video setup and range/version queries")
        lines = []
        with self.lock:
            start = time.monotonic()
            self.socket.sendto(command.encode("ascii"), (self.host, self.remote_port))
            while time.monotonic()-start < seconds:
                self.socket.settimeout(max(.001, seconds-(time.monotonic()-start)))
                try:
                    raw, peer = self.socket.recvfrom(2048)
                except (socket.timeout, ConnectionResetError):
                    continue
                lines.append((round(1000*(time.monotonic()-start)), peer[0],
                              raw.decode("ascii", errors="replace").strip()))
        return lines

    def connect(self, range_mode="none", video=True):
        if range_mode not in ("none", "stock", "custom"):
            raise ValueError("Unknown range mode")
        self.request("command", 5.)
        if video:
            self.request("streamon", 5.)
            self.streaming = True
        if range_mode != "none":
            self.worker = threading.Thread(target=self._poll, args=(range_mode,), daemon=True)
            self.worker.start()

    def _poll(self, mode):
        while not self.stop.is_set():
            try:
                reply, start, end = self.request("EXT oprange?" if mode == "custom" else "EXT tof?")
                self.range_sample = (self.parser.parse(reply, start, end) if mode == "custom"
                                     else parse_ext_tof(reply, start))
            except (OSError, TimeoutError, RuntimeError):
                self.range_sample = RangeSample(None, time.monotonic(), "forward_custom", "query_failed")
            self.stop.wait(.2)

    def close(self):
        self.stop.set()
        if self.worker:
            self.worker.join(timeout=1.)
        try:
            if self.streaming:
                self.request("streamoff")
        except (OSError, TimeoutError, RuntimeError):
            pass
        finally:
            self.socket.close()
