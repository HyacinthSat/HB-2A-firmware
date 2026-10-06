"""Ground side of CAPTR/REQF -> HSPAR -> HSCON/HSTST -> CFRM."""
from dataclasses import dataclass
import math

from wire import ACTUAL_RATES, DEPTHS, RATES, coded_length


@dataclass(frozen=True)
class Profile:
    rate: int
    code: int = 1
    depth: int = 1

    def fields(self):
        return [str(self.rate), str(self.code), str(self.depth)]

    def probe_seconds(self):
        return (128 + coded_length(32 + 2040 * self.depth, self.code)) / ACTUAL_RATES[self.rate]


class Negotiator:
    def __init__(self, send, log, threshold=3.5, max_rate=5, code=1, depth=1):
        if not math.isfinite(threshold) or not 3.0 <= threshold <= 3.5:
            raise ValueError("Es/N0 threshold must be between 3.0 and 3.5 dB")
        if max_rate not in range(6) or code not in range(4) or depth not in DEPTHS:
            raise ValueError("Invalid profile")
        self.send, self.log = send, log
        self.threshold, self.max_rate, self.code, self.depth = threshold, max_rate, code, depth
        self.state, self.profile, self.deadline = "idle", None, 0.0
        self.candidates = []
        self.request = None
        self.probe_after = math.inf
        self.retries = 0
        self.last_command = None
        self.last_success = None
        self.cached_probe = None
        self.image_complete = False
        self.rate_status = {}

    def transmit(self, command, fields):
        self.last_command = (command, list(fields))
        self.send(command, fields)

    def transmitted(self, command, now):
        if self.last_command and command == self.last_command[0]:
            if self.state in ("support", "connection"):
                # ACK + HSSUP are two 0.9 s downlink frames; allow their airtime
                # in addition to the flowchart's 2.5 s reply budget.
                self.deadline = now + (4.5 if self.state == "support" else 3.5)

    def start(self, command, fields, now):
        if self.state != "idle":
            raise ValueError("A negotiation or transfer is already active")
        if command not in ("CAPTR", "REQF"):
            raise ValueError("Expected CAPTR or REQF")
        self.request = command
        self.rate_status = {i: "Awaiting support" for i in range(len(RATES))}
        self.state, self.deadline = "support", now + 14
        self.retries = 0
        self.transmit(command, fields)
        self.log("Waiting for HSSUP")

    def cancel(self):
        if self.state != "idle":
            self.send("CANCEL", [])
        self.reset("Cancelled")

    def reset(self, reason):
        self.state, self.profile, self.candidates = "idle", None, []
        self.probe_after = math.inf
        self.image_complete = False
        self.log(reason)

    def next_candidate(self, now, first=False):
        if not self.candidates:
            self.send("CANCEL", [])
            self.reset("No tested rate reached the Es/N0 threshold")
            return
        self.profile = self.candidates.pop(0)
        self.rate_status[self.profile.rate] = "Awaiting HSCON"
        self.state, self.deadline, self.probe_after = "connection", now + 4, math.inf
        self.cached_probe = None
        self.retries = 0
        self.transmit("HSPAR" if first else "CHPRAM", self.profile.fields())
        self.log("Testing %d bit/s (profile %s)" %
                 (RATES[self.profile.rate], ",".join(self.profile.fields())))

    def text(self, text, rate, code, depth, esn0, now, received_at):
        fields = text.split(",")
        if fields[0] == "HSSUP" and self.state == "support" and rate == 0 and code == 1:
            try:
                if len(fields) != 4:
                    return
                rates, codes, depths = (int(value, 16) for value in fields[1:])
            except ValueError:
                return
            if not codes & (1 << self.code):
                self.code = next((c for c in (1, 2, 3, 0) if codes & (1 << c)), -1)
            if not depths & (1 << DEPTHS.index(self.depth)):
                self.depth = next((d for d in (4, 8, 5, 3, 2, 1) if depths & (1 << DEPTHS.index(d))), 0)
            if self.code < 0 or self.depth == 0:
                self.cancel()
                self.log("Requested code/interleave not advertised")
                return
            self.candidates = [Profile(i, self.code, self.depth) for i in
                               reversed(range(self.max_rate + 1)) if rates & (1 << i)]
            supported = {p.rate for p in self.candidates}
            self.rate_status = {i: "Not tested yet" if i in supported else "Not offered"
                                for i in range(len(RATES))}
            self.next_candidate(now, True)
        elif fields[0] == "HSCON" and self.state in ("connection", "probe"):
            if rate != 0 or code != 1 or fields[1:] != self.profile.fields():
                return
            if self.state == "connection":
                self.state = "probe"
                self.rate_status[self.profile.rate] = "Testing"
                self.probe_after = received_at
                self.deadline = now + self.profile.probe_seconds() + 1.3
                if self.cached_probe:
                    cached, self.cached_probe = self.cached_probe, None
                    self.text(*cached[:5], now, cached[6])
        elif fields[0] == "HSTST" and self.state == "connection":
            # Different rate branches can deliver in a different wall-clock
            # order. Only use this after a matching earlier-airtime HSCON.
            if fields[1:] == self.profile.fields():
                self.cached_probe = (text, rate, code, depth, esn0, now, received_at)
        elif fields[0] == "HSTST" and self.state == "probe":
            if fields[1:] != self.profile.fields() or (rate, code, depth) != (
                    self.profile.rate, self.profile.code, self.profile.depth):
                return
            if received_at <= self.probe_after or now > self.deadline:
                return
            if esn0 is None or not math.isfinite(esn0):
                self.rate_status[self.profile.rate] = "Es/N0 unavailable"
                self.log("Probe decoded; Es/N0 unavailable, no confirmation")
                return
            self.log("Probe Es/N0 %.2f dB; threshold %.2f dB" % (esn0, self.threshold))
            if esn0 >= self.threshold:
                self.state, self.deadline = "transfer", now + 60
                self.last_success = self.profile
                self.rate_status[self.profile.rate] = "Selected"
                self.transmit("CFRM", [])
                self.log("CFRM sent; waiting for image data")
            else:
                self.rate_status[self.profile.rate] = "Below threshold"
                self.next_candidate(now)
        elif fields[0] == "HSCON" and self.state == "transfer" and rate == 0 and code == 1:
            if fields[1:] == self.profile.fields() and self.retries < 2:
                # A repeated HSCON is followed by another RF probe. Wait for
                # its end before keying CFRM; the balloon is half duplex.
                self.retries += 1
                self.state = "probe"
                self.rate_status[self.profile.rate] = "Testing again"
                self.probe_after = received_at
                self.deadline = now + self.profile.probe_seconds() + 1.3
        elif fields[0] == "HSREJ" and self.state != "idle" and rate == 0 and code == 1:
            if text == "HSREJ,PARAM" and self.state in ("connection", "probe"):
                self.rate_status[self.profile.rate] = "Rejected"
                self.next_candidate(now)
            elif len(fields) >= 2 and fields[1] == "TRANSFER":
                self.reset("Transfer rejected" +
                           (" (result %s)" % ",".join(fields[2:]) if len(fields) > 2 else ""))
            else:
                self.reset(text)
        elif text == "HSDONE" and self.state == "transfer" and rate == 0 and code == 1:
            self.reset("Business completed")
        elif text == "ACK,CANCEL" and rate == 0 and code == 1:
            self.reset(text)

    def image(self, eoi, now):
        if self.state == "transfer":
            self.deadline = now + 30
            if eoi:
                self.log("Image EOI received; waiting for HSDONE")
                self.deadline = now + 5
                self.image_complete = True

    def tick(self, now):
        if self.state == "idle" or now < self.deadline:
            return
        if self.state == "probe":
            self.rate_status[self.profile.rate] = "No qualified probe"
            self.log("%d bit/s: no qualified probe before timeout; trying the next rate" %
                     RATES[self.profile.rate])
            self.next_candidate(now)
        elif self.state in ("support", "connection") and self.retries < 2:
            self.retries += 1
            self.transmit(*self.last_command)
            self.deadline = now + 4.5
            self.log("Retry %d/3" % (self.retries + 1))
        else:
            if self.state == "transfer" and self.image_complete:
                # The JPEG is already complete. Older flight firmware can
                # lose HSDONE while switching back to the robust channel.
                self.reset("Image complete; HSDONE missing")
                return
            # A missing HSCON does not prove the balloon entered WAIT_CONFIRM.
            # Avoid blindly retrying non-idempotent CAPTR/CFRM commands.
            self.cancel()
            self.log("Session timeout")


class BusinessLoop:
    """Continuous jobs; one queued REQF is consumed once, then CAPTR resumes."""
    def __init__(self, send, log, parameters, threshold=3.5):
        self.send, self.log, self.parameters, self.threshold = send, log, parameters, threshold
        self.session = Negotiator(send, log, threshold, depth=4)
        self.enabled, self.state, self.deadline = False, "stopped", 0
        self.number, self.retries = 0, 0
        self.last_control = 0
        self.parameter_error = None
        self.pending_request = None

    def start(self, now):
        self.enabled, self.state, self.retries = True, "enter", 0
        self.send("EXPR", [])
        self.deadline = now + 4.5
        self.log("Automatic link acquisition")

    def stop(self):
        self.enabled = False
        self.pending_request = None
        # Stop must also work after the transmitter has disarmed itself on an
        # I/O fault. The balloon's existing handshake timeout releases it.
        self.session.reset("Automatic stopped")
        self.state = "stopped"

    def next_business(self, now):
        try:
            command, fields = self.pending_request or self.parameters()
        except ValueError as exc:
            message = str(exc)
            if message != self.parameter_error:
                self.log("Invalid business settings: %s; edit settings to resume automatically" % message)
            self.parameter_error = message
            self.state, self.deadline = "parameters", now + 1
            return
        self.pending_request = None
        self.parameter_error = None
        if self.state == "parameters":
            # Editing may take longer than the balloon's EXPR lease. Renew
            # before sending the validated request, including one-shot REQF.
            self.pending_request = (command, fields)
            self.log("Business settings valid; reacquiring link")
            self.start(now)
            return
        self.number += 1
        self.log("Business %d: %s %s" % (self.number, command, " ".join(fields)))
        self.session.start(command, fields, now)
        self.state = "business"

    def transmitted(self, command, now):
        self.session.transmitted(command, now)
        if command == "EXPR" and self.state == "enter":
            self.deadline = now + 3.5

    def group(self, group, now):
        if not self.enabled:
            return
        for frame in group["frames"]:
            if frame["type"] == "text":
                text = frame["text"]
                if group["rate"] == 0 and group["code"] == 1:
                    self.last_control = now
                    if self.state == "enter" and text == "ACK,EXPR":
                        self.state, self.deadline = "ready", now + 1.0
                self.session.text(text, group["rate"], group["code"], group["depth"],
                                  group["esn0"], now, group["received_at"])
            elif self.session.state == "transfer" and self.session.profile is not None:
                if (group["rate"], group["code"]) == (self.session.profile.rate, self.session.profile.code):
                    self.session.image(frame["eoi"], now)

    def tick(self, now):
        if not self.enabled:
            return
        self.session.tick(now)
        if self.state == "business" and self.session.state == "idle":
            self.state, self.deadline = "renew", now + 1
        if now < self.deadline:
            return
        if self.state in ("ready", "parameters"):
            self.next_business(now)
        elif self.state == "renew":
            self.start(now)
        elif self.state == "enter":
            self.retries += 1
            if self.retries < 3:
                self.send("EXPR", [])
                self.deadline = now + 4.5
            else:
                self.log("Link acquisition failed three times; retry in 15 seconds")
                self.state, self.deadline = "renew", now + 15
