"""USB command format. RF BCH/NRZI/CRC encoding happens on the board."""
import re

COUNTS = {"TRANPD": 5, "EXPR": 1, "CAPTR": 5, "REQF": 2,
          "HSPAR": 4, "CHPRAM": 4, "CFRM": 1, "CANCEL": 1}
CALL = re.compile(r"[A-Z0-9]{1,6}(?:-(?:[0-9]|1[0-5]))?\Z")


def command_data(command, fields):
    if command not in COUNTS or len(fields) != COUNTS[command] - 1:
        raise ValueError("Wrong command or field count")
    parts = [command, *fields]
    encoded = []
    for i, field in enumerate(parts):
        try:
            raw = field.encode("ascii")
        except UnicodeEncodeError as exc:
            raise ValueError(f"Field {i + 1}: ASCII characters only") from exc
        if len(raw) > 7 or any(c < 32 or c > 126 for c in raw):
            raise ValueError(f"Field {i + 1}: maximum 7 printable ASCII bytes")
        encoded.append(raw.ljust(7, b" "))
    if command == "TRANPD":
        for value in fields[:2]:
            if not CALL.fullmatch(value):
                raise ValueError("Invalid source/destination callsign (SSID 0..15)")
        if fields[2] != "HB-2A":
            raise ValueError("Relay path must be HB-2A")
        if not fields[3].strip():
            raise ValueError("Relay message is empty")
    elif command in ("HSPAR", "CHPRAM"):
        if any(not value.isdigit() or not 0 <= int(value) <= 255 for value in fields):
            raise ValueError("High speed parameters must be decimal bytes")
    elif command == "CAPTR":
        if fields[0] not in ("A", "B") or any(not value.isdigit() for value in fields[1:]):
            raise ValueError("CAPTR fields: camera A/B, width, height, quality")
        if not 1 <= int(fields[1]) <= 255 or not 1 <= int(fields[2]) <= 255 or not 0 <= int(fields[3]) <= 7:
            raise ValueError("CAPTR width/height 1..255 and quality 0..7")
    elif command == "REQF":
        if not fields[0].isdigit() or not 0 <= int(fields[0]) <= 255:
            raise ValueError("REQF file index must be 0..255")
    return b"".join(encoded)


def usb_command(command, fields, power=10):
    if isinstance(power, bool) or not isinstance(power, int) or not 2 <= power <= 17:
        raise ValueError("Power must be an integer from 2 to 17 dBm")
    return f"SEND {power} {command_data(command, fields).hex().upper()}\n".encode("ascii")
