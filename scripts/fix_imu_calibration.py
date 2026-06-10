#!/usr/bin/env python3
"""
One-shot EEPROM patch for OAK-D devices whose IMU calibration is not
readable by BasaltVIO ("IMU calibration data is not available on device yet").

Root cause: the EEPROM calibration version is 6 (BW1098OBC board profile),
which does not include IMU calibration fields.  Bumping to version 7 with the
correct OAK-D board identity makes the IMU calibration block visible.

Run ONCE per device — changes are written to the device EEPROM:

  docker compose --profile basalt_bridge run --rm basalt_bridge \
    bash -c "DEPTHAI_ALLOW_FACTORY_FLASHING=868632271 python3 /scripts/fix_imu_calibration.py"

Or on the host (with depthai installed):

  DEPTHAI_ALLOW_FACTORY_FLASHING=868632271 python3 scripts/fix_imu_calibration.py
"""

import json
import os
import sys
import tempfile
from pathlib import Path

import depthai as dai

ALLOW_FLASH_ENV = "DEPTHAI_ALLOW_FACTORY_FLASHING"
ALLOW_FLASH_VAL = "868632271"
BACKUP_FILE = Path(__file__).parent / "depthai_calib_backup.json"


def main() -> None:
    if os.environ.get(ALLOW_FLASH_ENV) != ALLOW_FLASH_VAL:
        print(
            f"ERROR: environment variable {ALLOW_FLASH_ENV}={ALLOW_FLASH_VAL} "
            "must be set before running this script.",
            file=sys.stderr,
        )
        sys.exit(1)

    device = dai.Device(dai.UsbSpeed.HIGH)
    mx_id = device.getMxId()
    print(f"Connected to device: {mx_id}")

    # Back up current calibration before touching anything
    cal = device.readCalibration()
    cal.eepromToJsonFile(str(BACKUP_FILE))
    print(f"Backup written to: {BACKUP_FILE}")

    data = json.loads(BACKUP_FILE.read_text())

    print(f"  boardName (before): {data.get('boardName')!r}")
    print(f"  boardRev  (before): {data.get('boardRev')!r}")
    print(f"  version   (before): {data.get('version')!r}")

    changed = False

    if data.get("boardName") != "OAK-D":
        data["boardName"] = "OAK-D"
        changed = True

    if data.get("boardRev") != "R1M0E1":
        data["boardRev"] = "R1M0E1"
        changed = True

    if int(data.get("version", 0)) < 7:
        data["version"] = 7
        changed = True

    if not changed:
        print("Calibration is already correct — nothing to do.")
        device.close()
        return

    print(f"  boardName  (after): {data['boardName']!r}")
    print(f"  boardRev   (after): {data['boardRev']!r}")
    print(f"  version    (after): {data['version']!r}")

    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        json.dump(data, f)
        patched_path = f.name

    try:
        patched_cal = dai.CalibrationHandler(patched_path)
        device.flashCalibration(patched_cal)
        print(f"Calibration flashed successfully to device {mx_id}.")
    except Exception as ex:
        print(f"Failed flashing calibration: {ex}", file=sys.stderr)
        sys.exit(1)
    finally:
        os.unlink(patched_path)
        device.close()


if __name__ == "__main__":
    main()
