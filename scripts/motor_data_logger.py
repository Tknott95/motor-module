import csv
import time
from pathlib import Path


class MotorDataLogger:
    def __init__(self, filename: str):
        self.filename = filename

        self.file = open(filename, "w", newline="")
        self.writer = csv.writer(self.file)

        self.writer.writerow(
            [
                "timestamp",
                "cmd_pos",
                "cmd_vel",
                "cmd_tau",
                "act_pos",
                "act_vel",
                "act_current",
                "temperature",
            ]
        )

    def log(
        self,
        cmd_pos,
        cmd_vel,
        cmd_tau,
        act_pos,
        act_vel,
        act_current,
        temperature,
    ):
        self.writer.writerow(
            [
                time.time(),
                cmd_pos,
                cmd_vel,
                cmd_tau,
                act_pos,
                act_vel,
                act_current,
                temperature,
            ]
        )

        self.file.flush()

    def close(self):
        self.file.close()
