# Copyright (c) 2017 Linaro Limited.
# Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
# Copyright (c) 2026 Muhammed Asif P
# SPDX-License-Identifier: Apache-2.0

import argparse
import os
import shlex
from pathlib import Path

import yaml
from runners.core import RunnerCaps, RunnerConfig, ZephyrBinaryRunner


class MPLABIPEBinaryRunner(ZephyrBinaryRunner):
    """Runner for Flashing Application using Microchip MPLAB IPE"""

    def __init__(
        self,
        cfg: RunnerConfig,
        tool: str,
        part: str,
        dev_id: str | None = None,
        erase: bool = False,
        verify: bool = False,
        merged_hex: bool = False,
    ):
        super().__init__(cfg)
        self.tool = tool
        self.part = part
        self.dev_id = dev_id
        self.erase = erase
        self.verify = verify
        self.merged_hex = merged_hex

    @classmethod
    def name(cls):
        return "mplab_ipe"

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={"flash"}, erase=True, dev_id=True)

    @classmethod
    def do_add_parser(cls, parser):
        parser.add_argument(
            "--tool",
            required=True,
            help="Programming probe model, e.g. PKOB4. Passed unchanged to the MLAB IPE",
        )
        parser.add_argument(
            "--part", required=True, help="Target device name, e.g. 32CX1025SG61128"
        )
        parser.add_argument(
            "--verify", action=argparse.BooleanOptionalAction, help="Verify after programming"
        )
        parser.add_argument(
            "--merged-hex",
            action=argparse.BooleanOptionalAction,
            help="In sysbuild, flash the merged hex of all images in a single run (when "
            "flashing the last domain) instead of each image separately. Needed because "
            "every IPE run erases the whole chip.",
        )

    @classmethod
    def do_create(cls, cfg: RunnerConfig, args):
        return MPLABIPEBinaryRunner(
            cfg,
            tool=args.tool,  # probe type
            part=args.part,  # part number of the SoC
            dev_id=args.dev_id,
            erase=args.erase,
            verify=args.verify,
            merged_hex=args.merged_hex,
        )

    def do_run(self, command: str, **kwargs):
        if command == "flash":
            self.flash()
        else:
            raise ValueError(f"Unsupported command: {command}")

    def sysbuild_merged_hex(self):
        """Return the sysbuild merged hex if it should be flashed now, else None.

        The merged hex is flashed when processing the last domain in the flash order; the
        earlier domains are skipped. Falls back to the image's own hex if there is none.
        """
        build_dir = Path(self.cfg.build_dir).resolve()
        top_dir = build_dir.parent
        domains_file = top_dir / 'domains.yaml'
        merged = sorted(top_dir.glob('merged_*.hex'))
        if not domains_file.is_file() or not merged:
            self.ensure_output('hex')
            return self.cfg.hex_file

        with open(domains_file) as f:
            domains = yaml.safe_load(f)
        order = domains.get('flash_order') or [d['name'] for d in domains['domains']]
        last = next(d['build_dir'] for d in domains['domains'] if d['name'] == order[-1])
        if Path(last).resolve() != build_dir:
            self.logger.info("Skipping %s: merged hex is flashed with the last domain", build_dir)
            return None
        if len(merged) > 1:
            raise RuntimeError(f"Multiple merged hex files found: {merged}")
        return str(merged[0])

    def flash(self):
        hex_file = self.cfg.hex_file
        if self.merged_hex:
            hex_file = self.sysbuild_merged_hex()
            if hex_file is None:
                return
        else:
            self.ensure_output('hex')

        exe = self.require("ipecmd.exe" if os.name == "nt" else "ipecmd.sh")

        cmd = [
            exe,
            f"-P{self.part}",  # part number of the SoC
            f"-F{hex_file}",
            "-M",  # Program all memories
            "-OL",  # Release from reset
            "-OK",  # Silent connect with the tool and hardware
        ]

        if self.dev_id:
            cmd.append(f"-TS{self.dev_id}")  # Select programmer by serial number
        elif self.tool is not None:
            cmd.append(f"-TP{self.tool}")  # Select programmer by name of the tool

        if self.erase:
            cmd.append("-E")

        if self.verify:
            cmd.append("-V")

        self.logger.info("Running: %s", " ".join(shlex.quote(p) for p in cmd))
        self.check_call(cmd)
