# Copyright (c) 2026 Nordic Semiconductor ASA.
#
# SPDX-License-Identifier: Apache-2.0

'''Runner for flashing nRF7120 (Wezen) devices that are in the Empty LCS.'''

import json
import subprocess
import sys
from pathlib import Path

from runners.nrfutil import NrfUtilBinaryRunner

# On Wezen 1.0 silicon in the Empty life cycle state, the MRAM controller is
# only left in a writable state after two soft resets have been performed on a
# freshly erased device. Programming has to be done with the erase disabled,
# since any erase issued by nrfutil as part of the program operation would put
# the device back into the state the soft resets are meant to leave. This is
# fixed in Wezen 1.1.
SOFT_RESETS_AFTER_ERASE = 2

# A Wezen device in the Empty LCS reads back a part number of 0x00000000, so
# automatic detection fails and the real part number has to be forced. nrfutil
# only honours --x-partno on direct operations: it is not recorded in a batch
# file, and 'batch-execute' ignores the option. Every operation is therefore run
# directly instead of being collected into a batch.
X_PARTNO = '0x2c'


class NrfUtilWezenBinaryRunner(NrfUtilBinaryRunner):
    '''Runner front-end for nrfutil with the Wezen Empty LCS workaround.'''

    @classmethod
    def name(cls):
        return 'wezen_empty_lcs'

    @classmethod
    def do_create(cls, cfg, args):
        return NrfUtilWezenBinaryRunner(cfg, args.nrf_family, args.softreset,
                                        args.pinreset, args.dev_id,
                                        erase=args.erase,
                                        erase_mode=args.erase_mode,
                                        ext_erase_mode=args.ext_erase_mode,
                                        reset=args.reset,
                                        tool_opt=args.tool_opt,
                                        force=args.force, recover=args.recover,
                                        ext_mem_config_file=args.ext_mem_config_file,
                                        dry_run=args.dry_run)

    def _exec(self, args, force=False):
        # 'list' is the only operation issued by this runner that rejects
        # --x-partno, since it enumerates probes rather than targeting a device.
        if args and args[0] != 'list':
            args = args + ['--x-partno', X_PARTNO]

        cmd = ['nrfutil', '--json', 'device'] + args
        self._log_cmd(cmd)

        if self.dry_run and not force:
            return {}

        jout_all = []
        err_code = None

        with subprocess.Popen(cmd, stdout=subprocess.PIPE) as p:
            for line in iter(p.stdout.readline, b''):
                # https://github.com/ndjson/ndjson-spec
                jout = json.loads(line.decode(sys.getdefaultencoding()))
                jout_all.append(jout)

                if jout['type'] == 'task_progress':
                    progress = jout['data']['progress']
                    if progress['progressPercentage'] == 0:
                        self.logger.info(progress['description'])
                elif jout['type'] == 'task_end' and jout['data']['error']:
                    err_code = jout['data']['error']['code']

        if p.returncode != 0:
            # nrfutil exits with 1 regardless of the failure, so report the
            # error code from the task instead. The base class inspects it to
            # detect e.g. a protected device or a failed verification.
            raise subprocess.CalledProcessError(err_code or p.returncode, cmd)

        return jout_all

    def _direct_cmd(self, op):
        _op = op['operation']
        op_type = _op['type']

        cmd = [op_type]

        if op_type == 'program':
            cmd += ['--firmware', _op['firmware']['file']]
            opts = _op['options']
            cli_opts = f"chip_erase_mode={opts['chip_erase_mode']}"
            if opts.get('ext_mem_erase_mode'):
                cli_opts += f",ext_mem_erase_mode={opts['ext_mem_erase_mode']}"
            if opts.get('verify'):
                cli_opts += f",verify={opts['verify']}"
            cmd += ['--options', cli_opts]
        elif op_type == 'reset':
            cmd += ['--reset-kind', _op['kind']]
        elif op_type == 'erase':
            cmd.append(f'--{_op["kind"]}')
        elif op_type == 'x-provision-keys':
            cmd += ['--key-file', _op['keyfile']]

        cmd += ['--core', op['core']] if op.get('core') else []
        cmd += ['--x-family', f'{self.family}']
        cmd += ['--serial-number', self._format_dev_ids()]
        return cmd

    def _exec_batch(self):
        ops, self._ops = self._ops, []
        self._op_id = 1

        precmd = []
        if self.ext_mem_config_file:
            precmd = ['--x-ext-mem-config-file', self.ext_mem_config_file]

        for op in ops:
            self._exec(precmd + self._direct_cmd(op))

    def _erase_and_soft_reset(self):
        # Each step is flushed separately so that it runs as its own nrfutil
        # invocation, matching the documented manual sequence.
        self.exec_op('erase', kind='all')
        self.flush(force=True)

        for _ in range(SOFT_RESETS_AFTER_ERASE):
            self.exec_op('reset', kind='RESET_SOFT')
            self.flush(force=True)

    def program_hex(self):
        if self.family != 'nrf71':
            raise RuntimeError(
                f'The {self.name()} runner only supports the nRF71 family, '
                f'but the build targets {self.family}.')

        if self.erase_mode == 'none' and not self.erase:
            self.logger.info('Erase disabled, skipping the soft reset sequence')
        else:
            if self.erase_mode == 'ranges':
                self.logger.warning(
                    '--erase-mode=ranges is not supported by this runner, '
                    'performing a full erase instead')
            self._erase_and_soft_reset()

        self.logger.info(f'Flashing file: {self.hex_}')
        self.op_program(self.hex_, 'ERASE_NONE', None, defer=True)

        for keyfile in ('prot_ram_inv_slots.json', 'keyfile.json'):
            path = Path(self.cfg.build_dir).parent / keyfile
            if path.exists():
                self.logger.info(f'Provisioning key file: {path}')
                self.exec_op('x-provision-keys', keyfile=str(path), defer=True)

        self.flush(force=True)
