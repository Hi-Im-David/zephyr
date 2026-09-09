# Copyright (c) 2026 Nordic Semiconductor ASA.
#
# SPDX-License-Identifier: Apache-2.0

'''Runner for flashing nRF7120 (Wezen) devices that are in the Empty LCS.'''

from pathlib import Path

from runners.nrfutil import NrfUtilBinaryRunner

# On Wezen 1.0 silicon in the Empty life cycle state, the MRAM controller is
# only left in a writable state after two soft resets have been performed on a
# freshly erased device. Programming has to be done with the erase disabled,
# since any erase issued by nrfutil as part of the program operation would put
# the device back into the state the soft resets are meant to leave. This is
# fixed in Wezen 1.1.
SOFT_RESETS_AFTER_ERASE = 2


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
