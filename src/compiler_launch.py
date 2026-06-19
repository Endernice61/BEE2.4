"""Launches the correct compiler."""

# ruff: noqa: E402  # Ignore import order, we want to set up logging as early as possible.
import os
import sys


if hasattr(sys, 'frozen'):
    app_name = os.path.basename(sys.executable).casefold()
    # On Linux, we're in bin/linux32/, not bin/. So everything else works as expected,
    # move back.
    folder = os.path.basename(os.getcwd())
    if folder.casefold() == 'linux32':
        os.chdir(os.path.dirname(os.getcwd()))
else:
    # Sourcecode-launch - check first sys arg.
    app_name = sys.argv.pop(1).casefold()

if 'original' in app_name:
    sys.exit('Original compilers were replaced! Please verify game files in Steam!')

if not app_name.startswith(('vbsp', 'vrad')):
    sys.exit(f'Invalid application name: "{app_name}"! Filename must start with "vbsp" or "vrad" to set application behavior.')

# Remove the latter half of the app_name after the initial valid application behavior checks have been done.
app_name = app_name.split(".")[0]
app_name = app_name.split("_")[0]

if app_name == 'vrad' and '--errorserver' in sys.argv:
    app_name = 'error_server'

from srctools.logger import init_logging
LOGGER = init_logging(f'bee2/{app_name}.log')
LOGGER.info('Arguments: {}', sys.argv)

import utils

LOGGER.info('Running "{}", version {}:', app_name, utils.BEE_VERSION)

if app_name == 'vbsp':
    import vbsp
    func = vbsp.main
elif app_name == 'vrad':
    import vrad
    func = vrad.main
elif app_name == 'error_server':
    import error_server
    func = error_server.main
else:
    raise AssertionError(app_name)

from trio_debug import Tracer
tracer = Tracer() if utils.CODE_DEV_MODE else None

import trio
trio.run(
    func, sys.argv,
    strict_exception_groups=True,  # Opt into 3.11-style semantics.
    instruments=[tracer] if tracer is not None else [],
)

if tracer is not None:
    tracer.display_slow()
