# SPDX-License-Identifier: Apache-2.0

# ST-Link V2 - giam SWD freq tranh loi "Unable to get core ID"
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=hw" "--tool-opt=freq=1800")
board_runner_args(openocd "--target=stm32f4x")

include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd-stm32.board.cmake)
