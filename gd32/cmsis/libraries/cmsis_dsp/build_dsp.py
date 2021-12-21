from os.path import join, realpath
Import("env")

# global build environment is for all libraries and code
global_env = DefaultEnvironment()

board = env.BoardConfig()
board_mcu = board.get('build.cpu', "")

mcu_to_lib = {
    # ARMv7-M types
    "cortex-m3": "arm_cortexM3l_math",
    "cortex-m4": "arm_cortexM4l_math", # all GD32 Cortex-M4 devices have the FPU
    # ARMv8 types
    "cortex-m23": "arm_ARMv8MBLl_math", # e.g., E10x
    #"cortex-m33": "arm_ARMv8MMLlfsp_math" # E50x series has DSP and FPU
    "cortex-m33": "arm_ARMv8MMLldfsp_math" # E50x and W51x series has DSP and FPU
}

if board_mcu in mcu_to_lib.keys():   
    lib_file = mcu_to_lib[board_mcu]
    env.Append(
        LIBPATH=[realpath('lib')],
        LIBS=[ lib_file ]
    )
    print("Added CMSIS-DSP library for %s" % lib_file)
else:
    print("WARNING: Fitting CMSIS-DSP library was not found for %s.." % board_mcu)