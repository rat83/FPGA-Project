set_global_assignment -entity "Computer_System" -library "lib_Computer_System" -name IP_TOOL_NAME "Qsys"
set_global_assignment -entity "Computer_System" -library "lib_Computer_System" -name IP_TOOL_VERSION "18.1"
set_global_assignment -entity "Computer_System" -library "lib_Computer_System" -name IP_TOOL_ENV "Qsys"
set_global_assignment -library "lib_Computer_System" -name SPD_FILE [file join $::quartus(sip_path) "../Computer_System.spd"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "../../Computer_System.qsys"]

set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "Computer_System.v"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/altera_reset_controller.v"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/altera_reset_synchronizer.v"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/altera_reset_controller.sdc"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/Computer_System_irq_mapper.sv"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/Computer_System_vga_pio.vo"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/Computer_System_m10k_pll.vo"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/Computer_System_System_PLL.v"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/altera_up_avalon_reset_from_locked_signal.v"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/Computer_System_System_PLL_sys_pll.vo"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/Computer_System_ARM_A9_HPS.v"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/Computer_System_ARM_A9_HPS_hps_io.v"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/verbosity_pkg.sv"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/avalon_utilities_pkg.sv"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/avalon_mm_pkg.sv"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/altera_avalon_mm_slave_bfm.sv"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/altera_avalon_interrupt_sink.sv"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/altera_avalon_clock_source.sv"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/altera_avalon_reset_source.sv"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/Computer_System_ARM_A9_HPS_hps_io_border_memory.sv"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/Computer_System_ARM_A9_HPS_hps_io_border_hps_io.sv"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/Computer_System_ARM_A9_HPS_hps_io_border.sv"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/questa_mvc_svapi.svh"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/mgc_common_axi.sv"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/mgc_axi_master.sv"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/mgc_axi_slave.sv"]
set_global_assignment -library "lib_Computer_System" -name MISC_FILE [file join $::quartus(sip_path) "submodules/Computer_System_ARM_A9_HPS_fpga_interfaces.sv"]
