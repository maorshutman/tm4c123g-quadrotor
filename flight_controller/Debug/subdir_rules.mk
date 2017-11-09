################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
buffer.obj: ../buffer.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156/examples/boards/ek-tm4c123gxl-boostxl-senshub" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156" --advice:power=all -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_TM4C123_RB1 --define=UART_BUFFERED --diag_wrap=off --diag_warning=225 --display_error_number --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="buffer.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

comp_dcm.obj: ../comp_dcm.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156/examples/boards/ek-tm4c123gxl-boostxl-senshub" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156" --advice:power=all -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_TM4C123_RB1 --define=UART_BUFFERED --diag_wrap=off --diag_warning=225 --display_error_number --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="comp_dcm.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

controller.obj: ../controller.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156/examples/boards/ek-tm4c123gxl-boostxl-senshub" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156" --advice:power=all -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_TM4C123_RB1 --define=UART_BUFFERED --diag_wrap=off --diag_warning=225 --display_error_number --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="controller.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

escpwm.obj: ../escpwm.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156/examples/boards/ek-tm4c123gxl-boostxl-senshub" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156" --advice:power=all -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_TM4C123_RB1 --define=UART_BUFFERED --diag_wrap=off --diag_warning=225 --display_error_number --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="escpwm.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

hc12.obj: ../hc12.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156/examples/boards/ek-tm4c123gxl-boostxl-senshub" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156" --advice:power=all -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_TM4C123_RB1 --define=UART_BUFFERED --diag_wrap=off --diag_warning=225 --display_error_number --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="hc12.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

main.obj: ../main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156/examples/boards/ek-tm4c123gxl-boostxl-senshub" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156" --advice:power=all -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_TM4C123_RB1 --define=UART_BUFFERED --diag_wrap=off --diag_warning=225 --display_error_number --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="main.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

mpu9150mod.obj: ../mpu9150mod.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156/examples/boards/ek-tm4c123gxl-boostxl-senshub" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156" --advice:power=all -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_TM4C123_RB1 --define=UART_BUFFERED --diag_wrap=off --diag_warning=225 --display_error_number --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="mpu9150mod.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

startup_ccs.obj: ../startup_ccs.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156/examples/boards/ek-tm4c123gxl-boostxl-senshub" --include_path="/Users/maorshutman/ti/TivaWare_C_Series-2.1.3.156" --advice:power=all -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_TM4C123_RB1 --define=UART_BUFFERED --diag_wrap=off --diag_warning=225 --display_error_number --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="startup_ccs.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


