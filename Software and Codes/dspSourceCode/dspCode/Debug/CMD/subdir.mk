################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../CMD/DSP2833x_Headers_nonBIOS.cmd \
../CMD/F28334.cmd 


# Each subdirectory must supply rules for building sources it contributes
CMD/DSP2833x_Headers_nonBIOS.out: ../CMD/DSP2833x_Headers_nonBIOS.cmd $(GEN_CMDS)
	@echo 'Building file: $<'
	@echo 'Invoking: Linker'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --diag_warning=225 --large_memory_model --float_support=fpu32 -z -m"../CMD/Debug/AD.map" --stack_size=1000 --heap_size=1000 --warn_sections -i"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/lib" -i"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" -i"D:/gg201409/Good-ggSpmV2Module/dspCode" --reread_libs --entry_point=code_start --rom_model -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMD/F28334.out: ../CMD/F28334.cmd $(GEN_CMDS)
	@echo 'Building file: $<'
	@echo 'Invoking: Linker'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --diag_warning=225 --large_memory_model --float_support=fpu32 -z -m"../CMD/Debug/AD.map" --stack_size=1000 --heap_size=1000 --warn_sections -i"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/lib" -i"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" -i"D:/gg201409/Good-ggSpmV2Module/dspCode" --reread_libs --entry_point=code_start --rom_model -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


