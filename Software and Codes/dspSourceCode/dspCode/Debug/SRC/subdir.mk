################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SRC/DSP2833x_Adc.c \
../SRC/DSP2833x_CpuTimers.c \
../SRC/DSP2833x_DefaultIsr.c \
../SRC/DSP2833x_EPwm.c \
../SRC/DSP2833x_GlobalVariableDefs.c \
../SRC/DSP2833x_PieCtrl.c \
../SRC/DSP2833x_PieVect.c \
../SRC/DSP2833x_SysCtrl.c \
../SRC/DSP2833x_Xintf.c \
../SRC/main.c 

ASM_SRCS += \
../SRC/DSP2833x_ADC_cal.asm \
../SRC/DSP2833x_CSMPasswords.asm \
../SRC/DSP2833x_CodeStartBranch.asm \
../SRC/DSP2833x_usDelay.asm 

ASM_DEPS += \
./SRC/DSP2833x_ADC_cal.pp \
./SRC/DSP2833x_CSMPasswords.pp \
./SRC/DSP2833x_CodeStartBranch.pp \
./SRC/DSP2833x_usDelay.pp 

OBJS += \
./DSP2833x_ADC_cal.obj \
./DSP2833x_Adc.obj \
./DSP2833x_CSMPasswords.obj \
./DSP2833x_CodeStartBranch.obj \
./DSP2833x_CpuTimers.obj \
./DSP2833x_DefaultIsr.obj \
./DSP2833x_EPwm.obj \
./DSP2833x_GlobalVariableDefs.obj \
./DSP2833x_PieCtrl.obj \
./DSP2833x_PieVect.obj \
./DSP2833x_SysCtrl.obj \
./DSP2833x_Xintf.obj \
./DSP2833x_usDelay.obj \
./main.obj 

C_DEPS += \
./SRC/DSP2833x_Adc.pp \
./SRC/DSP2833x_CpuTimers.pp \
./SRC/DSP2833x_DefaultIsr.pp \
./SRC/DSP2833x_EPwm.pp \
./SRC/DSP2833x_GlobalVariableDefs.pp \
./SRC/DSP2833x_PieCtrl.pp \
./SRC/DSP2833x_PieVect.pp \
./SRC/DSP2833x_SysCtrl.pp \
./SRC/DSP2833x_Xintf.pp \
./SRC/main.pp 

OBJS__QTD += \
".\DSP2833x_ADC_cal.obj" \
".\DSP2833x_Adc.obj" \
".\DSP2833x_CSMPasswords.obj" \
".\DSP2833x_CodeStartBranch.obj" \
".\DSP2833x_CpuTimers.obj" \
".\DSP2833x_DefaultIsr.obj" \
".\DSP2833x_EPwm.obj" \
".\DSP2833x_GlobalVariableDefs.obj" \
".\DSP2833x_PieCtrl.obj" \
".\DSP2833x_PieVect.obj" \
".\DSP2833x_SysCtrl.obj" \
".\DSP2833x_Xintf.obj" \
".\DSP2833x_usDelay.obj" \
".\main.obj" 

ASM_DEPS__QTD += \
".\SRC\DSP2833x_ADC_cal.pp" \
".\SRC\DSP2833x_CSMPasswords.pp" \
".\SRC\DSP2833x_CodeStartBranch.pp" \
".\SRC\DSP2833x_usDelay.pp" 

C_DEPS__QTD += \
".\SRC\DSP2833x_Adc.pp" \
".\SRC\DSP2833x_CpuTimers.pp" \
".\SRC\DSP2833x_DefaultIsr.pp" \
".\SRC\DSP2833x_EPwm.pp" \
".\SRC\DSP2833x_GlobalVariableDefs.pp" \
".\SRC\DSP2833x_PieCtrl.pp" \
".\SRC\DSP2833x_PieVect.pp" \
".\SRC\DSP2833x_SysCtrl.pp" \
".\SRC\DSP2833x_Xintf.pp" \
".\SRC\main.pp" 

ASM_SRCS_QUOTED += \
"../SRC/DSP2833x_ADC_cal.asm" \
"../SRC/DSP2833x_CSMPasswords.asm" \
"../SRC/DSP2833x_CodeStartBranch.asm" \
"../SRC/DSP2833x_usDelay.asm" 

C_SRCS_QUOTED += \
"../SRC/DSP2833x_Adc.c" \
"../SRC/DSP2833x_CpuTimers.c" \
"../SRC/DSP2833x_DefaultIsr.c" \
"../SRC/DSP2833x_EPwm.c" \
"../SRC/DSP2833x_GlobalVariableDefs.c" \
"../SRC/DSP2833x_PieCtrl.c" \
"../SRC/DSP2833x_PieVect.c" \
"../SRC/DSP2833x_SysCtrl.c" \
"../SRC/DSP2833x_Xintf.c" \
"../SRC/main.c" 


# Each subdirectory must supply rules for building sources it contributes
./DSP2833x_ADC_cal.obj: ../SRC/DSP2833x_ADC_cal.asm $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Program Files/Texas Instruments/xdais_7_10_00_06/packages/ti/xdais" --include_path="../include" --diag_warning=225 --large_memory_model --float_support=fpu32 --preproc_with_compile --preproc_dependency="SRC/DSP2833x_ADC_cal.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

./DSP2833x_Adc.obj: ../SRC/DSP2833x_Adc.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Program Files/Texas Instruments/xdais_7_10_00_06/packages/ti/xdais" --include_path="../include" --diag_warning=225 --large_memory_model --float_support=fpu32 --preproc_with_compile --preproc_dependency="SRC/DSP2833x_Adc.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

./DSP2833x_CSMPasswords.obj: ../SRC/DSP2833x_CSMPasswords.asm $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Program Files/Texas Instruments/xdais_7_10_00_06/packages/ti/xdais" --include_path="../include" --diag_warning=225 --large_memory_model --float_support=fpu32 --preproc_with_compile --preproc_dependency="SRC/DSP2833x_CSMPasswords.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

./DSP2833x_CodeStartBranch.obj: ../SRC/DSP2833x_CodeStartBranch.asm $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Program Files/Texas Instruments/xdais_7_10_00_06/packages/ti/xdais" --include_path="../include" --diag_warning=225 --large_memory_model --float_support=fpu32 --preproc_with_compile --preproc_dependency="SRC/DSP2833x_CodeStartBranch.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

./DSP2833x_CpuTimers.obj: ../SRC/DSP2833x_CpuTimers.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Program Files/Texas Instruments/xdais_7_10_00_06/packages/ti/xdais" --include_path="../include" --diag_warning=225 --large_memory_model --float_support=fpu32 --preproc_with_compile --preproc_dependency="SRC/DSP2833x_CpuTimers.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

./DSP2833x_DefaultIsr.obj: ../SRC/DSP2833x_DefaultIsr.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Program Files/Texas Instruments/xdais_7_10_00_06/packages/ti/xdais" --include_path="../include" --diag_warning=225 --large_memory_model --float_support=fpu32 --preproc_with_compile --preproc_dependency="SRC/DSP2833x_DefaultIsr.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

./DSP2833x_EPwm.obj: ../SRC/DSP2833x_EPwm.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Program Files/Texas Instruments/xdais_7_10_00_06/packages/ti/xdais" --include_path="../include" --diag_warning=225 --large_memory_model --float_support=fpu32 --preproc_with_compile --preproc_dependency="SRC/DSP2833x_EPwm.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

./DSP2833x_GlobalVariableDefs.obj: ../SRC/DSP2833x_GlobalVariableDefs.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Program Files/Texas Instruments/xdais_7_10_00_06/packages/ti/xdais" --include_path="../include" --diag_warning=225 --large_memory_model --float_support=fpu32 --preproc_with_compile --preproc_dependency="SRC/DSP2833x_GlobalVariableDefs.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

./DSP2833x_PieCtrl.obj: ../SRC/DSP2833x_PieCtrl.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Program Files/Texas Instruments/xdais_7_10_00_06/packages/ti/xdais" --include_path="../include" --diag_warning=225 --large_memory_model --float_support=fpu32 --preproc_with_compile --preproc_dependency="SRC/DSP2833x_PieCtrl.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

./DSP2833x_PieVect.obj: ../SRC/DSP2833x_PieVect.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Program Files/Texas Instruments/xdais_7_10_00_06/packages/ti/xdais" --include_path="../include" --diag_warning=225 --large_memory_model --float_support=fpu32 --preproc_with_compile --preproc_dependency="SRC/DSP2833x_PieVect.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

./DSP2833x_SysCtrl.obj: ../SRC/DSP2833x_SysCtrl.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Program Files/Texas Instruments/xdais_7_10_00_06/packages/ti/xdais" --include_path="../include" --diag_warning=225 --large_memory_model --float_support=fpu32 --preproc_with_compile --preproc_dependency="SRC/DSP2833x_SysCtrl.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

./DSP2833x_Xintf.obj: ../SRC/DSP2833x_Xintf.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Program Files/Texas Instruments/xdais_7_10_00_06/packages/ti/xdais" --include_path="../include" --diag_warning=225 --large_memory_model --float_support=fpu32 --preproc_with_compile --preproc_dependency="SRC/DSP2833x_Xintf.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

./DSP2833x_usDelay.obj: ../SRC/DSP2833x_usDelay.asm $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Program Files/Texas Instruments/xdais_7_10_00_06/packages/ti/xdais" --include_path="../include" --diag_warning=225 --large_memory_model --float_support=fpu32 --preproc_with_compile --preproc_dependency="SRC/DSP2833x_usDelay.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

./main.obj: ../SRC/main.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Program Files/Texas Instruments/xdais_7_10_00_06/packages/ti/xdais" --include_path="../include" --diag_warning=225 --large_memory_model --float_support=fpu32 --preproc_with_compile --preproc_dependency="SRC/main.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '


