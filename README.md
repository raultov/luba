BE CAREFUL!!!

I am changing a lot this project, is far from being stable.

MAIN COMPONENTS:

STM32F4 discovery

Freetos 1.6
HC-SR04
DHT11
nRF24L01
APC220

GENERAL INSTRUCTIONS
	- Install eclipse C/C++
	- Install arm linaro eabi libraries 
	- Instal st-flash
	- Install openocd
	

PARTICULAR ECLIPSE SETTINGS
	C/C++ build:
		Discovery Options:
			Cross G++ Compiler
				Compiler Invocation Command --> arm-none-eabi-g++
			Cross GCC Compiler
				Compiler Invocation Command --> arm-none-eabi-gcc

		Environment:
			Add to Path --> /opt/ARM/arm-linaro-eabi/bin

		Settings:
			Tool Settings:
				Cross settings:
					Prefix --> arm-none-eabi-
					Path --> /opt/ARM/arm-linaro-eabi/bin
				Cross gcc compiler:
					Command --> gcc
					All options --> -DUSE_STM32F4_DISCOVERY -DSTM32F4XX -DUSE_STDPERIPH_DRIVER 
							-I"/.../STM32F4-Discovery" -I"/.../STM32F4xx_StdPeriph_Driver/inc" -I"/.../CMSIS/Include" 
							-I"/.../CMSIS/ST/STM32F4xx/Include" -I"/.../luba3/src" -I"/.../luba3/FreeRTOS/Source/include" 
							-I"/.../luba3/FreeRTOS/Source/portable/GCC/ARM_CM4F" -O0 -g3 -Wall -c  -g  
							-std=gnu99 -gdwarf-2 -ffunction-sections -fdata-sections -fsingle-precision-constant 
							-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
				Cross g++ compiler:
					Command --> g++
					All options --> -DUSE_STM32F4_DISCOVERY -DSTM32F4XX -DUSE_STDPERIPH_DRIVER -I../luba3/src 
							-I../luba3/Libraries/CMSIS/Include -I../luba3/Libraries/STM32F4xx_StdPeriph_Driver/inc 
							-I../luba3/Libraries/Device/STM32F4xx/Include 
							-O0 -g3 -Wall -c  -g  -std=gnu99 -gdwarf-2 -ffunction-sections -fdata-sections 
							-fsingle-precision-constant -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
				Cross g++ linker:
					Command --> gcc
					All options --> -L/opt/ARM/arm-linaro-eabi/lib/gcc/arm-none-eabi/4.6.2/thumb/cortex-m4/float-abi-hard/fpuv4-sp-d16 
							-L/opt/ARM/arm-linaro-eabi/arm-none-eabi/lib/thumb/cortex-m4/float-abi-hard/fpuv4-sp-d16 
							-O0  -g -Wall  -fno-exceptions -ffunction-sections -fdata-sections  -nostartfiles 
							-Wl,--gc-sections -Wl,-Map=linker.map,--gc-sections,-T "/.../luba3/src/stm32_flash.ld" 
							-fsingle-precision-constant -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 
							-lgcc -lc -lg -lm

				GCC asembler:
					Command --> arm-none-eabi-as
					All options --> -Wa -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I../luba3/src 
							-I../luba3/Libraries/CMSIS/Include 
							-I../luba3/Libraries/STM32F4xx_StdPeriph_Driver/inc 
							-I../luba3/Libraries/Device/STM32F4xx/Include 
							-I"/home/raul/git/electronica/luba3/FreeRTOS/Source/include" 
							-I"/home/raul/git/electronica/luba3/FreeRTOS/Source/portable/GCC/ARM_CM4F"

			Build steps:
				Command --> arm-none-eabi-objcopy  -O binary  "${ProjName}" "${ProjName}.bin" 
				Description --> Binary

		Tool Chain Editor:
			Current Toolchain --> Cros GCC
			Current builder --> CDT Internal Builder
			Used Tools --> Cross GCC Compiler, Cross G++ Compiler, Cross G++ Linker, GCC Assembler

	C/C++ General:
		A xml file with the configuration can be found together with this readme file.

			































