TARGET = RCOutput_Test RCOutput_Dump
 
all: $(TARGET)

 RCOutput_Test:  RCOutput_Test.o
	@echo 'Linking target: $@'
	@echo 'Invoking: GCC Linker'
	gcc -Wall -O2  RCOutput_Test.o -o  RCOutput_Test
	@echo 'Finished building target: $@'
	@echo ' '

RCOutput_Dump:  RCOutput_Dump.o
	@echo 'Linking target: $@'
	@echo 'Invoking: GCC Linker'
	gcc -Wall -O2  RCOutput_Dump.o -o  RCOutput_Dump
	@echo 'Finished building target: $@'
	@echo ' '

RCOutput_Test.o: RCOutput_Test.c
	@echo 'Compiling target: $@'
	@echo 'Invoking: GCC  compiler'
	gcc -Wall -O2 -c -o "$@" "$<" 
	@echo 'Finished building target: $@'
	@echo ' '

RCOutput_Dump.o: RCOutput_Dump.c
	@echo 'Compiling target: $@'
	@echo 'Invoking: GCC  compiler'
	gcc -Wall -O2 -c -o "$@" "$<"
	@echo 'Finished building target: $@'
	@echo ' '

clean:
	$(RM)  RCOutput_Test RCOutput_Dump  RCOutput_Test.o  RCOutput_Dump.o
