# ColdFire FreeRTOS

This is an up-to-date ColdFire port of FreeRTOS. The existing ColdFire ports of FreeRTOS was quite old and did not take advantage of the ColdFire including nested interrupts, user and supervisor stacks as well as some optimizations.

By implementing a switch between User and Supervisor modes during the context switch, we can give processes a little more protection as well as shift the burden of interrupt stack frames onto the single system stack instead of onto whichever task the interrupt occurs in. This can significantly reduce the memory requirements of FreeRTOS when there are many tasks and especially when nested interrupts are enabled.

Swapping between user and supervisor states is a little more involved, but the extra overhead is pretty small as compared to the yield handler itself. The basic processes is:

1. Using the USP, save the interrupt stack frame back into the user stack
2. Save all the user registers (D0~D7 and A0~A6)
3. Clear the forced software interrupt
4. Save the USP into the task control block (TCB)
5. Call the normal FreeRTOS yield handler.
6. Reverse the above steps
7. Perform an 'RTE' and return to user mode

Additionally, critical sections cannot simply be entered through setting and clearing the CPUSR interrupt-level bits (since this is a privellaged operation). Instead, two traps have been made (trap 12 and trap 13) which can be invoked from user land. With a little stack twiddling, the interrupt mask is set through the RTE instruction making enabling and disabling of the interrupts an atomic operation.

Lastly, since the ColdFire does have the FF1 instruction (similar to the ARM CLZ instruction), 'configUSE_PORT_OPTIMISED_TASK_SELECTION' may be enabled to improve context switching times slightly. No benchmarks have been performened with these changes, so the actual performance gains may be negligible.

# Summary Changed or New Definitions (FreeRTOSConfig.h or portmacro.h)
portUSING_PRIVILEGE_MODE
- Splits MPU operation (memory protection) from PRIVILEGED operation (Supervisor/User state)

configIDLE_TASK_NAME
- Specify the name of the idle task in the configuration file

configUSE_CONFIG_ASSERT
- Simply enable and disable the configASSERT defintion

configKERNEL_INTERRUPT_PRIORITY
- Actually affects the SWI interrupt level
- Can be any value from 1 thru 7

configMAX_SYSCALL_INTERRUPT_PRIORITY
- Allows higher priority interrupts to run in a critical section
- configKERNEL_INTERRUPT_PRIORITY should be equal to or less than this value
- for example, '5' would allow interrupts at levels 6 and 7 to still occur
