# freertos_stm32_motor_control
 
FreeRTOS implementation, developed in Atollic TrueSTUDIO, that controls a DC motor.

Runs on STM32 Nucleo64 F401RE board.

https://www.st.com/en/evaluation-tools/nucleo-f401re.html

Uses the IHM04A1 motor driver expansion board, utilizing the L6206 brushed DC motor driver.

https://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32-nucleo-expansion-boards/stm32-ode-move-actuate-hw/x-nucleo-ihm04a1.html#sw-tools-scroll

The first commit of the project is basically "hello world!" for FreeRTOS motor control, with a blinking LED, UART text output, and push button motor on/off control.  The STM32 SPN4 motor control software documentation can be found here:

https://www.st.com/content/st_com/en/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32cube-expansion-packages/x-cube-spn4.html

You will notice that the STM32 SPN4 motor control examples are not implemented in FreeRTOS, and therefore this project was constructed by transferring applicable portions of the F401RE example to an existing FreeRTOS project for F401RE.

The project was constructed as follows:

1.  Create a new FreeRTOS project for the target board using CubeMX.  Be sure to assign a hardware timer to the timebase source under SYS, select the IDE as TrueSTUDIO, enable NVIC for GPIO interrupts, and select FreeRTOS middleware.

2.  Save the project and open in TrueSTUDIO.

3.  To get the LED to blink, create a FreeRTOS task that writes set and reset values the the appropriate GPIO pin.

4.  To use the pushbutton, find the IRQ handler function in the stm32f4xx_it.c file, and add code to take action when the button is pressed.  The vector interrupt controller uses a table to branch to this function when the button is pressed.

5.  To interface with the motor control board:

    a.  find the board specific sample in the SPN4 example project folders.

    b.  copy the board support package, the BSP folder, to the Drivers folder of the FreeRTOS project.

    c.  only copy files specific to the board you are using, in this case the STM32F401RE.

    d.  copy any missing files from the Inc and Src folders as well.

    e.  add any missing includes, macro definitions, or function definitions found in any of the source and header files.
    
    f.  there may be other missing elements, but these will only be found when you try to build.
    
    g.  once you get it to build, add code in main.c to configure the motor control board at start up.
    
    h.  finally, add a task in main.c to turn the motor on and off on button press, using the main.c in SPN4 as a guide.
    
    i.  you may need to use a shared global variable that is written to in the IRQ handler and read in the motor task.
    
6.  Alternatively, clone my project to a temporary location, and copy files into a new empty project.

    a.  clone the project to you TrueSTUDIO project folder.
    
    b.  rename by adding "temp" to the end of the directory name.
    
    c.  create a new empty project using STM32CubeMX.
    
        1) use FreeRTOS middleware, enable CMSIS_V1
        2) use Timer4 for instead of systick
    
    d.  copy and replace files from the cloned project to the new project:
    
        1) Inc
        2) Src
        3) Drivers
    
    e.  edit the project includes dependencies (see the screen shot image "includes.png"):
        
        project->properties->C/C++ General->Paths and Symbols->Includes->Add
        
        add path to Drivers->BSP, Drivers->CMSIS, Drivers->STM32HAL... as needed, the build errors will indicate the 
        header files that the compiler cannot find, just locate where the files are in the project folder and add
        the appropriate path to the includes.
    
    e.  build the project.
    
7.  once you have a project that builds fro either method above:
    
    a.  debug as C/C++.
    
    b.  verify LED is flashing.
    
    c.  open minicom and verify UART output, 115000, 8, 1, 1.
    
    d.  press the blue button and verify motor turns.
    
    e.  verify text output to the minicom terminal.
    
