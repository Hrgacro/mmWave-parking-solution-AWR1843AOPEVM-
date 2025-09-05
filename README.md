# mmWave-parking-solution-AWR1843AOPEVM-

Description:
This project demonstrates a parking solution using the AWR1843AOPEVM radar sensor, Nucleo board, custom PCB, and DRV5055 ground sensors. I used GitHub Copilot to generate Python code for sensor data processing, even though I had no prior Python experience. The system integrates hardware and software to detect vehicles and trigger the radar for parking automation.

Key Features:
Real-time sensor data processing with Python.
Hardware-software integration (Nucleo + AWR1843AOPEVM).
AI-assisted coding via GitHub Copilot.
Custom PCB for stable communication between components.

Technologies Used:
Hardware: AWR1843AOPEVM, Nucleo-WL55JC1, DRV5055 sensors, custom PCB.
Software: Python (via GitHub Copilot), STM32CubeIDE.
Tools: Tera Term/PuTTY for serial communication.

What I Learned:
How to combine hardware and software for real-world applications.
How to use AI tools (GitHub Copilot) to accelerate coding without prior experience.
The importance of documenting complex projects for clarity.

Instructions to Use:
Download Tools: Use Tera Term or PuTTY for serial communication.
Connect Hardware: Link AWR1843AOPEVM to your PC via USB.
Configure Serial Settings:
Baud rate: 115200
8N1 (no parity, 1 stop bit)
Delay type: Per line, 20ms
Upload Configuration File:
Use PuTTY to send awr1843.cfg via "File > Send File".
Ensure "Bulk Read" is selected and "Binary" is off.
Start the System: Press Enter after uploading the .cfg file.
(Optional) Nucleo + PCB Integration:
Upload main.c to Nucleo-WL55JC1 via STM32CubeIDE.
Use your custom PCB to bridge AWR1843AOPEVM and Nucleo.
Watch the Prototype: [https://youtu.be/zbJpwU5Eh9E]
