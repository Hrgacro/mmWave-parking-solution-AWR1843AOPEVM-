# mmWave Parking Solution with AWR1843AOPEVM

## Project Overview
> A hands-on parking solution using the **AWR1843AOPEVM** radar sensor, **Nucleo board**, **custom PCB**, and **DRV5055 ground sensors**. Leveraged **GitHub Copilot** to generate Python code for sensor data processing, even without prior Python experience. Demonstrates hardware-software integration and AI-assisted coding.

## Key Features
- Real-time sensor data processing with Python.
- Hardware-software integration (Nucleo + AWR1843AOPEVM).
- AI-assisted coding via GitHub Copilot.
- Custom PCB for stable communication between components.

## Technologies Used
### Hardware
- AWR1843AOPEVM radar sensor  
- Nucleo-WL55JC1 board  
- DRV5055 ground sensors  
- Custom PCB  

### Software
- Python (via GitHub Copilot)  
- STM32CubeIDE (for Nucleo firmware)  

### Tools
- Tera Term/PuTTY (serial communication)  
- GitHub Copilot (code generation)  

## What I Learned
- How to combine hardware and software for real-world applications.  
- How to use AI tools (GitHub Copilot) to accelerate coding without prior experience.  
- The importance of documenting complex projects for clarity and replication.  

## Instructions to Use
1. **Download Tools:** Install Tera Term or PuTTY for serial communication.  
2. **Connect Hardware:** Link the AWR1843AOPEVM to your PC via USB.  
3. **Configure Serial Settings:**  
   - Baud rate: 115200  
   - 8N1 (no parity, 1 stop bit)  
   - Delay type: Per line, 20ms  
4. **Upload Configuration File:**  
   - Use PuTTY to send `awr1843.cfg` via "File > Send File".  
   - Ensure "Bulk Read" is selected and "Binary" is off.  
5. **Start the System:** Press Enter after uploading the `.cfg` file.  
6. **(Optional) Nucleo + PCB Integration:**  
   - Upload `main.c` to Nucleo-WL55JC1 via STM32CubeIDE.  
   - Use your custom PCB to bridge AWR1843AOPEVM and Nucleo.  
7. **Watch the Prototype:** (https://youtu.be/zbJpwU5Eh9E)  

## Files Included
- `awr1843.cfg` (configuration file)  
- `main.c` (Nucleo firmware)  
- `nucleoradar_code.py` (Python radar logic)  
- `code-iterations/` (code versions showing learning process)  
- `mx_pinouts.png` (hardware pinout diagrams)  

## Why This Project Matters
This project showcases my ability to:  
- Solve real-world problems using AI tools (GitHub Copilot).  
- Integrate hardware and software without prior Python experience.  
- Document complex systems for replication and scalability.  

Watch the Prototype: [https://youtu.be/zbJpwU5Eh9E]

