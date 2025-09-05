# mmWave Parking Solution with AWR1843AOPEVM

## Project Overview
> A hands-on parking solution using the **AWR1843AOPEVM** radar sensor, **Nucleo board**, **custom PCB**, and **DRV5055 ground sensors**. Leveraged **GitHub Copilot** to generate Python code for sensor data processing, with begginer Python experience. This project demonstrates hardware-software integration, AI-assisted coding, and real-world problem-solving.

## Key Features
- **Real-time sensor data processing** with Python.  
- **Hardware-software integration** (Nucleo + AWR1843AOPEVM).  
- **AI-assisted coding** via GitHub Copilot.  
- **Custom PCB design** for stable communication between components.  

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

## Challenges  
- **Learning Python on the Fly**:  
  I had begginer Python experience but used GitHub Copilot to generate code for sensor data processing. This required rapid learning of Python syntax and libraries while debugging.  

- **Hardware-Software Integration**:  
  Combining the AWR1843AOPEVM radar with the Nucleo board and custom PCB was complex. Ensuring stable communication between hardware components required troubleshooting signal integrity and timing issues.  

- **Documentation for Replication**:  
  Creating clear, step-by-step instructions for others to replicate the project was time-consuming. I had to balance technical details (e.g., baud rates, PCB layouts) with simplicity.  

- **Debugging Sensor Data**:  
  Parsing raw radar data from the AWR1843AOPEVM and filtering false positives (e.g., distinguishing cars from background noise) required iterative testing and adjustments to detection thresholds.  

## Skills Gained  
### Technical Skills  
- **Programming**:  
  - Python (via GitHub Copilot) for sensor data processing and logic.  
  - C (for Nucleo firmware in `main.c`).  
- **Hardware Tools**:  
  - STM32CubeIDE for Nucleo firmware development.  
  - Custom PCB design for hardware integration.  
- **Sensor Integration**:  
  - AWR1843AOPEVM radar sensor setup and data interpretation.  
  - DRV5055 ground sensors for vehicle detection.  
- **Tools & Platforms**:  
  - Tera Term/PuTTY for serial communication.  
  - GitHub Copilot for AI-assisted coding.  

### Soft Skills  
- **Problem-Solving**: Debugging hardware-software mismatches and refining detection algorithms.  
- **Documentation**: Writing clear instructions and technical documentation for complex systems.  
- **Adaptability**: Learning new tools (Python, radar sensors) quickly to meet project goals.  

- **Project Documentation**: Created clear documentation for replication and scalability.  
- **Problem-Solving**: Designed a custom PCB to ensure stable communication between hardware components.  

## Why This Project Matters
This project highlights my ability to:  
- **Solve real-world problems** using AI tools (GitHub Copilot).  
- **Learn rapidly** and adapt to new technologies (Python, radar sensors).  
- **Deliver end-to-end solutions** from hardware to software.  
- **Document complex systems** for clarity and future maintenance.  

## How to Replicate This Project
1. **Clone this repository**.  
2. **Connect the AWR1843AOPEVM** to your PC via USB.  
3. **Upload `awr1843.cfg`** using PuTTY/Tera Term.  
4. **Run `nucleoradar_code.py`** (Python script for radar logic).  
5. **Watch the prototype video** [here](https://youtu.be/zbJpwU5Eh9E) for a visual guide.  

## Files Included
- `awr1843.cfg` (configuration file for AWR1843AOPEVM)  
- `main.c` (Nucleo firmware for hardware integration)  
- `nucleoradar_code.py` (Python script for sensor data processing)  
- `code-iterations/` (code versions showing my learning journey)  
- `mx_pinouts.png` (hardware pinout diagrams)

## AI/ML Focus
- **GitHub Copilot**: Demonstrated ability to generate code without prior Python experience.  
- **Sensor Data Processing**: A foundational step for AI/ML applications (e.g., object detection, parking automation).  

## Contact
Feedback or questions? Reach out via [LinkedIn - https://www.linkedin.com/in/tonči-hrgović-315889248/]!  

Watch the Prototype: [https://youtu.be/zbJpwU5Eh9E]



