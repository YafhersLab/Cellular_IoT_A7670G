# 📶 Cellular IoT Monitoring System using A7670G and MQTT

## 📌 Project Overview
This project involves the development of a **cellular IoT monitoring and control system** using two custom electronic boards based on **A7670G cellular modules**.

Both boards transmit data over the **cellular network** to an **MQTT broker**. The first board focuses on environmental monitoring, while the second board integrates additional sensing, multimedia data acquisition, and remote actuation capabilities. Data collected from the MQTT broker is visualized and stored using a **LabVIEW-based interface**, enabling real-time monitoring and local database generation.

This project demonstrates skills in:
- Cellular IoT communications  
- MQTT-based data transmission  
- Embedded systems design  
- Data visualization and logging  
- End-to-end IoT system integration  

---

## 🎯 Motivation
The goal of this project was to design a **scalable cellular IoT architecture** capable of handling heterogeneous data sources, including environmental sensors, distance measurements, multimedia inputs, and remote actuation.

By integrating MQTT communication with LabVIEW-based visualization, the system provides a complete workflow from **data acquisition to analysis**, suitable for industrial, research, or monitoring applications.

---

## 🧠 System Description
- **Board 1 (Environmental Node):**
  - Measures temperature and humidity
  - Tracks time using an RTC module
  - Sends data to an MQTT broker via cellular connectivity

- **Board 2 (Advanced Monitoring & Control Node):**
  - Measures distance
  - Includes RTC-based time tracking
  - Captures camera and audio data
  - Controls a water pump via relay (remote activation)
  - Transmits all data via cellular MQTT

- **Backend & Visualization:**
  - Data is retrieved from the MQTT broker
  - LabVIEW interface displays real-time graphs
  - Local databases are generated for data logging and analysis

---

## 🧩 Project Structure

1. **code/**  
   Firmware source code for both cellular IoT boards, including sensor acquisition, MQTT communication, and control logic.

2. **apk/**  
   Mobile application files developed using MIT App Inventor.

3. **schematic/**  
   Electrical schematics of the custom-designed PCBs.

---

## 🛠️ Hardware Used
- **Cellular Module:** Lilygo A7670G  
- **Microcontroller:** ESP32-S3  
- **Custom Hardware:**  
  - PCBs designed using Altium Designer  
  - Relay module for pump control  
  - Environmental, distance, camera, and audio sensors  

---

## 💻 Software & Tools
- **Altium Designer** – PCB design  
- **Arduino IDE** – Firmware development  
- **SolidWorks** – Mechanical and enclosure design  
- **MIT App Inventor** – Mobile application development  
- **LabVIEW** – Data visualization and local database generation  

---

## 🚧 Project Status
**Experimental**

This system is currently in an experimental phase and serves as a test platform for advanced cellular IoT monitoring and control applications.

---

## 📚 Future Work
- Secure MQTT communication (TLS)
- Cloud-based data analytics
- Multimedia data compression and optimization
- Power optimization for long-term deployment
- Integration with AI-based data analysis pipelines

---

## 📄 License
This project is intended for **educational and research purposes**.  
License to be defined.

---

## ✍️ Author
**Yafhers Mendoza**  
Cellular IoT & Embedded Systems Design