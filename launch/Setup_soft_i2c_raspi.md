# 🧪 Setting Up Software I2C (Bit-Banged I2C) on a Raspberry Pi

Software I2C lets you emulate I2C using **any two GPIO pins** via the `i2c-gpio` kernel module. This is useful when:

- The hardware I2C bus is already in use.
- You need more than one I2C bus.

---

## ✅ Step-by-Step Guide

### 1. Choose GPIO Pins

Pick two unused GPIOs for SDA and SCL. Example:

- **SDA** → `GPIO23` (physical pin **16**)  
- **SCL** → `GPIO24` (physical pin **18**)

You can change these to any other available GPIOs.

---

### 2. Open a Terminal

Edit the boot configuration file:

```bash
sudo nano /boot/firmware/config.txt
```
    or
```bash
sudo gedit /boot/firmware/config.txt
```

Add the following line at the end:
```bash
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=23,i2c_gpio_scl=24
```
- `bus=3` creates a new software I2C bus `/dev/i2c-3`  
- `i2c_gpio_sda=23` and `i2c_gpio_scl=24` set the GPIO pins
---

### 💾 Save and Exit

- `Ctrl + O` → Write changes  
- `Enter` → Confirm  
- `Ctrl + X` → Exit the editor

---

if you use gedit you can just click save

### 🔁 Reboot the Pi

```bash
sudo reboot
```

### 3. Install I2C Tools

Install `i2c-tools` if it's not already installed:

```bash
sudo apt update
sudo apt install -y i2c-tools
```

### 4. Verify the New I2C Bus

After reboot, check available I2C devices:

```bash
ls /dev/i2c-*
```
you should see:
```bash
/dev/i2c-3
```
Scan the software I2C bus to detect connected devices
```bash
sudo i2cdetect -r -y 3
```

### Notes
- You can create multiple software I2C buses by adding more dtoverlay lines with unique bus numbers and GPIO pins.

- Software I2C is slower and less reliable than hardware I2C, but it works well for most sensors and peripherals.