# Raspberry Pi Zero 2 W — USB Gadget Networking (Raspberry Pi OS Bookworm)

This document describes the **minimal, repeatable steps** required to enable **USB Ethernet gadget mode** on Raspberry Pi OS Bookworm (NetworkManager-based).

After completing this setup, plugging the Pi into any computer via USB will automatically create a network link:

- **Pi USB IP:** `192.168.7.1`
- **Host laptop IP:** automatically assigned via DHCP (e.g., `192.168.7.x`)
- **SSH:**
  ```bash
  ssh pi@192.168.7.1
  ```

This configuration works without Wi-Fi and is ideal for FTC robotics development, fieldwork, or offline environments.

---

## 1. Enable USB Gadget Mode (`g_ether`)

### Edit `/boot/firmware/config.txt`

Add at the end (or inside `[all]`):

```ini
dtoverlay=dwc2
```

### Edit `/boot/firmware/cmdline.txt`

This file must remain **one single line**.

After the word `rootwait`, add:

```
modules-load=dwc2,g_ether
```

Example:

```
console=serial0,115200 console=tty1 root=PARTUUID=XXXX rootfstype=ext4 fsck.repair=yes rootwait modules-load=dwc2,g_ether cfg80211.ieee80211_regdom=AU
```

### Enable SSH

```bash
sudo touch /boot/firmware/ssh
```

Reboot the Pi.

---

## 2. Tell NetworkManager to Manage `usb0`

Create the override directory:

```bash
sudo mkdir -p /etc/NetworkManager/conf.d
```

Create:

```bash
sudo nano /etc/NetworkManager/conf.d/10-usb0-managed.conf
```

Add:

```ini
[device]
match-device=interface-name:usb0
managed=true
```

Restart NetworkManager:

```bash
sudo systemctl restart NetworkManager
```

---

## 3. Create the USB Network Connection

With the Pi plugged into a host computer (so `usb0` exists):

```bash
sudo nmcli connection add   type ethernet   ifname usb0   con-name usb0   ipv4.method shared   ipv4.addresses 192.168.7.1/24   ipv6.method ignore   connection.autoconnect yes
```

Or, if the connection already exists:

```bash
sudo nmcli connection modify usb0   ipv4.method shared   ipv4.addresses 192.168.7.1/24   ipv6.method ignore   connection.autoconnect yes
```

Bring it up:

```bash
sudo nmcli connection up usb0
```

Reboot:

```bash
sudo reboot
```

---

## 4. Testing

### On the Pi:

```bash
ip addr show usb0
```

Expected:

```
inet 192.168.7.1/24
state UP
```

### On the laptop:

A new network interface (macOS: “RNDIS/Ethernet Gadget”) will show as **Connected**, with an IP such as:

```
192.168.7.179
Router: 192.168.7.1
```

### SSH to the Pi:

```bash
ssh pi@192.168.7.1
```

---

## 5. Results

| Feature | Status |
|--------|--------|
| USB gadget mode | ✔ Enabled via `dwc2` + `g_ether` |
| Pi IP | ✔ `192.168.7.1` (static) |
| Host DHCP | ✔ Provided automatically |
| Autoconnect at boot | ✔ Yes |
| Works on macOS/Windows/Linux | ✔ Yes |
| Wi-Fi unaffected | ✔ Yes |

---

## 6. (Optional) Enable mDNS for USB

```bash
sudo apt install avahi-daemon
```

Then SSH using:

```bash
ssh pi@raspberrypi.local
```

---

## 7. (Optional) macOS SSH Shortcut

Edit `~/.ssh/config`:

```ini
Host pi-usb
  HostName 192.168.7.1
  User pi
```

Connect with:

```bash
ssh pi-usb
```

---

## 8. Summary

This configuration provides:

- Fully automatic USB Ethernet gadget networking
- DHCP to any connected laptop
- Static Pi address (`192.168.7.1`)
- No need for Wi-Fi to develop on the Pi
- Clean Bookworm-compatible setup using NetworkManager only

This process can be repeated on any freshly flashed SD card.
