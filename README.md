# leon-1p-can

Dashboard για Seat Leon 1P (Cupra 2.0 TFSI) σε Waveshare ESP32-P4 KIT-B με οθόνη 10.1".

- CAN powertrain 500 kbit/s, **μόνο ακρόαση**, TX GPIO2 / RX GPIO3.
- Δείχνει μόνο: στροφές (0x280), νερό (0x288), αέρα εισαγωγής (0x380).
- ESP-IDF v6.1.1 + LVGL 9 + BSP της Waveshare (`waveshare/esp32_p4_platform`).

## Κατέβασμα .bin
Κάθε push στο `main` κάνει build στο GitHub Actions. Το `.bin` είναι στο tab **Actions** → τελευταίο run → **Artifacts**.
Για μόνιμη έκδοση: tag `v0.1.0` → εμφανίζεται στα **Releases**.

`leon-1p-can-full.bin` γράφεται στη διεύθυνση `0x0` (περιέχει bootloader + partition table + app).

## Κατάσταση
- [x] Οθόνη 10.1-DSI-TOUCH-A μέσω Waveshare BSP (δεν έχει δοκιμαστεί σε πλακέτα)
- [ ] Επιβεβαίωση κλιμάκων CAN με Autel
