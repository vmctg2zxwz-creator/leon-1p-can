# leon-1p-can

Dashboard για Seat Leon 1P (Cupra 2.0 TFSI) σε Waveshare ESP32-P4 KIT-B με οθόνη 10.1".

- CAN powertrain 500 kbit/s, **μόνο ακρόαση**, TX GPIO2 / RX GPIO3.
- Τιμές: στροφές (0x280), ταχύτητα (0x1A0), νερό (0x288), αέρας εισαγωγής (0x380).
- ESP-IDF v5.5 + LVGL 9.

## Κατέβασμα .bin
Κάθε push στο `main` κάνει build στο GitHub Actions. Το `.bin` είναι στο tab **Actions** → τελευταίο run → **Artifacts**.
Για μόνιμη έκδοση: tag `v0.1.0` → εμφανίζεται στα **Releases**.

`leon-1p-can-full.bin` γράφεται στη διεύθυνση `0x0` (περιέχει bootloader + partition table + app).

## Κατάσταση
- [ ] Driver οθόνης 10.1" (προς το παρόν placeholder, δεν δείχνει εικόνα)
- [ ] Επιβεβαίωση κλιμάκων CAN με Autel
