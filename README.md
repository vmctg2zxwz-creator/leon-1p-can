# leon-1p-can

CAN sniffer για Seat Leon 1P σε Waveshare ESP32-P4 KIT-B. **Χωρίς οθόνη.**

- CAN 500 kbit/s, **μόνο ακρόαση**: δεν στέλνει τίποτα στο αυτοκίνητο.
- TX GPIO2 / RX GPIO3 (Adafruit CAN Pal, termination OFF).
- Serial **921600 baud**. Κάθε γραμμή: `ID [DLC] data`.
- Κάθε 1 s: `--- μηνύματα/s | bus errors | rx err count | state | dropped ---`

## Κατέβασμα .bin
Tab **Actions** → τελευταίο run → **Artifacts** → `leon-1p-can-firmware`.
Γράψε το `leon-1p-can-full.bin` στη διεύθυνση `0x0`.
