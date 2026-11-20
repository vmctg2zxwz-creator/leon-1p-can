# leon-1p-can

CAN sniffer για Seat Leon 1P σε Waveshare ESP32-P4 KIT-B. Η οθόνη 10.1" δείχνει το ίδιο κείμενο με το serial.

- CAN 500 kbit/s, **μόνο ακρόαση**: δεν στέλνει τίποτα στο αυτοκίνητο.
- TX GPIO2 / RX GPIO3 (Adafruit CAN Pal, termination OFF).
- Serial **115200 baud**. Κάθε 1 s ένας πίνακας: μία γραμμή ανά ID με μηνύματα/s και τελευταία δεδομένα.
- Στο τέλος: msg/s, IDs, err/s ανά τύπο, total err, state.
- Χρονισμός CAN: SP 70%, SJW 4 (βρέθηκε στο αυτοκίνητο).

## Κατέβασμα .bin
Tab **Actions** → τελευταίο run → **Artifacts** → `leon-1p-can-firmware`.
Γράψε το `leon-1p-can-full.bin` στη διεύθυνση `0x0`.
