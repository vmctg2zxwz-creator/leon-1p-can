#pragma once
#include <stdbool.h>

/* Τιμές που διαβάζουμε από το αυτοκίνητο (OBD-II mode 01). */
typedef struct {
    float rpm;          /* στροφές */
    float speed_kmh;    /* ταχύτητα */
    float coolant_c;    /* θερμοκρασία νερού */
    float intake_c;     /* θερμοκρασία αέρα εισαγωγής */
    float map_kpa;      /* πίεση πολλαπλής (απόλυτη) */
    float baro_kpa;     /* ατμοσφαιρική πίεση */
    float throttle_pct; /* πεντάλ γκαζιού */
    float battery_v;    /* τάση μονάδας ελέγχου */
    bool  connected;    /* απαντάει το ECU; */
} car_data_t;

/* Ξεκινάει το CAN (TWAI) και ένα task που ρωτάει συνεχώς τα PID. */
void obd_start(void);

/* Αντίγραφο των τελευταίων τιμών (ασφαλές από οποιοδήποτε task). */
car_data_t obd_get(void);
