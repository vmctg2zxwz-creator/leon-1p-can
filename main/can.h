#pragma once
#include <stdbool.h>

/* Οι τρεις τιμές που δείχνουμε. */
typedef struct {
    float rpm;          /* στροφές */
    float coolant_c;    /* θερμοκρασία νερού */
    float intake_c;     /* θερμοκρασία αέρα εισαγωγής */
    bool  rpm_ok, coolant_ok, intake_ok;  /* ήρθε πρόσφατη τιμή; */
} car_data_t;

/* Ξεκινά το CAN σε λειτουργία μόνο ακρόασης. */
void can_start(void);

/* Οι τελευταίες τιμές. */
car_data_t can_get(void);
