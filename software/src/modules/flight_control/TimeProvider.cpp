#include "TimeProvider.h"

float TimeProvider::now()
{
    // millis() liefert ms seit Boot, wir wollen Sekunden
    return millis() / 1000.0f;
}