#include <hal/cmd_log.h>

#ifdef HAS_LOG_SOFTUART
SoftwareSerial SwSerial(0, SOFTUART_TX_PIN);  // RX, TX
#endif 
