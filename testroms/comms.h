#if !defined(COMMS_H)
#define COMMS_H 1

bool comms_check_active();
bool comms_update();
void comms_status(char *str, int len);
int comms_read(void *buffer, int maxlen);
int comms_write(const __far void *data, int len);

#endif // COMMS_H