
/**
 * @brief: initialize 4 IR sensors
 */
void IR_Init(void);

/*
 * @param index: the sensor index; J5 is 0 and J8 is 3
 * @return the latest measurement from that sensor, in unit of mm
 */
int IR_GetData(int index);
