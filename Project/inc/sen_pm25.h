#ifndef __SEN_PM25_H
#define __SEN_PM25_H

extern u16 pm25_value;
extern bool pm25_ready;
extern bool pm25_alive;

void pm25_init(void);

#endif /* __SEN_PM25_H */