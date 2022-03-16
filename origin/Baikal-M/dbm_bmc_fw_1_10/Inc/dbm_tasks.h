#ifndef DBM_TASKS_H
#define DBM_TASKS_H

void vInitFlashWriteThread(void);


void flashWriteThread(void *arg);
void flashEraseThread(void *arg);
void flashOperationThread(void *arg);

void systemStartThread(void *arg);

void ledGreenThread(void *arg);
void ledRedThread(void *arg);

void inputPinMonitorThread(void *arg);


#endif // DBM_TASKS_H
