#ifndef OPL_OFFLINE_H
#define OPL_OFFLINE_H

#include <stddef.h>
#include <stdint.h>

void OfflineOPL_RunUntilNextCallback(void);
void OfflineOPL_RunUntilQueueEmpty(uint64_t max_samples);
void OfflineOPL_RunSamples(uint64_t nsamples);
int OfflineOPL_QueueEmpty(void);
void OfflineOPL_SetCapture(int enabled);
uint64_t OfflineOPL_TotalSamples(void);
unsigned int OfflineOPL_Rate(void);
const int16_t *OfflineOPL_Captured(size_t *frames);
void OfflineOPL_ClearCapture(void);
int OfflineOPL_LastKeyOff(uint64_t *sample);
void OfflineOPL_ResetKeyOffTracking(void);
int OfflineOPL_VoicesKeyedOn(void);

#endif
