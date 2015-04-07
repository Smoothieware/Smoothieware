#ifndef _DEBUG_H
#define _DEBUG_H

#define ENTER_ISR() do {} while (0)
#define LEAVE_ISR() do {} while (0)

#define printf(...) iprintf(__VA_ARGS__)

#endif /* _DEBUG_H */
