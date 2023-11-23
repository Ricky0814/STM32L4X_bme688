#ifndef __MODE_H__
#define __MODE_H__

#ifdef __cplusplus
extern "C" {
#endif
    
#include "main.h"
#include <stdio.h>
#include "bme68x.h"
#include "common.h"

int self_test(void);

int forced_mode(void);

int parallel_mode(void);

int sequential_mode(void);




#ifdef __cplusplus
}
#endif

#endif /* __MODE_H__ */








