#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef enum CMissionResult {
  Ok,
  Err,
  ErrSkip,
} CMissionResult;

typedef struct CMissionPtr CMissionPtr;

typedef struct CTask CTask;

typedef struct MissionDataPtr MissionDataPtr;

typedef enum CMissionResult (*CTaskFunc)(struct MissionDataPtr *data);

typedef enum OptionFunction_CTaskFunc_Tag {
  Some_CTaskFunc,
  None_CTaskFunc,
} OptionFunction_CTaskFunc_Tag;

typedef struct OptionFunction_CTaskFunc {
  OptionFunction_CTaskFunc_Tag tag;
  union {
    struct {
      CTaskFunc some;
    };
  };
} OptionFunction_CTaskFunc;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

struct CTask *ctask_create(const char *name_ptr,
                           struct OptionFunction_CTaskFunc task_func,
                           struct OptionFunction_CTaskFunc repair_task_func);

struct CMissionPtr *cmission_create(const char *name_ptr, struct CTask *task_array, intptr_t size);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus
