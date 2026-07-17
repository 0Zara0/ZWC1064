#ifndef SOKOBAN_SOLVER_LOWMEM_H
#define SOKOBAN_SOLVER_LOWMEM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SOKOBAN_MAP_ROW
#define SOKOBAN_MAP_ROW 10
#endif

#ifndef SOKOBAN_MAP_COL
#define SOKOBAN_MAP_COL 14
#endif
#define SOKOBAN_MAX_STEP 256

/*
 * Low-RAM configuration.
 * Increase these only if the function returns SOKOBAN_ERR_SEARCH_OVERFLOW.
 * The default static RAM used by the solver workspace is about 40 KB.
 */
#ifndef SOKOBAN_MAX_OBJECTS
#define SOKOBAN_MAX_OBJECTS 16u
#endif

#ifndef SOKOBAN_BOX_NODE_MAX
#define SOKOBAN_BOX_NODE_MAX 1024u
#endif

/* A larger hash table reduces probing. Keep it >= 2 * SOKOBAN_BOX_NODE_MAX. */
#ifndef SOKOBAN_BOX_HASH_SIZE
#define SOKOBAN_BOX_HASH_SIZE 2048u
#endif

#ifndef SOKOBAN_BOMB_NODE_MAX
#define SOKOBAN_BOMB_NODE_MAX 128u
#endif

typedef enum {
    SOKOBAN_OK = 0,
    SOKOBAN_NOT_FOUND = 1,
    SOKOBAN_ERR_INVALID_ARGUMENT = -1,
    SOKOBAN_ERR_BUFFER_TOO_SMALL = -2,
    SOKOBAN_ERR_TOO_MANY_OBJECTS = -3,
    SOKOBAN_ERR_SEARCH_OVERFLOW = -4,
    SOKOBAN_ERR_INTERNAL = -5
} SokobanStatus;

/*
 * One-call MCU API.
 *
 * game_map encoding:
 *   0 road, 1 wall, 2 player, 3 unknown box, 4 unknown target,
 *   5 bomb, 'A'..'Z' boxes, 'a'..'z' matching targets.
 *
 * out_path receives W/S/A/D/Q/E plus observation codes 1/2/3/4.
 *   W: forward, S: backward, A: strafe left, D: strafe right
 *   Q: turn left (90deg CCW), E: turn right (90deg CW)
 * Use an output buffer of at least SOKOBAN_MAX_STEP bytes.
 *
 * If a bomb explodes, game_map is modified in-place by clearing walls
 * in the 3x3 explosion area, matching the original PC program.
 */
SokobanStatus Sokoban_Solve(int game_map[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL],
                            char *out_path,
                            size_t out_size);

const char *Sokoban_StatusString(SokobanStatus status);

/* Optional helper: compile-time static workspace estimate in bytes. */
uint32_t Sokoban_GetStaticWorkspaceBytes(void);

#ifdef __cplusplus
}
#endif

#endif /* SOKOBAN_SOLVER_LOWMEM_H */
