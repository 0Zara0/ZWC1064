#ifndef SOKO_MAP_TOOLS_H
#define SOKO_MAP_TOOLS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SOKOMAP_ROW
#define SOKOMAP_ROW 10u
#endif

#ifndef SOKOMAP_COL
#define SOKOMAP_COL 14u
#endif

#define SOKOMAP_ARRAY_LEN (SOKOMAP_ROW * SOKOMAP_COL)

typedef enum
{
    SOKOMAP_OK = 0,
    SOKOMAP_ERR_NULL = 1,
    SOKOMAP_ERR_INVALID_CHAR = 2,
    SOKOMAP_ERR_PLAYER_NOT_FOUND = 3,
    SOKOMAP_ERR_NO_TARGET = 4,
    SOKOMAP_ERR_INVALID_TAG = 5
} SokoMapStatus;

/*
*将uint8_t类型的原始地图数组转化为int类型的地图数组(不要用这个)
 * Convert received one-dimensional uint8_t ASCII map to one-dimensional int map.
 * Supported input:
 *   '0'..'5' -> 0..5
 *   'A'..'Z' -> ASCII value, for marked boxes
 *   'a'..'z' -> ASCII value, for marked goals
 * Other bytes are converted to -1 and SOKOMAP_ERR_INVALID_CHAR is returned.
 */
SokoMapStatus SokoMap_AsciiToIntArray(const uint8_t ascii[SOKOMAP_ARRAY_LEN],
                                      int num[SOKOMAP_ARRAY_LEN]);

/*
*将uint8_t类型的原始地图数组转化为int类型的地图数组
 * Convert received one-dimensional uint8_t ASCII map to one-dimensional int map,
 * but keep labels that already exist in num[].
 *
 * Use this after the first normal conversion and after you have called
 * SokoMap_MarkNearest(). It solves the problem where a later raw map only
 * contains '3'/'4' and would otherwise overwrite existing A/B/C... or a/b/c...
 * labels in your latest int one-dimensional map.
 *
 * Preserve rule:
 *   incoming '3' + old num[i] is 'A'..'Z' -> keep old label
 *   incoming '4' + old num[i] is 'a'..'z' -> keep old label
 * Other cells follow the latest received raw map.
 */
SokoMapStatus SokoMap_AsciiToIntArrayKeepMarks(const uint8_t ascii[SOKOMAP_ARRAY_LEN],
                                               int num[SOKOMAP_ARRAY_LEN]);

/*
 *将int类型的一维数组转化为二维数组
 * Convert one-dimensional int map to two-dimensional int map.
 */
SokoMapStatus SokoMap_IntArrayToMap(const int num[SOKOMAP_ARRAY_LEN],
                                    int map[SOKOMAP_ROW][SOKOMAP_COL]);

/*
 *给箱子和目的地标记
 * Mark the nearest unmarked object from player position 2.
 * tag = 'A'..'Y': mark nearest 3 as tag.
 * tag = 'a'..'y': mark nearest 4 as tag.
 * tag = 'Z' or 'z': special mode, mark all 3 as 'A' and all 4 as 'a'.
 * If exactly one 3 and one 4 remain, the function automatically assigns a pair.
 */
SokoMapStatus SokoMap_MarkNearest(int map[SOKOMAP_ROW][SOKOMAP_COL], uint8_t tag);



/* Optional helper: readable status string, no printf dependency. */
const char *SokoMap_StatusString(SokoMapStatus status);

#ifdef __cplusplus
}
#endif

#endif /* SOKO_MAP_TOOLS_H */
