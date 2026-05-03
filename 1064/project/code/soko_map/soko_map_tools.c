#include "soko_map_tools.h"

#define CELL_INDEX(r, c) ((uint8_t)((r) * SOKOMAP_COL + (c)))
#define CELL_ROW(idx)    ((uint8_t)((idx) / SOKOMAP_COL))
#define CELL_COL(idx)    ((uint8_t)((idx) % SOKOMAP_COL))

static uint8_t is_upper_ascii(uint8_t ch)
{
    return (uint8_t)((ch >= (uint8_t)'A') && (ch <= (uint8_t)'Z'));
}

static uint8_t is_lower_ascii(uint8_t ch)
{
    return (uint8_t)((ch >= (uint8_t)'a') && (ch <= (uint8_t)'z'));
}

static uint8_t is_upper_int(int value)
{
    return (uint8_t)((value >= (int)'A') && (value <= (int)'Z'));
}

static uint8_t is_lower_int(int value)
{
    return (uint8_t)((value >= (int)'a') && (value <= (int)'z'));
}

static uint8_t in_map_i16(int16_t r, int16_t c)
{
    return (uint8_t)((r >= 0) && (r < (int16_t)SOKOMAP_ROW) &&
                     (c >= 0) && (c < (int16_t)SOKOMAP_COL));
}

static SokoMapStatus ascii_cell_to_int(uint8_t ch, int *out_value)
{
    if (out_value == (int *)0) {
        return SOKOMAP_ERR_NULL;
    }

    if ((ch >= (uint8_t)'0') && (ch <= (uint8_t)'5')) {
        *out_value = (int)(ch - (uint8_t)'0');
        return SOKOMAP_OK;
    }

    if (is_upper_ascii(ch) || is_lower_ascii(ch)) {
        *out_value = (int)ch;
        return SOKOMAP_OK;
    }

    *out_value = -1;
    return SOKOMAP_ERR_INVALID_CHAR;
}

SokoMapStatus SokoMap_AsciiToIntArray(const uint8_t ascii[SOKOMAP_ARRAY_LEN],
                                      int num[SOKOMAP_ARRAY_LEN])
{
    uint16_t i;
    SokoMapStatus status = SOKOMAP_OK;

    if ((ascii == (const uint8_t *)0) || (num == (int *)0)) {
        return SOKOMAP_ERR_NULL;
    }

    for (i = 0u; i < (uint16_t)SOKOMAP_ARRAY_LEN; i++) {
        int value;
        SokoMapStatus cell_status = ascii_cell_to_int(ascii[i], &value);

        num[i] = value;
        if (cell_status != SOKOMAP_OK) {
            status = cell_status;
        }
    }

    return status;
}

SokoMapStatus SokoMap_AsciiToIntArrayKeepMarks(const uint8_t ascii[SOKOMAP_ARRAY_LEN],
                                               int num[SOKOMAP_ARRAY_LEN])
{
    uint16_t i;
    SokoMapStatus status = SOKOMAP_OK;

    if ((ascii == (const uint8_t *)0) || (num == (int *)0)) {
        return SOKOMAP_ERR_NULL;
    }

    for (i = 0u; i < (uint16_t)SOKOMAP_ARRAY_LEN; i++) {
        int new_value;
        int old_value = num[i];
        SokoMapStatus cell_status = ascii_cell_to_int(ascii[i], &new_value);

        if (cell_status != SOKOMAP_OK) {
            num[i] = new_value;
            status = cell_status;
            continue;
        }

        if ((new_value == 3) && (is_upper_int(old_value) != 0u)) {
            num[i] = old_value;
        } else if ((new_value == 4) && (is_lower_int(old_value) != 0u)) {
            num[i] = old_value;
        } else {
            num[i] = new_value;
        }
    }

    return status;
}

SokoMapStatus SokoMap_IntArrayToMap(const int num[SOKOMAP_ARRAY_LEN],
                                    int map[SOKOMAP_ROW][SOKOMAP_COL])
{
    uint16_t i;

    if ((num == (const int *)0) || (map == (int (*)[SOKOMAP_COL])0)) {
        return SOKOMAP_ERR_NULL;
    }

    for (i = 0u; i < (uint16_t)SOKOMAP_ARRAY_LEN; i++) {
        map[i / SOKOMAP_COL][i % SOKOMAP_COL] = num[i];
    }

    return SOKOMAP_OK;
}

SokoMapStatus SokoMap_MarkNearest(int map[SOKOMAP_ROW][SOKOMAP_COL], uint8_t tag)
{
    uint8_t visited[SOKOMAP_ARRAY_LEN];
    uint8_t queue[SOKOMAP_ARRAY_LEN];
    uint16_t i;
    uint16_t front;
    uint16_t rear;
    uint8_t px = 0u;
    uint8_t py = 0u;
    uint8_t found_player = 0u;
    uint8_t remain_box = 0u;
    uint8_t remain_goal = 0u;
    uint32_t upper_mask = 0u;
    uint32_t lower_mask = 0u;

    static const int8_t dr[4] = {1, -1, 0, 0};
    static const int8_t dc[4] = {0, 0, 1, -1};

    if (map == (int (*)[SOKOMAP_COL])0) {
        return SOKOMAP_ERR_NULL;
    }

    for (i = 0u; i < (uint16_t)SOKOMAP_ARRAY_LEN; i++) {
        uint8_t r = (uint8_t)(i / SOKOMAP_COL);
        uint8_t c = (uint8_t)(i % SOKOMAP_COL);
        int cell = map[r][c];

        if (cell == 2) {
            px = r;
            py = c;
            found_player = 1u;
        } else if (cell == 3) {
            if (remain_box < 255u) {
                remain_box++;
            }
        } else if (cell == 4) {
            if (remain_goal < 255u) {
                remain_goal++;
            }
        } else if ((cell >= (int)'A') && (cell <= (int)'Z')) {
            upper_mask |= (uint32_t)1u << (uint8_t)(cell - (int)'A');
        } else if ((cell >= (int)'a') && (cell <= (int)'z')) {
            lower_mask |= (uint32_t)1u << (uint8_t)(cell - (int)'a');
        } else {
            /* Other values are ignored here. */
        }
    }

    if (found_player == 0u) {
        return SOKOMAP_ERR_PLAYER_NOT_FOUND;
    }

    if ((tag == (uint8_t)'Z') || (tag == (uint8_t)'z')) {
        uint8_t changed = 0u;

        for (i = 0u; i < (uint16_t)SOKOMAP_ARRAY_LEN; i++) {
            uint8_t r = (uint8_t)(i / SOKOMAP_COL);
            uint8_t c = (uint8_t)(i % SOKOMAP_COL);

            if (map[r][c] == 3) {
                map[r][c] = (int)'A';
                changed = 1u;
            } else if (map[r][c] == 4) {
                map[r][c] = (int)'a';
                changed = 1u;
            } else {
                /* No action. */
            }
        }

        return (changed != 0u) ? SOKOMAP_OK : SOKOMAP_ERR_NO_TARGET;
    }

    if ((remain_box == 1u) && (remain_goal == 1u)) {
        uint8_t upper = 0u;
        uint8_t lower = 0u;

        for (i = 0u; i < 26u; i++) {
            uint32_t bit = (uint32_t)1u << i;

            if (((lower_mask & bit) != 0u) && ((upper_mask & bit) == 0u)) {
                upper = (uint8_t)((uint8_t)'A' + (uint8_t)i);
            }
            if (((upper_mask & bit) != 0u) && ((lower_mask & bit) == 0u)) {
                lower = (uint8_t)((uint8_t)'a' + (uint8_t)i);
            }
        }

        if ((upper == 0u) || (lower == 0u)) {
            for (i = 0u; i < 26u; i++) {
                uint32_t bit = (uint32_t)1u << i;

                if (((upper_mask & bit) == 0u) && ((lower_mask & bit) == 0u)) {
                    upper = (uint8_t)((uint8_t)'A' + (uint8_t)i);
                    lower = (uint8_t)((uint8_t)'a' + (uint8_t)i);
                    break;
                }
            }
        }

        if ((upper == 0u) || (lower == 0u)) {
            return SOKOMAP_ERR_NO_TARGET;
        }

        for (i = 0u; i < (uint16_t)SOKOMAP_ARRAY_LEN; i++) {
            uint8_t r = (uint8_t)(i / SOKOMAP_COL);
            uint8_t c = (uint8_t)(i % SOKOMAP_COL);

            if (map[r][c] == 3) {
                map[r][c] = (int)upper;
            } else if (map[r][c] == 4) {
                map[r][c] = (int)lower;
            } else {
                /* No action. */
            }
        }

        return SOKOMAP_OK;
    }

    if ((is_upper_ascii(tag) == 0u) && (is_lower_ascii(tag) == 0u)) {
        return SOKOMAP_ERR_INVALID_TAG;
    }

    if ((is_upper_ascii(tag) != 0u) && (remain_box == 0u)) {
        return SOKOMAP_ERR_NO_TARGET;
    }

    if ((is_lower_ascii(tag) != 0u) && (remain_goal == 0u)) {
        return SOKOMAP_ERR_NO_TARGET;
    }

    for (i = 0u; i < (uint16_t)SOKOMAP_ARRAY_LEN; i++) {
        visited[i] = 0u;
    }

    front = 0u;
    rear = 0u;
    queue[rear++] = CELL_INDEX(px, py);
    visited[CELL_INDEX(px, py)] = 1u;

    while (front < rear) {
        uint8_t cur = queue[front++];
        uint8_t cr = CELL_ROW(cur);
        uint8_t cc = CELL_COL(cur);
        uint8_t d;

        for (d = 0u; d < 4u; d++) {
            int16_t nr_i = (int16_t)cr + (int16_t)dr[d];
            int16_t nc_i = (int16_t)cc + (int16_t)dc[d];
            uint8_t nr;
            uint8_t nc;
            uint8_t nidx;

            if (in_map_i16(nr_i, nc_i) == 0u) {
                continue;
            }

            nr = (uint8_t)nr_i;
            nc = (uint8_t)nc_i;
            nidx = CELL_INDEX(nr, nc);

            if (visited[nidx] != 0u) {
                continue;
            }

            if (map[nr][nc] == 1) {
                continue;
            }

            visited[nidx] = 1u;

            if ((is_upper_ascii(tag) != 0u) && (map[nr][nc] == 3)) {
                map[nr][nc] = (int)tag;
                return SOKOMAP_OK;
            }

            if ((is_lower_ascii(tag) != 0u) && (map[nr][nc] == 4)) {
                map[nr][nc] = (int)tag;
                return SOKOMAP_OK;
            }

            if (rear < (uint16_t)SOKOMAP_ARRAY_LEN) {
                queue[rear++] = nidx;
            }
        }
    }

    return SOKOMAP_ERR_NO_TARGET;
}

const char *SokoMap_StatusString(SokoMapStatus status)
{
    switch (status) {
    case SOKOMAP_OK:
        return "OK";
    case SOKOMAP_ERR_NULL:
        return "NULL pointer";
    case SOKOMAP_ERR_INVALID_CHAR:
        return "Invalid input character";
    case SOKOMAP_ERR_PLAYER_NOT_FOUND:
        return "Player not found";
    case SOKOMAP_ERR_NO_TARGET:
        return "No target";
    case SOKOMAP_ERR_INVALID_TAG:
        return "Invalid tag";
    default:
        return "Unknown";
    }
}
