#include "sokoban_solver.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define CELL_COUNT          (SOKOBAN_MAP_ROW * SOKOBAN_MAP_COL)
#define DIR_STATE_COUNT     5u
#define HUMAN_STATE_COUNT   (CELL_COUNT * DIR_STATE_COUNT)
#define BOMB_STATE_COUNT    (CELL_COUNT * CELL_COUNT)
#define BOMB_VIS_WORDS      ((BOMB_STATE_COUNT + 31u) / 32u)
#define INF_U16             0xffffu
#define NODE_NONE           0xffffu

#define DIR_UP      0
#define DIR_DOWN    1
#define DIR_LEFT    2
#define DIR_RIGHT   3
#define DIR_NONE_I  4

typedef struct {
    int8_t dx;
    int8_t dy;
    char cmd;
} SokobanDir;

typedef struct {
    int8_t x;
    int8_t y;
    int8_t kind;
    char ch;
} Point;

typedef struct {
    uint32_t state_idx;
    uint16_t len;
    uint16_t turn;
    uint16_t prev_node;
    uint16_t heap_pos;
    uint8_t push_dir;
    uint8_t closed;
} BoxNode;

typedef struct {
    uint16_t state_idx;
    uint16_t prev_node;
    uint8_t push_dir;
    uint8_t last_dir;
    uint16_t len;
} BombNode;

static const SokobanDir s_dirs[4] = {
    {-1,  0, 'W'},
    { 1,  0, 'S'},
    { 0, -1, 'A'},
    { 0,  1, 'D'}
};

/* Parsed objects. */
static Point s_boxes[SOKOBAN_MAX_OBJECTS];
static Point s_dests[SOKOBAN_MAX_OBJECTS];
static Point s_bombs[SOKOBAN_MAX_OBJECTS];
static Point s_observe_targets[SOKOBAN_MAX_OBJECTS];
static Point s_player;
static uint8_t s_box_cnt;
static uint8_t s_dest_cnt;
static uint8_t s_bomb_cnt;
static uint8_t s_observe_cnt;

/* Human path search workspace, only 10*14*5 states. */
static uint8_t s_human_step[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL][DIR_STATE_COUNT];
static uint8_t s_human_turn[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL][DIR_STATE_COUNT];
static uint8_t s_human_done[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL][DIR_STATE_COUNT];
static int8_t  s_human_prev_x[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL][DIR_STATE_COUNT];
static int8_t  s_human_prev_y[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL][DIR_STATE_COUNT];
static int8_t  s_human_prev_dir[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL][DIR_STATE_COUNT];
static uint16_t s_human_heap[HUMAN_STATE_COUNT + 1u];
static uint16_t s_human_heap_size;

/* Low-memory box search: no full 98,000-state arrays. */
static BoxNode s_box_nodes[SOKOBAN_BOX_NODE_MAX];
static uint16_t s_box_node_count;
static uint16_t s_box_hash[SOKOBAN_BOX_HASH_SIZE];   /* stores node_id + 1, 0 means empty */
static uint16_t s_box_heap[SOKOBAN_BOX_NODE_MAX + 1u];
static uint16_t s_box_heap_size;

/* Bomb search: predecessor nodes instead of path per queue item. */
static BombNode s_bomb_nodes[SOKOBAN_BOMB_NODE_MAX];
static uint16_t s_bomb_queue[SOKOBAN_BOMB_NODE_MAX];
static uint16_t s_bomb_node_count;
static uint16_t s_bomb_front;
static uint16_t s_bomb_rear;
static uint32_t s_bomb_vis_bits[BOMB_VIS_WORDS];

static SokobanStatus s_last_error;

static bool in_map(int x, int y)
{
    return (x >= 0) && (x < SOKOBAN_MAP_ROW) && (y >= 0) && (y < SOKOBAN_MAP_COL);
}

static void append_char(char *dst, uint16_t *len, char c)
{
    if (*len < (uint16_t)(SOKOBAN_MAX_STEP - 1u)) {
        dst[*len] = c;
        (*len)++;
        dst[*len] = '\0';
    }
}

static void append_str(char *dst, uint16_t *len, const char *src)
{
    while ((*src != '\0') && (*len < (uint16_t)(SOKOBAN_MAX_STEP - 1u))) {
        dst[*len] = *src;
        (*len)++;
        src++;
    }
    dst[*len] = '\0';
}

static bool better_cost_u8(uint8_t len1, uint8_t turn1, uint8_t len2, uint8_t turn2)
{
    return (len1 < len2) || ((len1 == len2) && (turn1 < turn2));
}

static bool better_cost_u16(uint16_t len1, uint16_t turn1, uint16_t len2, uint16_t turn2)
{
    return (len1 < len2) || ((len1 == len2) && (turn1 < turn2));
}

static int cmd_to_dir(char c)
{
    if (c == 'W') { return DIR_UP; }
    if (c == 'S') { return DIR_DOWN; }
    if (c == 'A') { return DIR_LEFT; }
    if (c == 'D') { return DIR_RIGHT; }
    return DIR_NONE_I;
}

static int get_last_dir_from_path(const char *path, int fallback_dir)
{
    size_t len = strlen(path);
    if (len == 0u) {
        return fallback_dir;
    }
    return cmd_to_dir(path[len - 1u]);
}

static uint16_t get_turns_for_path(const char *path, int start_last_dir)
{
    uint16_t turns = 0u;
    int last_dir = start_last_dir;

    for (size_t i = 0u; path[i] != '\0'; i++) {
        int d = cmd_to_dir(path[i]);
        if ((last_dir != DIR_NONE_I) && (d != last_dir)) {
            turns++;
        }
        last_dir = d;
    }
    return turns;
}

static uint16_t cell_index(int x, int y)
{
    return (uint16_t)((x * SOKOBAN_MAP_COL) + y);
}

static void decode_cell(uint16_t c, int *x, int *y)
{
    *x = (int)(c / SOKOBAN_MAP_COL);
    *y = (int)(c % SOKOBAN_MAP_COL);
}

static uint32_t box_state_index(int px, int py, int bx, int by, int last_dir)
{
    uint32_t p = (uint32_t)cell_index(px, py);
    uint32_t b = (uint32_t)cell_index(bx, by);
    return (((p * CELL_COUNT) + b) * DIR_STATE_COUNT) + (uint32_t)last_dir;
}

static void decode_box_state(uint32_t idx, int *px, int *py, int *bx, int *by, int *last_dir)
{
    uint32_t tmp = idx;
    *last_dir = (int)(tmp % DIR_STATE_COUNT);
    tmp /= DIR_STATE_COUNT;

    uint32_t b = tmp % CELL_COUNT;
    uint32_t p = tmp / CELL_COUNT;

    *px = (int)(p / SOKOBAN_MAP_COL);
    *py = (int)(p % SOKOBAN_MAP_COL);
    *bx = (int)(b / SOKOBAN_MAP_COL);
    *by = (int)(b % SOKOBAN_MAP_COL);
}

static uint16_t bomb_state_index(int px, int py, int bx, int by)
{
    uint16_t p = cell_index(px, py);
    uint16_t b = cell_index(bx, by);
    return (uint16_t)((p * CELL_COUNT) + b);
}

static void decode_bomb_state(uint16_t idx, int *px, int *py, int *bx, int *by)
{
    uint16_t b = (uint16_t)(idx % CELL_COUNT);
    uint16_t p = (uint16_t)(idx / CELL_COUNT);
    decode_cell(p, px, py);
    decode_cell(b, bx, by);
}

static bool occupied_by_other_object(int x, int y, int ignore_x, int ignore_y)
{
    uint8_t i;

    for (i = 0u; i < s_box_cnt; i++) {
        if ((s_boxes[i].x == ignore_x) && (s_boxes[i].y == ignore_y)) {
            continue;
        }
        if ((s_boxes[i].x == x) && (s_boxes[i].y == y)) {
            return true;
        }
    }

    for (i = 0u; i < s_bomb_cnt; i++) {
        if ((s_bombs[i].x == ignore_x) && (s_bombs[i].y == ignore_y)) {
            continue;
        }
        if ((s_bombs[i].x == x) && (s_bombs[i].y == y)) {
            return true;
        }
    }

    for (i = 0u; i < s_observe_cnt; i++) {
        if (s_observe_targets[i].kind != 3) {
            continue;
        }
        if ((s_observe_targets[i].x == ignore_x) && (s_observe_targets[i].y == ignore_y)) {
            continue;
        }
        if ((s_observe_targets[i].x == x) && (s_observe_targets[i].y == y)) {
            return true;
        }
    }

    return false;
}

static bool can_go(int game_map[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL],
                   int x, int y,
                   int block_x, int block_y,
                   int ignore_x, int ignore_y)
{
    if (!in_map(x, y)) {
        return false;
    }
    if (game_map[x][y] == 1) {
        return false;
    }
    if ((x == block_x) && (y == block_y)) {
        return false;
    }
    if (occupied_by_other_object(x, y, ignore_x, ignore_y)) {
        return false;
    }
    return true;
}

/* ---------------- Human Dijkstra over tiny state space ---------------- */
static bool human_heap_better(uint16_t idx_a, uint16_t idx_b)
{
    uint16_t ax = (uint16_t)(idx_a / (SOKOBAN_MAP_COL * DIR_STATE_COUNT));
    uint16_t rem_a = (uint16_t)(idx_a % (SOKOBAN_MAP_COL * DIR_STATE_COUNT));
    uint16_t ay = (uint16_t)(rem_a / DIR_STATE_COUNT);
    uint16_t ad = (uint16_t)(rem_a % DIR_STATE_COUNT);

    uint16_t bx = (uint16_t)(idx_b / (SOKOBAN_MAP_COL * DIR_STATE_COUNT));
    uint16_t rem_b = (uint16_t)(idx_b % (SOKOBAN_MAP_COL * DIR_STATE_COUNT));
    uint16_t by = (uint16_t)(rem_b / DIR_STATE_COUNT);
    uint16_t bd = (uint16_t)(rem_b % DIR_STATE_COUNT);

    return better_cost_u8(s_human_step[ax][ay][ad], s_human_turn[ax][ay][ad],
                          s_human_step[bx][by][bd], s_human_turn[bx][by][bd]);
}

static void human_heap_swap(uint16_t a, uint16_t b)
{
    uint16_t t = s_human_heap[a];
    s_human_heap[a] = s_human_heap[b];
    s_human_heap[b] = t;
}

static bool human_heap_push(uint16_t idx)
{
    if (s_human_heap_size >= HUMAN_STATE_COUNT) {
        s_last_error = SOKOBAN_ERR_SEARCH_OVERFLOW;
        return false;
    }

    s_human_heap_size++;
    s_human_heap[s_human_heap_size] = idx;

    uint16_t cur = s_human_heap_size;
    while (cur > 1u) {
        uint16_t parent = (uint16_t)(cur / 2u);
        if (human_heap_better(s_human_heap[cur], s_human_heap[parent])) {
            human_heap_swap(cur, parent);
            cur = parent;
        } else {
            break;
        }
    }
    return true;
}

static int32_t human_heap_pop(void)
{
    if (s_human_heap_size == 0u) {
        return -1;
    }

    uint16_t ret = s_human_heap[1];
    s_human_heap[1] = s_human_heap[s_human_heap_size];
    s_human_heap_size--;

    uint16_t cur = 1u;
    while (1) {
        uint16_t left = (uint16_t)(cur * 2u);
        uint16_t right = (uint16_t)(left + 1u);
        uint16_t best = cur;

        if ((left <= s_human_heap_size) && human_heap_better(s_human_heap[left], s_human_heap[best])) {
            best = left;
        }
        if ((right <= s_human_heap_size) && human_heap_better(s_human_heap[right], s_human_heap[best])) {
            best = right;
        }
        if (best == cur) {
            break;
        }
        human_heap_swap(cur, best);
        cur = best;
    }

    return (int32_t)ret;
}

static bool bfs_human_pref(int game_map[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL],
                           int sx, int sy, int tx, int ty,
                           int block_x, int block_y,
                           int ignore_x, int ignore_y,
                           int start_last_dir,
                           char *path)
{
    path[0] = '\0';
    if (!in_map(sx, sy) || !in_map(tx, ty) ||
        (start_last_dir < 0) || (start_last_dir >= (int)DIR_STATE_COUNT)) {
        return false;
    }

    for (int i = 0; i < SOKOBAN_MAP_ROW; i++) {
        for (int j = 0; j < SOKOBAN_MAP_COL; j++) {
            for (int d = 0; d < (int)DIR_STATE_COUNT; d++) {
                s_human_step[i][j][d] = 0xffu;
                s_human_turn[i][j][d] = 0xffu;
                s_human_done[i][j][d] = 0u;
                s_human_prev_x[i][j][d] = -1;
                s_human_prev_y[i][j][d] = -1;
                s_human_prev_dir[i][j][d] = -1;
            }
        }
    }

    s_human_heap_size = 0u;
    s_human_step[sx][sy][start_last_dir] = 0u;
    s_human_turn[sx][sy][start_last_dir] = 0u;

    if (!human_heap_push((uint16_t)(((sx * SOKOBAN_MAP_COL) + sy) * (int)DIR_STATE_COUNT + start_last_dir))) {
        return false;
    }

    while (s_human_heap_size > 0u) {
        int32_t popped = human_heap_pop();
        if (popped < 0) {
            break;
        }

        uint16_t cur_idx = (uint16_t)popped;
        int cur_x = (int)(cur_idx / (SOKOBAN_MAP_COL * DIR_STATE_COUNT));
        uint16_t rem = (uint16_t)(cur_idx % (SOKOBAN_MAP_COL * DIR_STATE_COUNT));
        int cur_y = (int)(rem / DIR_STATE_COUNT);
        int cur_dir = (int)(rem % DIR_STATE_COUNT);

        if (s_human_done[cur_x][cur_y][cur_dir] != 0u) {
            continue;
        }
        s_human_done[cur_x][cur_y][cur_dir] = 1u;

        if ((cur_x == tx) && (cur_y == ty)) {
            char rev[SOKOBAN_MAX_STEP];
            uint16_t len = 0u;
            int x = cur_x;
            int y = cur_y;
            int d = cur_dir;

            while (!((x == sx) && (y == sy) && (d == start_last_dir))) {
                if ((d < 0) || (d >= 4)) {
                    return false;
                }
                if (len < (uint16_t)(SOKOBAN_MAX_STEP - 1u)) {
                    rev[len] = s_dirs[d].cmd;
                    len++;
                }
                int px = s_human_prev_x[x][y][d];
                int py = s_human_prev_y[x][y][d];
                int pd = s_human_prev_dir[x][y][d];
                if ((px < 0) || (py < 0) || (pd < 0)) {
                    return false;
                }
                x = px;
                y = py;
                d = pd;
            }

            for (uint16_t i = 0u; i < len; i++) {
                path[i] = rev[len - 1u - i];
            }
            path[len] = '\0';
            return true;
        }

        for (int nd = 0; nd < 4; nd++) {
            int nx = cur_x + s_dirs[nd].dx;
            int ny = cur_y + s_dirs[nd].dy;

            if (!can_go(game_map, nx, ny, block_x, block_y, ignore_x, ignore_y)) {
                continue;
            }

            uint16_t raw_step = (uint16_t)s_human_step[cur_x][cur_y][cur_dir] + 1u;
            if (raw_step > 254u) {
                continue;
            }
            uint8_t nstep = (uint8_t)raw_step;
            uint8_t nturn = s_human_turn[cur_x][cur_y][cur_dir];
            if ((cur_dir != DIR_NONE_I) && (cur_dir != nd) && (nturn < 254u)) {
                nturn++;
            }

            if (better_cost_u8(nstep, nturn, s_human_step[nx][ny][nd], s_human_turn[nx][ny][nd])) {
                s_human_step[nx][ny][nd] = nstep;
                s_human_turn[nx][ny][nd] = nturn;
                s_human_prev_x[nx][ny][nd] = (int8_t)cur_x;
                s_human_prev_y[nx][ny][nd] = (int8_t)cur_y;
                s_human_prev_dir[nx][ny][nd] = (int8_t)cur_dir;
                if (!human_heap_push((uint16_t)(((nx * SOKOBAN_MAP_COL) + ny) * (int)DIR_STATE_COUNT + nd))) {
                    return false;
                }
            }
        }
    }

    return false;
}

/* ---------------- Box low-memory Dijkstra ---------------- */
static uint16_t box_hash_slot(uint32_t idx, uint16_t probe)
{
    uint32_t h = idx * 2654435761u;
    return (uint16_t)((h + probe) % (uint32_t)SOKOBAN_BOX_HASH_SIZE);
}

static bool box_find_node(uint32_t state_idx, uint16_t *node_id)
{
    for (uint16_t p = 0u; p < (uint16_t)SOKOBAN_BOX_HASH_SIZE; p++) {
        uint16_t slot = box_hash_slot(state_idx, p);
        uint16_t stored = s_box_hash[slot];
        if (stored == 0u) {
            return false;
        }
        uint16_t id = (uint16_t)(stored - 1u);
        if (s_box_nodes[id].state_idx == state_idx) {
            *node_id = id;
            return true;
        }
    }
    return false;
}

static bool box_insert_hash(uint32_t state_idx, uint16_t node_id)
{
    for (uint16_t p = 0u; p < (uint16_t)SOKOBAN_BOX_HASH_SIZE; p++) {
        uint16_t slot = box_hash_slot(state_idx, p);
        if (s_box_hash[slot] == 0u) {
            s_box_hash[slot] = (uint16_t)(node_id + 1u);
            return true;
        }
    }
    s_last_error = SOKOBAN_ERR_SEARCH_OVERFLOW;
    return false;
}

static bool box_node_better(uint16_t node_a, uint16_t node_b)
{
    return better_cost_u16(s_box_nodes[node_a].len, s_box_nodes[node_a].turn,
                           s_box_nodes[node_b].len, s_box_nodes[node_b].turn);
}

static void box_heap_set(uint16_t pos, uint16_t node_id)
{
    s_box_heap[pos] = node_id;
    s_box_nodes[node_id].heap_pos = pos;
}

static void box_heap_swap(uint16_t a, uint16_t b)
{
    uint16_t na = s_box_heap[a];
    uint16_t nb = s_box_heap[b];
    box_heap_set(a, nb);
    box_heap_set(b, na);
}

static void box_heap_sift_up(uint16_t pos)
{
    while (pos > 1u) {
        uint16_t parent = (uint16_t)(pos / 2u);
        if (box_node_better(s_box_heap[pos], s_box_heap[parent])) {
            box_heap_swap(pos, parent);
            pos = parent;
        } else {
            break;
        }
    }
}

static void box_heap_sift_down(uint16_t pos)
{
    while (1) {
        uint16_t left = (uint16_t)(pos * 2u);
        uint16_t right = (uint16_t)(left + 1u);
        uint16_t best = pos;

        if ((left <= s_box_heap_size) && box_node_better(s_box_heap[left], s_box_heap[best])) {
            best = left;
        }
        if ((right <= s_box_heap_size) && box_node_better(s_box_heap[right], s_box_heap[best])) {
            best = right;
        }
        if (best == pos) {
            break;
        }
        box_heap_swap(pos, best);
        pos = best;
    }
}

static bool box_heap_push_or_fix(uint16_t node_id)
{
    if (s_box_nodes[node_id].heap_pos != 0u) {
        box_heap_sift_up(s_box_nodes[node_id].heap_pos);
        return true;
    }

    if (s_box_heap_size >= (uint16_t)SOKOBAN_BOX_NODE_MAX) {
        s_last_error = SOKOBAN_ERR_SEARCH_OVERFLOW;
        return false;
    }

    s_box_heap_size++;
    box_heap_set(s_box_heap_size, node_id);
    box_heap_sift_up(s_box_heap_size);
    return true;
}

static int32_t box_heap_pop(void)
{
    if (s_box_heap_size == 0u) {
        return -1;
    }

    uint16_t ret = s_box_heap[1];
    s_box_nodes[ret].heap_pos = 0u;

    if (s_box_heap_size > 1u) {
        uint16_t last = s_box_heap[s_box_heap_size];
        s_box_heap_size--;
        box_heap_set(1u, last);
        box_heap_sift_down(1u);
    } else {
        s_box_heap_size = 0u;
    }

    return (int32_t)ret;
}

static bool box_new_node(uint32_t state_idx, uint16_t len, uint16_t turn,
                         uint16_t prev, uint8_t push_dir, uint16_t *node_id)
{
    if (s_box_node_count >= (uint16_t)SOKOBAN_BOX_NODE_MAX) {
        s_last_error = SOKOBAN_ERR_SEARCH_OVERFLOW;
        return false;
    }

    uint16_t id = s_box_node_count;
    s_box_node_count++;

    s_box_nodes[id].state_idx = state_idx;
    s_box_nodes[id].len = len;
    s_box_nodes[id].turn = turn;
    s_box_nodes[id].prev_node = prev;
    s_box_nodes[id].heap_pos = 0u;
    s_box_nodes[id].push_dir = push_dir;
    s_box_nodes[id].closed = 0u;

    if (!box_insert_hash(state_idx, id)) {
        return false;
    }

    *node_id = id;
    return true;
}

static bool reconstruct_box_path(int game_map[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL],
                                 Point p_start,
                                 Point b_start,
                                 uint16_t end_node,
                                 char *out_path)
{
    uint16_t chain[SOKOBAN_MAX_STEP];
    uint16_t count = 0u;
    uint16_t cur = end_node;

    while (s_box_nodes[cur].prev_node != NODE_NONE) {
        if (count >= (uint16_t)SOKOBAN_MAX_STEP) {
            return false;
        }
        chain[count] = cur;
        count++;
        cur = s_box_nodes[cur].prev_node;
        if (cur >= s_box_node_count) {
            return false;
        }
    }

    out_path[0] = '\0';
    uint16_t out_len = 0u;
    int cur_px = p_start.x;
    int cur_py = p_start.y;
    int cur_bx = b_start.x;
    int cur_by = b_start.y;
    int last_dir = DIR_NONE_I;

    while (count > 0u) {
        count--;
        uint16_t next_node = chain[count];
        int d = s_box_nodes[next_node].push_dir;
        int p_need_x = cur_bx - s_dirs[d].dx;
        int p_need_y = cur_by - s_dirs[d].dy;
        char move_path[SOKOBAN_MAX_STEP];

        if (!bfs_human_pref(game_map,
                            cur_px, cur_py,
                            p_need_x, p_need_y,
                            cur_bx, cur_by,
                            b_start.x, b_start.y,
                            last_dir,
                            move_path)) {
            return false;
        }

        append_str(out_path, &out_len, move_path);
        append_char(out_path, &out_len, s_dirs[d].cmd);
        if (out_len >= (uint16_t)(SOKOBAN_MAX_STEP - 1u)) {
            return false;
        }

        cur_px = cur_bx;
        cur_py = cur_by;
        cur_bx += s_dirs[d].dx;
        cur_by += s_dirs[d].dy;
        last_dir = d;
    }

    return true;
}

static bool bfs_box(int game_map[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL],
                    Point p_start,
                    Point b_start,
                    Point dest,
                    Point *final_player,
                    Point *final_box,
                    int *final_last_dir,
                    char *out_path)
{
    memset(s_box_hash, 0, sizeof(s_box_hash));
    s_box_node_count = 0u;
    s_box_heap_size = 0u;

    uint32_t start_state = box_state_index(p_start.x, p_start.y, b_start.x, b_start.y, DIR_NONE_I);
    uint16_t start_node;
    if (!box_new_node(start_state, 0u, 0u, NODE_NONE, DIR_NONE_I, &start_node)) {
        return false;
    }
    if (!box_heap_push_or_fix(start_node)) {
        return false;
    }

    while (s_box_heap_size > 0u) {
        int32_t popped = box_heap_pop();
        if (popped < 0) {
            break;
        }

        uint16_t cur_node = (uint16_t)popped;
        if (s_box_nodes[cur_node].closed != 0u) {
            continue;
        }
        s_box_nodes[cur_node].closed = 1u;

        int cur_px;
        int cur_py;
        int cur_bx;
        int cur_by;
        int cur_last_dir;
        decode_box_state(s_box_nodes[cur_node].state_idx, &cur_px, &cur_py, &cur_bx, &cur_by, &cur_last_dir);

        if ((cur_bx == dest.x) && (cur_by == dest.y)) {
            if (!reconstruct_box_path(game_map, p_start, b_start, cur_node, out_path)) {
                s_last_error = SOKOBAN_ERR_INTERNAL;
                return false;
            }
            if (final_player != NULL) {
                final_player->x = (int8_t)cur_px;
                final_player->y = (int8_t)cur_py;
            }
            if (final_box != NULL) {
                final_box->x = (int8_t)cur_bx;
                final_box->y = (int8_t)cur_by;
            }
            if (final_last_dir != NULL) {
                *final_last_dir = cur_last_dir;
            }
            return true;
        }

        for (int d = 0; d < 4; d++) {
            int nbx = cur_bx + s_dirs[d].dx;
            int nby = cur_by + s_dirs[d].dy;
            int p_need_x = cur_bx - s_dirs[d].dx;
            int p_need_y = cur_by - s_dirs[d].dy;
            char move_path[SOKOBAN_MAX_STEP];

            if (!can_go(game_map, nbx, nby, -1, -1, b_start.x, b_start.y)) {
                continue;
            }
            if (!can_go(game_map, p_need_x, p_need_y, cur_bx, cur_by, b_start.x, b_start.y)) {
                continue;
            }

            if (!bfs_human_pref(game_map,
                                cur_px, cur_py,
                                p_need_x, p_need_y,
                                cur_bx, cur_by,
                                b_start.x, b_start.y,
                                cur_last_dir,
                                move_path)) {
                if (s_last_error == SOKOBAN_ERR_SEARCH_OVERFLOW) {
                    return false;
                }
                continue;
            }

            uint16_t move_len = (uint16_t)strlen(move_path);
            uint16_t move_turns = get_turns_for_path(move_path, cur_last_dir);
            int last_before_push = get_last_dir_from_path(move_path, cur_last_dir);
            uint16_t push_turn = ((last_before_push != DIR_NONE_I) && (last_before_push != d)) ? 1u : 0u;

            uint32_t raw_next_len = (uint32_t)s_box_nodes[cur_node].len + move_len + 1u;
            if (raw_next_len >= SOKOBAN_MAX_STEP) {
                continue;
            }
            uint16_t next_len = (uint16_t)raw_next_len;
            uint16_t next_turn = (uint16_t)(s_box_nodes[cur_node].turn + move_turns + push_turn);
            uint32_t next_state = box_state_index(cur_bx, cur_by, nbx, nby, d);
            uint16_t next_node;

            if (box_find_node(next_state, &next_node)) {
                if ((s_box_nodes[next_node].closed == 0u) &&
                    better_cost_u16(next_len, next_turn,
                                    s_box_nodes[next_node].len,
                                    s_box_nodes[next_node].turn)) {
                    s_box_nodes[next_node].len = next_len;
                    s_box_nodes[next_node].turn = next_turn;
                    s_box_nodes[next_node].prev_node = cur_node;
                    s_box_nodes[next_node].push_dir = (uint8_t)d;
                    if (!box_heap_push_or_fix(next_node)) {
                        return false;
                    }
                }
            } else {
                if (!box_new_node(next_state, next_len, next_turn, cur_node, (uint8_t)d, &next_node)) {
                    return false;
                }
                if (!box_heap_push_or_fix(next_node)) {
                    return false;
                }
            }
        }
    }

    return false;
}

/* ---------------- Bomb low-memory BFS ---------------- */
static void bomb_vis_clear(void)
{
    memset(s_bomb_vis_bits, 0, sizeof(s_bomb_vis_bits));
}

static bool bomb_vis_get(uint16_t idx)
{
    return (s_bomb_vis_bits[idx >> 5] & (1u << (idx & 31u))) != 0u;
}

static void bomb_vis_set(uint16_t idx)
{
    s_bomb_vis_bits[idx >> 5] |= (1u << (idx & 31u));
}

static bool bomb_add_node(uint16_t state_idx, uint16_t prev_node, uint8_t push_dir,
                          uint8_t last_dir, uint16_t len, uint16_t *node_id)
{
    if (s_bomb_node_count >= (uint16_t)SOKOBAN_BOMB_NODE_MAX) {
        s_last_error = SOKOBAN_ERR_SEARCH_OVERFLOW;
        return false;
    }
    uint16_t id = s_bomb_node_count;
    s_bomb_node_count++;
    s_bomb_nodes[id].state_idx = state_idx;
    s_bomb_nodes[id].prev_node = prev_node;
    s_bomb_nodes[id].push_dir = push_dir;
    s_bomb_nodes[id].last_dir = last_dir;
    s_bomb_nodes[id].len = len;
    *node_id = id;
    return true;
}

static bool bomb_enqueue(uint16_t node_id)
{
    if (s_bomb_rear >= (uint16_t)SOKOBAN_BOMB_NODE_MAX) {
        s_last_error = SOKOBAN_ERR_SEARCH_OVERFLOW;
        return false;
    }
    s_bomb_queue[s_bomb_rear] = node_id;
    s_bomb_rear++;
    return true;
}

static bool reconstruct_bomb_path(int game_map[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL],
                                  Point p_start,
                                  Point bomb_start,
                                  uint16_t end_node,
                                  char *out_path)
{
    uint16_t chain[SOKOBAN_MAX_STEP];
    uint16_t count = 0u;
    uint16_t cur = end_node;

    while (s_bomb_nodes[cur].prev_node != NODE_NONE) {
        if (count >= (uint16_t)SOKOBAN_MAX_STEP) {
            return false;
        }
        chain[count] = cur;
        count++;
        cur = s_bomb_nodes[cur].prev_node;
        if (cur >= s_bomb_node_count) {
            return false;
        }
    }

    out_path[0] = '\0';
    uint16_t out_len = 0u;
    int cur_px = p_start.x;
    int cur_py = p_start.y;
    int cur_bx = bomb_start.x;
    int cur_by = bomb_start.y;
    int last_dir = DIR_NONE_I;

    while (count > 0u) {
        count--;
        uint16_t next_node = chain[count];
        int d = s_bomb_nodes[next_node].push_dir;
        int p_need_x = cur_bx - s_dirs[d].dx;
        int p_need_y = cur_by - s_dirs[d].dy;
        char move_path[SOKOBAN_MAX_STEP];

        if (!bfs_human_pref(game_map,
                            cur_px, cur_py,
                            p_need_x, p_need_y,
                            cur_bx, cur_by,
                            bomb_start.x, bomb_start.y,
                            last_dir,
                            move_path)) {
            return false;
        }
        append_str(out_path, &out_len, move_path);
        append_char(out_path, &out_len, s_dirs[d].cmd);
        if (out_len >= (uint16_t)(SOKOBAN_MAX_STEP - 1u)) {
            return false;
        }

        cur_px = cur_bx;
        cur_py = cur_by;
        cur_bx += s_dirs[d].dx;
        cur_by += s_dirs[d].dy;
        last_dir = d;
    }

    return true;
}

static bool bfs_bomb(int game_map[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL],
                     Point p_start,
                     Point bomb_start,
                     Point *explode_center,
                     char *out_path)
{
    s_bomb_node_count = 0u;
    s_bomb_front = 0u;
    s_bomb_rear = 0u;
    bomb_vis_clear();
    out_path[0] = '\0';

    uint16_t start_state = bomb_state_index(p_start.x, p_start.y, bomb_start.x, bomb_start.y);
    uint16_t start_node;
    if (!bomb_add_node(start_state, NODE_NONE, DIR_NONE_I, DIR_NONE_I, 0u, &start_node)) {
        return false;
    }
    bomb_vis_set(start_state);
    if (!bomb_enqueue(start_node)) {
        return false;
    }

    while (s_bomb_front < s_bomb_rear) {
        uint16_t cur_node = s_bomb_queue[s_bomb_front];
        s_bomb_front++;

        int cur_px;
        int cur_py;
        int cur_bx;
        int cur_by;
        decode_bomb_state(s_bomb_nodes[cur_node].state_idx, &cur_px, &cur_py, &cur_bx, &cur_by);

        for (int d = 0; d < 4; d++) {
            int front_x = cur_bx + s_dirs[d].dx;
            int front_y = cur_by + s_dirs[d].dy;
            int p_need_x = cur_bx - s_dirs[d].dx;
            int p_need_y = cur_by - s_dirs[d].dy;
            char move_path[SOKOBAN_MAX_STEP];

            if (!can_go(game_map, p_need_x, p_need_y, cur_bx, cur_by, bomb_start.x, bomb_start.y)) {
                continue;
            }
            if (!bfs_human_pref(game_map,
                                cur_px, cur_py,
                                p_need_x, p_need_y,
                                cur_bx, cur_by,
                                bomb_start.x, bomb_start.y,
                                s_bomb_nodes[cur_node].last_dir,
                                move_path)) {
                if (s_last_error == SOKOBAN_ERR_SEARCH_OVERFLOW) {
                    return false;
                }
                continue;
            }

            uint16_t move_len = (uint16_t)strlen(move_path);
            uint32_t raw_len = (uint32_t)s_bomb_nodes[cur_node].len + move_len + 1u;
            if (raw_len >= SOKOBAN_MAX_STEP) {
                continue;
            }

            if (in_map(front_x, front_y) && (game_map[front_x][front_y] == 1)) {
                uint16_t out_len;
                if (!reconstruct_bomb_path(game_map, p_start, bomb_start, cur_node, out_path)) {
                    s_last_error = SOKOBAN_ERR_INTERNAL;
                    return false;
                }
                out_len = (uint16_t)strlen(out_path);
                append_str(out_path, &out_len, move_path);
                append_char(out_path, &out_len, s_dirs[d].cmd);
                explode_center->x = (int8_t)front_x;
                explode_center->y = (int8_t)front_y;
                explode_center->kind = 5;
                explode_center->ch = '5';
                return true;
            }

            if (!can_go(game_map, front_x, front_y, -1, -1, bomb_start.x, bomb_start.y)) {
                continue;
            }

            uint16_t next_state = bomb_state_index(cur_bx, cur_by, front_x, front_y);
            if (!bomb_vis_get(next_state)) {
                uint16_t next_node;
                if (!bomb_add_node(next_state, cur_node, (uint8_t)d, (uint8_t)d, (uint16_t)raw_len, &next_node)) {
                    return false;
                }
                bomb_vis_set(next_state);
                if (!bomb_enqueue(next_node)) {
                    return false;
                }
            }
        }
    }

    return false;
}

static void explode_wall_3x3(int game_map[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL], int cx, int cy)
{
    for (int i = cx - 1; i <= cx + 1; i++) {
        for (int j = cy - 1; j <= cy + 1; j++) {
            if (in_map(i, j) && (game_map[i][j] == 1)) {
                game_map[i][j] = 0;
            }
        }
    }
}

static char get_observe_code(int px, int py, int tx, int ty)
{
    if ((px == tx) && (py == ty - 1)) { return '1'; }
    if ((px == tx) && (py == ty + 1)) { return '2'; }
    if ((px == tx - 1) && (py == ty)) { return '3'; }
    if ((px == tx + 1) && (py == ty)) { return '4'; }
    return '0';
}

static bool observe_one(int game_map[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL],
                        Point p_start,
                        char *out_path)
{
    static const int8_t offx[4] = {0, 0, -1, 1};
    static const int8_t offy[4] = {-1, 1, 0, 0};
    out_path[0] = '\0';

    for (uint8_t i = 0u; i < s_observe_cnt; i++) {
        Point target = s_observe_targets[i];
        for (int k = 0; k < 4; k++) {
            int tx = target.x + offx[k];
            int ty = target.y + offy[k];
            char path[SOKOBAN_MAX_STEP];

            if (!can_go(game_map, tx, ty, -1, -1, -1, -1)) {
                continue;
            }
            if (bfs_human_pref(game_map, p_start.x, p_start.y, tx, ty,
                               -1, -1, -1, -1, DIR_NONE_I, path)) {
                uint16_t len = 0u;
                append_str(out_path, &len, path);
                append_char(out_path, &len, get_observe_code(tx, ty, target.x, target.y));
                return true;
            }
            if (s_last_error == SOKOBAN_ERR_SEARCH_OVERFLOW) {
                return false;
            }
        }
    }

    return false;
}

static bool go_to_nearest_finish(int game_map[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL],
                                 Point p_start,
                                 char *out_path)
{
    Point candidates[4] = {
        {(int8_t)(SOKOBAN_MAP_ROW / 2 - 1), 0, 0, 0},
        {(int8_t)(SOKOBAN_MAP_ROW / 2),     0, 0, 0},
        {(int8_t)(SOKOBAN_MAP_ROW / 2 - 1), (int8_t)(SOKOBAN_MAP_COL - 1), 0, 0},
        {(int8_t)(SOKOBAN_MAP_ROW / 2),     (int8_t)(SOKOBAN_MAP_COL - 1), 0, 0}
    };
    uint16_t best_len = INF_U16;
    uint16_t best_turn = INF_U16;
    bool found = false;
    out_path[0] = '\0';

    for (int i = 0; i < 4; i++) {
        int tx = candidates[i].x;
        int ty = candidates[i].y;
        char path[SOKOBAN_MAX_STEP];

        if (!can_go(game_map, tx, ty, -1, -1, -1, -1)) {
            continue;
        }
        if (bfs_human_pref(game_map, p_start.x, p_start.y, tx, ty,
                           -1, -1, -1, -1, DIR_NONE_I, path)) {
            uint16_t len = (uint16_t)strlen(path);
            uint16_t turns = get_turns_for_path(path, DIR_NONE_I);
            if (better_cost_u16(len, turns, best_len, best_turn)) {
                best_len = len;
                best_turn = turns;
                strcpy(out_path, path);
                found = true;
            }
        } else if (s_last_error == SOKOBAN_ERR_SEARCH_OVERFLOW) {
            return false;
        }
    }

    return found;
}

static bool is_other_unfinished_dest(int x, int y, char finished_dest_char)
{
    for (uint8_t i = 0u; i < s_dest_cnt; i++) {
        if (s_dests[i].ch == finished_dest_char) {
            continue;
        }
        if ((s_dests[i].x == x) && (s_dests[i].y == y)) {
            return true;
        }
    }
    return false;
}

static bool is_any_unfinished_dest(int x, int y, char finished_dest_char)
{
    for (uint8_t i = 0u; i < s_dest_cnt; i++) {
        if (s_dests[i].ch == finished_dest_char) {
            continue;
        }
        if ((s_dests[i].x == x) && (s_dests[i].y == y)) {
            return true;
        }
    }
    return false;
}

static void build_step_aside_order(int last_dir, int order[4])
{
    if ((last_dir == DIR_UP) || (last_dir == DIR_DOWN)) {
        order[0] = DIR_LEFT;
        order[1] = DIR_RIGHT;
        order[2] = last_dir;
        order[3] = (last_dir == DIR_UP) ? DIR_DOWN : DIR_UP;
        return;
    }
    if ((last_dir == DIR_LEFT) || (last_dir == DIR_RIGHT)) {
        order[0] = DIR_UP;
        order[1] = DIR_DOWN;
        order[2] = last_dir;
        order[3] = (last_dir == DIR_LEFT) ? DIR_RIGHT : DIR_LEFT;
        return;
    }
    order[0] = DIR_LEFT;
    order[1] = DIR_RIGHT;
    order[2] = DIR_UP;
    order[3] = DIR_DOWN;
}

static void step_aside_if_on_other_dest(int game_map[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL],
                                        Point player_pos,
                                        Point settled_box,
                                        char finished_dest_char,
                                        int last_dir,
                                        char *out_path)
{
    out_path[0] = '\0';
    if (!is_other_unfinished_dest(player_pos.x, player_pos.y, finished_dest_char)) {
        return;
    }

    int order[4];
    build_step_aside_order(last_dir, order);
    for (int i = 0; i < 4; i++) {
        int d = order[i];
        int nx = player_pos.x + s_dirs[d].dx;
        int ny = player_pos.y + s_dirs[d].dy;
        if (!can_go(game_map, nx, ny, settled_box.x, settled_box.y, -1, -1)) {
            continue;
        }
        if (is_any_unfinished_dest(nx, ny, finished_dest_char)) {
            continue;
        }
        out_path[0] = s_dirs[d].cmd;
        out_path[1] = '\0';
        return;
    }
}

static SokobanStatus add_point(Point *array, uint8_t *count, Point value)
{
    if (*count >= (uint8_t)SOKOBAN_MAX_OBJECTS) {
        return SOKOBAN_ERR_TOO_MANY_OBJECTS;
    }
    array[*count] = value;
    (*count)++;
    return SOKOBAN_OK;
}

static SokobanStatus parse_map(int game_map[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL])
{
    s_box_cnt = 0u;
    s_dest_cnt = 0u;
    s_bomb_cnt = 0u;
    s_observe_cnt = 0u;
    s_player.x = -1;
    s_player.y = -1;

    for (int i = 0; i < SOKOBAN_MAP_ROW; i++) {
        for (int j = 0; j < SOKOBAN_MAP_COL; j++) {
            int val = game_map[i][j];
            SokobanStatus st;

            if (val == 2) {
                s_player.x = (int8_t)i;
                s_player.y = (int8_t)j;
            } else if (val == 5) {
                st = add_point(s_bombs, &s_bomb_cnt, (Point){(int8_t)i, (int8_t)j, 5, '5'});
                if (st != SOKOBAN_OK) { return st; }
            } else if (val == 3) {
                st = add_point(s_observe_targets, &s_observe_cnt, (Point){(int8_t)i, (int8_t)j, 3, '3'});
                if (st != SOKOBAN_OK) { return st; }
            } else if (val == 4) {
                st = add_point(s_observe_targets, &s_observe_cnt, (Point){(int8_t)i, (int8_t)j, 4, '4'});
                if (st != SOKOBAN_OK) { return st; }
            } else if ((val >= 'A') && (val <= 'Z')) {
                st = add_point(s_boxes, &s_box_cnt, (Point){(int8_t)i, (int8_t)j, 100, (char)val});
                if (st != SOKOBAN_OK) { return st; }
            } else if ((val >= 'a') && (val <= 'z')) {
                st = add_point(s_dests, &s_dest_cnt, (Point){(int8_t)i, (int8_t)j, 101, (char)val});
                if (st != SOKOBAN_OK) { return st; }
            }
        }
    }

    if (!in_map(s_player.x, s_player.y)) {
        return SOKOBAN_ERR_INVALID_ARGUMENT;
    }

    return SOKOBAN_OK;
}

static SokobanStatus solve_internal(int game_map[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL], char *total)
{
    SokobanStatus st = parse_map(game_map);
    total[0] = '\0';
    if (st != SOKOBAN_OK) {
        return st;
    }

    if ((s_box_cnt == 0u) && (s_dest_cnt == 0u)) {
        if (go_to_nearest_finish(game_map, s_player, total)) {
            return SOKOBAN_OK;
        }
        return (s_last_error == SOKOBAN_ERR_SEARCH_OVERFLOW) ? s_last_error : SOKOBAN_NOT_FOUND;
    }

    for (uint8_t i = 0u; i < s_bomb_cnt; i++) {
        Point center;
        char bomb_path[SOKOBAN_MAX_STEP];
        if (bfs_bomb(game_map, s_player, s_bombs[i], &center, bomb_path)) {
            if (bomb_path[0] != '\0') {
                explode_wall_3x3(game_map, center.x, center.y);
                strcpy(total, bomb_path);
                return SOKOBAN_OK;
            }
        } else if ((s_last_error == SOKOBAN_ERR_SEARCH_OVERFLOW) ||
                   (s_last_error == SOKOBAN_ERR_INTERNAL)) {
            return s_last_error;
        }
    }

    if ((s_bomb_cnt == 0u) && (s_observe_cnt > 0u)) {
        char obs_path[SOKOBAN_MAX_STEP];
        if (observe_one(game_map, s_player, obs_path)) {
            if (obs_path[0] != '\0') {
                strcpy(total, obs_path);
                return SOKOBAN_OK;
            }
        } else if (s_last_error == SOKOBAN_ERR_SEARCH_OVERFLOW) {
            return s_last_error;
        }
    }

    for (uint8_t b = 0u; b < s_box_cnt; b++) {
        char box_char = s_boxes[b].ch;
        char target_char = (char)(box_char + 32);

        for (uint8_t t = 0u; t < s_dest_cnt; t++) {
            if (s_dests[t].ch == target_char) {
                Point final_player = {-1, -1, 0, 0};
                Point final_box = {-1, -1, 0, 0};
                int final_last_dir = DIR_NONE_I;
                char path[SOKOBAN_MAX_STEP];

                if (bfs_box(game_map,
                            s_player,
                            s_boxes[b],
                            s_dests[t],
                            &final_player,
                            &final_box,
                            &final_last_dir,
                            path)) {
                    if (path[0] != '\0') {
                        char aside[2];
                        uint16_t total_len;
                        strcpy(total, path);
                        step_aside_if_on_other_dest(game_map,
                                                    final_player,
                                                    final_box,
                                                    target_char,
                                                    final_last_dir,
                                                    aside);
                        if (aside[0] != '\0') {
                            total_len = (uint16_t)strlen(total);
                            append_str(total, &total_len, aside);
                        }
                        return SOKOBAN_OK;
                    }
                } else if ((s_last_error == SOKOBAN_ERR_SEARCH_OVERFLOW) ||
                           (s_last_error == SOKOBAN_ERR_INTERNAL)) {
                    return s_last_error;
                }
            }
        }
    }

    return SOKOBAN_NOT_FOUND;
}

SokobanStatus Sokoban_Solve(int game_map[SOKOBAN_MAP_ROW][SOKOBAN_MAP_COL],
                            char *out_path,
                            size_t out_size)
{
    char total[SOKOBAN_MAX_STEP];

    if ((game_map == NULL) || (out_path == NULL) || (out_size == 0u)) {
        return SOKOBAN_ERR_INVALID_ARGUMENT;
    }

    out_path[0] = '\0';
    s_last_error = SOKOBAN_OK;

    SokobanStatus st = solve_internal(game_map, total);
    if (st != SOKOBAN_OK) {
        return st;
    }

    size_t len = strlen(total);
    if ((len + 1u) > out_size) {
        return SOKOBAN_ERR_BUFFER_TOO_SMALL;
    }

    memcpy(out_path, total, len + 1u);
    return SOKOBAN_OK;
}

const char *Sokoban_StatusString(SokobanStatus status)
{
    switch (status) {
    case SOKOBAN_OK:
        return "OK";
    case SOKOBAN_NOT_FOUND:
        return "No path found";
    case SOKOBAN_ERR_INVALID_ARGUMENT:
        return "Invalid argument";
    case SOKOBAN_ERR_BUFFER_TOO_SMALL:
        return "Output buffer too small";
    case SOKOBAN_ERR_TOO_MANY_OBJECTS:
        return "Too many objects";
    case SOKOBAN_ERR_SEARCH_OVERFLOW:
        return "Search node pool overflow";
    case SOKOBAN_ERR_INTERNAL:
        return "Internal solver error";
    default:
        return "Unknown status";
    }
}

uint32_t Sokoban_GetStaticWorkspaceBytes(void)
{
    uint32_t total = 0u;
    total += (uint32_t)sizeof(s_boxes);
    total += (uint32_t)sizeof(s_dests);
    total += (uint32_t)sizeof(s_bombs);
    total += (uint32_t)sizeof(s_observe_targets);
    total += (uint32_t)sizeof(s_human_step);
    total += (uint32_t)sizeof(s_human_turn);
    total += (uint32_t)sizeof(s_human_done);
    total += (uint32_t)sizeof(s_human_prev_x);
    total += (uint32_t)sizeof(s_human_prev_y);
    total += (uint32_t)sizeof(s_human_prev_dir);
    total += (uint32_t)sizeof(s_human_heap);
    total += (uint32_t)sizeof(s_box_nodes);
    total += (uint32_t)sizeof(s_box_hash);
    total += (uint32_t)sizeof(s_box_heap);
    total += (uint32_t)sizeof(s_bomb_nodes);
    total += (uint32_t)sizeof(s_bomb_queue);
    total += (uint32_t)sizeof(s_bomb_vis_bits);
    return total;
}
