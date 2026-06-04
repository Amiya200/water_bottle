/**
 * @file json_lite.h
 * @brief Minimal JSON extraction for flat command objects + small builder.
 *        Not a general parser - tuned for the HydraSense command schema
 *        (flat keys, one level of nesting for reminder_window).
 */
#ifndef JSON_LITE_H
#define JSON_LITE_H
#include "board.h"

/* Find "key":"value" string. Copies into out (size lim). Returns true if found. */
bool json_get_str(const char *json, const char *key, char *out, uint16_t lim);
/* Find "key": number (int). Returns true and sets *out if found. */
bool json_get_int(const char *json, const char *key, int32_t *out);
/* Find nested value inside an object value of `parent`. */
bool json_get_int_in(const char *json, const char *parent, const char *key, int32_t *out);
bool json_get_str_in(const char *json, const char *parent, const char *key, char *out, uint16_t lim);

/* ---- builder: append into a fixed buffer, tracks position ---- */
typedef struct { char *buf; uint16_t cap; uint16_t len; } jbuild_t;
void jb_init(jbuild_t *b, char *buf, uint16_t cap);
void jb_obj_open(jbuild_t *b);
void jb_obj_close(jbuild_t *b);
void jb_kv_str(jbuild_t *b, const char *k, const char *v);
void jb_kv_int(jbuild_t *b, const char *k, int32_t v);
void jb_kv_bool(jbuild_t *b, const char *k, bool v);
void jb_kv_raw(jbuild_t *b, const char *k, const char *raw); /* raw value */
void jb_raw(jbuild_t *b, const char *s);

#endif
