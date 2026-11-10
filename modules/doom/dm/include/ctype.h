/* Doom-on-ML libc shim: ctype (ASCII only, no locale table). */
#ifndef DM_CTYPE_H
#define DM_CTYPE_H

static inline int dm_isdigit(int c)  { return c >= '0' && c <= '9'; }
static inline int dm_isupper(int c)  { return c >= 'A' && c <= 'Z'; }
static inline int dm_islower(int c)  { return c >= 'a' && c <= 'z'; }
static inline int dm_isalpha(int c)  { return dm_isupper(c) || dm_islower(c); }
static inline int dm_isalnum(int c)  { return dm_isalpha(c) || dm_isdigit(c); }
static inline int dm_isspace(int c)  { return c == ' ' || (c >= '\t' && c <= '\r'); }
static inline int dm_isxdigit(int c) { return dm_isdigit(c) || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'); }
static inline int dm_isprint(int c)  { return c >= 0x20 && c < 0x7f; }
static inline int dm_ispunct(int c)  { return dm_isprint(c) && !dm_isalnum(c) && c != ' '; }
static inline int dm_iscntrl(int c)  { return (c >= 0 && c < 0x20) || c == 0x7f; }
static inline int dm_toupper(int c)  { return dm_islower(c) ? c - 32 : c; }
static inline int dm_tolower(int c)  { return dm_isupper(c) ? c + 32 : c; }

#define isdigit  dm_isdigit
#define isupper  dm_isupper
#define islower  dm_islower
#define isalpha  dm_isalpha
#define isalnum  dm_isalnum
#define isspace  dm_isspace
#define isxdigit dm_isxdigit
#define isprint  dm_isprint
#define ispunct  dm_ispunct
#define iscntrl  dm_iscntrl
#define toupper  dm_toupper
#define tolower  dm_tolower

#endif
