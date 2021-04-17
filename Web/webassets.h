/*
 * webassets.h
 *
 *  Created on: Feb 13, 2021
 *      Author: Clayton
 */

#ifndef WEB_WEBASSETS_H_
#define WEB_WEBASSETS_H_

#ifdef __cplusplus
extern "C"
{
#endif

// Add linker symbols here
// NOTE: do not reference _size as it disables the final binary compression and the compiler is running.

extern const uint8_t _binary____Web_index_html_start[];
extern const uint8_t _binary____Web_index_html_end;

// @formatter:off
typedef enum WebAssetMap { ASSET_INDEX, ASSET_ERROR = 0xFF } web_FileAssetItem_Type;
typedef struct { web_FileAssetItem_Type key; const char *name; const size_t endaddr; const char *data; } web_FileAssetItem_t;
static __attribute__ ((unused)) web_FileAssetItem_t webAssets[] = {
  { .key = ASSET_INDEX,
    .name = "index.html",
    .endaddr = (const size_t) &_binary____Web_index_html_end,
    .data = (const char *) _binary____Web_index_html_start
  }
};
#define NWEBASSETS (sizeof(webAssets)/sizeof(web_FileAssetItem_t))
// @formatter:on

#ifdef __cplusplus
}
#endif

#endif /* WEB_WEBASSETS_H_ */
